//! Vehicle commands: connect, read parameters, download logs and write
//! parameter changes (backup, write, verify, reboot).

use std::collections::HashMap;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use ardutune_core::utc::{date_time, file_stamp};
use ardutune_core::{arming, autotune, config, gps, logging, motors, paramdoc, params, rc, serial, setup, switch, tune};
use ardutune_link::{plan, ports, CompassCal, Link, Params};
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager, Runtime};

use crate::drones::{self, Drone};
use crate::{backups, blocking, data_dir, lock, AppState, CmdResult, Reporter};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const REBOOT_TIMEOUT: Duration = Duration::from_secs(40);
const SETUP_LISTEN: Duration = Duration::from_secs(3);

pub struct Connected {
    pub(crate) link: Link,
    pub(crate) params: Params,
    pub(crate) address: String,
    /// Default values, once read from the vehicle (`None` until then, or
    /// when the firmware doesn't offer them).
    pub(crate) defaults: Option<HashMap<String, f32>>,
    /// The compass calibration being followed, if any.
    pub(crate) compass_cal: Option<CompassCal>,
    /// Which vehicle this is, when its flight controller reports a chip ID.
    pub(crate) drone: Option<Drone>,
    /// The latest pre-arm check, whose fixes can be written.
    pub(crate) last_arming: Option<arming::Report>,
    /// GPS page: a fix was missing at some point, and the time after power-up
    /// (s) the first 3D fix then came.
    pub(crate) gps_fix: GpsFixTiming,
    /// The latest GPS page, whose fixes can be written.
    pub(crate) last_gps: Option<gps::Page>,
}

#[derive(Default)]
pub(crate) struct GpsFixTiming {
    pub(crate) saw_no_fix: bool,
    pub(crate) first_fix_s: Option<f64>,
}

/// Changes shown to the user and waiting for confirmation. They are planned
/// again against the vehicle's parameters when written.
pub struct Pending {
    wanted: Vec<(String, f64)>,
    /// What is being done ("writing recommended filter settings"), for the
    /// backup's reason line.
    label: String,
}

#[derive(Serialize)]
pub struct VehicleInfo {
    address: String,
    kind: &'static str,
    ardupilot: bool,
    system_id: u8,
    armed: bool,
    params: usize,
    /// Flight controller chip ID, board and the pilot's name for it.
    uid: Option<String>,
    board: Option<String>,
    name: Option<String>,
    /// The name, or board and ID while unnamed.
    label: Option<String>,
}

impl Connected {
    pub fn params(&self) -> &Params {
        &self.params
    }

    pub fn system_id(&self) -> u8 {
        self.link.vehicle().system_id
    }

    /// A parameter's default, once read from the vehicle (Parameters page).
    pub fn default_of(&self, name: &str) -> Option<f64> {
        self.defaults.as_ref().and_then(|d| d.get(name)).map(|v| *v as f64)
    }

    pub fn defaults_unknown(&self) -> bool {
        self.defaults.is_none()
    }

    pub(crate) fn info(&self) -> VehicleInfo {
        let v = self.link.vehicle();
        VehicleInfo {
            address: self.address.clone(),
            kind: v.type_name(),
            ardupilot: v.is_ardupilot(),
            system_id: v.system_id,
            armed: v.armed,
            params: self.params.len(),
            uid: self.drone.as_ref().and_then(|d| d.uid.clone()),
            board: self.drone.as_ref().and_then(|d| d.board.clone()),
            name: self.drone.as_ref().and_then(|d| d.name.clone()),
            label: self.drone.as_ref().map(Drone::label),
        }
    }

    /// Re-reads this vehicle's entry (its name may have changed).
    pub(crate) fn refresh_drone<R: Runtime>(&mut self, app: &AppHandle<R>) {
        if let Some(uid) = self.drone.as_ref().map(|d| d.key().to_owned()) {
            if let Some(d) = drones::find(app, &uid) {
                self.drone = Some(d);
            }
        }
    }

    /// Asks the flight controller which one it is and records the visit.
    pub(crate) fn identify<R: Runtime>(&mut self, app: &AppHandle<R>) {
        // Not every autopilot reports a chip ID; backups then go untagged.
        let Ok(id) = self.link.identity() else { return };
        let Some(uid) = id.uid else { return };
        self.drone = drones::seen(app, &uid, id.serial.as_deref(), id.board.as_deref(), &date_time(now())).ok();
    }
}

/// Turns a link error into a message, forgetting a connection that is gone.
pub(crate) fn check<T>(slot: &mut Option<Connected>, r: ardutune_link::Result<T>) -> CmdResult<T> {
    r.map_err(|e| {
        if matches!(e, ardutune_link::Error::Disconnected) {
            *slot = None;
        }
        e.to_string()
    })
}

pub(crate) fn connected(slot: &mut Option<Connected>) -> CmdResult<&mut Connected> {
    slot.as_mut().ok_or_else(|| "Not connected to a vehicle.".to_owned())
}

/// Reads all parameters again, e.g. before a backup, in case another ground
/// station changed them.
pub fn refresh_params(slot: &mut Option<Connected>, progress: &mut dyn FnMut(usize, usize)) -> CmdResult<VehicleInfo> {
    let r = connected(slot)?.link.fetch_params(progress);
    let params = check(slot, r)?;
    let c = connected(slot)?;
    c.params = params;
    Ok(c.info())
}

/// Parameters the firmware owns: counters, versions and detected sensor IDs.
/// Writing old values back from a backup would be wrong or ignored.
fn vehicle_owned(name: &str) -> bool {
    name.starts_with("STAT_")
        || name.starts_with("SYSID_SW_")
        || name == "FORMAT_VERSION"
        || name.contains("DEV_ID")
        || name.contains("DEVID")
        || ((name.starts_with("INS_ACC") || name.starts_with("INS_GYR")) && name.ends_with("_ID"))
}

#[derive(Serialize)]
pub struct PortView {
    name: String,
    description: String,
    address: String,
    likely_autopilot: bool,
}

#[tauri::command]
pub fn list_ports() -> Vec<PortView> {
    ports::list()
        .into_iter()
        .map(|p| PortView { address: p.address(), name: p.name, description: p.description, likely_autopilot: p.likely_autopilot })
        .collect()
}

/// Connects (`auto` picks the flight controller's USB port) and reads all parameters.
#[tauri::command]
pub async fn connect<R: Runtime>(app: AppHandle<R>, address: String) -> CmdResult<VehicleInfo> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        *slot = None;
        let address = if address.trim().is_empty() || address == "auto" { ports::auto_address()? } else { address };
        let mut link = Link::connect(&address, CONNECT_TIMEOUT).map_err(|e| e.to_string())?;
        let mut progress = Reporter::new(app, "Reading parameters");
        let params = link.fetch_params(&mut |got, total| progress.update(got, total)).map_err(|e| e.to_string())?;
        let mut c =
            Connected { link, params, address, defaults: None, compass_cal: None, drone: None, last_arming: None, gps_fix: Default::default(), last_gps: None };
        c.identify(app);
        let info = c.info();
        *slot = Some(c);
        Ok(info)
    })
    .await
}

#[tauri::command]
pub async fn disconnect<R: Runtime>(app: AppHandle<R>) -> CmdResult<()> {
    blocking(app, |app| {
        *lock(&app.state::<AppState>().vehicle) = None;
        Ok(())
    })
    .await
}

/// The connected vehicle with a fresh armed state, or `None`.
#[tauri::command]
pub async fn vehicle_status<R: Runtime>(app: AppHandle<R>) -> CmdResult<Option<VehicleInfo>> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let Some(c) = slot.as_mut() else { return Ok(None) };
        let r = c.link.refresh_armed();
        check(&mut slot, r)?;
        Ok(slot.as_ref().map(Connected::info))
    })
    .await
}

#[derive(Serialize)]
pub struct LogView {
    id: u16,
    size_mb: f64,
    date: Option<String>,
}

#[tauri::command]
pub async fn list_vehicle_logs<R: Runtime>(app: AppHandle<R>) -> CmdResult<Vec<LogView>> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        log_list(&mut slot)
    })
    .await
}

/// The vehicle's logs, without the empty entries deleted files leave behind.
fn log_list(slot: &mut Option<Connected>) -> CmdResult<Vec<LogView>> {
    let r = connected(slot)?.link.list_logs();
    let entries = check(slot, r)?;
    Ok(entries
        .iter()
        .filter(|e| e.size > 0)
        // The flight controller's clock: wrong until GPS had a fix, so
        // dates before 2015 or in the future are left out.
        .map(|e| LogView {
            id: e.id,
            size_mb: e.size as f64 / 1e6,
            date: ardutune_core::history::plausible_time(i64::from(e.time_utc), now() as i64).then(|| date_time(e.time_utc as u64)),
        })
        .collect())
}

/// Deletes one log from the flight controller and returns the new list.
/// The newest log may still be open on the vehicle, so it needs
/// `reboot_first`: reboot, reconnect, then delete.
#[tauri::command]
pub async fn delete_vehicle_log<R: Runtime>(app: AppHandle<R>, id: u16, reboot_first: Option<bool>) -> CmdResult<Vec<LogView>> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let rebooted = reboot_first.unwrap_or(false);
        if rebooted {
            let c = slot.take().ok_or("Not connected to a vehicle.")?;
            let link = c.link.reboot_and_reconnect(&c.address, REBOOT_TIMEOUT).map_err(|e| format!("{e} (after rebooting)"))?;
            *slot = Some(Connected { link, ..c });
        }
        let r = connected(&mut slot)?.link.delete_log(id, rebooted);
        check(&mut slot, r)?;
        log_list(&mut slot)
    })
    .await
}

/// Erases every log on the flight controller.
#[tauri::command]
pub async fn erase_vehicle_logs<R: Runtime>(app: AppHandle<R>) -> CmdResult<Vec<LogView>> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.erase_all_logs();
        check(&mut slot, r)?;
        log_list(&mut slot)
    })
    .await
}

/// Downloads a log into Documents/ArduTune/logs and returns its path.
#[tauri::command]
pub async fn download_vehicle_log<R: Runtime>(app: AppHandle<R>, id: u16) -> CmdResult<String> {
    blocking(app, move |app| {
        let dir = data_dir(app, "logs")?;
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.list_logs();
        let entries = check(&mut slot, r)?;
        let entry = entries.into_iter().find(|e| e.id == id).ok_or(format!("The vehicle has no log {id}."))?;
        let mut progress = Reporter::new(app, "Downloading log");
        let r = connected(&mut slot)?.link.download_log(&entry, &mut |done, total| progress.update(done, total));
        let data = check(&mut slot, r)?;
        // The vehicle's name keeps logs of several vehicles apart (each
        // numbers its logs from 1); the log itself also records the ID.
        let who = connected(&mut slot)?.drone.as_ref().map(|d| format!("_{}", backups::slug(&d.label()))).unwrap_or_default();
        // No date: the flight controller's clock is only right after a GPS
        // fix. The power-up count logged inside tells flights apart instead
        // (log numbers restart after erasing), and downloading the same log
        // again replaces the copy.
        let boot = ardutune_core::log::Log::from_bytes(&data).param("STAT_BOOTCNT").filter(|b| *b > 0.0);
        let name = match boot {
            Some(b) => format!("log_{}_boot{}{who}.bin", entry.id, b as u32),
            None => format!("log_{}_downloaded-{}{who}.bin", entry.id, file_stamp(now())),
        };
        let path = dir.join(name);
        std::fs::write(&path, data).map_err(|e| format!("writing {}: {e}", path.display()))?;
        Ok(path.display().to_string())
    })
    .await
}

#[derive(Serialize)]
pub struct Row {
    name: String,
    current: f64,
    value: f64,
    reason: Option<String>,
}

#[derive(Serialize)]
pub struct PlanView {
    rows: Vec<Row>,
    unchanged: Vec<String>,
    unknown: Vec<String>,
    /// Left out of a file restore because the firmware owns them.
    skipped: Vec<String>,
    armed: bool,
    /// Set when a restored file was made on another vehicle.
    warning: Option<String>,
}

/// What some sources need besides their name: prop size and battery for the
/// initial tune, motor test results, captured radio ranges.
#[derive(Deserialize, Default)]
pub struct Inputs {
    tune: Option<TuneInput>,
    observations: Option<Vec<motors::Observation>>,
    /// (channel, min, max, centre) µs.
    captured: Option<Vec<(u8, u16, u16, u16)>>,
}

/// Works out what writing a set of recommendations would change on the
/// vehicle and keeps it for `apply_changes`. `source` names the
/// recommendations: `readiness`, `filters`, `filters_quieter`, `pid`,
/// `pid_autotune`, or `file` (a .param file at `path`).
#[tauri::command]
pub async fn prepare_changes<R: Runtime>(
    app: AppHandle<R>,
    source: String,
    path: Option<String>,
    label: Option<String>,
    values: Option<Vec<(String, f64)>>,
    inputs: Option<Inputs>,
) -> CmdResult<PlanView> {
    let Inputs { tune, observations, captured } = inputs.unwrap_or_default();
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        let mut wanted = match values {
            // Edits from the parameter list, with anything worth a second look.
            Some(values) if source == "edits" => values
                .into_iter()
                .map(|(n, v)| {
                    let warnings = paramdoc::warnings(&n, v);
                    (n, v, (!warnings.is_empty()).then(|| warnings.join("; ")))
                })
                .collect(),
            _ if source == "initial_tune" => {
                let t = tune.ok_or("No prop size given.")?;
                initial(&c.params, &t)?.changes.into_iter().map(|c| (c.name, c.value, Some(c.reason))).collect()
            }
            _ if source == "rc_calibration" => {
                let captured = captured.ok_or("No radio calibration captured.")?;
                rc::calibration(|n| c.params.get(n).map(|p| p.value as f64), &captured)?.into_iter().map(|c| (c.name, c.value, Some(c.reason))).collect()
            }
            _ if source == "motors" => {
                let seen = observations.ok_or("No motor test results given.")?;
                motor_verdict(&c.params, &seen)?.changes.into_iter().map(|c| (c.name, c.value, Some(c.reason))).collect()
            }
            _ => wanted(app, c, &source, path.clone())?,
        };
        let mut skipped = Vec::new();
        let warning = path.as_deref().filter(|_| source == "file").and_then(|p| backups::other_vehicle(app, p, c));
        if source == "file" {
            wanted.retain(|(n, ..)| {
                let owned = vehicle_owned(n);
                if owned && c.params.contains_key(n) {
                    skipped.push(n.clone());
                }
                !owned
            });
        }
        let pairs: Vec<(String, f64)> = wanted.iter().map(|(n, v, _)| (n.clone(), *v)).collect();
        let p = plan(&c.params, &pairs);
        let rows = p
            .changes
            .iter()
            .map(|ch| Row {
                name: ch.name.clone(),
                current: ch.current.value as f64,
                value: ch.value,
                reason: wanted.iter().find(|(n, ..)| *n == ch.name).and_then(|(.., r)| r.clone()),
            })
            .collect();
        let label = label.unwrap_or_else(|| format!("writing {source}"));
        *lock(&state.pending) = Some(Pending { wanted: pairs, label });
        Ok(PlanView { rows, unchanged: p.unchanged, unknown: p.unknown, skipped, armed: c.link.vehicle().armed, warning })
    })
    .await
}

use ardutune_core::sources::{self, Wanted};

fn wanted<R: Runtime>(app: &AppHandle<R>, c: &Connected, source: &str, path: Option<String>) -> CmdResult<Vec<Wanted>> {
    let live = &c.params;
    let ctx = sources::Context { last_arming: c.last_arming.as_ref(), last_gps: c.last_gps.as_ref() };
    if let Some(r) = sources::from_params(|n| live.get(n).map(|p| p.value as f64), source, &ctx) {
        return r;
    }
    if source == "file" {
        let path = path.ok_or("No parameter file given.")?;
        let text = std::fs::read_to_string(&path).map_err(|e| format!("reading {path}: {e}"))?;
        return Ok(params::parse(&text)?.into_iter().map(|(n, v)| (n, v, None)).collect());
    }
    let state = app.state::<AppState>();
    let slot = lock(&state.log);
    let loaded = slot.as_ref().ok_or("Open a log first.")?;
    let not_run = |what: &str| format!("Run the {what} analysis first.");
    Ok(match source {
        // Recomputed against the live values: they may have changed since the flight.
        "readiness" => {
            loaded.readiness.recommendations_for(|n| live.get(n).map(|p| p.value as f64)).into_iter().map(|c| (c.name, c.value, Some(c.reason))).collect()
        }
        "filters" | "filters_quieter" => {
            let r = loaded.filters.as_ref().ok_or_else(|| not_run("filter"))?;
            let changes = if source == "filters" { &r.changes } else { &r.quieter_changes };
            changes.iter().map(|c| (c.name.clone(), c.value, None)).collect()
        }
        "pid" | "pid_autotune" => {
            let r = loaded.pid.as_ref().ok_or_else(|| not_run("PID"))?;
            let chosen = if source == "pid" { &r.suggestions } else { &r.autotune };
            chosen.iter().map(|s| (s.name.clone(), s.value, Some(s.reason.clone()))).collect()
        }
        other => return Err(format!("unknown change source {other}")),
    })
}

/// Prop size and battery for the initial tune; cells and chemistry default
/// to the vehicle's MOT_BAT_VOLT_MAX/MIN.
#[derive(Deserialize)]
pub struct TuneInput {
    prop_in: f64,
    cells: Option<u32>,
    chemistry: Option<setup::Chemistry>,
}

fn initial(live: &Params, t: &TuneInput) -> CmdResult<tune::InitialTune> {
    tune::initial_tune(|n| live.get(n).map(|p| p.value as f64), t.prop_in, t.cells, t.chemistry)
}

#[derive(Serialize)]
pub struct PackView {
    cells: u32,
    chemistry: setup::Chemistry,
    from_vehicle: bool,
}

/// The battery the initial tune starts from, read from the vehicle.
#[tauri::command]
pub async fn vehicle_pack<R: Runtime>(app: AppHandle<R>) -> CmdResult<PackView> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        let (cells, chemistry, from_vehicle) = tune::vehicle_pack(|n| c.params.get(n).map(|p| p.value as f64));
        Ok(PackView { cells, chemistry, from_vehicle })
    })
    .await
}

/// Starting filter, acceleration and thrust values for a new build.
#[tauri::command]
pub async fn initial_tune<R: Runtime>(app: AppHandle<R>, tune: TuneInput) -> CmdResult<tune::InitialTune> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        initial(&connected(&mut slot)?.params, &tune)
    })
    .await
}

fn motor_verdict(live: &Params, seen: &[motors::Observation]) -> CmdResult<motors::Verdict> {
    motors::check(|n| live.get(n).map(|p| p.value as f64), seen)
}

/// The frame's motors in test order, with where each should be and which
/// way it should spin.
#[tauri::command]
pub async fn motor_layout<R: Runtime>(app: AppHandle<R>) -> CmdResult<motors::Layout> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        motors::layout(|n| c.params.get(n).map(|p| p.value as f64))
    })
    .await
}

/// Spins one motor (test letter `seq`, 1 = A) briefly; disarmed only.
#[tauri::command]
pub async fn motor_test<R: Runtime>(app: AppHandle<R>, seq: u8, throttle: f32, seconds: f32) -> CmdResult<()> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.motor_test(seq, throttle.clamp(3.0, 20.0), seconds.clamp(0.5, 3.0));
        check(&mut slot, r)
    })
    .await
}

/// What the motor test results mean, and the changes that fix them.
#[tauri::command]
pub async fn motor_check<R: Runtime>(app: AppHandle<R>, observations: Vec<motors::Observation>) -> CmdResult<motors::Verdict> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        motor_verdict(&connected(&mut slot)?.params, &observations)
    })
    .await
}

#[derive(Serialize)]
pub struct RcStatus {
    view: rc::RcView,
    /// (channel, min, max) seen while listening, for radio calibration.
    ranges: Vec<(u8, u16, u16)>,
    /// Signal strength in percent, when the receiver reports it.
    rssi_pct: Option<u8>,
    /// TUNE, TUNE_MIN, TUNE_MAX and the gains a tuning knob can set, for its defaults.
    tuning: std::collections::BTreeMap<String, f64>,
}

/// The RC channels now: values, what each does, switch positions, modes.
#[tauri::command]
pub async fn rc_status<R: Runtime>(app: AppHandle<R>) -> CmdResult<RcStatus> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.rc_sample(Duration::from_millis(250));
        let sample = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        let view = rc::view(|n| c.params.get(n).map(|p| p.value as f64), &sample.last);
        let rssi_pct = sample.rssi.map(|r| ((u32::from(r) * 100 + 127) / 254) as u8);
        let names = ["TUNE", "TUNE_MIN", "TUNE_MAX"].into_iter().chain((1..=61).filter_map(|t| rc::tune_target(f64::from(t))));
        let tuning = names.filter_map(|n| c.params.get(n).map(|p| (n.to_owned(), p.value as f64))).collect();
        Ok(RcStatus { view, ranges: sample.ranges, rssi_pct, tuning })
    })
    .await
}

/// The GPS page: each GPS live, the EKF's view, checks and settings.
#[tauri::command]
pub async fn gps_status<R: Runtime>(app: AppHandle<R>) -> CmdResult<gps::Page> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.gps_sample(Duration::from_millis(600));
        let sample = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        let fixes: Vec<gps::Fix> = sample
            .gps
            .iter()
            .map(|g| gps::Fix {
                instance: g.instance,
                fix_type: g.fix_type,
                satellites: g.satellites,
                hdop: g.hdop,
                speed: g.speed,
                course: g.course,
                position: g.position,
                h_acc: g.h_acc,
                v_acc: g.v_acc,
            })
            .collect();
        // Time to first fix, when the page saw it arrive.
        let has_fix = fixes.iter().any(|g| g.instance == 0 && g.fix_type >= 3);
        if !has_fix {
            c.gps_fix.saw_no_fix = true;
        } else if c.gps_fix.saw_no_fix && c.gps_fix.first_fix_s.is_none() {
            c.gps_fix.first_fix_s = sample.boot_s;
        }
        let live = gps::Live {
            gps: fixes,
            ekf: sample.ekf.map(|e| gps::Ekf { flags: e.flags, velocity: e.velocity, pos_horiz: e.pos_horiz, pos_vert: e.pos_vert, compass: e.compass }),
            armed: c.link.vehicle().armed,
            boot_s: sample.boot_s,
            first_fix_s: c.gps_fix.first_fix_s,
            messages: sample.messages,
            last_known: c.drone.as_ref().and_then(|d| d.last_position.clone()),
        };
        let page = gps::page(|n| c.params.get(n).map(|p| p.value as f64), &live);
        // Remember a trusted position, once it moved (or the day changed).
        if page.trusted {
            if let (Some(d), Some((lat, lon, _))) = (&c.drone, live.gps.iter().find(|g| g.fix_type >= 3).and_then(|g| g.position)) {
                let today = date_time(now())[..10].to_owned();
                let stale = d.last_position.as_ref().is_none_or(|k| k.when != today || gps::distance_km((k.lat, k.lon), (lat, lon)) > 1.0);
                if stale {
                    if let Ok(Some(updated)) = drones::set_position(app, d.key(), Some(gps::Known { lat, lon, when: today })) {
                        c.drone = Some(updated);
                    }
                }
            }
        }
        c.last_gps = Some(page.clone());
        Ok(page)
    })
    .await
}

/// Which logging the vehicle has (tuning or normal logs), and the changes to
/// switch between them.
#[tauri::command]
pub async fn logging_status<R: Runtime>(app: AppHandle<R>) -> CmdResult<logging::View> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        Ok(logging::view(|n| c.params.get(n).map(|p| p.value as f64)))
    })
    .await
}

/// The AutoTune page: tuning steps, checks, settings for the prop size,
/// QuikTune setup, and what the open log says AutoTune saved.
#[tauri::command]
pub async fn autotune_status<R: Runtime>(app: AppHandle<R>, prop_in: Option<f64>) -> CmdResult<autotune::View> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let log = lock(&state.log).as_ref().map(|l| l.log.clone());
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        Ok(autotune::view(|n| c.params.get(n).map(|p| p.value as f64), prop_in, log.as_deref()))
    })
    .await
}

/// The basic configuration page: frame, ESCs, board and battery settings.
#[tauri::command]
pub async fn vehicle_config<R: Runtime>(app: AppHandle<R>) -> CmdResult<config::View> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        Ok(config::view(|n| c.params.get(n).map(|p| p.value as f64)))
    })
    .await
}

/// The serial ports page: each port, what's live on it and checks.
#[tauri::command]
pub async fn serial_status<R: Runtime>(app: AppHandle<R>) -> CmdResult<serial::View> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.uart_info();
        let report = check(&mut slot, r)?;
        let r = connected(&mut slot)?.link.gps_sample(Duration::from_millis(300));
        let gps = check(&mut slot, r)?;
        let r = connected(&mut slot)?.link.rc_sample(Duration::from_millis(450));
        let rc = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        let uarts = report.map(|t| serial::parse_uarts(&t)).unwrap_or_default();
        let live = serial::Live {
            gps_fix: gps.gps.iter().map(|g| (g.instance, gps::fix_label(g.fix_type))).collect(),
            rc_signal: Some(rc.last.iter().take(4).any(|v| *v > 0 && *v != u16::MAX)),
        };
        Ok(serial::view(|n| c.params.get(n).map(|p| p.value as f64), &uarts, &live))
    })
    .await
}

/// "The vehicle really is here": the current position becomes the one to
/// compare with.
#[tauri::command]
pub async fn gps_accept_position<R: Runtime>(app: AppHandle<R>) -> CmdResult<()> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        let d = c.drone.as_ref().ok_or("This flight controller doesn't report a chip ID, so ArduTune can't remember its position.")?;
        let at = c.last_gps.as_ref().and_then(|p| p.units.iter().find_map(|u| u.position)).ok_or("No GPS position yet.")?;
        let today = date_time(now())[..10].to_owned();
        c.drone = drones::set_position(app, d.key(), Some(gps::Known { lat: at.0, lon: at.1, when: today }))?;
        Ok(())
    })
    .await
}

#[derive(Serialize)]
pub struct RcChoices {
    modes: Vec<(f64, String)>,
    options: Vec<(f64, String)>,
    /// TUNE choices for a tuning knob, with the parameter each one sets when known.
    tune: Vec<(f64, String, Option<&'static str>)>,
    knob_options: Vec<f64>,
}

/// Flight modes and switch functions to choose from.
#[tauri::command]
pub fn rc_choices() -> RcChoices {
    let (modes, options, tune) = rc::choices();
    let tune = tune.into_iter().map(|(v, n)| (v, n, rc::tune_target(v))).collect();
    RcChoices { modes, options, tune, knob_options: rc::KNOB_OPTIONS.to_vec() }
}

/// Watches the radio for `listen_s` seconds and returns the channel of the
/// switch the pilot flipped, checked to be free for `option` (RCn_OPTION).
#[tauri::command]
pub async fn detect_switch<R: Runtime>(app: AppHandle<R>, listen_s: Option<f64>, option: f64) -> CmdResult<u8> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let listen = Duration::from_secs_f64(listen_s.unwrap_or(6.0).clamp(1.0, 30.0));
        let r = connected(&mut slot)?.link.rc_ranges(listen);
        let moves = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        switch::flipped(|n| c.params.get(n).map(|p| p.value as f64), &moves, option)
    })
    .await
}

/// Starts calibrating every compass; follow with `compass_cal_poll`.
#[tauri::command]
pub async fn compass_cal_start<R: Runtime>(app: AppHandle<R>) -> CmdResult<()> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.compass_cal_start();
        check(&mut slot, r)?;
        connected(&mut slot)?.compass_cal = Some(CompassCal::default());
        Ok(())
    })
    .await
}

/// The calibration's progress after listening briefly.
#[tauri::command]
pub async fn compass_cal_poll<R: Runtime>(app: AppHandle<R>) -> CmdResult<CompassCal> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        let mut cal = c.compass_cal.take().ok_or("No compass calibration is running.")?;
        let r = c.link.compass_cal_poll(&mut cal, Duration::from_millis(400));
        connected(&mut slot)?.compass_cal = Some(cal.clone());
        check(&mut slot, r)?;
        Ok(cal)
    })
    .await
}

/// Cancels a running calibration, or accepts a finished one that wasn't saved.
#[tauri::command]
pub async fn compass_cal_finish<R: Runtime>(app: AppHandle<R>, accept: bool) -> CmdResult<()> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.compass_cal_finish(accept);
        check(&mut slot, r)?;
        if !accept {
            connected(&mut slot)?.compass_cal = None;
        }
        Ok(())
    })
    .await
}

#[derive(Serialize)]
pub struct WriteResult {
    name: String,
    value: Option<f32>,
    error: Option<String>,
}

#[derive(Serialize)]
pub struct ApplyView {
    backup: Option<String>,
    results: Vec<WriteResult>,
}

/// Writes and verifies the prepared changes, or only those named in `names`
/// (the ones the user kept ticked); with `backup`, saves all parameters first.
#[tauri::command]
pub async fn apply_changes<R: Runtime>(app: AppHandle<R>, names: Option<Vec<String>>, backup: Option<bool>) -> CmdResult<ApplyView> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let pending = lock(&state.pending).take().ok_or("Nothing to write.")?;
        let c = connected(&mut slot)?;
        let mut changes = plan(&c.params, &pending.wanted).changes;
        if let Some(names) = &names {
            changes.retain(|ch| names.contains(&ch.name));
        }
        if changes.is_empty() {
            return Ok(ApplyView { backup: None, results: vec![] });
        }
        let backup = match backup {
            Some(true) => Some(backups::save(app, c, &format!("Before {}", pending.label))?),
            _ => None,
        };

        let mut progress = Reporter::new(app, "Writing parameters");
        let mut written = 0;
        let r = c.link.apply(&changes, &mut |_, _| {
            written += 1;
            progress.update(written, changes.len());
        });
        let outcomes = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        let mut results = Vec::new();
        for (ch, r) in changes.iter().zip(outcomes) {
            match r {
                Ok(v) => {
                    if let Some(p) = c.params.get_mut(&ch.name) {
                        p.value = v;
                    }
                    results.push(WriteResult { name: ch.name.clone(), value: Some(v), error: None });
                }
                Err(e) => results.push(WriteResult { name: ch.name.clone(), value: None, error: Some(e.to_string()) }),
            }
        }
        let view = ApplyView { backup: backup.map(|b| b.display().to_string()), results };
        // For the AI assistant's last_write_result.
        lock(&state.mcp).last_write = serde_json::to_value(&view).ok();
        Ok(view)
    })
    .await
}

/// Runs the vehicle's pre-arm checks, listens for `listen_s` seconds (long
/// enough to try arming meanwhile) and explains what blocks arming.
#[tauri::command]
pub async fn arming_check<R: Runtime>(app: AppHandle<R>, listen_s: Option<f64>) -> CmdResult<arming::Report> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let listen = Duration::from_secs_f64(listen_s.unwrap_or(6.0).clamp(1.0, 120.0));
        let r = connected(&mut slot)?.link.arming_status(listen);
        let status = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        let report = arming::assess(&status.snapshot(), |n| c.params.get(n).map(|p| p.value as f64));
        c.last_arming = Some(report.clone());
        Ok(report)
    })
    .await
}

/// Checks the vehicle's setup step by step: its parameters plus a few
/// seconds of live status (receiver, sensors, logging, battery).
#[tauri::command]
pub async fn setup_guide<R: Runtime>(app: AppHandle<R>) -> CmdResult<setup::Guide> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.arming_status(SETUP_LISTEN);
        let status = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        Ok(setup::assess(|n| c.params.get(n).map(|p| p.value as f64), &status.setup_live()))
    })
    .await
}

#[derive(Serialize)]
pub struct ParamRow {
    name: String,
    value: f64,
    default: Option<f64>,
    /// Integer parameters take whole numbers only.
    integer: bool,
    doc: Option<&'static paramdoc::Doc>,
}

#[derive(Serialize)]
pub struct ParamTable {
    rows: Vec<ParamRow>,
    /// Whether defaults came from the vehicle.
    has_defaults: bool,
    docs_source: String,
    docs_fetched: String,
}

/// Every parameter with its current value, default and documentation. The
/// values and defaults are read fresh from the vehicle's packed parameter
/// file (fast), falling back to the values read on connect.
#[tauri::command]
pub async fn param_table<R: Runtime>(app: AppHandle<R>) -> CmdResult<ParamTable> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.param_defaults();
        let packed = check(&mut slot, r)?;
        let c = connected(&mut slot)?;
        if let Some(packed) = packed {
            for p in &packed {
                if let Some(param) = c.params.get_mut(&p.name) {
                    param.value = p.value;
                }
            }
            c.defaults = Some(packed.into_iter().filter_map(|p| Some((p.name, p.default?))).collect());
        }
        let rows = c
            .params
            .iter()
            .map(|(name, p)| ParamRow {
                name: name.clone(),
                value: p.value as f64,
                default: c.defaults.as_ref().and_then(|d| d.get(name)).map(|v| *v as f64),
                integer: p.is_integer(),
                doc: paramdoc::doc(name),
            })
            .collect();
        let docs = paramdoc::docs();
        Ok(ParamTable { rows, has_defaults: c.defaults.is_some(), docs_source: docs.source.clone(), docs_fetched: docs.fetched.clone() })
    })
    .await
}

/// Reboots the flight controller; the connection is closed.
#[tauri::command]
pub async fn reboot<R: Runtime>(app: AppHandle<R>) -> CmdResult<()> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.reboot();
        check(&mut slot, r)?;
        *slot = None;
        Ok(())
    })
    .await
}

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs())
}
