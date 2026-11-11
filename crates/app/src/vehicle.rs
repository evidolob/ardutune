//! Vehicle commands: connect, read parameters, download logs and write
//! parameter changes (backup, write, verify, reboot).

use std::collections::HashMap;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use ardutune_core::{arming, paramdoc, params, setup, tune};
use ardutune_core::utc::{date_time, file_stamp};
use ardutune_link::{plan, ports, Link, Params};
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager, Runtime};

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
}

impl Connected {
    pub fn params(&self) -> &Params {
        &self.params
    }

    pub fn system_id(&self) -> u8 {
        self.link.vehicle().system_id
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
        }
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
        .map(|p| PortView {
            address: p.address(),
            name: p.name,
            description: p.description,
            likely_autopilot: p.likely_autopilot,
        })
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
        let c = Connected { link, params, address, defaults: None };
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
        .map(|e| LogView { id: e.id, size_mb: e.size as f64 / 1e6, date: (e.time_utc > 0).then(|| date_time(e.time_utc as u64)) })
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
        let name = if entry.time_utc > 0 {
            format!("log_{}_{}.bin", entry.id, file_stamp(entry.time_utc as u64))
        } else {
            format!("log_{}_downloaded-{}.bin", entry.id, file_stamp(now()))
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
    tune: Option<TuneInput>,
) -> CmdResult<PlanView> {
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
            _ => wanted(app, &c.params, &source, path)?,
        };
        let mut skipped = Vec::new();
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
        Ok(PlanView { rows, unchanged: p.unchanged, unknown: p.unknown, skipped, armed: c.link.vehicle().armed })
    })
    .await
}

type Wanted = (String, f64, Option<String>);

fn wanted<R: Runtime>(app: &AppHandle<R>, live: &Params, source: &str, path: Option<String>) -> CmdResult<Vec<Wanted>> {
    if let Some(step) = source.strip_prefix("setup:") {
        return setup_changes(live, step);
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
        "readiness" => loaded
            .readiness
            .recommendations_for(|n| live.get(n).map(|p| p.value as f64))
            .into_iter()
            .map(|c| (c.name, c.value, Some(c.reason)))
            .collect(),
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

/// Changes a setup guide step proposes, worked out from the live values.
fn setup_changes(live: &Params, step: &str) -> CmdResult<Vec<Wanted>> {
    let guide = setup::assess(|n| live.get(n).map(|p| p.value as f64), &setup::Live::default());
    let step = guide.steps.into_iter().find(|s| s.id == step).ok_or(format!("unknown setup step {step}"))?;
    Ok(step.changes.into_iter().map(|c| (c.name, c.value, Some(c.reason))).collect())
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
            Some(true) => Some(backups::save(app, c.system_id(), &c.params, &format!("Before {}", pending.label))?),
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
        Ok(ApplyView { backup: backup.map(|b| b.display().to_string()), results })
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
        Ok(arming::assess(&status.snapshot(), |n| c.params.get(n).map(|p| p.value as f64)))
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
        Ok(ParamTable {
            rows,
            has_defaults: c.defaults.is_some(),
            docs_source: docs.source.clone(),
            docs_fetched: docs.fetched.clone(),
        })
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
