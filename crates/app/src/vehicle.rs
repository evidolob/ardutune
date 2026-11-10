//! Vehicle commands: connect, read parameters, download logs and write
//! parameter changes (backup, write, verify, reboot).

use std::path::PathBuf;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use ardutune_core::params;
use ardutune_core::utc::{date_time, file_stamp};
use ardutune_link::{plan, ports, Link, Params};
use serde::Serialize;
use tauri::{AppHandle, Manager, Runtime};

use crate::{blocking, data_dir, lock, AppState, CmdResult, Reporter};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

pub struct Connected {
    link: Link,
    params: Params,
    address: String,
}

/// Changes shown to the user and waiting for confirmation. They are planned
/// again against the vehicle's parameters when written.
pub struct Pending {
    wanted: Vec<(String, f64)>,
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
    fn info(&self) -> VehicleInfo {
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
fn check<T>(slot: &mut Option<Connected>, r: ardutune_link::Result<T>) -> CmdResult<T> {
    r.map_err(|e| {
        if matches!(e, ardutune_link::Error::Disconnected) {
            *slot = None;
        }
        e.to_string()
    })
}

fn connected(slot: &mut Option<Connected>) -> CmdResult<&mut Connected> {
    slot.as_mut().ok_or_else(|| "Not connected to a vehicle.".to_owned())
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
        let c = Connected { link, params, address };
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
#[tauri::command]
pub async fn delete_vehicle_log<R: Runtime>(app: AppHandle<R>, id: u16) -> CmdResult<Vec<LogView>> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.delete_log(id);
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
    armed: bool,
}

/// Works out what writing a set of recommendations would change on the
/// vehicle and keeps it for `apply_changes`. `source` names the
/// recommendations: `readiness`, `filters`, `filters_quieter`, `pid`,
/// `pid_autotune`, or `file` (a .param file at `path`).
#[tauri::command]
pub async fn prepare_changes<R: Runtime>(app: AppHandle<R>, source: String, path: Option<String>) -> CmdResult<PlanView> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let c = connected(&mut slot)?;
        let wanted = wanted(app, &c.params, &source, path)?;
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
        *lock(&state.pending) = Some(Pending { wanted: pairs });
        Ok(PlanView { rows, unchanged: p.unchanged, unknown: p.unknown, armed: c.link.vehicle().armed })
    })
    .await
}

type Wanted = (String, f64, Option<String>);

fn wanted<R: Runtime>(app: &AppHandle<R>, live: &Params, source: &str, path: Option<String>) -> CmdResult<Vec<Wanted>> {
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

/// Backs up all parameters, then writes and verifies the prepared changes.
#[tauri::command]
pub async fn apply_changes<R: Runtime>(app: AppHandle<R>) -> CmdResult<ApplyView> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let pending = lock(&state.pending).take().ok_or("Nothing to write.")?;
        let c = connected(&mut slot)?;
        let changes = plan(&c.params, &pending.wanted).changes;
        if changes.is_empty() {
            return Ok(ApplyView { backup: None, results: vec![] });
        }
        let backup = save_backup(app, c)?;

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
        Ok(ApplyView { backup: Some(backup.display().to_string()), results })
    })
    .await
}

/// Saves every parameter so a change can be undone by writing the file back.
fn save_backup<R: Runtime>(app: &AppHandle<R>, c: &Connected) -> CmdResult<PathBuf> {
    let now = now();
    let sys = c.link.vehicle().system_id;
    let path = data_dir(app, "backups")?.join(format!("backup-sys{sys}-{}.param", file_stamp(now)));
    let header = format!(
        "ArduTune parameter backup, {} UTC, system {sys}.\nRestore by writing this file to the vehicle.",
        date_time(now)
    );
    let text = params::write(&header, c.params.iter().map(|(n, p)| (n.as_str(), p.value)));
    std::fs::write(&path, text).map_err(|e| format!("writing {}: {e}", path.display()))?;
    Ok(path)
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
