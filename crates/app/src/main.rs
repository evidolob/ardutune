//! ArduTune desktop app: a Tauri window around the core analyses and the
//! vehicle link. The frontend in `ui/` is plain HTML/JS with no build step.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod analysis;
mod backups;
mod drones;
mod firmware;
mod history;
#[cfg(test)]
mod ipc_tests;
mod mcp;
mod vehicle;

use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use serde::Serialize;
use tauri::{AppHandle, Builder, Emitter, Manager, Runtime};

/// Errors reach the frontend as plain messages.
type CmdResult<T> = Result<T, String>;

/// Lock order, when more than one is needed: vehicle, then log.
#[derive(Default)]
pub struct AppState {
    pub log: Mutex<Option<analysis::Loaded>>,
    pub vehicle: Mutex<Option<vehicle::Connected>>,
    pub pending: Mutex<Option<vehicle::Pending>>,
    pub firmware: Mutex<Option<firmware::PendingFirmware>>,
    pub mcp: Mutex<mcp::State>,
}

/// Locks a mutex, recovering the data if a panicking command poisoned it.
fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// Runs blocking work (parsing, analysis, MAVLink) off the UI thread.
async fn blocking<R, T, F>(app: AppHandle<R>, f: F) -> CmdResult<T>
where
    R: Runtime,
    T: Send + 'static,
    F: FnOnce(&AppHandle<R>) -> CmdResult<T> + Send + 'static,
{
    tauri::async_runtime::spawn_blocking(move || f(&app)).await.map_err(|e| e.to_string())?
}

#[derive(Clone, Serialize)]
pub struct Progress<'a> {
    pub task: &'a str,
    pub done: usize,
    pub total: usize,
}

/// Sends `progress` events for a long task, about ten a second.
pub struct Reporter<'a, R: Runtime> {
    app: &'a AppHandle<R>,
    task: &'a str,
    last: Option<Instant>,
}

impl<'a, R: Runtime> Reporter<'a, R> {
    pub fn new(app: &'a AppHandle<R>, task: &'a str) -> Self {
        Self { app, task, last: None }
    }

    pub fn update(&mut self, done: usize, total: usize) {
        if self.last.is_some_and(|t| t.elapsed() < Duration::from_millis(100)) && done < total {
            return;
        }
        self.last = Some(Instant::now());
        let _ = self.app.emit("progress", Progress { task: self.task, done, total });
    }
}

/// A folder under Documents/ArduTune (or `ARDUTUNE_DATA_DIR`) for downloaded
/// logs and backups.
pub fn data_dir<R: Runtime>(app: &AppHandle<R>, sub: &str) -> CmdResult<PathBuf> {
    let root = match std::env::var_os("ARDUTUNE_DATA_DIR") {
        Some(dir) => PathBuf::from(dir),
        None => app.path().document_dir().map_err(|e| e.to_string())?.join("ArduTune"),
    };
    let dir = root.join(sub);
    std::fs::create_dir_all(&dir).map_err(|e| format!("creating {}: {e}", dir.display()))?;
    Ok(dir)
}

#[tauri::command]
async fn pick_file<R: Runtime>(app: AppHandle<R>, kind: String) -> CmdResult<Option<String>> {
    use tauri_plugin_dialog::DialogExt;
    blocking(app, move |app| {
        let dialog = app.dialog().file();
        let dialog = match kind.as_str() {
            "param" => dialog.set_title("Open a parameter file").add_filter("Parameter file", &["param", "parm", "txt"]),
            "apj" => dialog.set_title("Open a firmware file").add_filter("ArduPilot firmware", &["apj"]),
            _ => dialog.set_title("Open a flight log").add_filter("DataFlash log", &["bin", "BIN"]),
        };
        Ok(dialog.blocking_pick_file().and_then(|p| p.into_path().ok()).map(|p| p.display().to_string()))
    })
    .await
}

/// State and commands, shared by the app and the tests.
fn setup<R: Runtime>(builder: Builder<R>) -> Builder<R> {
    builder
        .manage(AppState::default())
        .setup(|app| {
            mcp::start_if_enabled(app.handle());
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            mcp::mcp_status,
            mcp::mcp_set_enabled,
            mcp::mcp_new_token,
            pick_file,
            analysis::open_log,
            analysis::close_log,
            analysis::open_sample,
            analysis::analysis,
            vehicle::list_ports,
            vehicle::connect,
            vehicle::disconnect,
            vehicle::vehicle_status,
            vehicle::list_vehicle_logs,
            vehicle::download_vehicle_log,
            vehicle::delete_vehicle_log,
            vehicle::erase_vehicle_logs,
            vehicle::prepare_changes,
            vehicle::apply_changes,
            vehicle::reboot,
            vehicle::arming_check,
            vehicle::setup_guide,
            vehicle::vehicle_pack,
            vehicle::initial_tune,
            vehicle::motor_layout,
            vehicle::motor_test,
            vehicle::motor_check,
            vehicle::detect_switch,
            vehicle::rc_status,
            vehicle::rc_choices,
            vehicle::gps_status,
            vehicle::gps_accept_position,
            vehicle::serial_status,
            vehicle::vehicle_config,
            vehicle::autotune_status,
            vehicle::compass_cal_start,
            vehicle::compass_cal_poll,
            vehicle::compass_cal_finish,
            vehicle::param_table,
            firmware::firmware_info,
            firmware::firmware_releases,
            firmware::firmware_prepare,
            firmware::firmware_flash,
            backups::backup_params,
            backups::list_backups,
            backups::delete_backup,
            backups::assign_backup,
            drones::name_drone,
            drones::list_drones,
            history::flight_history,
            history::import_log,
        ])
}

fn main() {
    setup(tauri::Builder::default().plugin(tauri_plugin_dialog::init())).run(tauri::generate_context!()).expect("error while running ArduTune");
}
