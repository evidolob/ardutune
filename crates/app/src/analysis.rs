//! Opening logs and running the analyses on them (`ardutune_core::views`).
//! Results are cached per log.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use ardutune_core::filters::FilterReport;
use ardutune_core::pid::PidReport;
use ardutune_core::synth::{sample_flight, Preset};
pub use ardutune_core::views::View;
use ardutune_core::{views, Log, Report};
use serde_json::{json, Value};
use tauri::{AppHandle, Manager, Runtime};

use crate::{blocking, lock, AppState, CmdResult};

/// The open log and what has been worked out from it so far.
pub struct Loaded {
    pub id: u64,
    pub log: Arc<Log>,
    pub readiness: Report,
    pub filters: Option<Arc<FilterReport>>,
    pub pid: Option<Arc<PidReport>>,
    views: HashMap<String, CmdResult<View>>,
}

/// Closes the open log: the analyses forget it.
#[tauri::command]
pub fn close_log<R: Runtime>(app: AppHandle<R>) {
    *lock(&app.state::<AppState>().log) = None;
}

#[tauri::command]
pub async fn open_log<R: Runtime>(app: AppHandle<R>, path: String) -> CmdResult<Value> {
    blocking(app, move |app| load(app, PathBuf::from(path))).await
}

/// Writes the synthetic sample flight and opens it, to try the app without a log.
#[tauri::command]
pub async fn open_sample<R: Runtime>(app: AppHandle<R>) -> CmdResult<Value> {
    blocking(app, |app| {
        let path = app.path().temp_dir().map_err(|e| e.to_string())?.join("ardutune-sample.bin");
        std::fs::write(&path, sample_flight(Preset::Ready)).map_err(|e| format!("writing {}: {e}", path.display()))?;
        load(app, path)
    })
    .await
}

pub fn load<R: Runtime>(app: &AppHandle<R>, path: PathBuf) -> CmdResult<Value> {
    let bytes = std::fs::read(&path).map_err(|e| format!("reading {}: {e}", path.display()))?;
    let (log, readiness, mut overview) = read(&path, &bytes);
    // Which vehicle flew it, from the flight controller ID in the banner.
    if let Some(b) = views::board(&log) {
        let drone = crate::drones::find(app, &b.serial);
        let label = drone.as_ref().map_or_else(|| format!("{} …{}", b.board, crate::drones::short(&b.serial)), |d| d.label());
        overview["vehicle"] = json!({ "label": label, "board": b.board, "named": drone.is_some_and(|d| d.name.is_some()) });
    }
    let state = app.state::<AppState>();
    let mut slot = lock(&state.log);
    let id = slot.as_ref().map_or(1, |l| l.id + 1);
    *slot = Some(Loaded { id, log: Arc::new(log), readiness, filters: None, pid: None, views: HashMap::new() });
    Ok(overview)
}

fn read(path: &Path, bytes: &[u8]) -> (Log, Report, Value) {
    views::open(&path.display().to_string(), path.file_name().map(|n| n.to_string_lossy().into_owned()), bytes)
}

#[tauri::command]
pub async fn analysis<R: Runtime>(app: AppHandle<R>, kind: String) -> CmdResult<View> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let (id, log) = {
            let slot = lock(&state.log);
            let loaded = slot.as_ref().ok_or("Open a log first.")?;
            if let Some(v) = loaded.views.get(&kind) {
                return v.clone();
            }
            (loaded.id, loaded.log.clone())
        };

        let computed = views::compute(&kind, &log)?;
        let result = computed.view;

        // Keep the result unless another log was opened meanwhile.
        let mut slot = lock(&state.log);
        if let Some(loaded) = slot.as_mut().filter(|l| l.id == id) {
            loaded.filters = computed.filters.map(Arc::new).or(loaded.filters.take());
            loaded.pid = computed.pid.map(Arc::new).or(loaded.pid.take());
            loaded.views.insert(kind, result.clone());
        }
        result
    })
    .await
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Writes what the app would show for a log to `dev/fixtures`, for
    /// previewing the UI in a browser (see `dev/preview.html`):
    /// `ARDUTUNE_FIXTURE_LOG=flight.bin cargo test -p ardutune-app -- --ignored export_fixtures`
    #[test]
    #[ignore]
    fn export_fixtures() {
        let path = PathBuf::from(std::env::var("ARDUTUNE_FIXTURE_LOG").expect("set ARDUTUNE_FIXTURE_LOG"));
        let bytes = std::fs::read(&path).unwrap();
        let (log, _, overview) = read(&path, &bytes);
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("dev/fixtures");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("overview.json"), overview.to_string()).unwrap();
        for kind in views::KINDS {
            let out = match views::compute(kind, &log).unwrap().view {
                Ok(v) => serde_json::to_value(v).unwrap(),
                Err(e) => json!({ "error": e }),
            };
            std::fs::write(dir.join(format!("{kind}.json")), out.to_string()).unwrap();
        }
    }
}
