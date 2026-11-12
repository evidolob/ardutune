//! Opening logs and running the analyses on them. Results are cached per log.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use ardutune_core::diagnose::diagnose;
use ardutune_core::filters::{self, Evaluation, FilterReport};
use ardutune_core::pid::{self, PidReport};
use ardutune_core::synth::{sample_flight, Preset};
use ardutune_core::{assess, log, noise, plot, Log, Report};
use serde::Serialize;
use serde_json::{json, Value};
use tauri::{AppHandle, Manager, Runtime};

use crate::{blocking, lock, AppState, CmdResult};

/// Numeric arrays longer than this are plot data the frontend gets as SVG.
const MAX_ARRAY: usize = 64;

/// The open log and what has been worked out from it so far.
pub struct Loaded {
    pub id: u64,
    pub log: Arc<Log>,
    pub readiness: Report,
    pub filters: Option<Arc<FilterReport>>,
    pub pid: Option<Arc<PidReport>>,
    views: HashMap<String, CmdResult<View>>,
}

/// One analysis as the frontend shows it.
#[derive(Clone, Serialize)]
pub struct View {
    report: Value,
    svg: Option<String>,
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
    if let Some(b) = log.text_messages().into_iter().find_map(ardutune_core::history::parse_board_line) {
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

/// Parses a log and builds the overview the frontend shows first.
fn read(path: &Path, bytes: &[u8]) -> (Log, Report, Value) {
    let log = Log::from_bytes(bytes);
    let readiness = assess(&log);
    let (flight, _) = log.flight();
    let overview = json!({
        "path": path.display().to_string(),
        "file_name": path.file_name().map(|n| n.to_string_lossy().into_owned()),
        "size_mb": bytes.len() as f64 / 1e6,
        "messages": log.stats.messages,
        "corrupt_dropped": log.stats.bad_timestamps,
        "truncated": log.stats.truncated,
        "params": log.params().len(),
        "flight_periods": flight.len(),
        "flight_s": log::total_duration(&flight),
        "readiness": readiness,
    });
    (log, readiness, overview)
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

        let (result, filter_report, pid_report) = compute(&kind, &log)?;

        // Keep the result unless another log was opened meanwhile.
        let mut slot = lock(&state.log);
        if let Some(loaded) = slot.as_mut().filter(|l| l.id == id) {
            loaded.filters = filter_report.or(loaded.filters.take());
            loaded.pid = pid_report.or(loaded.pid.take());
            loaded.views.insert(kind, result.clone());
        }
        result
    })
    .await
}

type Computed = (CmdResult<View>, Option<Arc<FilterReport>>, Option<Arc<PidReport>>);

/// Runs one analysis. The outer error is for an unknown kind; the inner one
/// is the analysis's own (for example, no gyro data) and is cached.
fn compute(kind: &str, log: &Log) -> CmdResult<Computed> {
    let mut filter_report = None;
    let mut pid_report = None;
    let result = match kind {
        "diagnose" => Ok(view(&diagnose(log), None)),
        "noise" => noise::analyse(log).map(|r| view(&r, Some(plot::render(&r)))),
        "filters" => filters::simulate(log, filters::DEFAULT_LAG_REF_HZ).map(|r| {
            let v = filter_view(&r);
            filter_report = Some(Arc::new(r));
            v
        }),
        "pid" => pid::analyse(log).map(|r| {
            let v = pid_view(&r);
            pid_report = Some(Arc::new(r));
            v
        }),
        other => return Err(format!("unknown analysis {other}")),
    };
    Ok((result, filter_report, pid_report))
}

fn view(report: &impl Serialize, svg: Option<String>) -> View {
    let mut report = serde_json::to_value(report).unwrap_or(Value::Null);
    slim(&mut report);
    View { report, svg }
}

/// Adds the human-readable settings summary to each evaluated filter set.
fn filter_view(r: &FilterReport) -> View {
    let mut v = view(r, Some(plot::render_filters(r)));
    let describe = |slot: Option<&mut Value>, e: &Evaluation| {
        if let Some(o) = slot.and_then(Value::as_object_mut) {
            o.insert("description".to_owned(), e.settings.describe().into());
        }
    };
    describe(v.report.get_mut("current"), &r.current);
    if let Some(e) = &r.recommended {
        describe(v.report.get_mut("recommended"), e);
    }
    if let Some(e) = &r.quieter {
        describe(v.report.get_mut("quieter"), e);
    }
    for (i, e) in r.frontier.iter().enumerate() {
        describe(v.report.get_mut("frontier").and_then(|f| f.get_mut(i)), e);
    }
    v
}

fn pid_view(r: &PidReport) -> View {
    let mut v = view(r, Some(plot::render_pid(r)));
    v.report["response_s"] = pid::RESPONSE_S.into();
    v
}

/// Drops long numeric arrays (spectra, step-response samples): the SVG shows them.
fn slim(v: &mut Value) {
    match v {
        Value::Object(o) => {
            o.retain(|_, x| !is_plot_data(x));
            o.values_mut().for_each(slim);
        }
        Value::Array(a) => a.iter_mut().for_each(slim),
        _ => {}
    }
}

fn is_plot_data(v: &Value) -> bool {
    let Some(a) = v.as_array() else { return false };
    match a.first() {
        Some(Value::Number(_)) => a.len() > MAX_ARRAY,
        // Per-axis spectra, spectrogram rows.
        Some(first @ Value::Array(_)) => is_plot_data(first),
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slim_drops_only_long_numeric_arrays() {
        let mut v = json!({
            "freq_hz": 31.7,
            "spectrum": (0..100).collect::<Vec<_>>(),
            "psd": [(0..100).collect::<Vec<_>>(), (0..100).collect::<Vec<_>>()],
            "range": [1, 2],
            "nested": [{"samples": (0..100).collect::<Vec<_>>(), "name": "x"}],
        });
        slim(&mut v);
        assert!(v.get("spectrum").is_none());
        assert!(v.get("psd").is_none());
        assert_eq!(v["range"], json!([1, 2]));
        assert_eq!(v["nested"], json!([{"name": "x"}]));
        assert_eq!(v["freq_hz"], json!(31.7));
    }

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
        for kind in ["diagnose", "noise", "filters", "pid"] {
            let out = match compute(kind, &log).unwrap().0 {
                Ok(v) => serde_json::to_value(v).unwrap(),
                Err(e) => json!({ "error": e }),
            };
            std::fs::write(dir.join(format!("{kind}.json")), out.to_string()).unwrap();
        }
    }
}
