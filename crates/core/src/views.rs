//! What the log analysis pages show: the overview of an opened log and each
//! analysis as a report plus an SVG plot. Shared by the desktop app and the
//! web app.

use serde::Serialize;
use serde_json::{json, Value};

use crate::diagnose::diagnose;
use crate::filters::{self, Evaluation, FilterReport};
use crate::pid::{self, PidReport};
use crate::{assess, log, noise, plot, Log, Report};

/// Numeric arrays longer than this are plot data the frontend gets as SVG.
const MAX_ARRAY: usize = 64;

/// The analyses the pages ask for.
pub const KINDS: [&str; 4] = ["diagnose", "noise", "filters", "pid"];

/// One analysis as the frontend shows it.
#[derive(Debug, Clone, Serialize)]
pub struct View {
    pub report: Value,
    pub svg: Option<String>,
}

/// Parses a log and builds the overview the frontend shows first. `path` is
/// what to show as its location (a file path, or the file name on the web).
pub fn open(path: &str, file_name: Option<String>, bytes: &[u8]) -> (Log, Report, Value) {
    let log = Log::from_bytes(bytes);
    let readiness = assess(&log);
    let (flight, _) = log.flight();
    let mut overview = json!({
        "path": path,
        "file_name": file_name,
        "size_mb": bytes.len() as f64 / 1e6,
        "messages": log.stats.messages,
        "corrupt_dropped": log.stats.bad_timestamps,
        "truncated": log.stats.truncated,
        "params": log.params().len(),
        "flight_periods": flight.len(),
        "flight_s": log::total_duration(&flight),
        "readiness": readiness,
    });
    if let Some(b) = board(&log) {
        overview["board"] = json!({ "board": b.board, "serial": b.serial });
    }
    (log, readiness, overview)
}

/// The flight controller that wrote the log, from its start-up banner.
pub fn board(log: &Log) -> Option<crate::history::BoardId> {
    log.text_messages().into_iter().find_map(crate::history::parse_board_line)
}

/// One analysis and the reports later steps reuse.
pub struct Computed {
    pub view: Result<View, String>,
    pub filters: Option<FilterReport>,
    pub pid: Option<PidReport>,
}

/// Runs one analysis. The outer error is for an unknown kind; the inner one
/// (in `view`) is the analysis's own, for example no gyro data.
pub fn compute(kind: &str, log: &Log) -> Result<Computed, String> {
    let mut out = Computed { view: Err(String::new()), filters: None, pid: None };
    out.view = match kind {
        "diagnose" => Ok(view(&diagnose(log), None)),
        "noise" => noise::analyse(log).map(|r| view(&r, Some(plot::render(&r)))),
        "filters" => filters::simulate(log, filters::DEFAULT_LAG_REF_HZ).map(|r| {
            let v = filter_view(&r);
            out.filters = Some(r);
            v
        }),
        "pid" => pid::analyse(log).map(|r| {
            let v = pid_view(&r);
            out.pid = Some(r);
            v
        }),
        other => return Err(format!("unknown analysis {other}")),
    };
    Ok(out)
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

    #[test]
    fn sample_flight_opens_and_analyses() {
        let bytes = crate::synth::sample_flight(crate::synth::Preset::Ready);
        let (log, readiness, overview) = open("sample.bin", Some("sample.bin".into()), &bytes);
        assert!(readiness.filter_ready, "{overview}");
        assert_eq!(overview["board"]["board"], json!("KakuteH7-bdshot"));
        for kind in KINDS {
            assert!(compute(kind, &log).unwrap().view.is_ok(), "{kind}");
        }
        assert!(compute("nope", &log).is_err());
    }
}
