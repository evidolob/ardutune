//! Flight history: every log in Documents/ArduTune/logs, grouped by the
//! vehicle it came from (the flight controller ID in the log's banner), with
//! a summary per flight and the tuning parameters changed since the
//! previous flight. Summaries are cached next to the logs, so each log is
//! read once.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::UNIX_EPOCH;

use ardutune_core::history::{self, FlightSummary};
use ardutune_core::log::Log;
use ardutune_core::utc::date_time;
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager, Runtime};

use crate::drones::{self, Drone};
use crate::{blocking, data_dir, lock, AppState, CmdResult, Reporter};

const CACHE: &str = ".ardutune-summaries.json";
/// Bump when [`FlightSummary`] changes, so old summaries are recomputed.
const CACHE_VERSION: u32 = 2;

#[derive(Serialize, Deserialize, Default)]
struct Cache {
    version: u32,
    /// File name → summary of that file as it was (size and time).
    entries: HashMap<String, Cached>,
}

#[derive(Serialize, Deserialize)]
struct Cached {
    size: u64,
    modified: u64,
    summary: FlightSummary,
}

#[derive(Serialize)]
pub struct Flight {
    path: String,
    file_name: String,
    size_mb: f64,
    /// 1 for the vehicle's first flight here, in flight order.
    number: usize,
    /// "YYYY-MM-DD HH:MM:SS" UTC from GPS time; `None` when the log never
    /// had a fix (the clock isn't trustworthy then).
    when: Option<String>,
    summary: FlightSummary,
    /// Tuning parameters changed since this vehicle's previous flight.
    changes: Vec<(String, Option<f64>, Option<f64>)>,
    #[serde(skip)]
    order: (u8, u32, i64, i64),
}

/// Flight order without trusting clocks: power-up count, then time since
/// power-up (several logs per power-up). Logs without a boot count (other
/// autopilots) go by GPS time, else the file's time, after the others.
fn order(s: &FlightSummary, modified: u64) -> (u8, u32, i64, i64) {
    match s.boot_count {
        Some(boot) => (0, boot, (s.after_boot_s * 1000.0) as i64, 0),
        None => (1, 0, s.started.unwrap_or(modified as i64), 0),
    }
}

#[derive(Serialize)]
pub struct VehicleFlights {
    /// The ID to name it by (`from_log`: the banner form), `None` for logs
    /// that don't say which flight controller they came from.
    id: Option<String>,
    from_log: bool,
    label: String,
    /// The pilot gave it a name (else the label is board and ID).
    named: bool,
    board: Option<String>,
    connected: bool,
    /// Newest first.
    flights: Vec<Flight>,
}

fn file_times(path: &Path) -> Option<(u64, u64)> {
    let m = std::fs::metadata(path).ok()?;
    let modified = m.modified().ok()?.duration_since(UNIX_EPOCH).ok()?.as_secs();
    Some((m.len(), modified))
}

/// Summaries of every log in the folder, reading only new or changed files.
fn summaries<R: Runtime>(app: &AppHandle<R>, dir: &Path) -> CmdResult<Vec<(PathBuf, u64, u64, FlightSummary)>> {
    let cache_path = dir.join(CACHE);
    let mut cache: Cache = std::fs::read_to_string(&cache_path)
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .filter(|c: &Cache| c.version == CACHE_VERSION)
        .unwrap_or_default();
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| format!("reading {}: {e}", dir.display()))?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("bin")))
        .collect();
    files.sort();
    let stale: Vec<&PathBuf> = files
        .iter()
        .filter(|p| {
            let name = p.file_name().unwrap_or_default().to_string_lossy();
            let now = file_times(p);
            cache.entries.get(name.as_ref()).is_none_or(|c| Some((c.size, c.modified)) != now)
        })
        .collect();
    let mut progress = Reporter::new(app, "Reading flight logs");
    let total = stale.len();
    for (i, path) in stale.into_iter().enumerate() {
        progress.update(i, total);
        let Some((size, modified)) = file_times(path) else { continue };
        // A file that can't be read is left out rather than failing the list.
        let Ok(log) = Log::open(path) else { continue };
        let name = path.file_name().unwrap_or_default().to_string_lossy().into_owned();
        cache.entries.insert(name, Cached { size, modified, summary: history::summarize(&log) });
    }
    progress.update(total, total);
    let names: Vec<String> = files.iter().map(|p| p.file_name().unwrap_or_default().to_string_lossy().into_owned()).collect();
    cache.entries.retain(|n, _| names.contains(n));
    cache.version = CACHE_VERSION;
    if total > 0 {
        let text = serde_json::to_string(&cache).map_err(|e| e.to_string())?;
        std::fs::write(&cache_path, text).map_err(|e| format!("writing {}: {e}", cache_path.display()))?;
    }
    Ok(files
        .into_iter()
        .zip(names)
        .filter_map(|(p, n)| {
            let c = cache.entries.remove(&n)?;
            Some((p, c.size, c.modified, c.summary))
        })
        .collect())
}

#[tauri::command]
pub async fn flight_history<R: Runtime>(app: AppHandle<R>) -> CmdResult<Vec<VehicleFlights>> {
    blocking(app, |app| {
        let dir = data_dir(app, "logs")?;
        let all = summaries(app, &dir)?;
        let known = drones::load(app)?;
        let connected: Option<Drone> = {
            let state = app.state::<AppState>();
            let slot = lock(&state.vehicle);
            slot.as_ref().and_then(|c| c.drone.clone())
        };

        // Group by vehicle: a known one when the log's ID matches it.
        let mut groups: Vec<VehicleFlights> = Vec::new();
        for (path, size, modified, summary) in all {
            let serial = summary.board.as_ref().map(|b| b.serial.clone());
            let drone = serial.as_deref().and_then(|s| known.iter().find(|d| d.is(s)));
            let (id, from_log, label, named, board) = match (drone, &summary.board) {
                (Some(d), _) => (Some(d.key().to_owned()), d.uid.is_none(), d.label(), d.name.is_some(), d.board.clone()),
                (None, Some(b)) => {
                    let d = Drone { board: Some(b.board.clone()), ..Drone::new(None, Some(b.serial.clone())) };
                    (Some(b.serial.clone()), true, d.label(), false, Some(b.board.clone()))
                }
                (None, None) => (None, false, "Unknown flight controller".to_owned(), false, None),
            };
            let flight = Flight {
                path: path.display().to_string(),
                file_name: path.file_name().unwrap_or_default().to_string_lossy().into_owned(),
                size_mb: size as f64 / 1e6,
                number: 0,
                when: summary.started.map(|t| date_time(t as u64)),
                order: order(&summary, modified),
                summary,
                changes: Vec::new(),
            };
            match groups.iter_mut().find(|g| g.id == id) {
                Some(g) => g.flights.push(flight),
                None => {
                    let is_connected = connected.as_ref().is_some_and(|c| id.as_deref().is_some_and(|i| c.is(i)));
                    groups.push(VehicleFlights { id, from_log, label, named, board, connected: is_connected, flights: vec![flight] });
                }
            }
        }
        for g in &mut groups {
            g.flights.sort_by_key(|f| f.order);
            for (i, f) in g.flights.iter_mut().enumerate() {
                f.number = i + 1;
            }
            for i in 1..g.flights.len() {
                let changes = history::changes(&g.flights[i - 1].summary, &g.flights[i].summary);
                g.flights[i].changes = changes;
            }
            g.flights.reverse();
        }
        // The connected vehicle first, then by name; unknown last.
        groups.sort_by_key(|g| (!g.connected, g.id.is_none(), g.label.to_lowercase()));
        Ok(groups)
    })
    .await
}

/// Copies a log file (from another ground station or the SD card) into the
/// logs folder so it shows in the history.
#[tauri::command]
pub async fn import_log<R: Runtime>(app: AppHandle<R>, path: String) -> CmdResult<String> {
    blocking(app, move |app| {
        let src = PathBuf::from(&path);
        let dir = data_dir(app, "logs")?;
        if src.parent().and_then(|p| p.canonicalize().ok()) == dir.canonicalize().ok() {
            return Ok(path);
        }
        let name = src.file_name().ok_or("Not a file.")?.to_string_lossy().into_owned();
        let stem = name.trim_end_matches(".bin").trim_end_matches(".BIN");
        let dest =
            (0..).map(|n| dir.join(if n == 0 { format!("{stem}.bin") } else { format!("{stem}-{n}.bin") })).find(|p| !p.exists()).expect("a free file name");
        std::fs::copy(&src, &dest).map_err(|e| format!("copying {path}: {e}"))?;
        Ok(dest.display().to_string())
    })
    .await
}
