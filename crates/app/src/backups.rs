//! Parameter backups in Documents/ArduTune/backups: made on request or
//! before a write, tagged with the vehicle they came from (flight
//! controller chip ID), listed newest first, restored through the normal
//! review-and-write flow (`prepare_changes` with a file).

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use ardutune_core::params;
use ardutune_core::utc::{date_time, file_stamp};
use serde::Serialize;
use tauri::{AppHandle, Manager, Runtime};

use crate::drones::{self, Drone};
use crate::vehicle::{self, Connected, VehicleInfo};
use crate::{blocking, data_dir, lock, AppState, CmdResult, Reporter};

const TITLE: &str = "ArduTune parameter backup";
const REASON: &str = "Reason: ";
const VEHICLE: &str = "Vehicle: ";
const CONTROLLER: &str = "Flight controller: ";

#[derive(Serialize)]
pub struct Backup {
    path: String,
    file_name: String,
    /// "YYYY-MM-DD HH:MM:SS" UTC, from the header; `None` for files not made here.
    saved: Option<String>,
    system_id: Option<u8>,
    reason: Option<String>,
    params: usize,
    /// The vehicle's chip ID and board, from the header.
    uid: Option<String>,
    board: Option<String>,
    /// What to call the vehicle: its current name, else the name in the
    /// file, else board and ID.
    vehicle: Option<String>,
    /// For a backup without a vehicle: how well it matches the connected one.
    #[serde(rename = "match")]
    likely: Option<Match>,
    #[serde(skip)]
    modified: Option<SystemTime>,
}

#[derive(Serialize, Debug, PartialEq)]
pub struct Match {
    /// "same": the accelerometer calibration is identical; "likely": almost
    /// every value is.
    level: &'static str,
    why: String,
}

/// Writes all of the vehicle's parameters to a new backup file.
pub fn save<R: Runtime>(app: &AppHandle<R>, c: &Connected, reason: &str) -> CmdResult<PathBuf> {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let dir = data_dir(app, "backups")?;
    let system_id = c.system_id();
    let who = match &c.drone {
        Some(d) => slug(&d.label()),
        None => format!("sys{system_id}"),
    };
    let stem = format!("backup-{who}-{}", file_stamp(now));
    let path = (0..)
        .map(|n| dir.join(if n == 0 { format!("{stem}.param") } else { format!("{stem}-{n}.param") }))
        .find(|p| !p.exists())
        .expect("a free file name");
    let mut header = format!("{TITLE}, {} UTC, system {system_id}.\n", date_time(now));
    if let Some(d) = &c.drone {
        header += &vehicle_lines(d);
    }
    header += &format!("{REASON}{}\nRestore it from the Backups screen, or write it with any ground station.", one_line(reason));
    let text = params::write(&header, c.params().iter().map(|(n, p)| (n.as_str(), p.value)));
    std::fs::write(&path, text).map_err(|e| format!("writing {}: {e}", path.display()))?;
    Ok(path)
}

/// The header lines that say which vehicle a backup belongs to.
fn vehicle_lines(d: &Drone) -> String {
    let name = d.name.as_deref().unwrap_or("unnamed");
    let board = d.board.as_deref().unwrap_or("unknown board");
    format!("{VEHICLE}{name}\n{CONTROLLER}{board}, ID {}\n", d.key())
}

/// A file-name-safe version of a vehicle name.
pub fn slug(s: &str) -> String {
    let mut out = String::new();
    for c in s.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_lowercase());
        } else if !out.ends_with('-') {
            out.push('-');
        }
    }
    let out = out.trim_matches('-');
    if out.is_empty() { "vehicle".to_owned() } else { out.chars().take(30).collect() }
}

fn one_line(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// Reads the vehicle's parameters again (they may have been changed
/// elsewhere) and backs them up with an optional note.
#[tauri::command]
pub async fn backup_params<R: Runtime>(app: AppHandle<R>, note: Option<String>) -> CmdResult<VehicleInfo> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let mut progress = Reporter::new(app, "Reading parameters");
        let info = vehicle::refresh_params(&mut slot, &mut |got, total| progress.update(got, total))?;
        let c = slot.as_ref().ok_or("Not connected to a vehicle.")?;
        let reason = note.filter(|n| !n.trim().is_empty()).unwrap_or_else(|| "Manual backup".to_owned());
        save(app, c, &reason)?;
        Ok(info)
    })
    .await
}

#[tauri::command]
pub async fn list_backups<R: Runtime>(app: AppHandle<R>) -> CmdResult<Vec<Backup>> {
    blocking(app, |app| {
        let dir = data_dir(app, "backups")?;
        let known = drones::load(app)?;
        // The connected vehicle's values, to match backups that don't say
        // which vehicle they came from.
        let live: Option<HashMap<String, f32>> = {
            let state = app.state::<AppState>();
            let slot = lock(&state.vehicle);
            slot.as_ref().filter(|c| c.drone.is_some()).map(|c| c.params().iter().map(|(n, p)| (n.clone(), p.value)).collect())
        };
        let mut backups: Vec<Backup> = std::fs::read_dir(&dir)
            .map_err(|e| format!("reading {}: {e}", dir.display()))?
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("param") || x.eq_ignore_ascii_case("parm")))
            .filter_map(|p| describe(&p, &known, live.as_ref()))
            .collect();
        // Newest first: by the time in the header, then by the file's time
        // (several backups can be made within a second).
        backups.sort_by(|a, b| (&b.saved, b.modified).cmp(&(&a.saved, a.modified)));
        Ok(backups)
    })
    .await
}

/// The header of a backup file and its values.
pub struct Header {
    pub saved: Option<String>,
    pub system_id: Option<u8>,
    pub reason: Option<String>,
    pub name: Option<String>,
    pub board: Option<String>,
    pub uid: Option<String>,
}

pub fn read_header(text: &str) -> Header {
    let comments: Vec<&str> = text.lines().filter_map(|l| l.strip_prefix("# ")).collect();
    let title = comments.first().and_then(|l| l.strip_prefix(TITLE)).and_then(|l| l.strip_prefix(", "));
    let find = |prefix: &str| comments.iter().find_map(|l| l.strip_prefix(prefix)).map(str::trim);
    let controller = find(CONTROLLER).and_then(|l| l.rsplit_once(", ID "));
    Header {
        saved: title.and_then(|l| l.split_once(" UTC")).map(|(d, _)| d.to_owned()),
        system_id: title.and_then(|l| l.split("system ").nth(1)).and_then(|s| s.trim_end_matches('.').parse().ok()),
        reason: find(REASON).map(str::to_owned),
        name: find(VEHICLE).filter(|n| *n != "unnamed").map(str::to_owned),
        board: controller.map(|(b, _)| b.to_owned()).filter(|b| b != "unknown board"),
        uid: controller.map(|(_, id)| id.trim().to_owned()).filter(|id| !id.is_empty()),
    }
}

fn describe(path: &Path, known: &[Drone], live: Option<&HashMap<String, f32>>) -> Option<Backup> {
    let text = std::fs::read_to_string(path).ok()?;
    let h = read_header(&text);
    let values = params::parse(&text).unwrap_or_default();
    let vehicle = h.uid.as_ref().map(|uid| match known.iter().find(|d| d.is(uid)) {
        Some(d) => d.label(),
        None => Drone { name: h.name.clone(), board: h.board.clone(), ..Drone::new(Some(uid.clone()), None) }.label(),
    });
    let likely = match (&h.uid, live) {
        (None, Some(live)) => match_vehicle(&values, live),
        _ => None,
    };
    Some(Backup {
        path: path.display().to_string(),
        file_name: path.file_name()?.to_string_lossy().into_owned(),
        saved: h.saved,
        system_id: h.system_id,
        reason: h.reason,
        params: values.len(),
        uid: h.uid,
        board: h.board,
        vehicle,
        likely,
        modified: std::fs::metadata(path).and_then(|m| m.modified()).ok(),
    })
}

/// Accelerometer calibration: unique to each flight controller and kept
/// until it is recalibrated.
const ACCEL_CAL: [&str; 12] = [
    "INS_ACCOFFS_X",
    "INS_ACCOFFS_Y",
    "INS_ACCOFFS_Z",
    "INS_ACCSCAL_X",
    "INS_ACCSCAL_Y",
    "INS_ACCSCAL_Z",
    "INS_ACC2OFFS_X",
    "INS_ACC2OFFS_Y",
    "INS_ACC2OFFS_Z",
    "INS_ACC2SCAL_X",
    "INS_ACC2SCAL_Y",
    "INS_ACC2SCAL_Z",
];

fn same(a: f32, b: f32) -> bool {
    (a - b).abs() <= 1e-6 * a.abs().max(b.abs()).max(1.0)
}

/// Whether a backup that doesn't name its vehicle looks like it came from
/// the connected one. Two vehicles with the same board and firmware share
/// most values (the defaults), so a near-total match is required, and an
/// identical accelerometer calibration is the strong sign.
fn match_vehicle(backup: &[(String, f64)], live: &HashMap<String, f32>) -> Option<Match> {
    let get = |n: &str| backup.iter().find(|(k, _)| k == n).map(|(_, v)| *v as f32);
    let cal: Vec<(f32, f32)> = ACCEL_CAL.iter().filter_map(|n| Some((get(n)?, *live.get(*n)?))).collect();
    // Uncalibrated values (0 offsets, 1 scales) prove nothing.
    let calibrated = cal.iter().filter(|(b, _)| *b != 0.0 && *b != 1.0).count();
    if calibrated >= 3 && cal.iter().all(|(b, l)| same(*b, *l)) {
        return Some(Match { level: "same", why: "Its accelerometer calibration is identical to the connected vehicle's.".to_owned() });
    }
    let shared: Vec<(f32, f32)> = backup.iter().filter_map(|(n, v)| Some((*v as f32, *live.get(n)?))).collect();
    if backup.is_empty() || shared.len() * 10 < backup.len() * 9 {
        return None;
    }
    let equal = shared.iter().filter(|(b, l)| same(*b, *l)).count();
    let pct = 100.0 * equal as f64 / shared.len() as f64;
    let recalibrated = calibrated >= 3;
    (pct >= 98.0 && !recalibrated).then(|| Match {
        level: "likely",
        why: format!("{pct:.0}% of its values match the connected vehicle's (no accelerometer calibration to compare)."),
    })
}

/// Tags a backup that doesn't say which vehicle it came from with the
/// connected vehicle.
#[tauri::command]
pub async fn assign_backup<R: Runtime>(app: AppHandle<R>, path: String) -> CmdResult<()> {
    blocking(app, move |app| {
        let file = in_backups(app, &path)?;
        let drone = {
            let state = app.state::<AppState>();
            let slot = lock(&state.vehicle);
            let c = slot.as_ref().ok_or("Not connected to a vehicle.")?;
            c.drone.clone().ok_or("The connected flight controller doesn't report a chip ID.")?
        };
        let text = std::fs::read_to_string(&file).map_err(|e| format!("reading {path}: {e}"))?;
        if read_header(&text).uid.is_some() {
            return Err("This backup already belongs to a vehicle.".to_owned());
        }
        let tag: String = vehicle_lines(&drone).lines().map(|l| format!("# {l}\n")).collect();
        // After our title line if there is one, else at the top.
        let text = match text.split_once('\n') {
            Some((first, rest)) if first.starts_with(&format!("# {TITLE}")) => format!("{first}\n{tag}{rest}"),
            _ => format!("{tag}{text}"),
        };
        std::fs::write(&file, text).map_err(|e| format!("writing {path}: {e}"))
    })
    .await
}

/// A file in the backups folder (only those can be changed or deleted here).
fn in_backups<R: Runtime>(app: &AppHandle<R>, path: &str) -> CmdResult<PathBuf> {
    let dir = data_dir(app, "backups")?.canonicalize().map_err(|e| e.to_string())?;
    let file = Path::new(path).canonicalize().map_err(|e| format!("{path}: {e}"))?;
    if file.parent() != Some(dir.as_path()) {
        return Err("Only files in the backups folder can be changed here.".to_owned());
    }
    Ok(file)
}

/// A warning when a parameter file was made on another vehicle than the
/// connected one.
pub fn other_vehicle<R: Runtime>(app: &AppHandle<R>, path: &str, c: &Connected) -> Option<String> {
    let text = std::fs::read_to_string(path).ok()?;
    let h = read_header(&text);
    let (theirs, ours) = (h.uid?, c.drone.as_ref()?);
    if ours.is(&theirs) {
        return None;
    }
    let d = drones::find(app, &theirs).unwrap_or(Drone { name: h.name, board: h.board, ..Drone::new(Some(theirs), None) });
    Some(format!(
        "This backup was made on {}, not on the connected vehicle ({}). Restoring it copies that vehicle's calibration and setup: untick anything that must stay, such as calibration (INS_ACC*, COMPASS_OFS*) and outputs.",
        d.label(),
        ours.label()
    ))
}

/// Deletes a backup file (only files in the backups folder).
#[tauri::command]
pub async fn delete_backup<R: Runtime>(app: AppHandle<R>, path: String) -> CmdResult<()> {
    blocking(app, move |app| {
        let file = in_backups(app, &path)?;
        std::fs::remove_file(&file).map_err(|e| format!("deleting {path}: {e}"))
    })
    .await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_backup_headers() {
        let dir = std::env::temp_dir().join(format!("ardutune-backup-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let ours = dir.join("backup-quad-20261111-100000.param");
        let header = format!(
            "{TITLE}, 2026-11-11 10:00:00 UTC, system 1.\n{VEHICLE}Quad (5\")\n{CONTROLLER}KakuteH7-bdshot, ID 003A0024\n{REASON}Before writing Recommended filter settings\nRestore it…"
        );
        std::fs::write(&ours, params::write(&header, [("A_B", 1.0f32), ("C_D", 2.5)])).unwrap();
        let mut known = Vec::new();
        let b = describe(&ours, &known, None).unwrap();
        assert_eq!(b.saved.as_deref(), Some("2026-11-11 10:00:00"));
        assert_eq!(b.system_id, Some(1));
        assert_eq!(b.reason.as_deref(), Some("Before writing Recommended filter settings"));
        assert_eq!((b.uid.as_deref(), b.board.as_deref()), (Some("003A0024"), Some("KakuteH7-bdshot")));
        assert_eq!(b.vehicle.as_deref(), Some("Quad (5\")"));
        assert_eq!(b.params, 2);
        // The current name wins over the one in the file.
        known.push(Drone { name: Some("Kakute".into()), ..Drone::new(Some("003A0024".into()), None) });
        assert_eq!(describe(&ours, &known, None).unwrap().vehicle.as_deref(), Some("Kakute"));

        let foreign = dir.join("mission-planner.param");
        std::fs::write(&foreign, "FRAME_CLASS,1\n").unwrap();
        let b = describe(&foreign, &known, None).unwrap();
        assert_eq!((b.saved, b.system_id, b.reason, b.uid, b.vehicle, b.params), (None, None, None, None, None, 1));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn matches_untagged_backups_to_the_connected_vehicle() {
        let mut live: HashMap<String, f32> =
            [("INS_ACCOFFS_X", 0.017), ("INS_ACCOFFS_Y", -0.01), ("INS_ACCOFFS_Z", 0.19), ("INS_ACCSCAL_X", 1.002)].iter().map(|(k, v)| (k.to_string(), *v)).collect();
        for i in 0..100 {
            live.insert(format!("P{i}"), i as f32);
        }
        let backup: Vec<(String, f64)> = live.iter().map(|(k, v)| (k.clone(), *v as f64)).collect();
        assert_eq!(match_vehicle(&backup, &live).unwrap().level, "same");

        // Another vehicle: different calibration, same defaults otherwise.
        let mut other = backup.clone();
        for (k, v) in other.iter_mut() {
            if k.starts_with("INS_ACC") {
                *v += 0.05;
            }
        }
        assert_eq!(match_vehicle(&other, &live), None);

        // No calibration in either: only a near-total match counts.
        let uncal: HashMap<String, f32> = live.iter().filter(|(k, _)| !k.starts_with("INS_")).map(|(k, v)| (k.clone(), *v)).collect();
        let mut b: Vec<(String, f64)> = uncal.iter().map(|(k, v)| (k.clone(), *v as f64)).collect();
        assert_eq!(match_vehicle(&b, &uncal).unwrap().level, "likely");
        for (_, v) in b.iter_mut().take(5) {
            *v += 1.0;
        }
        assert_eq!(match_vehicle(&b, &uncal), None);
    }

    #[test]
    fn slugs() {
        assert_eq!(slug("Kakute 5\" (new)"), "kakute-5-new");
        assert_eq!(slug("!!"), "vehicle");
    }
}
