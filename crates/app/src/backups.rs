//! Parameter backups in Documents/ArduTune/backups: made on request and
//! automatically before every write, listed newest first, restored through
//! the normal review-and-write flow (`prepare_changes` with a file).

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use ardutune_core::params;
use ardutune_core::utc::{date_time, file_stamp};
use ardutune_link::Params;
use serde::Serialize;
use tauri::{AppHandle, Manager, Runtime};

use crate::vehicle::{self, VehicleInfo};
use crate::{blocking, data_dir, lock, AppState, CmdResult, Reporter};

const TITLE: &str = "ArduTune parameter backup";
const REASON: &str = "Reason: ";

#[derive(Serialize)]
pub struct Backup {
    path: String,
    file_name: String,
    /// "YYYY-MM-DD HH:MM:SS" UTC, from the header; `None` for files not made here.
    saved: Option<String>,
    system_id: Option<u8>,
    reason: Option<String>,
    params: usize,
    #[serde(skip)]
    modified: Option<SystemTime>,
}

/// Writes all `params` to a new backup file and returns its path.
pub fn save<R: Runtime>(app: &AppHandle<R>, system_id: u8, params: &Params, reason: &str) -> CmdResult<PathBuf> {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let dir = data_dir(app, "backups")?;
    let stem = format!("backup-sys{system_id}-{}", file_stamp(now));
    let path = (0..)
        .map(|n| dir.join(if n == 0 { format!("{stem}.param") } else { format!("{stem}-{n}.param") }))
        .find(|p| !p.exists())
        .expect("a free file name");
    let header = format!(
        "{TITLE}, {} UTC, system {system_id}.\n{REASON}{}\nRestore it from the Parameters screen, or write it with any ground station.",
        date_time(now),
        one_line(reason)
    );
    let text = params::write(&header, params.iter().map(|(n, p)| (n.as_str(), p.value)));
    std::fs::write(&path, text).map_err(|e| format!("writing {}: {e}", path.display()))?;
    Ok(path)
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
        save(app, c.system_id(), c.params(), &reason)?;
        Ok(info)
    })
    .await
}

#[tauri::command]
pub async fn list_backups<R: Runtime>(app: AppHandle<R>) -> CmdResult<Vec<Backup>> {
    blocking(app, |app| {
        let dir = data_dir(app, "backups")?;
        let mut backups: Vec<Backup> = std::fs::read_dir(&dir)
            .map_err(|e| format!("reading {}: {e}", dir.display()))?
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("param") || x.eq_ignore_ascii_case("parm")))
            .filter_map(|p| describe(&p))
            .collect();
        // Newest first: by the time in the header, then by the file's time
        // (several backups can be made within a second).
        backups.sort_by(|a, b| (&b.saved, b.modified).cmp(&(&a.saved, a.modified)));
        Ok(backups)
    })
    .await
}

fn describe(path: &Path) -> Option<Backup> {
    let text = std::fs::read_to_string(path).ok()?;
    let comments: Vec<&str> = text.lines().filter_map(|l| l.strip_prefix("# ")).collect();
    let title = comments.first().and_then(|l| l.strip_prefix(TITLE)).and_then(|l| l.strip_prefix(", "));
    let saved = title.and_then(|l| l.split_once(" UTC")).map(|(d, _)| d.to_owned());
    let system_id = title.and_then(|l| l.split("system ").nth(1)).and_then(|s| s.trim_end_matches('.').parse().ok());
    let reason = comments.iter().find_map(|l| l.strip_prefix(REASON)).map(str::to_owned);
    Some(Backup {
        path: path.display().to_string(),
        file_name: path.file_name()?.to_string_lossy().into_owned(),
        saved,
        system_id,
        reason,
        params: params::parse(&text).map_or(0, |p| p.len()),
        modified: std::fs::metadata(path).and_then(|m| m.modified()).ok(),
    })
}

/// Deletes a backup file (only files in the backups folder).
#[tauri::command]
pub async fn delete_backup<R: Runtime>(app: AppHandle<R>, path: String) -> CmdResult<()> {
    blocking(app, move |app| {
        let dir = data_dir(app, "backups")?.canonicalize().map_err(|e| e.to_string())?;
        let file = Path::new(&path).canonicalize().map_err(|e| format!("{path}: {e}"))?;
        if file.parent() != Some(dir.as_path()) {
            return Err("Only files in the backups folder can be deleted here.".to_owned());
        }
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
        let ours = dir.join("backup-sys1-20261111-100000.param");
        let header = format!("{TITLE}, 2026-11-11 10:00:00 UTC, system 1.\n{REASON}Before writing Recommended filter settings\nRestore it…");
        std::fs::write(&ours, params::write(&header, [("A_B", 1.0f32), ("C_D", 2.5)])).unwrap();
        let b = describe(&ours).unwrap();
        assert_eq!(b.saved.as_deref(), Some("2026-11-11 10:00:00"));
        assert_eq!(b.system_id, Some(1));
        assert_eq!(b.reason.as_deref(), Some("Before writing Recommended filter settings"));
        assert_eq!(b.params, 2);

        let foreign = dir.join("mission-planner.param");
        std::fs::write(&foreign, "FRAME_CLASS,1\n").unwrap();
        let b = describe(&foreign).unwrap();
        assert_eq!((b.saved, b.system_id, b.reason, b.params), (None, None, None, 1));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
