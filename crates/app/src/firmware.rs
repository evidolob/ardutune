//! Firmware screen: what the vehicle runs, newer releases for its board,
//! and flashing a release or an `.apj` file (optionally backing up first),
//! then reconnecting and comparing parameters.

use std::collections::BTreeMap;
use std::time::Duration;

use ardutune_link::firmware::{self, Apj, Channel, Stage};
use ardutune_link::Link;
use serde::Serialize;
use tauri::{AppHandle, Emitter, Manager, Runtime};

use crate::vehicle::{check, connected, Connected, VehicleInfo};
use crate::{backups, blocking, lock, AppState, CmdResult};

const RECONNECT_TIMEOUT: Duration = Duration::from_secs(60);

/// Firmware waiting to be flashed, after the user has seen what it is.
pub struct PendingFirmware {
    apj: Apj,
    label: String,
}

#[derive(Serialize)]
pub struct Info {
    description: Option<String>,
    version: Option<String>,
    git_hash: Option<String>,
    board: Option<String>,
    board_id: Option<u32>,
    banner: Vec<String>,
}

#[tauri::command]
pub async fn firmware_info<R: Runtime>(app: AppHandle<R>) -> CmdResult<Info> {
    blocking(app, |app| {
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.firmware_info();
        let i = check(&mut slot, r)?;
        Ok(Info {
            description: i.description,
            version: i.version.map(|v| v.to_string()),
            git_hash: i.git_hash,
            board: i.board,
            board_id: i.board_id,
            banner: i.banner,
        })
    })
    .await
}

#[derive(Serialize)]
pub struct Release {
    channel: &'static str,
    version: Option<String>,
    newer: bool,
    error: Option<String>,
}

/// The Copter release each channel offers for `board` (needs internet).
#[tauri::command]
pub async fn firmware_releases<R: Runtime>(app: AppHandle<R>, board: String, current: Option<String>) -> CmdResult<Vec<Release>> {
    blocking(app, move |_| {
        let current = current.as_deref().and_then(firmware::Version::parse);
        Ok(firmware::available(&board)
            .into_iter()
            .map(|(c, r)| match r {
                Ok(v) => Release {
                    channel: c.dir(),
                    newer: v.as_ref().zip(current.as_ref()).is_some_and(|(v, cur)| v.is_newer_than(cur)),
                    version: v.map(|v| v.to_string()),
                    error: None,
                },
                Err(e) => Release { channel: c.dir(), version: None, newer: false, error: Some(e) },
            })
            .collect())
    })
    .await
}

#[derive(Serialize)]
pub struct Prepared {
    label: String,
    board_id: u32,
    board_name: Option<String>,
    size: usize,
    git_identity: Option<String>,
    /// Whether the firmware is for this vehicle's board (`None` if unknown).
    matches_board: Option<bool>,
    vehicle_board_id: Option<u32>,
}

/// Loads firmware (downloading a release, or reading a file) and checks it
/// is for this board. `source` is "file" (with `path`) or a channel.
#[tauri::command]
pub async fn firmware_prepare<R: Runtime>(app: AppHandle<R>, source: String, path: Option<String>) -> CmdResult<Prepared> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let info = {
            let mut slot = lock(&state.vehicle);
            let r = connected(&mut slot)?.link.firmware_info();
            check(&mut slot, r)?
        };
        let (bytes, label) = if source == "file" {
            let path = path.ok_or("No firmware file given.")?;
            let name = std::path::Path::new(&path).file_name().map_or(path.clone(), |n| n.to_string_lossy().into_owned());
            (std::fs::read(&path).map_err(|e| format!("reading {path}: {e}"))?, name)
        } else {
            let channel = Channel::from_dir(&source).ok_or(format!("unknown channel {source}"))?;
            let board = info.board.clone().ok_or("The vehicle didn't report its board name, so ArduTune can't pick its firmware.")?;
            let _ = app.emit("progress", crate::Progress { task: "Downloading firmware", done: 0, total: 0 });
            (firmware::download(channel, &board)?, format!("{} firmware for {board}", channel.dir()))
        };
        let apj = Apj::parse(&bytes)?;
        if apj.has_external_flash {
            return Err("This firmware also needs external flash, which ArduTune can't write yet.".to_owned());
        }
        let prepared = Prepared {
            label: label.clone(),
            board_id: apj.board_id,
            board_name: apj.summary.clone(),
            size: apj.image.len(),
            git_identity: apj.git_identity.clone(),
            matches_board: info.board_id.map(|id| id == apj.board_id),
            vehicle_board_id: info.board_id,
        };
        if prepared.matches_board == Some(false) {
            return Err(format!(
                "This firmware is for board ID {} ({}) but the flight controller is board ID {}. It would not run, so ArduTune won't flash it.",
                apj.board_id,
                apj.summary.as_deref().unwrap_or("another board"),
                info.board_id.unwrap_or(0)
            ));
        }
        *lock(&state.firmware) = Some(PendingFirmware { apj, label });
        Ok(prepared)
    })
    .await
}

#[derive(Serialize)]
pub struct ParamDiff {
    name: String,
    before: Option<f64>,
    after: Option<f64>,
}

#[derive(Serialize)]
pub struct Flashed {
    backup: Option<String>,
    version: Option<String>,
    reconnected: bool,
    /// Parameters whose value changed, appeared or disappeared.
    changed: Vec<ParamDiff>,
    vehicle: Option<VehicleInfo>,
}

fn stage_task(stage: Stage) -> (&'static str, usize, usize) {
    match stage {
        Stage::Erasing => ("Erasing flash (keep USB connected)", 0, 0),
        Stage::Programming { done, total } => ("Writing firmware (keep USB connected)", done, total),
        Stage::Verifying => ("Verifying", 0, 0),
        Stage::Rebooting => ("Starting the new firmware", 0, 0),
    }
}

/// Flashes the prepared firmware: optional backup, reboot into the
/// bootloader, erase/program/verify, then reconnect and compare parameters.
#[tauri::command]
pub async fn firmware_flash<R: Runtime>(app: AppHandle<R>, backup: Option<bool>) -> CmdResult<Flashed> {
    blocking(app, move |app| {
        let state = app.state::<AppState>();
        let pending = lock(&state.firmware).take().ok_or("No firmware prepared.")?;
        let mut slot = lock(&state.vehicle);
        let r = connected(&mut slot)?.link.refresh_armed();
        if check(&mut slot, r)? {
            return Err("The vehicle is armed; disarm before flashing.".to_owned());
        }
        let c = slot.take().ok_or("Not connected to a vehicle.")?;
        let before = c.params.clone();
        let backup = match backup {
            Some(true) => Some(backups::save(app, &c, &format!("Before flashing {}", pending.label))?.display().to_string()),
            _ => None,
        };
        let address = c.address.clone();
        let drone = c.drone.clone();
        let emit = |task: &str, done: usize, total: usize| {
            let _ = app.emit("progress", crate::Progress { task, done, total });
        };
        emit("Rebooting into the bootloader", 0, 0);
        c.link.reboot_to_bootloader().map_err(|e| format!("{e} (nothing was written)"))?;
        emit("Waiting for the bootloader", 0, 0);
        ardutune_link::flash_firmware(&pending.apj, &mut |stage| {
            let (task, done, total) = stage_task(stage);
            emit(task, done, total);
        })?;

        emit("Reconnecting to the new firmware", 0, 0);
        let Ok(mut link) = Link::connect_when_back(&address, RECONNECT_TIMEOUT) else {
            return Ok(Flashed { backup, version: None, reconnected: false, changed: vec![], vehicle: None });
        };
        let version = link.firmware_info().ok().and_then(|i| i.version).map(|v| v.to_string());
        let mut progress = crate::Reporter::new(app, "Reading parameters");
        let after = link.fetch_params(&mut |got, total| progress.update(got, total)).map_err(|e| e.to_string())?;
        let changed = diff(&before, &after);
        let c = Connected {
            link,
            params: after,
            address,
            defaults: None,
            compass_cal: None,
            drone,
            last_arming: None,
            gps_fix: Default::default(),
            last_gps: None,
        };
        let vehicle = Some(c.info());
        *slot = Some(c);
        Ok(Flashed { backup, version, reconnected: true, changed, vehicle })
    })
    .await
}

/// Parameters that differ between two reads (values compared as stored).
fn diff(before: &ardutune_link::Params, after: &ardutune_link::Params) -> Vec<ParamDiff> {
    let mut names: BTreeMap<&str, ()> = BTreeMap::new();
    names.extend(before.keys().map(|k| (k.as_str(), ())));
    names.extend(after.keys().map(|k| (k.as_str(), ())));
    names
        .into_keys()
        // Counters and sensor IDs change by themselves.
        .filter(|n| !n.starts_with("STAT_"))
        .filter_map(|n| {
            let (b, a) = (before.get(n).map(|p| p.value), after.get(n).map(|p| p.value));
            (b != a).then(|| ParamDiff { name: n.to_owned(), before: b.map(f64::from), after: a.map(f64::from) })
        })
        .collect()
}
