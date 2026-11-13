//! The vehicles this computer has seen, by flight controller chip ID, with
//! the names the pilot gave them. Kept in Documents/ArduTune/drones.json so
//! backups and logs can say which vehicle they belong to. A vehicle is known
//! by its chip ID as AUTOPILOT_VERSION reports it (`uid`), as the start-up
//! banner and every log show it (`serial`), or both.

use std::path::PathBuf;

use ardutune_core::history::serial_from_uid;
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager, Runtime};

use crate::vehicle::VehicleInfo;
use crate::{blocking, data_dir, lock, AppState, CmdResult};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Drone {
    /// Flight controller chip ID (hex), from a connection.
    #[serde(default)]
    pub uid: Option<String>,
    /// The same ID as the banner and logs show it.
    #[serde(default)]
    pub serial: Option<String>,
    /// Given by the pilot; `None` until named.
    pub name: Option<String>,
    /// Board name, e.g. "KakuteH7-bdshot".
    pub board: Option<String>,
    /// "YYYY-MM-DD HH:MM:SS" UTC of the last connection.
    pub last_seen: Option<String>,
    /// Where its GPS was last trusted, to spot a spoofed position.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_position: Option<ardutune_core::gps::Known>,
}

impl Drone {
    pub fn new(uid: Option<String>, serial: Option<String>) -> Self {
        Drone { uid, serial, name: None, board: None, last_seen: None, last_position: None }
    }

    /// The ID it is filed under: the chip ID, else the banner one.
    pub fn key(&self) -> &str {
        self.uid.as_deref().or(self.serial.as_deref()).unwrap_or_default()
    }

    /// Whether `id` (either form) names this flight controller.
    pub fn is(&self, id: &str) -> bool {
        self.uid.as_deref() == Some(id) || self.serial.as_deref() == Some(id) || self.uid.as_deref().and_then(serial_from_uid).as_deref() == Some(id)
    }

    /// What to call it: its name, else its board and the end of its ID.
    pub fn label(&self) -> String {
        match (&self.name, &self.board) {
            (Some(n), _) => n.clone(),
            (None, Some(b)) => format!("{b} …{}", short(self.key())),
            (None, None) => format!("Flight controller …{}", short(self.key())),
        }
    }
}

/// The last six characters of a chip ID: enough to tell boards apart.
pub fn short(uid: &str) -> &str {
    &uid[uid.len().saturating_sub(6)..]
}

fn file<R: Runtime>(app: &AppHandle<R>) -> CmdResult<PathBuf> {
    Ok(data_dir(app, "")?.join("drones.json"))
}

pub fn load<R: Runtime>(app: &AppHandle<R>) -> CmdResult<Vec<Drone>> {
    let path = file(app)?;
    match std::fs::read_to_string(&path) {
        Ok(text) => serde_json::from_str(&text).map_err(|e| format!("reading {}: {e}", path.display())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Vec::new()),
        Err(e) => Err(format!("reading {}: {e}", path.display())),
    }
}

fn store<R: Runtime>(app: &AppHandle<R>, drones: &[Drone]) -> CmdResult<()> {
    let path = file(app)?;
    let text = serde_json::to_string_pretty(drones).map_err(|e| e.to_string())?;
    std::fs::write(&path, text).map_err(|e| format!("writing {}: {e}", path.display()))
}

/// Remembers where a vehicle's GPS was trusted; returns the updated record.
pub fn set_position<R: Runtime>(app: &AppHandle<R>, key: &str, known: Option<ardutune_core::gps::Known>) -> CmdResult<Option<Drone>> {
    let mut drones = load(app)?;
    let Some(d) = drones.iter_mut().find(|d| d.is(key)) else { return Ok(None) };
    d.last_position = known;
    let d = d.clone();
    store(app, &drones)?;
    Ok(Some(d))
}

/// The vehicle with this ID (either form).
pub fn find<R: Runtime>(app: &AppHandle<R>, id: &str) -> Option<Drone> {
    load(app).ok()?.into_iter().find(|d| d.is(id))
}

/// Records a connection: adds the vehicle if new (or completes one known
/// only from its logs), updates board and time.
pub fn seen<R: Runtime>(app: &AppHandle<R>, uid: &str, serial: Option<&str>, board: Option<&str>, now: &str) -> CmdResult<Drone> {
    let mut drones = load(app)?;
    let at = drones.iter().position(|d| d.is(uid) || serial.is_some_and(|s| d.is(s)));
    let d = match at {
        Some(i) => &mut drones[i],
        None => {
            drones.push(Drone::new(None, None));
            drones.last_mut().unwrap()
        }
    };
    d.uid = Some(uid.to_owned());
    if serial.is_some() {
        d.serial = serial.map(str::to_owned);
    }
    if board.is_some() {
        d.board = board.map(str::to_owned);
    }
    d.last_seen = Some(now.to_owned());
    let d = d.clone();
    store(app, &drones)?;
    Ok(d)
}

/// Names a vehicle by either form of its ID (any vehicle seen, also one
/// known only from logs: `from_log` with its `board`); returns the connected
/// vehicle's info so the header can update.
#[tauri::command]
pub async fn name_drone<R: Runtime>(
    app: AppHandle<R>,
    uid: String,
    name: String,
    board: Option<String>,
    from_log: Option<bool>,
) -> CmdResult<Option<VehicleInfo>> {
    blocking(app, move |app| {
        let name = name.split_whitespace().collect::<Vec<_>>().join(" ");
        if name.is_empty() {
            return Err("Give the vehicle a name.".to_owned());
        }
        if name.chars().count() > 40 {
            return Err("Keep the name under 40 characters.".to_owned());
        }
        let mut drones = load(app)?;
        if drones.iter().any(|d| !d.is(&uid) && d.name.as_deref().is_some_and(|n| n.eq_ignore_ascii_case(&name))) {
            return Err(format!("Another vehicle is already called \"{name}\"."));
        }
        match drones.iter_mut().find(|d| d.is(&uid)) {
            Some(d) => d.name = Some(name),
            None => {
                // Known only from logs or backups: file it under that ID.
                let mut d = if from_log == Some(true) { Drone::new(None, Some(uid.clone())) } else { Drone::new(Some(uid.clone()), None) };
                d.name = Some(name);
                d.board = board;
                drones.push(d);
            }
        }
        store(app, &drones)?;
        let state = app.state::<AppState>();
        let mut slot = lock(&state.vehicle);
        Ok(match slot.as_mut() {
            Some(c) => {
                c.refresh_drone(app);
                Some(c.info())
            }
            None => None,
        })
    })
    .await
}

/// Every vehicle seen on this computer.
#[tauri::command]
pub async fn list_drones<R: Runtime>(app: AppHandle<R>) -> CmdResult<Vec<Drone>> {
    blocking(app, |app| load(app)).await
}
