//! The vehicles this computer has seen, by flight controller chip ID, with
//! the names the pilot gave them. Kept in Documents/ArduTune/drones.json so
//! backups can say which vehicle they belong to.

use std::path::PathBuf;

use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager, Runtime};

use crate::vehicle::VehicleInfo;
use crate::{blocking, data_dir, lock, AppState, CmdResult};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Drone {
    /// Flight controller chip ID (hex).
    pub uid: String,
    /// Given by the pilot; `None` until named.
    pub name: Option<String>,
    /// Board name, e.g. "KakuteH7-bdshot".
    pub board: Option<String>,
    /// "YYYY-MM-DD HH:MM:SS" UTC of the last connection.
    pub last_seen: Option<String>,
}

impl Drone {
    /// What to call it: its name, else its board and the end of its ID.
    pub fn label(&self) -> String {
        match (&self.name, &self.board) {
            (Some(n), _) => n.clone(),
            (None, Some(b)) => format!("{b} …{}", short(&self.uid)),
            (None, None) => format!("Flight controller …{}", short(&self.uid)),
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

pub fn find<R: Runtime>(app: &AppHandle<R>, uid: &str) -> Option<Drone> {
    load(app).ok()?.into_iter().find(|d| d.uid == uid)
}

/// Records a connection: adds the vehicle if new, updates board and time.
pub fn seen<R: Runtime>(app: &AppHandle<R>, uid: &str, board: Option<&str>, now: &str) -> CmdResult<Drone> {
    let mut drones = load(app)?;
    let d = match drones.iter_mut().position(|d| d.uid == uid) {
        Some(i) => &mut drones[i],
        None => {
            drones.push(Drone { uid: uid.to_owned(), name: None, board: None, last_seen: None });
            drones.last_mut().unwrap()
        }
    };
    if board.is_some() {
        d.board = board.map(str::to_owned);
    }
    d.last_seen = Some(now.to_owned());
    let d = d.clone();
    store(app, &drones)?;
    Ok(d)
}

/// Names a vehicle (any seen before, not only the connected one); returns
/// the connected vehicle's info so the header can update.
#[tauri::command]
pub async fn name_drone<R: Runtime>(app: AppHandle<R>, uid: String, name: String) -> CmdResult<Option<VehicleInfo>> {
    blocking(app, move |app| {
        let name = name.split_whitespace().collect::<Vec<_>>().join(" ");
        if name.is_empty() {
            return Err("Give the vehicle a name.".to_owned());
        }
        if name.chars().count() > 40 {
            return Err("Keep the name under 40 characters.".to_owned());
        }
        let mut drones = load(app)?;
        if drones.iter().any(|d| d.uid != uid && d.name.as_deref().is_some_and(|n| n.eq_ignore_ascii_case(&name))) {
            return Err(format!("Another vehicle is already called \"{name}\"."));
        }
        match drones.iter_mut().find(|d| d.uid == uid) {
            Some(d) => d.name = Some(name),
            None => drones.push(Drone { uid: uid.clone(), name: Some(name), board: None, last_seen: None }),
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
