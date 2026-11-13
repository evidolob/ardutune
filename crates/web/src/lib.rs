//! The web app's backend, compiled to WebAssembly and run in a Web Worker
//! (`www/worker.js`): the desktop app's log analysis commands, and its
//! vehicle commands over Web Serial (`vehicle`, `serial`). Results are JSON
//! strings, the same shapes the desktop app's commands return.

use std::cell::RefCell;
use std::collections::HashMap;

use ardutune_core::filters::FilterReport;
use ardutune_core::pid::PidReport;
use ardutune_core::synth::{sample_flight, Preset};
use ardutune_core::views::{self, View};
use ardutune_core::{Log, Report};
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
mod serial;
pub mod vehicle;

/// The open log and what has been worked out from it.
pub struct Loaded {
    pub log: Log,
    pub readiness: Report,
    pub filters: Option<FilterReport>,
    pub pid: Option<PidReport>,
    views: HashMap<String, Result<View, String>>,
}

thread_local! {
    static OPEN: RefCell<Option<Loaded>> = const { RefCell::new(None) };
    static VEHICLE: RefCell<Option<vehicle::Session>> = const { RefCell::new(None) };
}

fn to_json(v: &impl serde::Serialize) -> String {
    serde_json::to_string(v).unwrap_or_else(|_| "null".to_owned())
}

/// Opens a log from its bytes; returns the overview JSON.
#[wasm_bindgen]
pub fn open_log(name: &str, bytes: &[u8]) -> String {
    let (log, readiness, overview) = views::open(name, Some(name.to_owned()), bytes);
    OPEN.with(|o| *o.borrow_mut() = Some(Loaded { log, readiness, filters: None, pid: None, views: HashMap::new() }));
    overview.to_string()
}

/// Opens the synthetic sample flight.
#[wasm_bindgen]
pub fn open_sample() -> String {
    open_log("ardutune-sample.bin", &sample_flight(Preset::Ready))
}

/// Runs (or returns the cached) analysis `kind`; the view JSON, or an error.
#[wasm_bindgen]
pub fn analysis(kind: &str) -> Result<String, JsError> {
    run_analysis(kind).map_err(|e| JsError::new(&e))
}

fn run_analysis(kind: &str) -> Result<String, String> {
    OPEN.with(|o| {
        let mut slot = o.borrow_mut();
        let loaded = slot.as_mut().ok_or("Open a log first.")?;
        if !loaded.views.contains_key(kind) {
            let computed = views::compute(kind, &loaded.log)?;
            loaded.filters = computed.filters.or(loaded.filters.take());
            loaded.pid = computed.pid.or(loaded.pid.take());
            loaded.views.insert(kind.to_owned(), computed.view);
        }
        loaded.views[kind].clone().map(|v| to_json(&v))
    })
}

/// Forgets the open log.
#[wasm_bindgen]
pub fn close_log() {
    OPEN.with(|o| *o.borrow_mut() = None);
}

/// Connects over Web Serial: the page shares the ring buffer it reads the
/// port into (`ctrl`, `data`) and a `write` callback; `progress(done, total)`
/// follows the parameter download. Returns the vehicle info JSON.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub fn vehicle_connect(ctrl: js_sys::Int32Array, data: js_sys::Uint8Array, write: js_sys::Function, progress: js_sys::Function) -> Result<String, JsError> {
    use std::time::Duration;
    VEHICLE.with(|v| *v.borrow_mut() = None);
    let transport = serial::WebSerial::new(ctrl, data, write);
    let mut link = ardutune_link::Link::open(Box::new(transport), Duration::from_secs(8)).map_err(|e| JsError::new(&e.to_string()))?;
    let params = link
        .fetch_params(&mut |done, total| {
            let _ = progress.call2(&JsValue::NULL, &(done as u32).into(), &(total as u32).into());
        })
        .map_err(|e| JsError::new(&e.to_string()))?;
    let session = vehicle::Session::new(link, params);
    let info = session.info().to_string();
    VEHICLE.with(|v| *v.borrow_mut() = Some(session));
    Ok(info)
}

/// Runs a vehicle command (`args` as JSON); the result as JSON.
#[wasm_bindgen]
pub fn vehicle_call(cmd: &str, args: &str) -> Result<String, JsError> {
    let args: serde_json::Value = serde_json::from_str(args).unwrap_or(serde_json::Value::Null);
    OPEN.with(|o| {
        let log = o.borrow();
        VEHICLE.with(|v| vehicle::call(&mut v.borrow_mut(), cmd, &args, log.as_ref()))
    })
    .map(|v| v.to_string())
    .map_err(|e| JsError::new(&e))
}

#[cfg(test)]
mod tests {
    #[test]
    fn sample_round_trip() {
        let overview: serde_json::Value = serde_json::from_str(&super::open_sample()).unwrap();
        assert_eq!(overview["file_name"], "ardutune-sample.bin");
        let pid: serde_json::Value = serde_json::from_str(&super::run_analysis("pid").unwrap()).unwrap();
        assert!(pid["svg"].as_str().is_some_and(|s| s.starts_with("<svg")));
        super::close_log();
        assert_eq!(super::run_analysis("pid").unwrap_err(), "Open a log first.");
    }
}
