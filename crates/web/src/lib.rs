//! The web app's backend: the desktop app's log analysis commands, compiled
//! to WebAssembly and run in a Web Worker (`www/worker.js`). Results are JSON
//! strings, the same shapes the desktop app's commands return.

use std::cell::RefCell;
use std::collections::HashMap;

use ardutune_core::synth::{sample_flight, Preset};
use ardutune_core::views::{self, View};
use ardutune_core::Log;
use wasm_bindgen::prelude::*;

struct Loaded {
    log: Log,
    views: HashMap<String, Result<View, String>>,
}

thread_local! {
    static OPEN: RefCell<Option<Loaded>> = const { RefCell::new(None) };
}

fn to_json(v: &impl serde::Serialize) -> String {
    serde_json::to_string(v).unwrap_or_else(|_| "null".to_owned())
}

/// Opens a log from its bytes; returns the overview JSON.
#[wasm_bindgen]
pub fn open_log(name: &str, bytes: &[u8]) -> String {
    let (log, _, overview) = views::open(name, Some(name.to_owned()), bytes);
    OPEN.with(|o| *o.borrow_mut() = Some(Loaded { log, views: HashMap::new() }));
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
