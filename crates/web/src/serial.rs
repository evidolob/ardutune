//! The link's transport in the browser: the page reads the Web Serial port
//! into a shared ring buffer, and this side, running in a Web Worker, waits
//! on it with `Atomics.wait`, so the desktop link's blocking code runs as it
//! is. Writes go back to the page through a callback.
//!
//! Ring layout: `ctrl[0]` counts bytes the page has written (wrapping),
//! `ctrl[1]` is 1 once the port is gone; `data` holds the bytes.

use std::time::Duration;

use ardutune_link::frames::{encode, Decoder};
use ardutune_link::{gcs_heartbeat, Instant, MavHeader, Transport, HEARTBEAT_INTERVAL};
use js_sys::{Atomics, Function, Int32Array, Uint8Array};
use mavlink::dialects::ardupilotmega::MavMessage;
use wasm_bindgen::JsValue;

pub struct WebSerial {
    ctrl: Int32Array,
    data: Uint8Array,
    /// Bytes read so far (wrapping, like `ctrl[0]`).
    read: u32,
    write: Function,
    decoder: Decoder,
    sequence: u8,
    last_heartbeat: Option<Instant>,
    /// Never notified: `sleep` waits on it.
    nap: Int32Array,
}

impl WebSerial {
    pub fn new(ctrl: Int32Array, data: Uint8Array, write: Function) -> Self {
        let nap = Int32Array::new(&js_sys::SharedArrayBuffer::new(4));
        WebSerial { ctrl, data, read: 0, write, decoder: Decoder::default(), sequence: 0, last_heartbeat: None, nap }
    }

    /// Moves what the page has written into the decoder; false if nothing new.
    fn pull(&mut self) -> bool {
        let written = Atomics::load(&self.ctrl, 0).unwrap_or(0) as u32;
        let cap = self.data.length();
        let mut avail = written.wrapping_sub(self.read);
        if avail == 0 {
            return false;
        }
        // Fell behind by more than the ring holds: skip what was overwritten.
        if avail > cap {
            self.read = written.wrapping_sub(cap);
            avail = cap;
        }
        let mut bytes = vec![0u8; avail as usize];
        let start = self.read % cap;
        let first = (cap - start).min(avail);
        self.data.subarray(start, start + first).copy_to(&mut bytes[..first as usize]);
        if first < avail {
            self.data.subarray(0, avail - first).copy_to(&mut bytes[first as usize..]);
        }
        self.read = written;
        self.decoder.push(&bytes);
        true
    }

    fn closed(&self) -> bool {
        Atomics::load(&self.ctrl, 1).unwrap_or(0) != 0
    }

    fn heartbeat_if_due(&mut self) {
        if self.last_heartbeat.is_none_or(|t| t.elapsed() >= HEARTBEAT_INTERVAL) {
            self.last_heartbeat = Some(Instant::now());
            let _ = self.send(&gcs_heartbeat());
        }
    }
}

impl Transport for WebSerial {
    fn send(&mut self, msg: &MavMessage) -> Result<(), String> {
        let bytes = encode(self.sequence, msg);
        self.sequence = self.sequence.wrapping_add(1);
        self.write.call1(&JsValue::NULL, &Uint8Array::from(&bytes[..])).map(|_| ()).map_err(|e| format!("{e:?}"))
    }

    fn recv(&mut self, timeout: Duration) -> Result<Option<(MavHeader, MavMessage)>, String> {
        let deadline = Instant::now() + timeout;
        loop {
            self.heartbeat_if_due();
            if let Some(m) = self.decoder.next_message() {
                return Ok(Some(m));
            }
            if self.pull() {
                continue;
            }
            if self.closed() {
                return Err("the USB port closed".to_owned());
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            // Wake on new bytes, at the deadline, or for the next heartbeat.
            let wait = left.min(HEARTBEAT_INTERVAL);
            let seen = Atomics::load(&self.ctrl, 0).unwrap_or(0);
            let _ = Atomics::wait_with_timeout(&self.ctrl, 0, seen, wait.as_secs_f64() * 1000.0);
        }
    }

    fn sleep(&mut self, duration: Duration) {
        let _ = Atomics::wait_with_timeout(&self.nap, 0, 0, duration.as_secs_f64() * 1000.0);
    }
}
