//! MAVLink link to an ArduPilot vehicle: parameters, reboot, log download and
//! deletion.
//!
//! A background thread reads incoming messages into a channel so every
//! request can wait with a timeout; another sends a GCS heartbeat once a
//! second, which also lets UDP vehicles learn our address.

use std::collections::BTreeMap;
use std::fmt;
use std::io;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavParamType, MavResult, MavState, MavType, COMMAND_LONG_DATA,
    FILE_TRANSFER_PROTOCOL_DATA, GpsFixType, HEARTBEAT_DATA, LOG_ERASE_DATA, LOG_REQUEST_DATA_DATA, LOG_REQUEST_END_DATA, LOG_REQUEST_LIST_DATA, PARAM_REQUEST_LIST_DATA,
    PARAM_REQUEST_READ_DATA, PARAM_SET_DATA,
};
use mavlink::error::MessageReadError;
use mavlink::{MavConnection, MavHeader};

pub mod ftp;
pub mod mock;
pub mod ports;

pub use mavlink::dialects::ardupilotmega::MavParamType as ParamType;

/// Mission Planner's component id; ArduPilot treats it as a normal GCS.
const GCS_COMPONENT_ID: u8 = 190;
const GCS_SYSTEM_ID: u32 = 255;
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);
const POLL_SLEEP: Duration = Duration::from_millis(1);
/// Bytes per LOG_DATA message.
const LOG_CHUNK: usize = 90;

type Conn = Arc<dyn MavConnection<MavMessage> + Send + Sync>;
type Incoming = std::result::Result<(MavHeader, MavMessage), io::Error>;

#[derive(Debug)]
pub enum Error {
    Connect(io::Error),
    NoVehicle,
    Disconnected,
    Armed,
    Timeout(String),
    UnknownParam(String),
    Rejected { name: String, wanted: f64, got: f32 },
    Command(String),
    Send(String),
    Ftp(String),
    /// The newest log may still be open for writing: ArduPilot keeps it open
    /// after disarming. Deleting an open file can corrupt the SD card.
    NewestLogOpen(u16),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Connect(e) => write!(f, "could not open connection: {e}"),
            Error::NoVehicle => write!(f, "no ArduPilot heartbeat received; check the connection and baud rate"),
            Error::Disconnected => write!(f, "connection to the vehicle was lost"),
            Error::Armed => write!(f, "the vehicle is armed; disarm before changing parameters"),
            Error::Timeout(what) => write!(f, "timed out {what}"),
            Error::UnknownParam(name) => write!(f, "the vehicle has no parameter {name}"),
            Error::Rejected { name, wanted, got } => {
                write!(f, "{name}: vehicle kept {got} instead of {wanted} (value out of range?)")
            }
            Error::Command(e) => write!(f, "command failed: {e}"),
            Error::Send(e) => write!(f, "send failed: {e}"),
            Error::Ftp(e) => write!(f, "file access on the vehicle failed: {e}"),
            Error::NewestLogOpen(id) => write!(
                f,
                "log {id} is the newest log, which the flight controller may still have open; reboot it first, then delete"
            ),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

/// The autopilot we are talking to, from its heartbeat.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vehicle {
    pub system_id: u8,
    pub component_id: u8,
    pub mav_type: MavType,
    pub autopilot: MavAutopilot,
    pub armed: bool,
}

impl Vehicle {
    pub fn is_ardupilot(&self) -> bool {
        self.autopilot == MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA
    }

    pub fn type_name(&self) -> &'static str {
        use MavType::*;
        match self.mav_type {
            MAV_TYPE_QUADROTOR => "quadcopter",
            MAV_TYPE_HEXAROTOR => "hexacopter",
            MAV_TYPE_OCTOROTOR => "octocopter",
            MAV_TYPE_TRICOPTER => "tricopter",
            MAV_TYPE_COAXIAL => "coaxial copter",
            MAV_TYPE_DODECAROTOR => "dodecacopter",
            MAV_TYPE_HELICOPTER => "helicopter",
            MAV_TYPE_FIXED_WING => "plane",
            MAV_TYPE_GROUND_ROVER | MAV_TYPE_SURFACE_BOAT => "rover",
            MAV_TYPE_SUBMARINE => "sub",
            _ => "other vehicle",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Param {
    pub value: f32,
    pub kind: MavParamType,
}

impl Param {
    pub fn is_integer(&self) -> bool {
        !matches!(self.kind, MavParamType::MAV_PARAM_TYPE_REAL32 | MavParamType::MAV_PARAM_TYPE_REAL64)
    }
}

pub type Params = BTreeMap<String, Param>;

/// Whether the value the vehicle reports back matches what we asked for.
pub fn values_match(wanted: f64, got: f32, integer: bool) -> bool {
    if integer {
        wanted.round() == (got as f64).round()
    } else {
        // Parameters travel as f32.
        (wanted - got as f64).abs() <= (wanted.abs() * 1e-6).max(1e-6)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogEntry {
    pub id: u16,
    pub size: u32,
    /// Seconds since 1970, or 0 when the vehicle had no GPS time.
    pub time_utc: u32,
}

/// One parameter to write, with the vehicle's current value.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedChange {
    pub name: String,
    pub current: Param,
    pub value: f64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Plan {
    pub changes: Vec<PlannedChange>,
    /// Already at the wanted value.
    pub unchanged: Vec<String>,
    /// Not present on the vehicle (wrong firmware version or a typo).
    pub unknown: Vec<String>,
}

/// Compares wanted values with the vehicle's parameters.
pub fn plan(params: &Params, wanted: &[(String, f64)]) -> Plan {
    let mut plan = Plan::default();
    for (name, value) in wanted {
        match params.get(name) {
            None => plan.unknown.push(name.clone()),
            Some(p) if values_match(*value, p.value, p.is_integer()) => plan.unchanged.push(name.clone()),
            Some(p) => plan.changes.push(PlannedChange { name: name.clone(), current: *p, value: *value }),
        }
    }
    plan
}

pub struct Link {
    conn: Conn,
    rx: Receiver<Incoming>,
    stop: Arc<AtomicBool>,
    sequence: Arc<AtomicU8>,
    vehicle: Vehicle,
    ftp_seq: u16,
}

impl Drop for Link {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

fn header(sequence: &AtomicU8) -> MavHeader {
    MavHeader {
        system_id: GCS_SYSTEM_ID,
        component_id: GCS_COMPONENT_ID,
        sequence: sequence.fetch_add(1, Ordering::Relaxed),
    }
}

fn gcs_heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

fn is_transient(e: &io::Error) -> bool {
    use io::ErrorKind::*;
    matches!(e.kind(), WouldBlock | TimedOut | Interrupted | ConnectionRefused | ConnectionReset)
}

fn spawn_reader(conn: Conn, stop: Arc<AtomicBool>, tx: Sender<Incoming>) {
    thread::spawn(move || {
        while !stop.load(Ordering::Relaxed) {
            match conn.try_recv() {
                Ok(m) => {
                    if tx.send(Ok(m)).is_err() {
                        break;
                    }
                }
                Err(MessageReadError::Io(e)) if is_transient(&e) => thread::sleep(POLL_SLEEP),
                Err(MessageReadError::Io(e)) => {
                    let _ = tx.send(Err(e));
                    break;
                }
                // Corrupt or unknown message: skip it.
                Err(MessageReadError::Parse(_)) => {}
            }
        }
    });
}

fn spawn_heartbeat(conn: Conn, stop: Arc<AtomicBool>, sequence: Arc<AtomicU8>) {
    thread::spawn(move || {
        while !stop.load(Ordering::Relaxed) {
            let _ = conn.send(&header(&sequence), &gcs_heartbeat());
            let until = Instant::now() + HEARTBEAT_INTERVAL;
            while Instant::now() < until && !stop.load(Ordering::Relaxed) {
                thread::sleep(Duration::from_millis(20));
            }
        }
    });
}

impl Link {
    /// Opens `address` (`serial:<port>:<baud>`, `udpin:<ip>:<port>`,
    /// `udpout:…`, `tcpout:…`) and waits for an ArduPilot heartbeat.
    pub fn connect(address: &str, timeout: Duration) -> Result<Self> {
        let conn: Conn = Arc::new(mavlink::connect::<MavMessage>(address).map_err(Error::Connect)?);
        let stop = Arc::new(AtomicBool::new(false));
        let sequence = Arc::new(AtomicU8::new(0));
        let (tx, rx) = mpsc::channel();
        spawn_reader(conn.clone(), stop.clone(), tx);
        spawn_heartbeat(conn.clone(), stop.clone(), sequence.clone());

        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(remaining) {
                Ok(Ok((h, MavMessage::HEARTBEAT(hb)))) if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID => {
                    let vehicle = Vehicle {
                        system_id: h.system_id as u8,
                        component_id: h.component_id,
                        mav_type: hb.mavtype,
                        autopilot: hb.autopilot,
                        armed: hb.base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
                    };
                    return Ok(Self { conn, rx, stop, sequence, vehicle, ftp_seq: 0 });
                }
                Ok(Ok(_)) => {}
                Ok(Err(_)) | Err(RecvTimeoutError::Disconnected) => {
                    stop.store(true, Ordering::Relaxed);
                    return Err(Error::Disconnected);
                }
                Err(RecvTimeoutError::Timeout) => {
                    stop.store(true, Ordering::Relaxed);
                    return Err(Error::NoVehicle);
                }
            }
        }
    }

    /// Reboots the vehicle and connects to it again at `address` once it is
    /// back (USB ports disappear while it restarts).
    pub fn reboot_and_reconnect(mut self, address: &str, timeout: Duration) -> Result<Self> {
        self.reboot()?;
        drop(self);
        // Give it time to go down before the first attempt.
        thread::sleep(Duration::from_secs(2));
        let deadline = Instant::now() + timeout;
        loop {
            match Link::connect(address, Duration::from_secs(3)) {
                Ok(link) => return Ok(link),
                Err(_) if Instant::now() < deadline => thread::sleep(Duration::from_millis(500)),
                Err(e) => return Err(e),
            }
        }
    }

    pub fn vehicle(&self) -> &Vehicle {
        &self.vehicle
    }

    fn send(&self, msg: &MavMessage) -> Result<()> {
        self.conn.send(&header(&self.sequence), msg).map(|_| ()).map_err(|e| Error::Send(e.to_string()))
    }

    /// Next message from our vehicle, or `None` at `deadline`. Keeps the
    /// armed state current from heartbeats.
    fn next(&mut self, deadline: Instant) -> Result<Option<MavMessage>> {
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            match self.rx.recv_timeout(remaining) {
                Ok(Ok((h, msg))) => {
                    if h.system_id as u8 != self.vehicle.system_id {
                        continue;
                    }
                    if let MavMessage::HEARTBEAT(hb) = &msg {
                        if h.component_id == self.vehicle.component_id {
                            self.vehicle.armed = hb.base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                        }
                    }
                    return Ok(Some(msg));
                }
                Ok(Err(_)) | Err(RecvTimeoutError::Disconnected) => return Err(Error::Disconnected),
                Err(RecvTimeoutError::Timeout) => return Ok(None),
            }
        }
    }

    /// Waits up to `timeout` for a message that `f` accepts.
    fn wait_for<T>(&mut self, timeout: Duration, mut f: impl FnMut(&MavMessage) -> Option<T>) -> Result<Option<T>> {
        let deadline = Instant::now() + timeout;
        while let Some(msg) = self.next(deadline)? {
            if let Some(v) = f(&msg) {
                return Ok(Some(v));
            }
        }
        Ok(None)
    }

    /// Waits for a fresh heartbeat and returns whether the vehicle is armed.
    pub fn refresh_armed(&mut self) -> Result<bool> {
        let is_autopilot_heartbeat =
            |m: &MavMessage| matches!(m, MavMessage::HEARTBEAT(hb) if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID);
        self.wait_for(Duration::from_millis(2500), |m| is_autopilot_heartbeat(m).then_some(()))?
            .ok_or(Error::Timeout("waiting for a heartbeat".to_owned()))?;
        Ok(self.vehicle.armed)
    }

    /// Downloads every parameter. `progress(received, total)`.
    pub fn fetch_params(&mut self, progress: &mut dyn FnMut(usize, usize)) -> Result<Params> {
        let (sys, comp) = (self.vehicle.system_id, self.vehicle.component_id);
        self.send(&MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA { target_system: sys, target_component: comp }))?;

        let mut got: BTreeMap<u16, (String, Param)> = BTreeMap::new();
        let mut total: Option<u16> = None;
        let mut last_rx = Instant::now();
        let mut retries = 0;
        loop {
            if let Some(MavMessage::PARAM_VALUE(p)) = self.next(Instant::now() + Duration::from_millis(100))? {
                total = Some(p.param_count);
                // Index 65535 marks a value sent outside the list (e.g. after a set).
                if p.param_index < p.param_count {
                    let name = p.param_id.to_str().unwrap_or_default().to_owned();
                    got.insert(p.param_index, (name, Param { value: p.param_value, kind: p.param_type }));
                    progress(got.len(), p.param_count as usize);
                }
                last_rx = Instant::now();
            }
            if total.is_some_and(|t| got.len() >= t as usize) {
                break;
            }
            if last_rx.elapsed() > Duration::from_secs(1) {
                retries += 1;
                if retries > 5 {
                    return Err(Error::Timeout(format!(
                        "downloading parameters ({} of {} received)",
                        got.len(),
                        total.unwrap_or(0)
                    )));
                }
                match total {
                    None => self.send(&MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
                        target_system: sys,
                        target_component: comp,
                    }))?,
                    Some(t) => {
                        for idx in (0..t).filter(|i| !got.contains_key(i)).take(64) {
                            self.send(&MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                                param_index: idx as i16,
                                target_system: sys,
                                target_component: comp,
                                param_id: "".into(),
                            }))?;
                        }
                    }
                }
                last_rx = Instant::now();
            }
        }
        Ok(got.into_values().collect())
    }

    /// Writes one parameter and returns the value the vehicle confirmed.
    pub fn set_param(&mut self, name: &str, value: f64, kind: MavParamType) -> Result<f32> {
        if name.is_empty() || name.len() > 16 {
            return Err(Error::UnknownParam(name.to_owned()));
        }
        let integer = Param { value: 0.0, kind }.is_integer();
        for _ in 0..3 {
            self.send(&MavMessage::PARAM_SET(PARAM_SET_DATA {
                param_value: value as f32,
                target_system: self.vehicle.system_id,
                target_component: self.vehicle.component_id,
                param_id: name.into(),
                param_type: kind,
            }))?;
            let reply = self.wait_for(Duration::from_secs(1), |m| match m {
                MavMessage::PARAM_VALUE(p) if p.param_id.to_str() == Ok(name) => Some(p.param_value),
                _ => None,
            })?;
            if let Some(got) = reply {
                return if values_match(value, got, integer) {
                    Ok(got)
                } else {
                    Err(Error::Rejected { name: name.to_owned(), wanted: value, got })
                };
            }
        }
        Err(Error::UnknownParam(name.to_owned()))
    }

    /// Writes all planned changes, refusing if the vehicle is armed.
    /// Returns one result per change, in order.
    pub fn apply(
        &mut self,
        changes: &[PlannedChange],
        progress: &mut dyn FnMut(&PlannedChange, &Result<f32>),
    ) -> Result<Vec<Result<f32>>> {
        if self.refresh_armed()? {
            return Err(Error::Armed);
        }
        let mut results = Vec::with_capacity(changes.len());
        for c in changes {
            // Heartbeats seen while waiting for replies keep this current.
            if self.vehicle.armed {
                return Err(Error::Armed);
            }
            let r = self.set_param(&c.name, c.value, c.current.kind);
            if matches!(r, Err(Error::Disconnected)) {
                return Err(Error::Disconnected);
            }
            progress(c, &r);
            results.push(r);
        }
        Ok(results)
    }

    /// Reboots the flight controller (only accepted while disarmed).
    pub fn reboot(&mut self) -> Result<()> {
        if self.refresh_armed()? {
            return Err(Error::Armed);
        }
        let cmd = MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        self.send(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 1.0,
            command: cmd,
            target_system: self.vehicle.system_id,
            target_component: self.vehicle.component_id,
            ..Default::default()
        }))?;
        let result = self.wait_for(Duration::from_secs(3), |m| match m {
            MavMessage::COMMAND_ACK(a) if a.command == cmd => Some(a.result),
            _ => None,
        })?;
        match result {
            Some(MavResult::MAV_RESULT_ACCEPTED) => Ok(()),
            Some(r) => Err(Error::Command(format!("reboot refused ({r:?})"))),
            None => Err(Error::Timeout("waiting for the reboot to be acknowledged".to_owned())),
        }
    }

    /// Lists the logs stored on the flight controller, oldest first.
    pub fn list_logs(&mut self) -> Result<Vec<LogEntry>> {
        for _ in 0..3 {
            self.send(&MavMessage::LOG_REQUEST_LIST(LOG_REQUEST_LIST_DATA {
                start: 0,
                end: 0xffff,
                target_system: self.vehicle.system_id,
                target_component: self.vehicle.component_id,
            }))?;
            let mut entries: BTreeMap<u16, LogEntry> = BTreeMap::new();
            let mut total = None;
            let deadline_gap = Duration::from_millis(1500);
            while let Some(e) = self.wait_for(deadline_gap, |m| match m {
                MavMessage::LOG_ENTRY(e) => Some(e.clone()),
                _ => None,
            })? {
                total = Some(e.num_logs as usize);
                if e.num_logs == 0 {
                    return Ok(Vec::new());
                }
                entries.insert(e.id, LogEntry { id: e.id, size: e.size, time_utc: e.time_utc });
                if entries.len() >= e.num_logs as usize {
                    return Ok(entries.into_values().collect());
                }
            }
            if total.is_none() {
                continue;
            }
        }
        Err(Error::Timeout("listing logs".to_owned()))
    }

    /// Downloads one log, re-requesting any chunks lost on the way.
    /// `progress(bytes, total)`.
    pub fn download_log(&mut self, entry: &LogEntry, progress: &mut dyn FnMut(usize, usize)) -> Result<Vec<u8>> {
        let size = entry.size as usize;
        if size == 0 {
            return Ok(Vec::new());
        }
        let n_chunks = size.div_ceil(LOG_CHUNK);
        let mut buf = vec![0u8; size];
        let mut have = vec![false; n_chunks];
        let mut received = 0;

        // The vehicle serves one request at a time, so ask for one missing
        // range, and ask for the next when it ends or the stream stalls.
        let mut req_end = self.request_range(entry.id, &have, size)?;
        let mut last_rx = Instant::now();
        let mut stalls = 0;
        while received < n_chunks {
            let mut request_next = false;
            if let Some(MavMessage::LOG_DATA(d)) = self.next(Instant::now() + Duration::from_millis(50))? {
                let ofs = d.ofs as usize;
                if d.id == entry.id && ofs.is_multiple_of(LOG_CHUNK) && ofs < size {
                    let k = ofs / LOG_CHUNK;
                    let n = LOG_CHUNK.min(size - ofs);
                    if !have[k] && d.count as usize >= n {
                        buf[ofs..ofs + n].copy_from_slice(&d.data[..n]);
                        have[k] = true;
                        received += 1;
                        progress((received * LOG_CHUNK).min(size), size);
                    }
                    last_rx = Instant::now();
                    stalls = 0;
                    request_next = k + 1 >= req_end;
                }
            }
            if last_rx.elapsed() > Duration::from_millis(500) {
                stalls += 1;
                if stalls > 10 {
                    return Err(Error::Timeout(format!(
                        "downloading log {} ({} of {size} bytes)",
                        entry.id,
                        (received * LOG_CHUNK).min(size)
                    )));
                }
                request_next = true;
            }
            if request_next && received < n_chunks {
                req_end = self.request_range(entry.id, &have, size)?;
                last_rx = Instant::now();
            }
        }
        // Lets the vehicle resume normal logging.
        self.send(&MavMessage::LOG_REQUEST_END(LOG_REQUEST_END_DATA {
            target_system: self.vehicle.system_id,
            target_component: self.vehicle.component_id,
        }))?;
        Ok(buf)
    }

    /// Requests the first run of missing chunks; returns the chunk index it ends at.
    fn request_range(&self, id: u16, have: &[bool], size: usize) -> Result<usize> {
        let Some(start) = have.iter().position(|h| !h) else { return Ok(have.len()) };
        let end = have[start..].iter().position(|h| *h).map_or(have.len(), |n| start + n);
        let ofs = start * LOG_CHUNK;
        self.send(&MavMessage::LOG_REQUEST_DATA(LOG_REQUEST_DATA_DATA {
            ofs: ofs as u32,
            count: ((end * LOG_CHUNK).min(size) - ofs) as u32,
            id,
            target_system: self.vehicle.system_id,
            target_component: self.vehicle.component_id,
        }))?;
        Ok(end)
    }
}

/// What a vehicle reports about its readiness to arm, gathered by
/// [`Link::arming_status`].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ArmingStatus {
    /// Status texts received while listening, in order, without repeats.
    pub messages: Vec<String>,
    /// GPS_RAW_INT (instance 0) and GPS2_RAW (instance 1), when received.
    pub gps: Vec<GpsStatus>,
    /// SYS_STATUS sensor bits: (present, enabled, healthy).
    pub sensors: Option<(u32, u32, u32)>,
    /// Flight mode number from the heartbeat.
    pub custom_mode: Option<u32>,
    pub armed: bool,
    /// Latest RC input in µs, channels 1-8 (0 when not received).
    pub rc: Option<[u16; 8]>,
    /// Battery voltage from SYS_STATUS.
    pub battery_v: Option<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsStatus {
    pub instance: u8,
    /// MAVLink GPS_FIX_TYPE: 0 no GPS, 1 no fix, 2 2D, 3 3D, 4+ better.
    pub fix_type: u8,
    pub satellites: Option<u8>,
    pub hdop: Option<f32>,
}

const GPS_RAW_INT_ID: f32 = 24.0;
const GPS2_RAW_ID: f32 = 124.0;
const SYS_STATUS_ID: f32 = 1.0;
const RC_CHANNELS_ID: f32 = 65.0;

impl ArmingStatus {
    /// The input ArduTune's arming explanation works from.
    pub fn snapshot(&self) -> ardutune_core::arming::Snapshot {
        ardutune_core::arming::Snapshot {
            messages: self.messages.clone(),
            gps: self
                .gps
                .iter()
                .map(|g| ardutune_core::arming::Gps { instance: g.instance, fix_type: g.fix_type, satellites: g.satellites, hdop: g.hdop })
                .collect(),
            sensors: self.sensors,
            mode: self.custom_mode,
            armed: self.armed,
            rc: self.rc,
        }
    }

    /// The live state the setup guide uses.
    pub fn setup_live(&self) -> ardutune_core::setup::Live {
        ardutune_core::setup::Live { sensors: self.sensors, battery_v: self.battery_v, messages: self.messages.clone() }
    }
}

fn gps_status(instance: u8, fix: GpsFixType, satellites: u8, eph: u16) -> GpsStatus {
    GpsStatus {
        instance,
        fix_type: fix as u8,
        satellites: (satellites != u8::MAX).then_some(satellites),
        hdop: (eph != u16::MAX).then_some(eph as f32 / 100.0),
    }
}

impl Link {
    /// Asks the vehicle to run its pre-arm checks and listens for `listen`
    /// to the reasons it gives, the GPS state and sensor health.
    pub fn arming_status(&mut self, listen: Duration) -> Result<ArmingStatus> {
        for id in [GPS_RAW_INT_ID, GPS2_RAW_ID, SYS_STATUS_ID, RC_CHANNELS_ID] {
            self.command(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, [id, 500_000.0, 0.0, 0.0, 0.0, 0.0, 0.0])?;
        }
        self.command(MavCmd::MAV_CMD_RUN_PREARM_CHECKS, [0.0; 7])?;

        let mut status = ArmingStatus::default();
        let mut pending: Option<String> = None;
        let deadline = Instant::now() + listen;
        while let Some(msg) = self.next(deadline)? {
            match msg {
                MavMessage::STATUSTEXT(t) => {
                    let raw = t.text.to_str().unwrap_or_default().to_owned();
                    // Longer texts arrive in full 50-character pieces.
                    let text = match pending.take() {
                        Some(head) => head + &raw,
                        None => raw.clone(),
                    };
                    if raw.len() >= STATUSTEXT_LEN {
                        pending = Some(text);
                    } else {
                        push_text(&mut status.messages, text);
                    }
                }
                MavMessage::GPS_RAW_INT(g) => upsert(&mut status.gps, gps_status(0, g.fix_type, g.satellites_visible, g.eph)),
                MavMessage::GPS2_RAW(g) => upsert(&mut status.gps, gps_status(1, g.fix_type, g.satellites_visible, g.eph)),
                MavMessage::SYS_STATUS(s) => {
                    status.sensors = Some((
                        s.onboard_control_sensors_present.bits(),
                        s.onboard_control_sensors_enabled.bits(),
                        s.onboard_control_sensors_health.bits(),
                    ));
                    status.battery_v = (s.voltage_battery != u16::MAX).then_some(s.voltage_battery as f32 / 1000.0);
                }
                MavMessage::HEARTBEAT(hb) if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID => {
                    status.custom_mode = Some(hb.custom_mode);
                }
                MavMessage::RC_CHANNELS(rc) => {
                    status.rc = Some([
                        rc.chan1_raw,
                        rc.chan2_raw,
                        rc.chan3_raw,
                        rc.chan4_raw,
                        rc.chan5_raw,
                        rc.chan6_raw,
                        rc.chan7_raw,
                        rc.chan8_raw,
                    ]);
                }
                _ => {}
            }
        }
        if let Some(text) = pending {
            push_text(&mut status.messages, text);
        }
        status.gps.sort_by_key(|g| g.instance);
        status.armed = self.vehicle.armed;
        Ok(status)
    }

    /// Sends a command without waiting for its acknowledgement.
    fn command(&self, command: MavCmd, p: [f32; 7]) -> Result<()> {
        self.send(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command,
            param1: p[0],
            param2: p[1],
            param3: p[2],
            param4: p[3],
            param5: p[4],
            param6: p[5],
            param7: p[6],
            target_system: self.vehicle.system_id,
            target_component: self.vehicle.component_id,
            ..Default::default()
        }))
    }
}

const STATUSTEXT_LEN: usize = 50;

fn push_text(messages: &mut Vec<String>, text: String) {
    let text = text.trim().to_owned();
    if !text.is_empty() && !messages.contains(&text) {
        messages.push(text);
    }
}

fn upsert(list: &mut Vec<GpsStatus>, g: GpsStatus) {
    match list.iter_mut().find(|x| x.instance == g.instance) {
        Some(x) => *x = g,
        None => list.push(g),
    }
}

/// Where ArduPilot keeps logs: on the SD card of ChibiOS boards, and in
/// `logs` relative to the working directory in SITL.
const LOG_DIRS: [&str; 2] = ["/APM/LOGS", "logs"];
const ERASE_TIMEOUT: Duration = Duration::from_secs(60);

/// A log file on the vehicle: (number, name, size).
pub type LogFile = (u32, String, u64);

impl Link {
    /// Erases every log on the flight controller and waits until the list is
    /// empty. MAVLink's log protocol can only erase all logs.
    pub fn erase_all_logs(&mut self) -> Result<()> {
        if self.refresh_armed()? {
            return Err(Error::Armed);
        }
        self.send(&MavMessage::LOG_ERASE(LOG_ERASE_DATA {
            target_system: self.vehicle.system_id,
            target_component: self.vehicle.component_id,
        }))?;
        // No reply comes: poll the list (it may not answer while erasing).
        let deadline = Instant::now() + ERASE_TIMEOUT;
        while Instant::now() < deadline {
            thread::sleep(Duration::from_millis(500));
            match self.list_logs() {
                Ok(logs) if logs.iter().all(|l| l.size == 0) => return Ok(()),
                Ok(_) | Err(Error::Timeout(_)) => {}
                Err(e) => return Err(e),
            }
        }
        Err(Error::Timeout("waiting for the logs to be erased".to_owned()))
    }

    /// Deletes one log's file over MAVLink FTP. The log list numbers logs from
    /// the oldest file, so log `id` is file number oldest + id - 1; that is
    /// only trusted when every listed log's size matches its file.
    ///
    /// ArduPilot keeps the newest log open after disarming, so it is only
    /// deleted when `just_rebooted` says the vehicle hasn't armed since boot.
    pub fn delete_log(&mut self, id: u16, just_rebooted: bool) -> Result<()> {
        if self.refresh_armed()? {
            return Err(Error::Armed);
        }
        let logs = self.list_logs()?;
        if !logs.iter().any(|l| l.id == id && l.size > 0) {
            return Err(Error::Command(format!("the vehicle has no log {id}")));
        }
        let newest = logs.iter().filter(|l| l.size > 0).map(|l| l.id).max();
        if newest == Some(id) && !just_rebooted {
            return Err(Error::NewestLogOpen(id));
        }
        let (dir, files) = self.log_files()?;
        let oldest = files.iter().map(|f| f.0).min().unwrap_or(0);
        let file_for = |id: u16| files.iter().find(|f| f.0 == oldest + id as u32 - 1);
        let consistent = logs.iter().filter(|l| l.size > 0).all(|l| file_for(l.id).is_some_and(|f| f.2 == l.size as u64));
        if !consistent {
            return Err(Error::Ftp(
                "the log list doesn't match the files on the SD card, so nothing was deleted; \
                 remove the file from the card directly or erase all logs"
                    .to_owned(),
            ));
        }
        let (_, name, _) = file_for(id).expect("checked above");
        let path = format!("{dir}/{name}");
        let reply = self.ftp(ftp::REMOVE_FILE, 0, path.as_bytes())?;
        match reply.opcode {
            ftp::ACK => Ok(()),
            _ => Err(Error::Ftp(format!("could not delete {path} ({})", nak_reason(&reply)))),
        }
    }

    /// The log directory and the log files in it, read over MAVLink FTP.
    pub fn log_files(&mut self) -> Result<(&'static str, Vec<LogFile>)> {
        let mut last_err = None;
        for dir in LOG_DIRS {
            match self.list_dir(dir) {
                Ok(entries) => {
                    let files = entries
                        .into_iter()
                        .filter_map(|e| match e {
                            ftp::Entry::File { name, size } => ftp::log_number(&name).map(|n| (n, name, size)),
                            _ => None,
                        })
                        .collect();
                    return Ok((dir, files));
                }
                Err(e) => last_err = Some(e),
            }
        }
        Err(last_err.unwrap_or_else(|| Error::Ftp("no log directory".to_owned())))
    }

    pub fn list_dir(&mut self, path: &str) -> Result<Vec<ftp::Entry>> {
        let mut entries = Vec::new();
        loop {
            let reply = self.ftp(ftp::LIST_DIRECTORY, entries.len() as u32, path.as_bytes())?;
            if reply.opcode == ftp::NAK {
                if reply.data.first() == Some(&ftp::ERR_EOF) {
                    return Ok(entries);
                }
                return Err(Error::Ftp(format!("listing {path} ({})", nak_reason(&reply))));
            }
            let batch = ftp::parse_entries(&reply.data);
            if batch.is_empty() {
                return Ok(entries);
            }
            entries.extend(batch);
        }
    }

    /// Sends an FTP request and waits for its reply, resending on silence.
    fn ftp(&mut self, opcode: u8, offset: u32, data: &[u8]) -> Result<ftp::Packet> {
        self.ftp_in_session(0, opcode, offset, data)
    }

    fn ftp_in_session(&mut self, session: u8, opcode: u8, offset: u32, data: &[u8]) -> Result<ftp::Packet> {
        self.ftp_seq = self.ftp_seq.wrapping_add(1);
        let req = ftp::Packet { session, ..ftp::Packet::new(self.ftp_seq, opcode, offset, data) };
        let expected = req.seq.wrapping_add(1);
        for _ in 0..3 {
            self.send(&MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
                target_network: 0,
                target_system: self.vehicle.system_id,
                target_component: self.vehicle.component_id,
                payload: req.encode(),
            }))?;
            let reply = self.wait_for(Duration::from_secs(1), |m| match m {
                MavMessage::FILE_TRANSFER_PROTOCOL(f) => {
                    let p = ftp::Packet::decode(&f.payload);
                    (p.seq == expected && p.req_opcode == opcode).then_some(p)
                }
                _ => None,
            })?;
            if let Some(p) = reply {
                return Ok(p);
            }
        }
        Err(Error::Timeout("waiting for a file transfer reply (does this firmware support MAVLink FTP?)".to_owned()))
    }
}

impl Link {
    /// Reads a whole file from the vehicle over MAVLink FTP.
    /// `progress(bytes)` after each piece.
    pub fn read_file(&mut self, path: &str, progress: &mut dyn FnMut(usize)) -> Result<Vec<u8>> {
        // Close a session a previous program may have left open.
        self.ftp(ftp::RESET_SESSIONS, 0, &[])?;
        let open = self.ftp(ftp::OPEN_FILE_RO, 0, path.as_bytes())?;
        if open.opcode != ftp::ACK {
            return Err(Error::Ftp(format!("opening {path} ({})", nak_reason(&open))));
        }
        let session = open.session;
        let mut data = Vec::new();
        let result = loop {
            // The request's size field asks for that many bytes.
            let reply = self.ftp_in_session(session, ftp::READ_FILE, data.len() as u32, &[0; ftp::DATA_MAX])?;
            if reply.opcode == ftp::NAK {
                if reply.data.first() == Some(&ftp::ERR_EOF) {
                    break Ok(());
                }
                break Err(Error::Ftp(format!("reading {path} ({})", nak_reason(&reply))));
            }
            if reply.offset as usize != data.len() {
                continue;
            }
            data.extend_from_slice(&reply.data);
            progress(data.len());
            if reply.data.len() < ftp::DATA_MAX {
                break Ok(());
            }
        };
        let _ = self.ftp_in_session(session, ftp::TERMINATE_SESSION, 0, &[]);
        result.map(|_| data)
    }

    /// Every parameter with its default value, from ArduPilot's packed
    /// parameter file (4.1+). `None` when the firmware doesn't offer it.
    pub fn param_defaults(&mut self) -> Result<Option<Vec<ftp::PackedParam>>> {
        match self.read_file("@PARAM/param.pck?withdefaults=1", &mut |_| {}) {
            Ok(bytes) => ftp::parse_param_pck(&bytes).map(Some).map_err(Error::Ftp),
            Err(Error::Ftp(_)) | Err(Error::Timeout(_)) => Ok(None),
            Err(e) => Err(e),
        }
    }
}

fn nak_reason(p: &ftp::Packet) -> String {
    match p.data.first() {
        Some(&ftp::ERR_FILE_NOT_FOUND) => "file not found".to_owned(),
        Some(&ftp::ERR_EOF) => "end of file".to_owned(),
        Some(2) => format!("system error {}", p.data.get(1).copied().unwrap_or(0)),
        Some(9) => "file is protected".to_owned(),
        Some(code) => format!("error {code}"),
        None => "no reason given".to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(value: f32, kind: MavParamType) -> Param {
        Param { value, kind }
    }

    #[test]
    fn matching_respects_type() {
        assert!(values_match(0.135, 0.135f32, false));
        assert!(!values_match(0.135, 0.14, false));
        assert!(values_match(180223.0, 180223.0, true));
        assert!(values_match(9.0, 9.2, true));
        assert!(!values_match(9.0, 10.0, true));
    }

    #[test]
    fn plans_changes() {
        let mut params = Params::new();
        params.insert("LOG_BITMASK".into(), p(176126.0, MavParamType::MAV_PARAM_TYPE_INT32));
        params.insert("INS_RAW_LOG_OPT".into(), p(9.0, MavParamType::MAV_PARAM_TYPE_INT8));
        let wanted = vec![
            ("LOG_BITMASK".to_owned(), 180223.0),
            ("INS_RAW_LOG_OPT".to_owned(), 9.0),
            ("NOPE".to_owned(), 1.0),
        ];
        let plan = plan(&params, &wanted);
        assert_eq!(plan.changes.len(), 1);
        assert_eq!(plan.changes[0].name, "LOG_BITMASK");
        assert_eq!(plan.unchanged, vec!["INS_RAW_LOG_OPT"]);
        assert_eq!(plan.unknown, vec!["NOPE"]);
    }
}
