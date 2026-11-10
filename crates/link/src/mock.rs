//! A simulated ArduPilot vehicle speaking just enough MAVLink for tests and
//! demos: heartbeats, the parameter protocol, log listing/download and reboot.
//! It can drop messages to exercise retry paths.

use std::collections::{BTreeMap, BTreeSet};
use std::io;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavParamType, MavResult, MavState, MavType, COMMAND_ACK_DATA,
    HEARTBEAT_DATA, LOG_DATA_DATA, LOG_ENTRY_DATA, PARAM_VALUE_DATA,
};
use mavlink::{MavConnection, MavHeader};

const LOG_CHUNK: usize = 90;
/// LOG_DATA messages sent per loop pass, so requests are still served while streaming.
const CHUNKS_PER_PASS: usize = 20;

#[derive(Debug, Clone)]
pub struct MockState {
    /// Name → (value, type), in index order.
    pub params: BTreeMap<String, (f32, MavParamType)>,
    /// Allowed range per parameter; writes are clamped like the firmware does.
    pub limits: BTreeMap<String, (f32, f32)>,
    pub armed: bool,
    pub logs: Vec<Vec<u8>>,
    /// Drop one LOG_DATA chunk in every `n`, the first time it is sent.
    pub drop_log_chunk_every: Option<usize>,
    /// Parameter indices left out of the first full list.
    pub drop_param_indices: BTreeSet<u16>,
    pub reboots: usize,
    pub param_sets: usize,
}

impl MockState {
    /// A quad with Copter's default logging and a few hundred parameters.
    pub fn quad(logs: Vec<Vec<u8>>) -> Self {
        use MavParamType::*;
        let mut params = BTreeMap::new();
        let base: [(&str, f32, MavParamType); 17] = [
            ("LOG_BITMASK", 176_126.0, MAV_PARAM_TYPE_INT32),
            ("INS_RAW_LOG_OPT", 0.0, MAV_PARAM_TYPE_INT8),
            ("INS_LOG_BAT_MASK", 0.0, MAV_PARAM_TYPE_INT8),
            ("INS_LOG_BAT_OPT", 0.0, MAV_PARAM_TYPE_INT8),
            ("INS_GYRO_FILTER", 20.0, MAV_PARAM_TYPE_INT16),
            ("INS_HNTCH_ENABLE", 0.0, MAV_PARAM_TYPE_INT8),
            ("INS_HNTCH_MODE", 1.0, MAV_PARAM_TYPE_INT8),
            ("INS_HNTCH_FREQ", 80.0, MAV_PARAM_TYPE_REAL32),
            ("INS_HNTCH_BW", 40.0, MAV_PARAM_TYPE_REAL32),
            ("INS_HNTCH_ATT", 40.0, MAV_PARAM_TYPE_REAL32),
            ("INS_HNTCH_HMNCS", 3.0, MAV_PARAM_TYPE_INT8),
            ("INS_HNTCH_OPTS", 0.0, MAV_PARAM_TYPE_INT16),
            ("INS_HNTCH_FM_RAT", 1.0, MAV_PARAM_TYPE_REAL32),
            ("INS_HNTCH_REF", 0.0, MAV_PARAM_TYPE_REAL32),
            ("ATC_RAT_RLL_P", 0.135, MAV_PARAM_TYPE_REAL32),
            ("MOT_THST_HOVER", 0.35, MAV_PARAM_TYPE_REAL32),
            ("SCHED_LOOP_RATE", 400.0, MAV_PARAM_TYPE_INT16),
        ];
        for (n, v, t) in base {
            params.insert(n.to_owned(), (v, t));
        }
        for i in 0..300 {
            params.insert(format!("ZZ_FILL_{i:03}"), (i as f32, MAV_PARAM_TYPE_REAL32));
        }
        let mut limits = BTreeMap::new();
        limits.insert("INS_GYRO_FILTER".to_owned(), (0.0, 256.0));
        Self {
            params,
            limits,
            armed: false,
            logs,
            drop_log_chunk_every: None,
            drop_param_indices: BTreeSet::new(),
            reboots: 0,
            param_sets: 0,
        }
    }
}

pub struct MockVehicle {
    state: Arc<Mutex<MockState>>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
    client_address: String,
}

impl Drop for MockVehicle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl MockVehicle {
    /// Starts on a free localhost UDP port; connect with [`Self::client_address`].
    pub fn start(state: MockState) -> io::Result<Self> {
        let port = UdpSocket::bind("127.0.0.1:0")?.local_addr()?.port();
        Self::start_on(&format!("udpin:127.0.0.1:{port}"), &format!("udpout:127.0.0.1:{port}"), state)
    }

    /// Starts listening on `listen` (a `udpin:`/`tcpin:` address).
    pub fn start_on(listen: &str, client_address: &str, state: MockState) -> io::Result<Self> {
        let conn = mavlink::connect::<MavMessage>(listen)?;
        let state = Arc::new(Mutex::new(state));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = {
            let (state, stop) = (state.clone(), stop.clone());
            thread::spawn(move || run(conn, state, stop))
        };
        Ok(Self { state, stop, handle: Some(handle), client_address: client_address.to_owned() })
    }

    pub fn client_address(&self) -> &str {
        &self.client_address
    }

    pub fn state(&self) -> MutexGuard<'_, MockState> {
        self.state.lock().unwrap()
    }
}

struct Stream {
    id: u16,
    ofs: usize,
    end: usize,
}

fn run(conn: impl MavConnection<MavMessage>, state: Arc<Mutex<MockState>>, stop: Arc<AtomicBool>) {
    let mut seq = 0u8;
    let mut send = |msg: MavMessage| {
        seq = seq.wrapping_add(1);
        let _ = conn.send(&MavHeader { system_id: 1, component_id: 1, sequence: seq }, &msg);
    };
    let mut last_heartbeat = Instant::now() - Duration::from_secs(1);
    let mut stream: Option<Stream> = None;
    let mut dropped: BTreeSet<usize> = BTreeSet::new();

    while !stop.load(Ordering::Relaxed) {
        if last_heartbeat.elapsed() > Duration::from_millis(200) {
            let armed = state.lock().unwrap().armed;
            send(heartbeat(armed));
            last_heartbeat = Instant::now();
        }

        let incoming = conn.try_recv();
        if let Ok((_, msg)) = incoming {
            let mut st = state.lock().unwrap();
            for reply in handle(&mut st, msg, &mut stream) {
                send(reply);
            }
        } else if stream.is_none() {
            thread::sleep(Duration::from_millis(1));
        }

        if let Some(s) = &mut stream {
            let st = state.lock().unwrap();
            let log = &st.logs[s.id as usize - 1];
            for _ in 0..CHUNKS_PER_PASS {
                if s.ofs >= s.end {
                    stream = None;
                    break;
                }
                let n = LOG_CHUNK.min(log.len() - s.ofs);
                let k = s.ofs / LOG_CHUNK;
                let drop = st.drop_log_chunk_every.is_some_and(|every| k % every == every / 2) && dropped.insert(s.ofs);
                if !drop {
                    let mut data = [0u8; LOG_CHUNK];
                    data[..n].copy_from_slice(&log[s.ofs..s.ofs + n]);
                    send(MavMessage::LOG_DATA(LOG_DATA_DATA { ofs: s.ofs as u32, id: s.id, count: n as u8, data }));
                }
                s.ofs += LOG_CHUNK;
            }
        }
    }
}

fn heartbeat(armed: bool) -> MavMessage {
    let mut mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if armed {
        mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mode,
        system_status: if armed { MavState::MAV_STATE_ACTIVE } else { MavState::MAV_STATE_STANDBY },
        mavlink_version: 3,
    })
}

fn param_value(st: &MockState, index: usize) -> Option<MavMessage> {
    let (name, (value, kind)) = st.params.iter().nth(index)?;
    Some(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
        param_value: *value,
        param_count: st.params.len() as u16,
        param_index: index as u16,
        param_id: name.as_str().into(),
        param_type: *kind,
    }))
}

fn handle(st: &mut MockState, msg: MavMessage, stream: &mut Option<Stream>) -> Vec<MavMessage> {
    match msg {
        MavMessage::PARAM_REQUEST_LIST(_) => {
            let skip = std::mem::take(&mut st.drop_param_indices);
            (0..st.params.len()).filter(|i| !skip.contains(&(*i as u16))).filter_map(|i| param_value(st, i)).collect()
        }
        MavMessage::PARAM_REQUEST_READ(r) => {
            let index = if r.param_index >= 0 {
                Some(r.param_index as usize)
            } else {
                let name = r.param_id.to_str().unwrap_or_default();
                st.params.keys().position(|k| k == name)
            };
            index.and_then(|i| param_value(st, i)).into_iter().collect()
        }
        MavMessage::PARAM_SET(s) => {
            let name = s.param_id.to_str().unwrap_or_default().to_owned();
            let Some(index) = st.params.keys().position(|k| *k == name) else { return Vec::new() };
            let limits = st.limits.get(&name).copied();
            let entry = st.params.get_mut(&name).unwrap();
            let mut value = s.param_value;
            if !matches!(entry.1, MavParamType::MAV_PARAM_TYPE_REAL32) {
                value = value.round();
            }
            if let Some((lo, hi)) = limits {
                value = value.clamp(lo, hi);
            }
            entry.0 = value;
            st.param_sets += 1;
            param_value(st, index).into_iter().collect()
        }
        MavMessage::LOG_REQUEST_LIST(_) => {
            let n = st.logs.len() as u16;
            if n == 0 {
                return vec![MavMessage::LOG_ENTRY(LOG_ENTRY_DATA { time_utc: 0, size: 0, id: 0, num_logs: 0, last_log_num: 0 })];
            }
            st.logs
                .iter()
                .enumerate()
                .map(|(i, log)| {
                    MavMessage::LOG_ENTRY(LOG_ENTRY_DATA {
                        time_utc: 1_762_000_000 + i as u32 * 3600,
                        size: log.len() as u32,
                        id: i as u16 + 1,
                        num_logs: n,
                        last_log_num: n,
                    })
                })
                .collect()
        }
        MavMessage::LOG_REQUEST_DATA(r) => {
            if let Some(log) = st.logs.get((r.id as usize).wrapping_sub(1)) {
                let ofs = r.ofs as usize;
                let end = (ofs as u64 + r.count as u64).min(log.len() as u64) as usize;
                *stream = Some(Stream { id: r.id, ofs, end });
            }
            Vec::new()
        }
        MavMessage::LOG_REQUEST_END(_) => {
            *stream = None;
            Vec::new()
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN => {
            // Like ArduPilot, refuse to reboot while armed.
            let result = if st.armed {
                MavResult::MAV_RESULT_FAILED
            } else {
                st.reboots += 1;
                MavResult::MAV_RESULT_ACCEPTED
            };
            vec![MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command: c.command, result })]
        }
        _ => Vec::new(),
    }
}
