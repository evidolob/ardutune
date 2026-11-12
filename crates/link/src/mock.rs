//! A simulated ArduPilot vehicle speaking just enough MAVLink for tests and
//! demos: heartbeats, the parameter protocol, log listing/download/erase,
//! MAVLink FTP listing and deletion of log files, and reboot.
//! It can drop messages to exercise retry paths.

use std::collections::{BTreeMap, BTreeSet};
use std::io;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavParamType, MavResult, MavState, MavType, AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA,
    FILE_TRANSFER_PROTOCOL_DATA, GPS_RAW_INT_DATA, GpsFixType, HEARTBEAT_DATA, LOG_DATA_DATA, LOG_ENTRY_DATA, MavSeverity,
    PARAM_VALUE_DATA, RC_CHANNELS_DATA, STATUSTEXT_DATA, MAG_CAL_PROGRESS_DATA, MAG_CAL_REPORT_DATA, MagCalStatus,
};
use mavlink::{MavConnection, MavHeader};

use crate::ftp::{self, Entry, Packet};

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
    /// Log files, oldest first: `logs[i]` is file `first_log_num + i`.
    pub logs: Vec<Vec<u8>>,
    pub first_log_num: u32,
    /// Indices into `logs` whose files were deleted.
    pub deleted: BTreeSet<usize>,
    /// Drop one LOG_DATA chunk in every `n`, the first time it is sent.
    pub drop_log_chunk_every: Option<usize>,
    /// Parameter indices left out of the first full list.
    pub drop_param_indices: BTreeSet<u16>,
    pub reboots: usize,
    pub param_sets: usize,
    /// Reasons sent as "PreArm: …" when the pre-arm checks are run.
    pub prearm_failures: Vec<String>,
    /// GPS fix type, satellites and HDOP; `None` sends no GPS messages.
    pub gps: Option<(GpsFixType, u8, f32)>,
    /// What the GPS reports: ground speed (m/s), latitude, longitude.
    pub gps_motion: (f32, f64, f64),
    pub mode: u32,
    pub rc: [u16; 8],
    /// Default values; parameters not listed default to their starting value.
    pub defaults: BTreeMap<String, f32>,
    pub board_id: u32,
    pub board_name: String,
    /// Reboots that asked to stay in the bootloader.
    pub bootloader_reboots: usize,
    /// The chip ID sent in AUTOPILOT_VERSION.
    pub uid2: [u8; 18],
    /// Motor tests run: (test letter number, throttle %, seconds).
    pub motor_tests: Vec<(u8, f32, f32)>,
    /// When set, motor tests are refused with this "Motor Test: …" reason.
    pub motor_test_refusal: Option<String>,
    /// Like ArduCopter, the vehicle reports itself armed while a motor test runs.
    pub motor_test_until: Option<Instant>,
    /// When the running compass calibration started.
    pub mag_cal_started: Option<Instant>,
    /// How long a compass calibration takes.
    pub mag_cal_duration: Duration,
    /// How the calibration ends: success, or this failure.
    pub mag_cal_failure: Option<MagCalStatus>,
    pub mag_cal_cancels: usize,
}

impl MockState {
    /// Indices of the oldest and newest log files still present. Like
    /// ArduPilot, log ids count from the oldest file, so a deleted file in
    /// between leaves an empty entry.
    fn log_span(&self) -> Option<(usize, usize)> {
        let present = |i: &usize| !self.deleted.contains(i);
        let first = (0..self.logs.len()).find(present)?;
        let last = (0..self.logs.len()).rev().find(present)?;
        Some((first, last))
    }

    /// Index into `logs` of log `id`, if its file exists.
    fn log_index(&self, id: u16) -> Option<usize> {
        let (first, last) = self.log_span()?;
        let i = (first + id as usize).checked_sub(1)?;
        (id > 0 && i <= last && !self.deleted.contains(&i)).then_some(i)
    }

    fn log_dir(&self) -> Vec<Entry> {
        let mut entries = vec![Entry::Dir(".".into()), Entry::Dir("..".into()), Entry::File { name: "LASTLOG.TXT".into(), size: 3 }];
        entries.extend(
            self.logs
                .iter()
                .enumerate()
                .filter(|(i, _)| !self.deleted.contains(i))
                .map(|(i, log)| Entry::File { name: format!("{:08}.BIN", self.first_log_num as usize + i), size: log.len() as u64 }),
        );
        entries
    }

    /// A quad with Copter's default logging and a few hundred parameters.
    pub fn quad(logs: Vec<Vec<u8>>) -> Self {
        use MavParamType::*;
        let mut params = BTreeMap::new();
        let base: [(&str, f32, MavParamType); 23] = [
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
            ("ATC_RAT_RLL_I", 0.135, MAV_PARAM_TYPE_REAL32),
            ("ATC_RAT_RLL_D", 0.0036, MAV_PARAM_TYPE_REAL32),
            ("ATC_RAT_RLL_SMAX", 0.0, MAV_PARAM_TYPE_REAL32),
            ("ATC_RAT_PIT_SMAX", 0.0, MAV_PARAM_TYPE_REAL32),
            ("ATC_ANG_RLL_P", 4.5, MAV_PARAM_TYPE_REAL32),
            ("ATC_ACCEL_R_MAX", 110_000.0, MAV_PARAM_TYPE_REAL32),
            ("MOT_THST_HOVER", 0.35, MAV_PARAM_TYPE_REAL32),
            ("SCHED_LOOP_RATE", 400.0, MAV_PARAM_TYPE_INT16),
        ];
        for (n, v, t) in base {
            params.insert(n.to_owned(), (v, t));
        }
        // A BetaFlight X quad with DShot600, motors 1-4 on outputs 1-4.
        for (n, v) in [("FRAME_CLASS", 1.0), ("FRAME_TYPE", 18.0), ("MOT_PWM_TYPE", 6.0), ("SERVO_BLH_RVMASK", 0.0)] {
            params.insert(n.to_owned(), (v, MAV_PARAM_TYPE_INT8));
        }
        for a in ["X", "Y", "Z"] {
            params.insert(format!("COMPASS_OFS_{a}"), (0.0, MAV_PARAM_TYPE_REAL32));
        }
        // GPS, failsafes returning home, an arm switch on 5 and modes on 6.
        for (n, v) in [("GPS1_TYPE", 1.0), ("FS_THR_ENABLE", 1.0), ("BATT_FS_LOW_ACT", 2.0), ("FLTMODE_CH", 6.0)] {
            params.insert(n.to_owned(), (v, MAV_PARAM_TYPE_INT8));
        }
        for ch in 5..=16 {
            params.insert(format!("RC{ch}_OPTION"), (if ch == 5 { 153.0 } else { 0.0 }, MAV_PARAM_TYPE_INT16));
        }
        for o in 1..=4 {
            params.insert(format!("SERVO{o}_FUNCTION"), (32.0 + o as f32, MAV_PARAM_TYPE_INT16));
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
            first_log_num: 1,
            deleted: BTreeSet::new(),
            prearm_failures: Vec::new(),
            gps: Some((GpsFixType::GPS_FIX_TYPE_3D_FIX, 14, 0.8)),
            gps_motion: (0.02, 50.4501, 30.5234),
            mode: 0,
            rc: [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000],
            defaults: BTreeMap::new(),
            board_id: 1048,
            board_name: "KakuteH7-bdshot".to_owned(),
            bootloader_reboots: 0,
            uid2: [0x00, 0x3A, 0x00, 0x24, 0x32, 0x33, 0x51, 0x0A, 0x33, 0x35, 0x35, 0x33, 0, 0, 0, 0, 0, 0],
            motor_tests: Vec::new(),
            motor_test_refusal: None,
            motor_test_until: None,
            mag_cal_started: None,
            mag_cal_duration: Duration::from_millis(1500),
            mag_cal_failure: None,
            mag_cal_cancels: 0,
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

    /// The state, to change from another thread while a call is running.
    pub fn shared(&self) -> Arc<Mutex<MockState>> {
        self.state.clone()
    }

    pub fn state(&self) -> MutexGuard<'_, MockState> {
        self.state.lock().unwrap()
    }
}

struct Stream {
    id: u16,
    index: usize,
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
            let st = state.lock().unwrap().clone();
            let testing = st.motor_test_until.is_some_and(|t| Instant::now() < t);
            send(heartbeat(st.armed || testing, st.mode));
            if let Some((fix_type, satellites, hdop)) = st.gps {
                send(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                    fix_type,
                    satellites_visible: satellites,
                    eph: (hdop * 100.0) as u16,
                    epv: u16::MAX,
                    vel: (st.gps_motion.0 * 100.0) as u16,
                    cog: 700,
                    lat: (st.gps_motion.1 * 1e7) as i32,
                    lon: (st.gps_motion.2 * 1e7) as i32,
                    alt: 186_000,
                    ..Default::default()
                }));
            }
            let rc = st.rc;
            send(MavMessage::RC_CHANNELS(RC_CHANNELS_DATA {
                chancount: 8,
                chan1_raw: rc[0],
                chan2_raw: rc[1],
                chan3_raw: rc[2],
                chan4_raw: rc[3],
                chan5_raw: rc[4],
                chan6_raw: rc[5],
                chan7_raw: rc[6],
                chan8_raw: rc[7],
                ..Default::default()
            }));
            last_heartbeat = Instant::now();
            for msg in mag_cal(&mut state.lock().unwrap()) {
                send(msg);
            }
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
            let log = &st.logs[s.index];
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

/// Progress of a running compass calibration; once complete, the report
/// (sent repeatedly, like ArduPilot) and, on success, the saved offsets.
fn mag_cal(st: &mut MockState) -> Vec<MavMessage> {
    let Some(started) = st.mag_cal_started else { return Vec::new() };
    let frac = (started.elapsed().as_secs_f32() / st.mag_cal_duration.as_secs_f32()).min(1.0);
    if frac < 1.0 {
        let covered = (frac * 80.0) as usize;
        let mut completion_mask = [0u8; 10];
        for bit in 0..covered {
            completion_mask[bit / 8] |= 1 << (bit % 8);
        }
        return vec![MavMessage::MAG_CAL_PROGRESS(MAG_CAL_PROGRESS_DATA {
            compass_id: 0,
            cal_mask: 1,
            cal_status: MagCalStatus::MAG_CAL_RUNNING_STEP_ONE,
            attempt: 1,
            completion_pct: (frac * 100.0) as u8,
            completion_mask,
            ..Default::default()
        })];
    }
    let ofs = [-38.6, 117.8, -129.7];
    let ok = st.mag_cal_failure.is_none();
    if ok && st.params["COMPASS_OFS_X"].0 == 0.0 {
        for (a, v) in ["X", "Y", "Z"].iter().zip(ofs) {
            st.params.get_mut(&format!("COMPASS_OFS_{a}")).unwrap().0 = v;
        }
    }
    vec![MavMessage::MAG_CAL_REPORT(MAG_CAL_REPORT_DATA {
        compass_id: 0,
        cal_mask: 1,
        cal_status: st.mag_cal_failure.unwrap_or(MagCalStatus::MAG_CAL_SUCCESS),
        autosaved: u8::from(ok),
        fitness: if ok { 7.4 } else { 31.0 },
        ofs_x: ofs[0],
        ofs_y: ofs[1],
        ofs_z: ofs[2],
        ..Default::default()
    })]
}

fn ack(command: MavCmd, result: MavResult) -> MavMessage {
    MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command, result, ..Default::default() })
}

fn heartbeat(armed: bool, custom_mode: u32) -> MavMessage {
    let mut mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if armed {
        mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
    }
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode,
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
            let Some((first, last)) = st.log_span() else {
                return vec![MavMessage::LOG_ENTRY(LOG_ENTRY_DATA { time_utc: 0, size: 0, id: 0, num_logs: 0, last_log_num: 0 })];
            };
            let n = (last - first + 1) as u16;
            (first..=last)
                .map(|i| {
                    let size = if st.deleted.contains(&i) { 0 } else { st.logs[i].len() as u32 };
                    MavMessage::LOG_ENTRY(LOG_ENTRY_DATA {
                        time_utc: 1_762_000_000 + i as u32 * 3600,
                        size,
                        id: (i - first) as u16 + 1,
                        num_logs: n,
                        last_log_num: n,
                    })
                })
                .collect()
        }
        MavMessage::LOG_REQUEST_DATA(r) => {
            if let Some(index) = st.log_index(r.id) {
                let ofs = r.ofs as usize;
                let end = (ofs as u64 + r.count as u64).min(st.logs[index].len() as u64) as usize;
                *stream = Some(Stream { id: r.id, index, ofs, end });
            }
            Vec::new()
        }
        MavMessage::LOG_ERASE(_) => {
            *stream = None;
            st.logs.clear();
            st.deleted.clear();
            Vec::new()
        }
        MavMessage::FILE_TRANSFER_PROTOCOL(f) => {
            let req = Packet::decode(&f.payload);
            let reply = handle_ftp(st, &req);
            vec![MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
                target_network: 0,
                target_system: 255,
                target_component: 190,
                payload: reply.encode(),
            })]
        }
        MavMessage::LOG_REQUEST_END(_) => {
            *stream = None;
            Vec::new()
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_RUN_PREARM_CHECKS => {
            let mut out = vec![ack(c.command, MavResult::MAV_RESULT_ACCEPTED)];
            for reason in &st.prearm_failures {
                out.extend(status_texts(&format!("PreArm: {reason}")));
            }
            out
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_REQUEST_MESSAGE && c.param1 == 148.0 => {
            let mut hash = [0u8; 8];
            hash.copy_from_slice(b"d0b31066");
            vec![
                ack(c.command, MavResult::MAV_RESULT_ACCEPTED),
                MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
                    flight_sw_version: 0x0407_0000,
                    board_version: st.board_id << 16,
                    uid2: st.uid2,
                    flight_custom_version: hash,
                    ..Default::default()
                }),
            ]
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_DO_SEND_BANNER => {
            let mut out = vec![ack(c.command, MavResult::MAV_RESULT_ACCEPTED)];
            for line in ["ArduCopter V4.7.0-dev (d0b31066)", "ChibiOS: 1fa4b8e0", &format!("{} 24003A00 0A513332 33353533", st.board_name)] {
                out.extend(status_texts(line));
            }
            out
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
            vec![ack(c.command, MavResult::MAV_RESULT_ACCEPTED)]
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_DO_START_MAG_CAL => {
            let result = if st.armed {
                MavResult::MAV_RESULT_FAILED
            } else {
                st.mag_cal_started = Some(Instant::now());
                MavResult::MAV_RESULT_ACCEPTED
            };
            vec![ack(c.command, result)]
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_DO_CANCEL_MAG_CAL => {
            st.mag_cal_started = None;
            st.mag_cal_cancels += 1;
            vec![ack(c.command, MavResult::MAV_RESULT_ACCEPTED)]
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_DO_MOTOR_TEST => {
            // Like ArduCopter, explain a refusal before acknowledging it.
            let refusal = if st.armed { Some("vehicle is armed".to_owned()) } else { st.motor_test_refusal.clone() };
            let mut out = Vec::new();
            let result = match refusal {
                Some(why) => {
                    out.extend(status_texts(&format!("Motor Test: {why}")));
                    MavResult::MAV_RESULT_FAILED
                }
                None => {
                    st.motor_tests.push((c.param1 as u8, c.param3, c.param4));
                    st.motor_test_until = Some(Instant::now() + Duration::from_secs_f32(c.param4));
                    MavResult::MAV_RESULT_ACCEPTED
                }
            };
            out.push(ack(c.command, result));
            out
        }
        MavMessage::COMMAND_LONG(c) if c.command == MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN => {
            // Like ArduPilot, refuse to reboot while armed.
            let result = if st.armed {
                MavResult::MAV_RESULT_FAILED
            } else {
                st.reboots += 1;
                if c.param1 == 3.0 {
                    st.bootloader_reboots += 1;
                }
                MavResult::MAV_RESULT_ACCEPTED
            };
            vec![ack(c.command, result)]
        }
        _ => Vec::new(),
    }
}

/// The packed parameter file; defaults are the values the quad started with.
fn param_pck(st: &MockState) -> Vec<u8> {
    let params: Vec<(String, f32, f32, u8)> = st
        .params
        .iter()
        .map(|(name, (value, kind))| {
            let default = st.defaults.get(name).copied().unwrap_or(*value);
            let code = match kind {
                MavParamType::MAV_PARAM_TYPE_INT8 | MavParamType::MAV_PARAM_TYPE_UINT8 => 1,
                MavParamType::MAV_PARAM_TYPE_INT16 | MavParamType::MAV_PARAM_TYPE_UINT16 => 2,
                MavParamType::MAV_PARAM_TYPE_INT32 | MavParamType::MAV_PARAM_TYPE_UINT32 => 3,
                _ => 4,
            };
            (name.clone(), *value, default, code)
        })
        .collect();
    ftp::encode_param_pck(&params)
}

/// A status text, split into 50-character pieces like ArduPilot does.
fn status_texts(text: &str) -> Vec<MavMessage> {
    text.as_bytes()
        .chunks(50)
        .map(|chunk| {
            MavMessage::STATUSTEXT(STATUSTEXT_DATA {
                severity: MavSeverity::MAV_SEVERITY_CRITICAL,
                text: String::from_utf8_lossy(chunk).as_ref().into(),
                ..Default::default()
            })
        })
        .collect()
}

/// FTP over a single directory, `/APM/LOGS`, like a ChibiOS board's SD card.
fn handle_ftp(st: &mut MockState, req: &Packet) -> Packet {
    const DIR: &str = "/APM/LOGS";
    let nak = |code: u8| Packet::reply(req, ftp::NAK, &[code]);
    let path = String::from_utf8_lossy(&req.data).trim_end_matches('\0').to_owned();
    match req.opcode {
        ftp::LIST_DIRECTORY if path == DIR => {
            let entries = st.log_dir();
            let rest = entries.iter().skip(req.offset as usize);
            if req.offset as usize >= entries.len() {
                return nak(ftp::ERR_EOF);
            }
            Packet::reply(req, ftp::ACK, &ftp::encode_entries(rest))
        }
        ftp::LIST_DIRECTORY => nak(ftp::ERR_FILE_NOT_FOUND),
        ftp::RESET_SESSIONS | ftp::TERMINATE_SESSION => Packet::reply(req, ftp::ACK, &[]),
        ftp::OPEN_FILE_RO if path.starts_with("@PARAM/param.pck") => {
            let file = param_pck(st);
            let mut reply = Packet::reply(req, ftp::ACK, &(file.len() as u32).to_le_bytes());
            reply.session = 1;
            reply
        }
        ftp::OPEN_FILE_RO => nak(ftp::ERR_FILE_NOT_FOUND),
        ftp::READ_FILE => {
            let file = param_pck(st);
            let ofs = req.offset as usize;
            if ofs >= file.len() {
                return nak(ftp::ERR_EOF);
            }
            let end = (ofs + ftp::DATA_MAX).min(file.len());
            let mut reply = Packet::reply(req, ftp::ACK, &file[ofs..end]);
            reply.offset = req.offset;
            reply
        }
        ftp::REMOVE_FILE => {
            let index = path
                .strip_prefix(DIR)
                .and_then(|p| p.strip_prefix('/'))
                .and_then(ftp::log_number)
                .and_then(|n| n.checked_sub(st.first_log_num))
                .map(|i| i as usize)
                .filter(|i| *i < st.logs.len() && !st.deleted.contains(i));
            match index {
                Some(i) => {
                    st.deleted.insert(i);
                    Packet::reply(req, ftp::ACK, &[])
                }
                None => nak(ftp::ERR_FILE_NOT_FOUND),
            }
        }
        _ => nak(ftp::ERR_FAIL),
    }
}
