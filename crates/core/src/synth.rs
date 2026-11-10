//! Writer for synthetic DataFlash logs, used by tests and by the CLI's
//! `sample` command so the tool can be tried without a real flight.

use std::collections::HashMap;
use std::f64::consts::TAU;

use crate::dataflash::{FieldKind, FMT_LEN, FMT_TYPE, HEAD1, HEAD2, HEADER_LEN};

/// A value to encode; numbers are converted to the field's storage type.
#[derive(Debug, Clone, Copy)]
pub enum V<'a> {
    I(i64),
    F(f64),
    S(&'a str),
    A([i16; 32]),
}

impl V<'_> {
    fn num(&self) -> f64 {
        match *self {
            V::I(i) => i as f64,
            V::F(f) => f,
            _ => panic!("expected a number, got {self:?}"),
        }
    }

    fn int(&self) -> i64 {
        match *self {
            V::I(i) => i,
            V::F(f) => f.round() as i64,
            _ => panic!("expected a number, got {self:?}"),
        }
    }
}

#[derive(Default)]
pub struct LogWriter {
    buf: Vec<u8>,
    next_type: u8,
    formats: HashMap<u8, String>,
}

impl LogWriter {
    pub fn new() -> Self {
        Self { next_type: 1, ..Default::default() }
    }

    /// Declares a message type and returns its id.
    pub fn define(&mut self, name: &str, format: &str, labels: &str) -> u8 {
        let length = HEADER_LEN
            + format
                .chars()
                .map(|c| FieldKind::from_char(c).expect("unsupported format char").prim.size())
                .sum::<usize>();
        self.define_raw(name, format, labels, length)
    }

    /// Declares a message type with an explicit length, even if inconsistent.
    pub fn define_raw(&mut self, name: &str, format: &str, labels: &str, length: usize) -> u8 {
        let t = self.next_type;
        self.next_type += 1;
        if self.next_type == FMT_TYPE {
            self.next_type += 1;
        }
        let start = self.buf.len();
        self.buf.extend_from_slice(&[HEAD1, HEAD2, FMT_TYPE, t, length as u8]);
        self.text(name, 4);
        self.text(format, 16);
        self.text(labels, 64);
        debug_assert_eq!(self.buf.len() - start, FMT_LEN);
        self.formats.insert(t, format.to_owned());
        t
    }

    pub fn write(&mut self, t: u8, values: &[V]) {
        let format = self.formats[&t].clone();
        assert_eq!(format.len(), values.len(), "value count for type {t}");
        self.buf.extend_from_slice(&[HEAD1, HEAD2, t]);
        for (c, v) in format.chars().zip(values) {
            match c {
                'b' => self.buf.push(v.int() as i8 as u8),
                'B' | 'M' => self.buf.push(v.int() as u8),
                'h' => self.put(&(v.int() as i16).to_le_bytes()),
                'H' => self.put(&(v.int() as u16).to_le_bytes()),
                'i' => self.put(&(v.int() as i32).to_le_bytes()),
                'I' => self.put(&(v.int() as u32).to_le_bytes()),
                'q' => self.put(&v.int().to_le_bytes()),
                'Q' => self.put(&(v.int() as u64).to_le_bytes()),
                'f' => self.put(&(v.num() as f32).to_le_bytes()),
                'd' => self.put(&v.num().to_le_bytes()),
                'c' => self.put(&((v.num() * 100.0).round() as i16).to_le_bytes()),
                'C' => self.put(&((v.num() * 100.0).round() as u16).to_le_bytes()),
                'e' => self.put(&((v.num() * 100.0).round() as i32).to_le_bytes()),
                'E' => self.put(&((v.num() * 100.0).round() as u32).to_le_bytes()),
                'L' => self.put(&((v.num() * 1e7).round() as i32).to_le_bytes()),
                'n' | 'N' | 'Z' => {
                    let V::S(s) = v else { panic!("expected text for '{c}'") };
                    self.text(s, FieldKind::from_char(c).unwrap().prim.size());
                }
                'a' => {
                    let V::A(a) = v else { panic!("expected array for 'a'") };
                    for x in a {
                        self.put(&x.to_le_bytes());
                    }
                }
                _ => panic!("writer does not support format char '{c}'"),
            }
        }
    }

    /// Writes a message with a raw payload, bypassing the format.
    pub fn write_raw(&mut self, t: u8, payload: &[u8]) {
        self.buf.extend_from_slice(&[HEAD1, HEAD2, t]);
        self.put(payload);
    }

    /// Appends arbitrary bytes, e.g. to simulate corruption.
    pub fn raw(&mut self, bytes: &[u8]) {
        self.put(bytes);
    }

    pub fn into_bytes(self) -> Vec<u8> {
        self.buf
    }

    fn put(&mut self, b: &[u8]) {
        self.buf.extend_from_slice(b);
    }

    fn text(&mut self, s: &str, n: usize) {
        let mut b = s.as_bytes().to_vec();
        b.resize(n, 0);
        self.put(&b);
    }
}

/// Which kind of sample flight to synthesise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    /// Logging set up for tuning, with a hover and stick snaps.
    Ready,
    /// Default logging and a plain hover: not enough for filter or PID tuning.
    DefaultLogging,
}

/// Copter's default `LOG_BITMASK`: no fast attitude, no PID logging.
pub const DEFAULT_LOG_BITMASK: u32 = 176_126;

const ARM_S: f64 = 2.0;
const HOVER_START_S: f64 = 7.0;
const SNAPS_START_S: f64 = 37.0;
const LAND_START_S: f64 = 57.0;
const DISARM_S: f64 = 62.0;
const END_S: f64 = 63.0;
const HOVER_THROTTLE: f64 = 0.35;
const HOVER_RPM: f64 = 9000.0;

fn us(t: f64) -> V<'static> {
    V::I((t * 1e6).round() as i64)
}

/// Steps through `[from, to)` at `hz`.
fn ticks(from: f64, to: f64, hz: f64) -> impl Iterator<Item = f64> {
    let n = ((to - from) * hz).floor() as usize;
    (0..n).map(move |i| from + i as f64 / hz)
}

fn throttle_at(t: f64) -> f64 {
    let wobble = 0.01 * (TAU * 0.7 * t).sin();
    if t < ARM_S {
        0.0
    } else if t < HOVER_START_S {
        HOVER_THROTTLE * (t - ARM_S) / (HOVER_START_S - ARM_S)
    } else if t < LAND_START_S {
        HOVER_THROTTLE + wobble
    } else if t < DISARM_S {
        HOVER_THROTTLE * (DISARM_S - t) / (DISARM_S - LAND_START_S)
    } else {
        0.0
    }
}

/// Stick-snap rate target (deg/s) for axis 0=roll, 1=pitch, 2=yaw:
/// 0.5 s pulses of alternating sign, one axis after another.
fn snap_target(t: f64, axis: usize) -> f64 {
    let window = (LAND_START_S - SNAPS_START_S) / 3.0;
    let start = SNAPS_START_S + axis as f64 * window;
    if t < start || t >= start + window {
        return 0.0;
    }
    let k = (t - start).floor();
    if t - start - k >= 0.5 {
        return 0.0;
    }
    let amplitude = if axis == 2 { 90.0 } else { 150.0 };
    if k as i64 % 2 == 0 {
        amplitude
    } else {
        -amplitude
    }
}

/// Builds a ~60 s quad flight: arm, climb, 30 s hover, (stick snaps), land.
pub fn sample_flight(preset: Preset) -> Vec<u8> {
    let ready = preset == Preset::Ready;
    let mut w = LogWriter::new();
    let parm = w.define("PARM", "QNff", "TimeUS,Name,Value,Default");
    let msg = w.define("MSG", "QZ", "TimeUS,Message");
    let ev = w.define("EV", "QB", "TimeUS,Id");
    let vibe = w.define("VIBE", "QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip");
    let ctun = w.define("CTUN", "Qfff", "TimeUS,ThI,ThO,ThH");
    let pid_labels = "TimeUS,Tar,Act,Err,P,I,D,FF,DFF,Dmod,SRate,Flags";
    let pids = ["PIDR", "PIDP", "PIDY"].map(|n| w.define(n, "QffffffffffB", pid_labels));
    let esc = w.define("ESC", "QBffff", "TimeUS,Instance,RPM,Volt,Curr,Temp");
    let gyr = w.define("GYR", "QBIfff", "TimeUS,I,SampleUS,GyrX,GyrY,GyrZ");
    let dsf = w.define("DSF", "QIH", "TimeUS,Dp,Blk");
    let rate = w.define("RATE", "Qffffffffffff", "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut");

    w.write(msg, &[us(0.5), V::S("ArduCopter V4.6.2 (a1b2c3d4)")]);
    w.write(msg, &[us(0.5), V::S("Frame: QUAD/X")]);

    let log_bitmask = if ready { DEFAULT_LOG_BITMASK | 1 | (1 << 12) } else { DEFAULT_LOG_BITMASK };
    let params: [(&str, f64); 8] = [
        ("LOG_BITMASK", log_bitmask as f64),
        ("INS_RAW_LOG_OPT", if ready { 9.0 } else { 0.0 }),
        ("INS_LOG_BAT_MASK", 0.0),
        ("INS_LOG_BAT_OPT", 0.0),
        ("INS_HNTCH_ENABLE", 0.0),
        ("INS_GYRO_FILTER", 20.0),
        ("SCHED_LOOP_RATE", 400.0),
        ("MOT_THST_HOVER", HOVER_THROTTLE),
    ];
    for (name, value) in params {
        w.write(parm, &[us(0.6), V::S(name), V::F(value), V::F(value)]);
    }

    w.write(ev, &[us(ARM_S), V::I(10)]);
    w.write(ev, &[us(DISARM_S), V::I(11)]);

    for t in ticks(0.0, END_S, 10.0) {
        let th = throttle_at(t);
        w.write(ctun, &[us(t), V::F(th), V::F(th), V::F(HOVER_THROTTLE)]);
        let flying = if th > 0.0 { 1.0 } else { 0.0 };
        let wob = (TAU * 0.3 * t).sin();
        w.write(
            vibe,
            &[us(t), V::I(0), V::F(flying * (12.0 + wob)), V::F(flying * (13.0 - wob)), V::F(flying * (20.0 + 2.0 * wob)), V::I(0)],
        );
        if ready {
            let rpm = if th > 0.05 { HOVER_RPM * (th / HOVER_THROTTLE).sqrt() } else { 0.0 };
            for m in 0..4 {
                w.write(esc, &[us(t), V::I(m), V::F(rpm), V::F(16.2), V::F(th * 30.0), V::F(35.0)]);
            }
        }
    }

    // Rate-loop PIDs: loop rate with ATTITUDE_FAST, 10 Hz otherwise. Each
    // axis responds as a second-order system (natural frequency 6 Hz); pitch
    // is the least damped, so its step response overshoots about 16%.
    let pid_hz = if ready { 400.0 } else { 10.0 };
    let (wn, zeta) = (TAU * 6.0, [0.7, 0.5, 0.8]);
    let mut act = [0.0f64; 3];
    let mut vel = [0.0f64; 3];
    for t in ticks(0.0, END_S, pid_hz) {
        let mut tars = [0.0; 3];
        for (axis, &id) in pids.iter().enumerate() {
            let tar = if ready { snap_target(t, axis) } else { 0.0 };
            tars[axis] = tar;
            // Substeps keep the integration stable at the 10 Hz rate too.
            for _ in 0..10 {
                let dt = 0.1 / pid_hz;
                vel[axis] += (wn * wn * (tar - act[axis]) - 2.0 * zeta[axis] * wn * vel[axis]) * dt;
                act[axis] += vel[axis] * dt;
            }
            let err = tar - act[axis];
            w.write(
                id,
                &[us(t), V::F(tar), V::F(act[axis]), V::F(err), V::F(0.002 * err), V::F(0.0), V::F(0.0), V::F(0.0), V::F(0.0), V::F(1.0), V::F(0.0), V::I(0)],
            );
        }
        if ready {
            let out = |a: usize| V::F(0.002 * (tars[a] - act[a]));
            w.write(
                rate,
                &[
                    us(t),
                    V::F(tars[0]),
                    V::F(act[0]),
                    out(0),
                    V::F(tars[1]),
                    V::F(act[1]),
                    out(1),
                    V::F(tars[2]),
                    V::F(act[2]),
                    out(2),
                    V::F(0.0),
                    V::F(0.0),
                    V::F(throttle_at(t)),
                ],
            );
        }
    }

    // Raw pre-filter gyro at 2 kHz: motor noise at the rotor frequency plus
    // its second harmonic, scaled with throttle.
    if ready {
        let mut phase = 0.0;
        for t in ticks(0.0, END_S, 2000.0) {
            let th = throttle_at(t);
            let f = HOVER_RPM / 60.0 * (th / HOVER_THROTTLE).max(0.0).sqrt();
            phase += TAU * f / 2000.0;
            let noise = if th > 0.05 { 0.08 * phase.sin() + 0.03 * (2.0 * phase).sin() } else { 0.0 };
            w.write(gyr, &[us(t), V::I(0), V::I((t * 1e6) as i64), V::F(noise), V::F(-noise), V::F(0.5 * noise)]);
        }
    }

    w.write(dsf, &[us(END_S), V::I(0), V::I(0)]);
    w.into_bytes()
}
