//! A parsed log: one columnar [`Table`] per message type, plus helpers for
//! the things every analysis needs (parameters, firmware, armed time).

use std::collections::BTreeMap;
use std::path::Path;

use serde::Serialize;

use crate::dataflash::{FieldKind, Prim};

#[derive(Debug, Clone)]
pub enum Column {
    /// Every numeric field, already scaled to real units.
    Num(Vec<f64>),
    Text(Vec<String>),
    I16x32(Vec<[i16; 32]>),
}

impl Column {
    pub fn len(&self) -> usize {
        match self {
            Column::Num(v) => v.len(),
            Column::Text(v) => v.len(),
            Column::I16x32(v) => v.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// All messages of one type, stored column by column.
#[derive(Debug, Clone)]
pub struct Table {
    pub name: String,
    pub labels: Vec<String>,
    pub columns: Vec<Column>,
    pub(crate) kinds: Vec<FieldKind>,
}

impl Table {
    pub(crate) fn new(name: &str, labels: &[String], kinds: &[FieldKind]) -> Self {
        let columns = kinds
            .iter()
            .map(|k| match k.prim {
                Prim::Text(_) => Column::Text(Vec::new()),
                Prim::I16x32 => Column::I16x32(Vec::new()),
                _ => Column::Num(Vec::new()),
            })
            .collect();
        Self { name: name.to_owned(), labels: labels.to_vec(), columns, kinds: kinds.to_vec() }
    }

    pub fn len(&self) -> usize {
        self.columns.first().map_or(0, Column::len)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn column(&self, field: &str) -> Option<&Column> {
        let i = self.labels.iter().position(|l| l == field)?;
        self.columns.get(i)
    }

    pub fn num(&self, field: &str) -> Option<&[f64]> {
        match self.column(field)? {
            Column::Num(v) => Some(v),
            _ => None,
        }
    }

    /// The first of `fields` that exists; field names vary across firmware versions.
    pub fn num_any(&self, fields: &[&str]) -> Option<&[f64]> {
        fields.iter().find_map(|f| self.num(f))
    }

    pub fn text(&self, field: &str) -> Option<&[String]> {
        match self.column(field)? {
            Column::Text(v) => Some(v),
            _ => None,
        }
    }

    pub fn arrays(&self, field: &str) -> Option<&[[i16; 32]]> {
        match self.column(field)? {
            Column::I16x32(v) => Some(v),
            _ => None,
        }
    }

    /// Removes rows whose timestamp is impossible: beyond any plausible time
    /// since boot, or a lone jump away from two agreeing neighbours. Such rows
    /// are corrupt messages that happened to be framed correctly.
    fn drop_timestamp_spikes(&mut self) -> usize {
        const MAX_TIME_S: f64 = 1e7;
        const SPIKE_S: f64 = 5.0;
        let Some(t) = self.time_s() else { return 0 };
        let n = t.len();
        let keep: Vec<bool> = (0..n)
            .map(|i| {
                if t[i] > MAX_TIME_S {
                    return false;
                }
                if i == 0 || i + 1 >= n {
                    return true;
                }
                let (p, x, q) = (t[i - 1], t[i], t[i + 1]);
                !((x - p).abs() > SPIKE_S && (x - q).abs() > SPIKE_S && (q - p).abs() <= SPIKE_S)
            })
            .collect();
        let dropped = keep.iter().filter(|k| !**k).count();
        if dropped > 0 {
            fn retain<T>(v: &mut Vec<T>, keep: &[bool]) {
                let mut k = keep.iter();
                v.retain(|_| *k.next().unwrap());
            }
            for c in &mut self.columns {
                match c {
                    Column::Num(v) => retain(v, &keep),
                    Column::Text(v) => retain(v, &keep),
                    Column::I16x32(v) => retain(v, &keep),
                }
            }
        }
        dropped
    }

    /// Timestamps in seconds, from `TimeUS` or (older logs) `TimeMS`.
    pub fn time_s(&self) -> Option<Vec<f64>> {
        if let Some(t) = self.num("TimeUS") {
            return Some(t.iter().map(|v| v * 1e-6).collect());
        }
        self.num("TimeMS").map(|t| t.iter().map(|v| v * 1e-3).collect())
    }
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct ParseStats {
    pub bytes: usize,
    pub messages: usize,
    /// Bytes skipped while searching for a valid message header.
    pub skipped_bytes: usize,
    /// Messages whose format could not be decoded.
    pub undecodable: usize,
    /// The file ended in the middle of a message.
    pub truncated: bool,
    /// Messages dropped because their timestamp was impossible (corrupt data).
    pub bad_timestamps: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum Vehicle {
    Copter,
    Plane,
    Rover,
    Sub,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Firmware {
    pub vehicle: Vehicle,
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
    /// The firmware banner as logged, e.g. `ArduCopter V4.6.2 (a1b2c3d4)`.
    pub text: String,
}

impl Firmware {
    pub fn at_least(&self, major: u32, minor: u32) -> bool {
        (self.major, self.minor) >= (major, minor)
    }

    /// Parses a banner such as `ArduCopter V4.5.7 (2a3dc4b7)`.
    pub fn parse_banner(s: &str) -> Option<Self> {
        const VEHICLES: [(&str, Vehicle); 5] = [
            ("ArduCopter", Vehicle::Copter),
            ("ArduPlane", Vehicle::Plane),
            ("ArduRover", Vehicle::Rover),
            ("ArduSub", Vehicle::Sub),
            ("Rover", Vehicle::Rover),
        ];
        let (vehicle, rest) = VEHICLES.iter().find_map(|(p, v)| s.find(p).map(|i| (v.clone(), &s[i + p.len()..])))?;
        let rest = rest.trim_start().strip_prefix('V')?;
        let ver: String = rest.chars().take_while(|c| c.is_ascii_digit() || *c == '.').collect();
        let mut parts = ver.split('.').map(|p| p.parse::<u32>().ok());
        let major = parts.next()??;
        let minor = parts.next().flatten().unwrap_or(0);
        let patch = parts.next().flatten().unwrap_or(0);
        Some(Self { vehicle, major, minor, patch, text: s.trim().to_owned() })
    }
}

/// Where [`Log::flight`] got the flight periods from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FlightSource {
    /// ARM messages or armed/disarmed events.
    Arming,
    /// Arming wasn't logged; periods with the motors spinning.
    MotorOutput,
    /// Nothing to go on; the whole log.
    WholeLog,
}

/// A time span in seconds since boot.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Interval {
    pub start: f64,
    pub end: f64,
}

impl Interval {
    pub fn duration(&self) -> f64 {
        (self.end - self.start).max(0.0)
    }

    pub fn contains(&self, t: f64) -> bool {
        t >= self.start && t <= self.end
    }
}

pub fn total_duration(intervals: &[Interval]) -> f64 {
    intervals.iter().map(Interval::duration).sum()
}

pub fn in_any(intervals: &[Interval], t: f64) -> bool {
    intervals.iter().any(|i| i.contains(t))
}

#[derive(Debug, Clone)]
pub struct Log {
    tables: BTreeMap<String, Table>,
    params: BTreeMap<String, f64>,
    pub stats: ParseStats,
}

/// Motor-running periods closer than this are merged.
const MERGE_GAP_S: f64 = 3.0;
const MIN_LEN_S: f64 = 2.0;

// ARM/EV ids used by every Copter version since 3.x.
const EV_ARMED: f64 = 10.0;
const EV_DISARMED: f64 = 11.0;

impl Log {
    pub(crate) fn new(mut tables: BTreeMap<String, Table>, mut stats: ParseStats) -> Self {
        for t in tables.values_mut() {
            stats.bad_timestamps += t.drop_timestamp_spikes();
        }
        let mut params = BTreeMap::new();
        if let Some(parm) = tables.get("PARM") {
            if let (Some(names), Some(values)) = (parm.text("Name"), parm.num("Value")) {
                // Later values win: parameters can change during a flight.
                for (n, v) in names.iter().zip(values) {
                    params.insert(n.clone(), *v);
                }
            }
        }
        Self { tables, params, stats }
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        crate::dataflash::parse(bytes)
    }

    pub fn open(path: impl AsRef<Path>) -> std::io::Result<Self> {
        Ok(Self::from_bytes(&std::fs::read(path)?))
    }

    pub fn tables(&self) -> impl Iterator<Item = &Table> {
        self.tables.values()
    }

    pub fn table(&self, name: &str) -> Option<&Table> {
        self.tables.get(name).filter(|t| !t.is_empty())
    }

    pub fn params(&self) -> &BTreeMap<String, f64> {
        &self.params
    }

    pub fn param(&self, name: &str) -> Option<f64> {
        self.params.get(name).copied()
    }

    /// Text from MSG messages, in log order.
    pub fn text_messages(&self) -> Vec<&str> {
        self.table("MSG").and_then(|t| t.text("Message")).map(|m| m.iter().map(String::as_str).collect()).unwrap_or_default()
    }

    pub fn firmware(&self) -> Option<Firmware> {
        if let Some(fw) = self.text_messages().into_iter().find_map(Firmware::parse_banner) {
            return Some(fw);
        }
        let ver = self.table("VER")?;
        let get = |f: &str| ver.num(f).and_then(|v| v.first()).map(|v| *v as u32);
        Some(Firmware { vehicle: Vehicle::Unknown, major: get("Maj")?, minor: get("Min").unwrap_or(0), patch: get("Pat").unwrap_or(0), text: String::new() })
    }

    /// First and last timestamp across all messages.
    pub fn time_span(&self) -> Option<Interval> {
        let mut span: Option<Interval> = None;
        for t in self.tables.values() {
            let Some(ts) = t.time_s() else { continue };
            for &v in ts.iter().filter(|v| **v > 0.0) {
                let s = span.get_or_insert(Interval { start: v, end: v });
                s.start = s.start.min(v);
                s.end = s.end.max(v);
            }
        }
        span
    }

    /// The periods to analyse as flight: armed periods when the log records
    /// arming, otherwise periods with motor output, otherwise the whole log.
    /// Logs often miss arming: logging starts at arming and the end of the
    /// file can be lost.
    pub fn flight(&self) -> (Vec<Interval>, FlightSource) {
        let armed = self.armed_intervals();
        if !armed.is_empty() {
            return (armed, FlightSource::Arming);
        }
        let motors = self.motor_running_intervals();
        if !motors.is_empty() {
            return (motors, FlightSource::MotorOutput);
        }
        (self.time_span().into_iter().collect(), FlightSource::WholeLog)
    }

    /// Periods with the motors spinning, from ESC RPM telemetry or, without
    /// it, throttle output (CTUN.ThO). Short dips are merged.
    fn motor_running_intervals(&self) -> Vec<Interval> {
        let series = |table: &str, field: &str| {
            let t = self.table(table)?;
            Some((t.time_s()?, t.num(field)?.to_vec()))
        };
        let Some((t, v)) = series("ESC", "RPM").or_else(|| series("CTUN", "ThO")) else { return Vec::new() };
        let mut samples: Vec<(f64, f64)> = t.into_iter().zip(v).collect();
        samples.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut out: Vec<Interval> = Vec::new();
        for (t, v) in samples {
            if v <= 0.0 {
                continue;
            }
            match out.last_mut() {
                Some(last) if t - last.end <= MERGE_GAP_S => last.end = t,
                _ => out.push(Interval { start: t, end: t }),
            }
        }
        out.retain(|i| i.duration() >= MIN_LEN_S);
        out
    }

    /// Armed periods, from ARM messages (4.1+) or EV armed/disarmed events.
    /// Empty when the log records neither.
    pub fn armed_intervals(&self) -> Vec<Interval> {
        let mut edges: Vec<(f64, bool)> = Vec::new();
        if let Some(arm) = self.table("ARM") {
            if let (Some(t), Some(state)) = (arm.time_s(), arm.num("ArmState")) {
                edges = t.into_iter().zip(state.iter().map(|s| *s != 0.0)).collect();
            }
        }
        if edges.is_empty() {
            if let Some(ev) = self.table("EV") {
                if let (Some(t), Some(id)) = (ev.time_s(), ev.num("Id")) {
                    edges = t.into_iter().zip(id).filter(|(_, id)| **id == EV_ARMED || **id == EV_DISARMED).map(|(t, id)| (t, *id == EV_ARMED)).collect();
                }
            }
        }
        edges.sort_by(|a, b| a.0.total_cmp(&b.0));

        let mut out = Vec::new();
        let mut armed_at: Option<f64> = None;
        for (t, armed) in edges {
            match (armed, armed_at) {
                (true, None) => armed_at = Some(t),
                (false, Some(start)) => {
                    out.push(Interval { start, end: t });
                    armed_at = None;
                }
                _ => {}
            }
        }
        if let (Some(start), Some(span)) = (armed_at, self.time_span()) {
            out.push(Interval { start, end: span.end });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{LogWriter, V};

    #[test]
    fn parses_firmware_banners() {
        let fw = Firmware::parse_banner("ArduCopter V4.5.7 (2a3dc4b7)").unwrap();
        assert_eq!((fw.vehicle.clone(), fw.major, fw.minor, fw.patch), (Vehicle::Copter, 4, 5, 7));
        assert!(fw.at_least(4, 5) && !fw.at_least(4, 6));
        let fw = Firmware::parse_banner("ArduCopter V4.6.0-beta2 (abc)").unwrap();
        assert_eq!((fw.major, fw.minor, fw.patch), (4, 6, 0));
        assert_eq!(Firmware::parse_banner("ArduRover V4.2.3").unwrap().vehicle, Vehicle::Rover);
        assert!(Firmware::parse_banner("Frame: QUAD/X").is_none());
    }

    #[test]
    fn last_parameter_value_wins() {
        let mut w = LogWriter::new();
        let parm = w.define("PARM", "QNff", "TimeUS,Name,Value,Default");
        w.write(parm, &[V::I(1), V::S("ATC_RAT_RLL_P"), V::F(0.135), V::F(0.135)]);
        w.write(parm, &[V::I(2), V::S("ATC_RAT_RLL_P"), V::F(0.1), V::F(0.135)]);
        let log = Log::from_bytes(&w.into_bytes());
        assert!((log.param("ATC_RAT_RLL_P").unwrap() - 0.1).abs() < 1e-6);
        assert_eq!(log.param("MISSING"), None);
    }

    #[test]
    fn armed_intervals_from_events() {
        let mut w = LogWriter::new();
        let ev = w.define("EV", "QB", "TimeUS,Id");
        for (t, id) in [(1_000_000, 10), (5_000_000, 11), (8_000_000, 10)] {
            w.write(ev, &[V::I(t), V::I(id)]);
        }
        let other = w.define("X", "Qf", "TimeUS,V");
        w.write(other, &[V::I(10_000_000), V::F(0.0)]);
        let log = Log::from_bytes(&w.into_bytes());
        let a = log.armed_intervals();
        assert_eq!(a, vec![Interval { start: 1.0, end: 5.0 }, Interval { start: 8.0, end: 10.0 }]);
    }

    #[test]
    fn drops_corrupt_timestamps() {
        let mut w = LogWriter::new();
        let t = w.define("X", "Qf", "TimeUS,V");
        for (us, v) in [(1_000_000, 1.0), (1_002_500, 2.0), (2_590_106_135_395, 9.0), (1_007_500, 3.0), (20_000_000_000_000, 9.0)] {
            w.write(t, &[V::I(us), V::F(v)]);
        }
        let log = Log::from_bytes(&w.into_bytes());
        assert_eq!(log.table("X").unwrap().num("V").unwrap(), &[1.0, 2.0, 3.0]);
        assert_eq!(log.stats.bad_timestamps, 2);
    }

    #[test]
    fn flight_from_motor_output_when_arming_missing() {
        let mut w = LogWriter::new();
        let ctun = w.define("CTUN", "Qf", "TimeUS,ThO");
        for i in 0..400 {
            let t = i as f64 * 0.1;
            // Throttle 5-20 s and 30-35 s, with a 1 s dip at 10 s.
            let th = if (5.0..20.0).contains(&t) && !(10.0..11.0).contains(&t) || (30.0..35.0).contains(&t) { 0.3 } else { 0.0 };
            w.write(ctun, &[V::I((t * 1e6) as i64), V::F(th)]);
        }
        let log = Log::from_bytes(&w.into_bytes());
        let (flight, source) = log.flight();
        assert_eq!(source, FlightSource::MotorOutput);
        assert_eq!(flight.len(), 2);
        assert!((flight[0].start - 5.0).abs() < 1e-6 && (flight[0].end - 19.9).abs() < 1e-6);
        assert!((flight[1].start - 30.0).abs() < 1e-6);
    }

    #[test]
    fn armed_intervals_prefer_arm_messages() {
        let mut w = LogWriter::new();
        let arm = w.define("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method");
        w.write(arm, &[V::I(2_000_000), V::I(1), V::I(0), V::I(0), V::I(0)]);
        w.write(arm, &[V::I(3_000_000), V::I(0), V::I(0), V::I(0), V::I(0)]);
        let ev = w.define("EV", "QB", "TimeUS,Id");
        w.write(ev, &[V::I(1_000_000), V::I(10)]);
        let log = Log::from_bytes(&w.into_bytes());
        assert_eq!(log.armed_intervals(), vec![Interval { start: 2.0, end: 3.0 }]);
    }
}
