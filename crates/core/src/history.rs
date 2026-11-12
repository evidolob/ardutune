//! Which flight controller a log came from, and a small summary of each
//! flight for comparing flights of one vehicle over time.

use serde::{Deserialize, Serialize};

use crate::log::Log;
use crate::{noise, pid, readiness};

/// A flight controller as its start-up banner names it: ChibiOS boards log
/// "<board> <chip ID in hex>", the ID in one block or in 4-byte groups
/// ("KakuteH7-bdshot 004D002D 33335101 38333837").
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoardId {
    pub board: String,
    /// The chip ID as the banner shows it, without spaces.
    pub serial: String,
}

pub fn parse_board_line(text: &str) -> Option<BoardId> {
    let words: Vec<&str> = text.split_whitespace().collect();
    let (board, id) = words.split_first()?;
    let digits: usize = id.iter().map(|w| w.len()).sum();
    let hex = !id.is_empty() && id.iter().all(|w| w.chars().all(|c| c.is_ascii_hexdigit()));
    (hex && digits >= 16 && !board.starts_with("Ardu")).then(|| BoardId { board: (*board).to_owned(), serial: id.concat().to_ascii_uppercase() })
}

/// The banner form of a chip ID read from AUTOPILOT_VERSION (`uid2`, the raw
/// bytes): ChibiOS prints each 4-byte word most significant byte first,
/// i.e. every group of four bytes reversed.
pub fn serial_from_uid(uid: &str) -> Option<String> {
    if uid.len() != 24 || !uid.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    let bytes: Vec<&str> = (0..12).map(|i| &uid[i * 2..i * 2 + 2]).collect();
    Some(bytes.chunks(4).flat_map(|w| w.iter().rev().copied()).collect::<String>().to_ascii_uppercase())
}

/// One flight's numbers that matter for tuning progress.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FlightSummary {
    pub board: Option<BoardId>,
    pub firmware: Option<String>,
    /// Unix time the log started, from GPS time; `None` without a GPS fix.
    pub started: Option<i64>,
    pub flight_s: f64,
    /// Worst axis' 95th percentile during flight, m/s².
    pub vibration: Option<f64>,
    /// Gyro noise above 40 Hz before and after the filters, deg/s RMS.
    pub noise_pre: Option<f64>,
    pub noise_post: Option<f64>,
    /// Motor noise frequency in hover.
    pub hover_hz: Option<f64>,
    pub axes: Vec<AxisSummary>,
    pub autotune: bool,
    /// Tuning parameters at the time (filters, rate and angle loops,
    /// acceleration limits, thrust), to show what changed between flights.
    pub params: Vec<(String, f64)>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct AxisSummary {
    pub axis: String,
    pub overshoot_pct: Option<f64>,
    pub rise_ms: Option<f64>,
    /// Frequency of an oscillation found in the rate loop.
    pub oscillation_hz: Option<f64>,
}

/// Parameters worth comparing between flights.
fn tuning_param(name: &str) -> bool {
    const PREFIXES: [&str; 8] = ["ATC_", "INS_GYRO_FILTER", "INS_HNTCH_", "INS_HNTC2_", "MOT_THST_", "MOT_SPIN_", "PSC_", "SCHED_LOOP_RATE"];
    PREFIXES.iter().any(|p| name.starts_with(p))
}

const GPS_EPOCH: i64 = 315_964_800; // 1980-01-06
const LEAP_SECONDS: i64 = 18;

/// When the log started: GPS week and milliseconds at the first good fix,
/// moved back to the start of the log.
fn started(log: &Log) -> Option<i64> {
    let gps = log.table("GPS")?;
    let (t, week, ms) = (gps.time_s()?, gps.num("GWk")?, gps.num("GMS")?);
    let status = gps.num("Status");
    let i = (0..t.len()).find(|&i| week[i] > 0.0 && status.is_none_or(|s| s[i] >= 3.0))?;
    let at_fix = GPS_EPOCH + week[i] as i64 * 604_800 + (ms[i] / 1000.0) as i64 - LEAP_SECONDS;
    let start = log.time_span().map_or(t[i], |s| s.start);
    Some(at_fix - (t[i] - start).round() as i64)
}

pub fn summarize(log: &Log) -> FlightSummary {
    let (flight, _) = log.flight();
    let noise = noise::analyse(log).ok();
    let pid = pid::analyse(log).ok();
    FlightSummary {
        board: log.text_messages().into_iter().find_map(parse_board_line),
        firmware: log.firmware().map(|f| f.text),
        started: started(log),
        flight_s: crate::log::total_duration(&flight),
        vibration: readiness::vibration_level(log),
        noise_pre: noise.as_ref().map(|n| n.noise_rms_pre),
        noise_post: noise.as_ref().and_then(|n| n.noise_rms_post),
        hover_hz: noise.as_ref().and_then(|n| n.hover_motor_hz),
        axes: pid
            .as_ref()
            .map(|p| {
                p.axes
                    .iter()
                    .map(|a| AxisSummary {
                        axis: a.axis.to_owned(),
                        overshoot_pct: a.step.as_ref().map(|s| s.overshoot_pct),
                        rise_ms: a.step.as_ref().and_then(|s| s.rise_ms),
                        oscillation_hz: a.oscillation.as_ref().map(|o| o.freq_hz),
                    })
                    .collect()
            })
            .unwrap_or_default(),
        autotune: pid.as_ref().is_some_and(|p| p.axes.iter().any(|a| a.includes_autotune)),
        params: log.params().iter().filter(|(n, _)| tuning_param(n)).map(|(n, v)| (n.clone(), *v)).collect(),
    }
}

/// Tuning parameters that differ between two flights: (name, before, after).
pub fn changes(before: &FlightSummary, after: &FlightSummary) -> Vec<(String, Option<f64>, Option<f64>)> {
    let get = |s: &FlightSummary, n: &str| s.params.iter().find(|(k, _)| k == n).map(|(_, v)| *v);
    let mut names: Vec<&String> = before.params.iter().chain(&after.params).map(|(n, _)| n).collect();
    names.sort();
    names.dedup();
    names
        .into_iter()
        .filter_map(|n| {
            let (b, a) = (get(before, n), get(after, n));
            let same = match (b, a) {
                (Some(x), Some(y)) => (x - y).abs() <= 1e-6 * x.abs().max(y.abs()).max(1.0),
                (None, None) => true,
                _ => false,
            };
            (!same).then(|| (n.clone(), b, a))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_lines() {
        let real = parse_board_line("KakuteH7-bdshot 004D002D 33335101 38333837").unwrap();
        assert_eq!((real.board.as_str(), real.serial.as_str()), ("KakuteH7-bdshot", "004D002D3333510138333837"));
        assert_eq!(parse_board_line("MatekH7A3 0035003D 4B564E0C 20204232").unwrap().serial, "0035003D4B564E0C20204232");
        assert!(parse_board_line("ArduCopter V4.7.2 (fb773567)").is_none());
        assert!(parse_board_line("ChibiOS: c6d0293e").is_none());
        assert!(parse_board_line("RCOut: DS600:1-4").is_none());
    }

    #[test]
    fn banner_form_of_a_raw_chip_id() {
        // ChibiOS memcpy's the ID bytes into uid2 and prints each word
        // most significant byte first.
        assert_eq!(serial_from_uid("2D004D000151333337383338").as_deref(), Some("004D002D3333510138333837"));
        assert_eq!(serial_from_uid("1234"), None);
    }

    #[test]
    fn summarizes_the_sample_flight() {
        let log = Log::from_bytes(&crate::synth::sample_flight(crate::synth::Preset::Ready));
        let s = summarize(&log);
        assert!(s.flight_s > 10.0, "{s:?}");
        assert!(s.noise_pre.is_some() && s.vibration.is_some());
        assert_eq!(s.axes.len(), 3);
        assert!(s.params.iter().any(|(n, _)| n == "INS_GYRO_FILTER"));
        let mut later = s.clone();
        later.params.retain(|(n, _)| n != "INS_GYRO_FILTER");
        later.params.push(("INS_GYRO_FILTER".into(), 45.0));
        let c = changes(&s, &later);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].2, Some(45.0));
    }
}

#[cfg(test)]
mod real {
    /// `ARDUTUNE_LOG=path cargo test -p ardutune-core real_log -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn real_log() {
        let path = std::env::var("ARDUTUNE_LOG").expect("ARDUTUNE_LOG");
        let started = std::time::Instant::now();
        let s = super::summarize(&crate::log::Log::open(path).unwrap());
        println!("{:#?}\n{} params, {:?}", super::FlightSummary { params: vec![], ..s.clone() }, s.params.len(), started.elapsed());
    }
}
