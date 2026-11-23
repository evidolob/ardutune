//! Two logging presets: tuning logs (everything ArduTune's analyses and an
//! AutoTune check need, with high-rate gyro data) and normal logs
//! (ArduPilot's default set, small enough for everyday flying).

use serde::Serialize;

use crate::readiness::{recommend_params, ParamChange, REQUIRED_LOG_BITS};
use crate::setup::{change, gyro_logging, Param};

/// ArduCopter's default LOG_BITMASK (the same since 4.0).
pub const COPTER_DEFAULT_LOG_BITMASK: f64 = 176_126.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// Has everything the noise, filter and PID analyses need.
    Tuning,
    /// No high-rate gyro logging.
    Normal,
    /// High-rate gyro logging on, but parts tuning needs are missing.
    Partial,
}

#[derive(Debug, Clone, Serialize)]
pub struct View {
    pub mode: Mode,
    /// What the vehicle logs now, in a sentence.
    pub summary: String,
    /// Changes to switch to tuning logs; empty when already there.
    pub tuning: Vec<ParamChange>,
    /// Changes to switch to normal logs; empty when already there.
    pub normal: Vec<ParamChange>,
}

/// Changes for normal logs: ArduPilot's default LOG_BITMASK and the
/// high-rate gyro logging off.
fn normal_changes(p: Param) -> Vec<ParamChange> {
    let mut out = Vec::new();
    out.extend(change(p, "LOG_BITMASK", COPTER_DEFAULT_LOG_BITMASK, "ArduPilot's default logging, enough for everyday flights and crash diagnosis"));
    out.extend(change(p, "INS_RAW_LOG_OPT", 0.0, "Raw gyro logging off: it makes logs many times larger"));
    out.extend(change(p, "INS_LOG_BAT_MASK", 0.0, "Batch sampler off: it makes logs larger"));
    out
}

pub fn view(param: impl Fn(&str) -> Option<f64>) -> View {
    let p: Param = &param;
    let (gyro, _) = gyro_logging(p);
    let tuning = recommend_params(p, gyro);
    let normal = normal_changes(p);
    let gyro_on = p("INS_RAW_LOG_OPT").is_some_and(|v| v != 0.0) || p("INS_LOG_BAT_MASK").is_some_and(|v| v != 0.0);
    let mask = p("LOG_BITMASK").map_or(0, |m| m as u32);
    let bits = REQUIRED_LOG_BITS.iter().all(|(b, _)| mask & (1 << b) != 0);
    let (mode, summary) = if tuning.is_empty() {
        (Mode::Tuning, "Tuning logs: fast attitude, PIDs and gyro before and after the filters. Logs are large, tens of MB per flight.")
    } else if !gyro_on {
        (
            Mode::Normal,
            if bits {
                "Normal logs with the PID and fast attitude data, but no high-rate gyro: enough for PIDs, not for the noise and filter analysis."
            } else {
                "Normal logs: enough for everyday flights and crash diagnosis, not for tuning."
            },
        )
    } else {
        (Mode::Partial, "High-rate gyro logging is on, but parts tuning needs are missing.")
    };
    View { mode, summary: summary.to_owned(), tuning, normal }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn view_of(values: &[(&str, f64)]) -> View {
        let m: HashMap<&str, f64> = values.iter().copied().collect();
        view(|n| m.get(n).copied())
    }

    #[test]
    fn default_logging_is_normal_and_tuning_turns_on_gyro_and_pids() {
        let v = view_of(&[("LOG_BITMASK", COPTER_DEFAULT_LOG_BITMASK), ("INS_RAW_LOG_OPT", 0.0), ("INS_LOG_BAT_MASK", 0.0), ("INS_LOG_BAT_OPT", 0.0)]);
        assert_eq!(v.mode, Mode::Normal);
        assert!(v.normal.is_empty(), "{:?}", v.normal);
        let names: Vec<&str> = v.tuning.iter().map(|c| c.name.as_str()).collect();
        assert!(names.contains(&"LOG_BITMASK") && names.contains(&"INS_RAW_LOG_OPT"), "{names:?}");
        let mask = v.tuning.iter().find(|c| c.name == "LOG_BITMASK").unwrap().value as u32;
        assert!(REQUIRED_LOG_BITS.iter().all(|(b, _)| mask & (1 << b) != 0));
    }

    #[test]
    fn tuning_logging_is_recognised_and_normal_undoes_it() {
        let mask = COPTER_DEFAULT_LOG_BITMASK as u32 | 1 | (1 << 12);
        let v = view_of(&[("LOG_BITMASK", f64::from(mask)), ("INS_RAW_LOG_OPT", 9.0), ("INS_LOG_BAT_MASK", 0.0), ("INS_LOG_BAT_OPT", 0.0)]);
        assert_eq!(v.mode, Mode::Tuning);
        assert!(v.tuning.is_empty(), "{:?}", v.tuning);
        let normal: Vec<(&str, f64)> = v.normal.iter().map(|c| (c.name.as_str(), c.value)).collect();
        assert_eq!(normal, [("LOG_BITMASK", COPTER_DEFAULT_LOG_BITMASK), ("INS_RAW_LOG_OPT", 0.0)]);
    }

    #[test]
    fn gyro_without_pid_logging_is_partial() {
        let v = view_of(&[("LOG_BITMASK", COPTER_DEFAULT_LOG_BITMASK), ("INS_RAW_LOG_OPT", 9.0), ("INS_LOG_BAT_MASK", 0.0)]);
        assert_eq!(v.mode, Mode::Partial);
        assert!(!v.tuning.is_empty() && !v.normal.is_empty());
    }
}
