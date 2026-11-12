//! RC channels as the pilot sees them: what each channel does (sticks,
//! flight mode, switch functions), where its switch is now, the flight mode
//! per switch position, and radio calibration from captured ranges.

use serde::Serialize;

use crate::paramdoc;
use crate::readiness::{ParamChange, Status};
use crate::setup::{change, item, Item, Param};
use crate::switch::option_name;

#[derive(Debug, Clone, Serialize)]
pub struct Channel {
    pub ch: u8,
    /// Live value in µs; `None` when the channel isn't received.
    pub value: Option<u16>,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub trim: Option<f64>,
    pub reversed: bool,
    /// "stick", "mode", "switch" or "free".
    pub kind: &'static str,
    /// What it does: "Throttle", "Flight mode", "Arm/disarm"…
    pub role: String,
    /// RCn_OPTION, for switch channels.
    pub option: Option<f64>,
    /// Switch position: "low", "middle" or "high".
    pub position: Option<&'static str>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ModePosition {
    /// 1-6, as FLTMODE1-6.
    pub position: u8,
    /// PWM band (µs) that selects it.
    pub band: (u16, u16),
    pub mode: Option<f64>,
    pub name: String,
    pub active: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct RcView {
    pub channels: Vec<Channel>,
    /// The flight mode channel, 0 when disabled.
    pub mode_channel: u8,
    pub modes: Vec<ModePosition>,
    /// Checks worth acting on (failsafe value, calibration).
    pub notes: Vec<Item>,
    /// Whether any channel is received.
    pub signal: bool,
}

/// ArduPilot's mode switch bands (µs) for FLTMODE1-6.
const MODE_BANDS: [(u16, u16); 6] = [(0, 1230), (1231, 1360), (1361, 1490), (1491, 1620), (1621, 1749), (1750, 2200)];
const STICKS: [(&str, &str, f64); 4] = [("RCMAP_ROLL", "Roll", 1.0), ("RCMAP_PITCH", "Pitch", 2.0), ("RCMAP_THROTTLE", "Throttle", 3.0), ("RCMAP_YAW", "Yaw", 4.0)];

/// Where an aux switch is: ArduPilot's thresholds are 1200 and 1800 µs.
pub fn position(value: u16) -> &'static str {
    if value < 1200 {
        "low"
    } else if value > 1800 {
        "high"
    } else {
        "middle"
    }
}

pub fn mode_label(mode: f64) -> String {
    paramdoc::doc("FLTMODE1")
        .and_then(|d| d.values.iter().find(|(v, _)| *v == mode).map(|(_, n)| n.clone()))
        .unwrap_or_else(|| format!("mode {mode}"))
}

/// A parameter value and its name.
pub type Named = (f64, String);

/// The flight modes and the switch functions to choose from.
pub fn choices() -> (Vec<Named>, Vec<Named>) {
    let values = |n: &str| paramdoc::doc(n).map(|d| d.values.clone()).unwrap_or_default();
    let options = values("RC5_OPTION").into_iter().filter(|(v, _)| *v != 0.0).collect();
    (values("FLTMODE1"), options)
}

pub fn view(param: impl Fn(&str) -> Option<f64>, live: &[u16]) -> RcView {
    let p: Param = &param;
    let mode_channel = p("FLTMODE_CH").unwrap_or(5.0) as u8;
    let value = |ch: u8| live.get(ch as usize - 1).copied().filter(|v| *v > 0 && *v != u16::MAX);
    let channels: Vec<Channel> = (1..=16u8)
        .map(|ch| {
            let num = |what: &str| p(&format!("RC{ch}_{what}"));
            let v = value(ch);
            let stick = STICKS.iter().find(|(n, _, d)| p(n).unwrap_or(*d) as u8 == ch);
            let option = num("OPTION").filter(|o| *o != 0.0);
            let (kind, role) = match (stick, option) {
                (Some((_, name, _)), _) => ("stick", (*name).to_owned()),
                _ if ch == mode_channel => ("mode", "Flight mode".to_owned()),
                (None, Some(o)) => ("switch", option_name(o)),
                (None, None) => ("free", String::new()),
            };
            Channel {
                ch,
                value: v,
                min: num("MIN"),
                max: num("MAX"),
                trim: num("TRIM"),
                reversed: num("REVERSED") == Some(1.0),
                kind,
                role,
                option: if kind == "switch" { option } else { None },
                position: if kind == "switch" { v.map(position) } else { None },
            }
        })
        .collect();
    let mode_value = if mode_channel > 0 { value(mode_channel) } else { None };
    let modes = (1..=6u8)
        .map(|i| {
            let mode = p(&format!("FLTMODE{i}"));
            let band = MODE_BANDS[i as usize - 1];
            ModePosition {
                position: i,
                band,
                mode,
                name: mode.map(mode_label).unwrap_or_default(),
                active: mode_value.is_some_and(|v| (band.0..=band.1).contains(&v)),
            }
        })
        .collect();
    RcView { signal: channels.iter().any(|c| c.value.is_some()), notes: notes(p), channels, mode_channel, modes }
}

fn notes(p: Param) -> Vec<Item> {
    let mut out = Vec::new();
    let thr = p("RCMAP_THROTTLE").unwrap_or(3.0) as u8;
    let thr_min = p(&format!("RC{thr}_MIN"));
    if let (Some(min), Some(fs), Some(on)) = (thr_min, p("FS_THR_VALUE"), p("FS_THR_ENABLE")) {
        if on > 0.0 && fs > min - 10.0 {
            out.push(item(
                Status::Fail,
                format!("The radio failsafe value ({fs:.0} µs) is not below the lowest throttle ({min:.0} µs)"),
                "The failsafe would trigger with the throttle down. Set FS_THR_VALUE about 10 µs below RC throttle minimum, or recalibrate.",
            ));
        }
    }
    let default = (1..=4).all(|ch| p(&format!("RC{ch}_MIN")) == Some(1100.0) && p(&format!("RC{ch}_MAX")) == Some(1900.0));
    if default && p("RC1_MIN").is_some() {
        out.push(item(Status::Warn, "The radio isn't calibrated", "The sticks still have the default 1100-1900 µs range: calibrate the radio below."));
    }
    out
}

/// Calibration from what was captured while the pilot moved every stick and
/// switch to its ends, then centred the sticks: (channel, min, max, centre).
/// Channels that barely moved are left alone.
pub fn calibration(param: impl Fn(&str) -> Option<f64>, captured: &[(u8, u16, u16, u16)]) -> Result<Vec<ParamChange>, String> {
    let p: Param = &param;
    let thr = p("RCMAP_THROTTLE").unwrap_or(3.0) as u8;
    let sticks: Vec<u8> = STICKS.iter().map(|(n, _, d)| p(n).unwrap_or(*d) as u8).collect();
    let moved: Vec<&(u8, u16, u16, u16)> = captured.iter().filter(|(_, lo, hi, _)| *lo > 0 && hi.saturating_sub(*lo) >= 400).collect();
    let missing: Vec<String> = sticks.iter().filter(|s| !moved.iter().any(|m| m.0 == **s)).map(|s| s.to_string()).collect();
    if !missing.is_empty() {
        return Err(format!("Channel {} didn't move through its range: move every stick to all its ends, then try again.", missing.join(", ")));
    }
    let mut out = Vec::new();
    for &&(ch, lo, hi, centre) in &moved {
        let name = |what: &str| format!("RC{ch}_{what}");
        out.extend(change(p, &name("MIN"), f64::from(lo), format!("Channel {ch} lowest")));
        out.extend(change(p, &name("MAX"), f64::from(hi), format!("Channel {ch} highest")));
        // Throttle trim sits at the bottom; the other sticks centre.
        let trim = if ch == thr { lo } else if sticks.contains(&ch) { centre } else { lo + (hi - lo) / 2 };
        out.extend(change(p, &name("TRIM"), f64::from(trim), format!("Channel {ch} centre")));
    }
    if let (Some(fs), Some(&&(_, lo, _, _))) = (p("FS_THR_VALUE"), moved.iter().find(|m| m.0 == thr)) {
        if fs > f64::from(lo) - 10.0 {
            out.extend(change(p, "FS_THR_VALUE", f64::from(lo) - 30.0, "Radio failsafe below the new lowest throttle"));
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn kakute() -> HashMap<String, f64> {
        let mut m: HashMap<String, f64> = [("FLTMODE_CH", 6.0), ("RC5_OPTION", 153.0), ("RC7_OPTION", 65.0), ("FS_THR_ENABLE", 1.0), ("FS_THR_VALUE", 975.0)]
            .iter()
            .map(|(k, v)| (k.to_string(), *v))
            .collect();
        for ch in 1..=16 {
            m.entry(format!("RC{ch}_OPTION")).or_insert(0.0);
            m.insert(format!("RC{ch}_MIN"), 988.0);
            m.insert(format!("RC{ch}_MAX"), 2011.0);
            m.insert(format!("RC{ch}_TRIM"), 1500.0);
            m.insert(format!("RC{ch}_REVERSED"), 0.0);
        }
        for (i, mode) in [0.0, 0.0, 2.0, 2.0, 5.0, 5.0].iter().enumerate() {
            m.insert(format!("FLTMODE{}", i + 1), *mode);
        }
        m
    }

    #[test]
    fn roles_positions_and_modes() {
        let m = kakute();
        let v = view(|n| m.get(n).copied(), &[1500, 1500, 988, 1500, 2011, 1550, 999, 1500]);
        let roles: Vec<(&str, &str)> = v.channels.iter().take(7).map(|c| (c.kind, c.role.as_str())).collect();
        assert_eq!(roles[..4], [("stick", "Roll"), ("stick", "Pitch"), ("stick", "Throttle"), ("stick", "Yaw")]);
        assert_eq!(v.channels[4].kind, "switch");
        assert_eq!(v.channels[4].position, Some("high"));
        assert_eq!((v.channels[5].kind, v.channels[5].role.as_str()), ("mode", "Flight mode"));
        assert_eq!((v.channels[6].role.as_str(), v.channels[6].position), ("GPS Disable", Some("low")));
        assert_eq!(v.channels[8].value, None, "not received");
        // 1550 µs is position 4: AltHold.
        let active: Vec<&str> = v.modes.iter().filter(|m| m.active).map(|m| m.name.as_str()).collect();
        assert_eq!(active, ["AltHold"]);
        assert!(v.signal && v.notes.is_empty(), "{:?}", v.notes);
    }

    #[test]
    fn failsafe_above_throttle_minimum() {
        let mut m = kakute();
        m.insert("FS_THR_VALUE".into(), 990.0);
        let v = view(|n| m.get(n).copied(), &[]);
        assert!(!v.signal);
        assert_eq!(v.notes[0].status, Status::Fail);
    }

    #[test]
    fn calibration_from_captured_ranges() {
        let m = kakute();
        let p = |n: &str| m.get(n).copied();
        let captured = [(1, 990, 2010, 1502), (2, 990, 2010, 1499), (3, 985, 2012, 985), (4, 990, 2010, 1500), (5, 999, 2011, 999), (9, 1500, 1500, 1500)];
        let c = calibration(p, &captured).unwrap();
        let get = |n: &str| c.iter().find(|x| x.name == n).map(|x| x.value);
        assert_eq!((get("RC3_MIN"), get("RC3_TRIM")), (Some(985.0), Some(985.0)), "throttle trim at the bottom");
        assert_eq!(get("RC1_TRIM"), Some(1502.0));
        assert_eq!(get("RC5_TRIM"), Some(1505.0), "switches centre between their ends");
        assert!(get("RC9_MIN").is_none(), "unmoved channels are left alone");
        assert!(get("FS_THR_VALUE").is_none(), "975 is already below 985");
        assert!(calibration(p, &captured[..2]).unwrap_err().contains("Channel 3, 4"));
    }
}
