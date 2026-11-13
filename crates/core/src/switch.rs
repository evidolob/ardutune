//! Putting a function on a transmitter switch: find the channel the pilot
//! flipped, check it's free, and set its `RCn_OPTION`.

use crate::paramdoc;
use crate::readiness::ParamChange;
use crate::setup::{change, Param};

/// RCn_OPTION "GPS Disable": high turns the GPS off.
pub const GPS_DISABLE: f64 = 65.0;

/// A switch counts as flipped when its channel moved this far (µs).
const FLIPPED_US: u16 = 300;

/// The name of an RCn_OPTION value, from ArduPilot's parameter docs.
pub fn option_name(option: f64) -> String {
    paramdoc::doc("RC5_OPTION").and_then(|d| d.values.iter().find(|(v, _)| *v == option).map(|(_, n)| n.clone())).unwrap_or_else(|| format!("option {option}"))
}

/// The stick channels (roll, pitch, throttle, yaw).
fn sticks(p: Param) -> Vec<u8> {
    ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW"].iter().zip([1.0, 2.0, 3.0, 4.0]).map(|(n, d)| p(n).unwrap_or(d) as u8).collect()
}

/// Why a channel can't take a new function, if it can't.
fn taken(p: Param, ch: u8, option: f64) -> Option<String> {
    if !(1..=16).contains(&ch) {
        return Some(format!("Channel {ch} doesn't exist: ArduPilot reads channels 1-16."));
    }
    if sticks(p).contains(&ch) {
        return Some(format!("Channel {ch} is a stick (roll, pitch, throttle or yaw)."));
    }
    if p("FLTMODE_CH").unwrap_or(5.0) as u8 == ch {
        return Some(format!("Channel {ch} selects the flight mode (FLTMODE_CH)."));
    }
    match p(&format!("RC{ch}_OPTION")) {
        None => Some(format!("This vehicle has no RC{ch}_OPTION.")),
        Some(o) if o != 0.0 && o != option => Some(format!("Channel {ch} already does \"{}\" (RC{ch}_OPTION = {o}).", option_name(o))),
        _ => None,
    }
}

/// The switch the pilot flipped, from each channel's lowest and highest
/// value while watching: (channel, min µs, max µs).
pub fn flipped(param: impl Fn(&str) -> Option<f64>, moves: &[(u8, u16, u16)], option: f64) -> Result<u8, String> {
    let p: Param = &param;
    let sticks = sticks(p);
    let mut moved: Vec<(u8, u16)> = moves
        .iter()
        .filter(|(ch, lo, hi)| *lo > 0 && hi.saturating_sub(*lo) >= FLIPPED_US && !sticks.contains(ch))
        .map(|(ch, lo, hi)| (*ch, hi - lo))
        .collect();
    moved.sort_by_key(|(_, range)| std::cmp::Reverse(*range));
    match moved.as_slice() {
        [] if moves.iter().all(|(_, lo, _)| *lo == 0) => {
            Err("No radio input: turn the transmitter on (and the receiver: many boards only power it from the battery).".to_owned())
        }
        [] => Err("No switch moved. Flip the switch from one end to the other a few times while ArduTune watches.".to_owned()),
        [(ch, _)] => match taken(p, *ch, option) {
            Some(why) => Err(format!("{why} Choose another switch.")),
            None => Ok(*ch),
        },
        more => {
            let list: Vec<String> = more.iter().map(|(ch, _)| ch.to_string()).collect();
            Err(format!("Several channels moved ({}): flip only the one switch.", list.join(", ")))
        }
    }
}

/// Putting `option` on channel `ch`; option 0 removes the channel's function.
pub fn assign(param: impl Fn(&str) -> Option<f64>, ch: u8, option: f64) -> Result<Vec<ParamChange>, String> {
    let p: Param = &param;
    if option == 0.0 {
        let now = p(&format!("RC{ch}_OPTION")).ok_or(format!("This vehicle has no RC{ch}_OPTION."))?;
        let reason = format!("Channel {ch} no longer does \"{}\"", option_name(now));
        return Ok(change(p, &format!("RC{ch}_OPTION"), 0.0, reason).into_iter().collect());
    }
    if let Some(why) = taken(p, ch, option) {
        return Err(why);
    }
    let reason =
        if option == GPS_DISABLE { format!("Channel {ch} turns the GPS off when high, on when low") } else { format!("Channel {ch}: {}", option_name(option)) };
    Ok(change(p, &format!("RC{ch}_OPTION"), option, reason).into_iter().collect())
}

/// Two functions on one three-position switch, which the transmitter splits
/// over two channels: `up` on channel `a` (high with the switch up), `down`
/// on channel `b` (high with the switch down); the middle leaves both low.
pub fn split(param: impl Fn(&str) -> Option<f64>, (a, up): (u8, f64), (b, down): (u8, f64)) -> Result<Vec<ParamChange>, String> {
    let p: Param = &param;
    if a == b {
        return Err("The two positions need two different channels.".to_owned());
    }
    let mut out = Vec::new();
    for (ch, option, pos) in [(a, up, "up"), (b, down, "down")] {
        if let Some(why) = taken(p, ch, option) {
            return Err(why);
        }
        let reason = format!("Switch {pos} (channel {ch} high): {}", option_name(option));
        out.extend(change(p, &format!("RC{ch}_OPTION"), option, reason));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn kakute() -> HashMap<String, f64> {
        let mut m: HashMap<String, f64> = [("FLTMODE_CH", 6.0), ("RC5_OPTION", 153.0)].iter().map(|(k, v)| (k.to_string(), *v)).collect();
        for ch in 1..=16 {
            m.entry(format!("RC{ch}_OPTION")).or_insert(0.0);
        }
        m
    }

    #[test]
    fn finds_the_flipped_switch() {
        let m = kakute();
        let p = |n: &str| m.get(n).copied();
        // Sticks wiggle, channel 7 goes from low to high.
        let moves = [(1, 1100, 1900), (3, 988, 1500), (5, 999, 999), (7, 999, 2011), (8, 1500, 1520)];
        assert_eq!(flipped(p, &moves, GPS_DISABLE), Ok(7));
        let changes = assign(p, 7, GPS_DISABLE).unwrap();
        assert_eq!((changes[0].name.as_str(), changes[0].value), ("RC7_OPTION", 65.0));
        assert!(changes[0].reason.contains("off when high"));
    }

    #[test]
    fn refuses_channels_in_use() {
        let m = kakute();
        let p = |n: &str| m.get(n).copied();
        assert!(flipped(p, &[(5, 999, 2011)], GPS_DISABLE).unwrap_err().contains("already does"));
        assert!(flipped(p, &[(6, 999, 2011)], GPS_DISABLE).unwrap_err().contains("flight mode"));
        assert!(flipped(p, &[(7, 999, 2011), (8, 999, 2011)], GPS_DISABLE).unwrap_err().contains("Several"));
        assert!(flipped(p, &[(7, 999, 1010)], GPS_DISABLE).unwrap_err().contains("No switch moved"));
        assert!(flipped(p, &[(7, 0, 0)], GPS_DISABLE).unwrap_err().contains("No radio input"));
        assert!(assign(p, 2, GPS_DISABLE).unwrap_err().contains("stick"));
        // Already set: nothing to change, not an error.
        let mut m = kakute();
        m.insert("RC7_OPTION".into(), 65.0);
        assert_eq!(assign(|n: &str| m.get(n).copied(), 7, GPS_DISABLE), Ok(vec![]));
        // Removing a function.
        let off = assign(|n: &str| m.get(n).copied(), 7, 0.0).unwrap();
        assert_eq!((off[0].value, off[0].reason.as_str()), (0.0, "Channel 7 no longer does \"GPS Disable\""));
    }

    #[test]
    fn three_position_split() {
        let m = kakute();
        let p = |n: &str| m.get(n).copied();
        let c = split(p, (7, 4.0), (8, GPS_DISABLE)).unwrap();
        let got: Vec<(&str, f64)> = c.iter().map(|c| (c.name.as_str(), c.value)).collect();
        assert_eq!(got, [("RC7_OPTION", 4.0), ("RC8_OPTION", 65.0)]);
        assert!(c[0].reason.starts_with("Switch up (channel 7 high)"));
        assert!(split(p, (7, 4.0), (7, 65.0)).is_err());
        assert!(split(p, (7, 4.0), (5, 65.0)).unwrap_err().contains("already does"));
    }
}
