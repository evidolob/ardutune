//! AutoTune setup: the steps before it (initial tune, notch filter,
//! QuikTune), checks before flying, AutoTune settings for the vehicle's size,
//! QuikTune setup, and what the AutoTune flight's log says was saved.
//! Follows ArduPilot's AutoTune and QuikTune documentation; the size-based
//! aggressiveness is ArduTune's own suggestion.

use serde::Serialize;

use crate::log::Log;
use crate::readiness::{ParamChange, Status};
use crate::setup::{change, Param};
use crate::tune;

/// RCn_OPTION values.
pub const AUTOTUNE_SWITCH: f64 = 17.0;
pub const TEST_GAINS_SWITCH: f64 = 180.0;
/// Scripting1, QuikTune's usual switch.
pub const SCRIPTING1: f64 = 300.0;
/// Copter modes.
const ALTHOLD: f64 = 2.0;
const AUTOTUNE_MODE: f64 = 15.0;
/// Props from this size get 10 Hz rate filters before AutoTune (ArduPilot docs).
const LARGE_PROP_IN: f64 = 13.0;

#[derive(Debug, Clone, Serialize)]
pub struct Step {
    pub id: &'static str,
    pub title: &'static str,
    /// "done", "todo", "optional" or "unknown".
    pub state: &'static str,
    pub detail: String,
    /// The ArduTune page that does it.
    pub view: &'static str,
}

#[derive(Debug, Clone, Serialize)]
pub struct Check {
    pub id: &'static str,
    pub status: Status,
    pub title: String,
    pub detail: String,
    /// Changes that fix it.
    pub changes: Vec<ParamChange>,
    /// A switch function to put on a switch (RCn_OPTION).
    pub switch: Option<f64>,
    /// An ArduTune page that fixes it.
    pub view: Option<&'static str>,
}

#[derive(Debug, Clone, Serialize)]
pub struct QuikTune {
    /// "builtin" (QUIK_* parameters present), "script" (scripting on, script
    /// missing), "scripting_off" or "unsupported".
    pub availability: &'static str,
    pub enabled: bool,
    /// The switch channel with QuikTune's function, if any.
    pub channel: Option<u8>,
    /// The RCn_OPTION QuikTune listens to.
    pub function: f64,
    pub changes: Vec<ParamChange>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Switches {
    pub autotune: Option<u8>,
    pub test_gains: Option<u8>,
    pub mode_channel: u8,
    /// Flight mode positions (1-6) with AltHold and with AutoTune.
    pub althold: Vec<u8>,
    pub autotune_mode: Vec<u8>,
}

#[derive(Debug, Clone, Serialize)]
pub struct LogResult {
    /// AutoTune ran in this log.
    pub ran: bool,
    /// Axes whose gains were saved ("Roll", "Pitch", "Yaw").
    pub saved: Vec<String>,
    /// Axes AutoTune reported finishing.
    pub completed: Vec<String>,
    pub messages: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct View {
    pub prop_in: Option<f64>,
    pub steps: Vec<Step>,
    pub checks: Vec<Check>,
    /// AutoTune settings for this size.
    pub recommended: Vec<ParamChange>,
    /// Why the recommendation, in a sentence or two.
    pub recommended_note: String,
    pub quiktune: QuikTune,
    pub switches: Switches,
    pub log: Option<LogResult>,
    /// Optional changes after a tune (stiffness, throttle mix).
    pub after: Vec<ParamChange>,
}

fn channel_with(p: Param, option: f64) -> Option<u8> {
    (1..=16u8).find(|c| p(&format!("RC{c}_OPTION")) == Some(option))
}

fn modes_with(p: Param, mode: f64) -> Vec<u8> {
    (1..=6u8).filter(|i| p(&format!("FLTMODE{i}")) == Some(mode)).collect()
}

/// AutoTune aggressiveness: ArduPilot's default 0.1 for small and medium
/// quads, 0.075 for large, more flexible frames (ArduTune's suggestion; the
/// docs give 0.05-0.1).
pub fn aggressiveness(prop_in: f64) -> f64 {
    if prop_in >= LARGE_PROP_IN {
        0.075
    } else {
        0.1
    }
}

fn check(id: &'static str, status: Status, title: impl Into<String>, detail: impl Into<String>) -> Check {
    Check { id, status, title: title.into(), detail: detail.into(), changes: vec![], switch: None, view: None }
}

pub fn view(param: impl Fn(&str) -> Option<f64>, prop_in: Option<f64>, log: Option<&Log>) -> View {
    let p: Param = &param;
    let prop = prop_in.filter(|v| (2.0..=40.0).contains(v));
    let large = prop.is_some_and(|v| v >= LARGE_PROP_IN);
    let switches = Switches {
        autotune: channel_with(p, AUTOTUNE_SWITCH),
        test_gains: channel_with(p, TEST_GAINS_SWITCH),
        mode_channel: p("FLTMODE_CH").unwrap_or(5.0) as u8,
        althold: modes_with(p, ALTHOLD),
        autotune_mode: modes_with(p, AUTOTUNE_MODE),
    };

    // The tuning path.
    let initial = prop.and_then(|v| tune::initial_tune(p, v, None, None).ok());
    let initial_state = match &initial {
        Some(t) if t.changes.is_empty() => ("done", format!("The vehicle has the starting values for {}-inch props.", fmt(prop.unwrap_or(0.0)))),
        Some(t) => ("todo", format!("{} value(s) differ from the starting values for {}-inch props.", t.changes.len(), fmt(prop.unwrap_or(0.0)))),
        None => ("unknown", "Enter the prop size to compare with the starting values.".to_owned()),
    };
    let notch = p("INS_HNTCH_ENABLE") == Some(1.0) && p("INS_HNTCH_MODE").is_some_and(|m| m != 0.0);
    let quik = quiktune(p);
    let ready = !switches.althold.is_empty() && (switches.autotune.is_some() || !switches.autotune_mode.is_empty());
    let log_result = log.map(log_result);
    let steps = vec![
        Step { id: "initial", title: "Starting values", state: initial_state.0, detail: initial_state.1, view: "tune" },
        Step {
            id: "notch",
            title: "Noise filter",
            state: if notch { "done" } else { "todo" },
            detail: if notch {
                "The harmonic notch filter is on.".to_owned()
            } else {
                "Fly a hover, open its log and set the harmonic notch from it (Filters).".to_owned()
            },
            view: "filters",
        },
        Step {
            id: "quiktune",
            title: "QuikTune",
            state: if quik.enabled { "done" } else { "optional" },
            detail: "Optional: a quick first tune of the rate gains while hovering, before AutoTune.".to_owned(),
            view: "autotune",
        },
        Step {
            id: "autotune",
            title: "AutoTune",
            state: if log_result.as_ref().is_some_and(|l| !l.saved.is_empty()) { "done" } else { "todo" },
            detail: if ready { "AltHold and AutoTune are on switches: ready to fly the tune.".to_owned() } else { "Put AltHold and AutoTune on switches (checks below).".to_owned() },
            view: "autotune",
        },
        Step {
            id: "log",
            title: "Check the result",
            state: if log_result.as_ref().is_some_and(|l| l.ran) { "done" } else { "todo" },
            detail: "Open the AutoTune flight's log: what was saved, and the step response before and after (PIDs).".to_owned(),
            view: "pid",
        },
    ];

    // Checks before flying.
    let mut checks = Vec::new();
    if switches.althold.is_empty() {
        let mut c = check(
            "althold",
            Status::Fail,
            "AltHold isn't on the flight mode switch",
            "AutoTune is started from a hover in AltHold, so the vehicle holds its height while it twitches. Set one mode switch position to AltHold (RC channels page).",
        );
        c.view = Some("rc");
        checks.push(c);
    }
    if switches.autotune.is_none() && switches.autotune_mode.is_empty() {
        let mut c = check(
            "switch",
            Status::Fail,
            "AutoTune isn't on a switch",
            "Put AutoTune on a two- or three-position switch: high starts the tune, low stops it and brings back the old gains.",
        );
        c.switch = Some(AUTOTUNE_SWITCH);
        checks.push(c);
    }
    if !notch {
        let mut c = check(
            "notch",
            Status::Warn,
            "The harmonic notch filter is off",
            "AutoTune pushes the gains as high as the noise allows. Without the notch, motor noise limits them, or the tune oscillates. Set it from a hover log first.",
        );
        c.view = Some("filters");
        checks.push(c);
    }
    if p("MOT_BAT_VOLT_MAX").is_some_and(|v| v <= 0.0) {
        let mut c = check(
            "voltage",
            Status::Warn,
            "Battery voltage compensation is off",
            "Without it the gains AutoTune finds on a full pack are too strong on an empty one. Initial tune sets MOT_BAT_VOLT_MAX and MOT_BAT_VOLT_MIN from the pack.",
        );
        c.view = Some("tune");
        checks.push(c);
    }
    if let Some(t) = &initial {
        if t.changes.iter().any(|c| c.name == "MOT_THST_EXPO") {
            let mut c = check(
                "expo",
                Status::Warn,
                "The thrust curve doesn't match the prop size",
                "A wrong MOT_THST_EXPO makes the motors respond unevenly across the throttle range, which AutoTune can't tune around. Initial tune sets it.",
            );
            c.view = Some("tune");
            checks.push(c);
        }
    }
    if p("MOT_SPIN_MIN").is_some_and(|v| v < 0.05) {
        checks.push(check(
            "spin",
            Status::Warn,
            "Spin minimum is very low",
            "AutoTune's twitches drop some motors to minimum; below about 0.05 they can stall or desync. Raise MOT_SPIN_MIN (Frame and hardware).",
        ));
    }
    if large {
        let filters: Vec<ParamChange> = ["ATC_RAT_RLL_FLTT", "ATC_RAT_RLL_FLTD", "ATC_RAT_PIT_FLTT", "ATC_RAT_PIT_FLTD"]
            .iter()
            .filter(|n| p(n).is_some_and(|v| v > 10.0))
            .filter_map(|n| change(p, n, 10.0, "10 Hz for props of 13 inches and more (ArduPilot's AutoTune advice)"))
            .collect();
        if !filters.is_empty() {
            let mut c = check(
                "large_filters",
                Status::Warn,
                "Rate filters are above 10 Hz for large props",
                "ArduPilot's AutoTune page asks for 10 Hz roll and pitch target and D filters on props of 13 inches and more.",
            );
            c.changes = filters;
            checks.push(c);
        }
    }
    checks.sort_by_key(|c| match c.status {
        Status::Fail => 0,
        Status::Warn => 1,
        _ => 2,
    });

    // AutoTune settings for this size.
    let mut recommended = Vec::new();
    let note = match prop {
        Some(v) => {
            let aggr = aggressiveness(v);
            recommended.extend(change(p, "AUTOTUNE_AGGR", aggr, if large { "0.075 for a large, more flexible frame" } else { "ArduPilot's default 0.1" }));
            recommended.extend(change(p, "AUTOTUNE_AXES", 7.0, "Roll, pitch and yaw"));
            if large {
                "Large props: a gentler AutoTune (0.075), and the tune may need more than one pack. If a pack doesn't last the whole tune, tune one axis per flight: set Axes to Roll, fly and save, then Pitch, then Yaw.".to_owned()
            } else {
                "ArduPilot's defaults suit this size: full aggressiveness and all three axes in one flight (about 5-10 minutes of twitching).".to_owned()
            }
        }
        None => "Enter the prop size for settings that suit the vehicle.".to_owned(),
    };

    // After a tune: the docs' "too stiff" reductions and the throttle mix for large props.
    let mut after = Vec::new();
    for n in ["ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_ANG_YAW_P"] {
        if p(n).is_some_and(|v| v > 10.0) {
            after.extend(change(p, n, 6.0, "Softer response if the tune feels too stiff (keep the high value on small aerobatic quads)"));
        }
    }
    if p("ATC_RAT_YAW_P").is_some_and(|v| v > 1.0) {
        after.extend(change(p, "ATC_RAT_YAW_P", 0.5, "Softer yaw if it feels too stiff"));
        after.extend(change(p, "ATC_RAT_YAW_I", 0.05, "Yaw I at a tenth of yaw P"));
    }
    if large && p("ATC_THR_MIX_MAX").is_some_and(|v| v < 0.9) {
        after.extend(change(p, "ATC_THR_MIX_MAX", 0.9, "Less pitch overshoot in AltHold on large props, after a good tune"));
    }

    View { prop_in: prop, steps, checks, recommended, recommended_note: note, quiktune: quik, switches, log: log_result, after }
}

fn fmt(v: f64) -> String {
    crate::params::format_value(v)
}

fn quiktune(p: Param) -> QuikTune {
    let function = p("QUIK_RC_FUNC").filter(|f| *f > 0.0).unwrap_or(SCRIPTING1);
    let channel = channel_with(p, function);
    if p("QUIK_ENABLE").is_some() {
        let enabled = p("QUIK_ENABLE") == Some(1.0);
        let mut changes: Vec<ParamChange> = change(p, "QUIK_ENABLE", 1.0, "Turn QuikTune on").into_iter().collect();
        changes.extend(change(p, "QUIK_AXES", 7.0, "Tune roll, pitch and yaw"));
        return QuikTune { availability: "builtin", enabled, channel, function, changes };
    }
    match p("SCR_ENABLE") {
        Some(1.0) => QuikTune { availability: "script", enabled: false, channel, function, changes: vec![] },
        Some(_) => QuikTune {
            availability: "scripting_off",
            enabled: false,
            channel,
            function,
            changes: change(p, "SCR_ENABLE", 1.0, "Turn Lua scripting on (after a reboot), to run the QuikTune script").into_iter().collect(),
        },
        None => QuikTune { availability: "unsupported", enabled: false, channel, function, changes: vec![] },
    }
}

/// What an AutoTune flight's log says: whether it ran, which axes finished
/// and which were saved.
pub fn log_result(log: &Log) -> LogResult {
    let texts: Vec<String> = log
        .table("MSG")
        .and_then(|m| m.text("Message"))
        .map(|t| t.iter().filter(|m| m.starts_with("AutoTune")).map(|m| m.to_string()).collect())
        .unwrap_or_default();
    let ran = !texts.is_empty() || log.table("ATUN").is_some();
    let axes = |key: &str| -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for m in texts.iter().filter(|m| m.contains(key)) {
            for axis in ["Roll", "Pitch", "Yaw"] {
                if m.contains(axis) && !out.iter().any(|a| a == axis) {
                    out.push(axis.to_owned());
                }
            }
        }
        out
    };
    let mut messages: Vec<String> = Vec::new();
    for m in &texts {
        if !messages.contains(m) && !m.contains("pilot overrides") {
            messages.push(m.clone());
        }
    }
    let keep = messages.len().saturating_sub(12);
    LogResult { ran, saved: axes("Saved gains"), completed: axes("complete"), messages: messages.split_off(keep) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn params() -> HashMap<String, f64> {
        let mut m: HashMap<String, f64> = [
            ("FLTMODE_CH", 6.0),
            ("FLTMODE1", 0.0),
            ("FLTMODE4", 2.0),
            ("FLTMODE6", 5.0),
            ("RC7_OPTION", 17.0),
            ("INS_HNTCH_ENABLE", 1.0),
            ("INS_HNTCH_MODE", 3.0),
            ("MOT_BAT_VOLT_MAX", 16.8),
            ("MOT_SPIN_MIN", 0.13),
            ("AUTOTUNE_AGGR", 0.1),
            ("AUTOTUNE_AXES", 7.0),
            ("ATC_RAT_RLL_FLTT", 20.0),
            ("ATC_RAT_RLL_FLTD", 20.0),
            ("ATC_RAT_PIT_FLTT", 20.0),
            ("ATC_RAT_PIT_FLTD", 20.0),
            ("ATC_THR_MIX_MAX", 0.5),
            ("SCR_ENABLE", 0.0),
        ]
        .iter()
        .map(|(k, v)| (k.to_string(), *v))
        .collect();
        for c in 1..=16 {
            m.entry(format!("RC{c}_OPTION")).or_insert(0.0);
        }
        m
    }

    #[test]
    fn ready_small_quad() {
        let m = params();
        let v = view(|n| m.get(n).copied(), Some(5.0), None);
        assert!(v.checks.is_empty(), "{:?}", v.checks);
        assert_eq!((v.switches.autotune, v.switches.althold.clone()), (Some(7), vec![4]));
        assert!(v.recommended.is_empty(), "defaults suit a 5-inch: {:?}", v.recommended);
        assert!(v.after.is_empty());
        assert_eq!(v.quiktune.availability, "scripting_off");
        assert_eq!(v.quiktune.changes[0].name, "SCR_ENABLE");
        let states: Vec<&str> = v.steps.iter().map(|s| s.state).collect();
        assert_eq!(states[1], "done", "notch on");
    }

    #[test]
    fn large_quad_and_missing_switches() {
        let mut m = params();
        m.insert("RC7_OPTION".into(), 0.0);
        m.insert("FLTMODE4".into(), 0.0);
        m.insert("INS_HNTCH_ENABLE".into(), 0.0);
        let v = view(|n| m.get(n).copied(), Some(15.0), None);
        let ids: Vec<&str> = v.checks.iter().map(|c| c.id).collect();
        assert_eq!(ids, ["althold", "switch", "notch", "large_filters"]);
        assert_eq!(v.checks[1].switch, Some(17.0));
        assert_eq!(v.checks[3].changes.len(), 4);
        assert_eq!((v.recommended[0].name.as_str(), v.recommended[0].value), ("AUTOTUNE_AGGR", 0.075));
        assert!(v.recommended_note.contains("one axis per flight"));
        assert!(v.after.iter().any(|c| c.name == "ATC_THR_MIX_MAX" && c.value == 0.9));
    }

    #[test]
    fn builtin_quiktune() {
        let mut m = params();
        m.extend([("QUIK_ENABLE".to_string(), 0.0), ("QUIK_AXES".into(), 7.0), ("QUIK_RC_FUNC".into(), 300.0), ("RC8_OPTION".into(), 300.0)]);
        let q = view(|n| m.get(n).copied(), None, None).quiktune;
        assert_eq!((q.availability, q.enabled, q.channel), ("builtin", false, Some(8)));
        assert_eq!(q.changes.len(), 1, "only QUIK_ENABLE");
    }

    #[test]
    fn stiff_after_tune() {
        let mut m = params();
        m.extend([("ATC_ANG_RLL_P".to_string(), 12.0), ("ATC_RAT_YAW_P".into(), 1.2), ("ATC_RAT_YAW_I".into(), 0.12)]);
        let v = view(|n| m.get(n).copied(), Some(5.0), None);
        let got: Vec<(&str, f64)> = v.after.iter().map(|c| (c.name.as_str(), c.value)).collect();
        assert_eq!(got, [("ATC_ANG_RLL_P", 6.0), ("ATC_RAT_YAW_P", 0.5), ("ATC_RAT_YAW_I", 0.05)]);
    }

    #[test]
    fn saved_axes_from_the_log() {
        use crate::synth::{LogWriter, V};
        let mut w = LogWriter::new();
        let msg = w.define("MSG", "QZ", "TimeUS,Message");
        for (t, m) in [(1, "AutoTune: Roll Rate D Up 10%"), (2, "AutoTune: pilot overrides active"), (3, "AutoTune: Roll complete"), (4, "AutoTune: Pitch complete"), (5, "AutoTune: Saved gains for Roll Pitch")] {
            w.write(msg, &[V::I(t * 1_000_000), V::S(m)]);
        }
        let log = Log::from_bytes(&w.into_bytes());
        let r = log_result(&log);
        assert!(r.ran);
        assert_eq!((r.completed, r.saved), (vec!["Roll".to_owned(), "Pitch".to_owned()], vec!["Roll".to_owned(), "Pitch".to_owned()]));
        assert!(!r.messages.iter().any(|m| m.contains("pilot overrides")));
    }
}
