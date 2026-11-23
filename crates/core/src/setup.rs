//! Configuration guide: ordered setup steps checked against the live
//! vehicle's parameters (and live status where it helps), each with plain
//! findings and the parameter changes that fix what can be fixed safely
//! without more information. Copter only.

use serde::Serialize;

use crate::diagnose::mode_name;
use crate::params::format_value;
use crate::readiness::{notch_config, recommend_params, ParamChange, Status, BAT_OPT_PRE_POST_BIT, RAW_LOG_OPT_PRE_POST_BIT, REQUIRED_LOG_BITS};

/// Live state that some checks use; all optional.
#[derive(Debug, Clone, Default)]
pub struct Live {
    /// SYS_STATUS sensor bits: (present, enabled, healthy).
    pub sensors: Option<(u32, u32, u32)>,
    pub battery_v: Option<f32>,
    /// Status texts the vehicle sent.
    pub messages: Vec<String>,
    /// RC input channels 1-8 in µs (0 when not received).
    pub rc: Option<[u16; 8]>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Guide {
    pub steps: Vec<Step>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Step {
    pub id: &'static str,
    pub title: &'static str,
    /// Why the step matters.
    pub why: &'static str,
    /// The worst of the step's findings.
    pub status: Status,
    pub items: Vec<Item>,
    /// Changes that fix this step's problems.
    pub changes: Vec<ParamChange>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Item {
    pub status: Status,
    pub title: String,
    pub detail: String,
}

pub(crate) type Param<'a> = &'a dyn Fn(&str) -> Option<f64>;

const SENSOR_RC: u32 = 1 << 16;
const SENSOR_LOGGING: u32 = 1 << 24;
const SENSOR_NAMES: [(u32, &str); 8] = [
    (1, "gyro"),
    (1 << 1, "accelerometer"),
    (1 << 2, "compass"),
    (1 << 3, "barometer"),
    (1 << 17, "second gyro"),
    (1 << 18, "second accelerometer"),
    (1 << 19, "second compass"),
    (1 << 21, "attitude estimate (AHRS)"),
];
/// Copter modes that need GPS.
const POSITION_MODES: [u32; 14] = [3, 4, 5, 6, 7, 11, 16, 17, 18, 19, 21, 23, 24, 27];

pub fn assess(param: impl Fn(&str) -> Option<f64>, live: &Live) -> Guide {
    let p: Param = &param;
    Guide { steps: vec![frame(p), esc_telemetry(p), radio(p, live), failsafes(p), battery(p, live), sensors(p, live), logging(p, live)] }
}

pub(crate) fn item(status: Status, title: impl Into<String>, detail: impl Into<String>) -> Item {
    Item { status, title: title.into(), detail: detail.into() }
}

/// A step's status is its worst warning or failure; a step with nothing to
/// check is Info ("not checked").
fn step(id: &'static str, title: &'static str, why: &'static str, items: Vec<Item>, changes: Vec<ParamChange>) -> Step {
    if items.is_empty() {
        let note = item(Status::Info, "Not checked", "The vehicle didn't report the parameters this step looks at.");
        return Step { id, title, why, status: Status::Info, items: vec![note], changes };
    }
    // Notes (Info) don't lower a step's status; warnings and failures do.
    let status = items.iter().map(|i| i.status).filter(|s| rank(*s) >= 2).max_by_key(|s| rank(*s)).unwrap_or(Status::Pass);
    Step { id, title, why, status, items, changes }
}

fn rank(s: Status) -> u8 {
    match s {
        Status::Pass => 0,
        Status::Info => 1,
        Status::Warn => 2,
        Status::Fail => 3,
    }
}

pub(crate) fn change(p: Param, name: &str, value: f64, reason: impl Into<String>) -> Option<ParamChange> {
    let current = p(name)?;
    // The vehicle stores 32-bit floats: 0.075 reads back as 0.07500000298.
    (current as f32 != value as f32).then(|| ParamChange { name: name.to_owned(), current: Some(current), value, reason: reason.into() })
}

pub(crate) fn int(p: Param, name: &str) -> Option<i64> {
    p(name).map(|v| v as i64)
}

/// Serial ports with `SERIALn_PROTOCOL == protocol`.
fn ports_with(p: Param, protocol: f64) -> Vec<u32> {
    (0..10).filter(|n| p(&format!("SERIAL{n}_PROTOCOL")) == Some(protocol)).collect()
}

fn port_list(ports: &[u32]) -> String {
    ports.iter().map(|n| format!("SERIAL{n}")).collect::<Vec<_>>().join(", ")
}

// ---------- 1. Frame and motors ----------

pub(crate) fn frame_type_name(t: i64) -> &'static str {
    match t {
        0 => "Plus",
        1 => "X",
        2 => "V",
        3 => "H",
        4 => "V-Tail",
        5 => "A-Tail",
        10 => "Y6B",
        6 => "Plus reversed",
        11 => "Y6F",
        12 => "BetaFlight X",
        13 => "DJI X",
        14 => "Clockwise X",
        15 => "I",
        16 => "NYT Plus",
        17 => "NYT X",
        18 => "BetaFlight X reversed",
        19 => "Y4",
        _ => "other",
    }
}

pub(crate) fn motor_count(class: i64) -> Option<usize> {
    match class {
        1 => Some(4),
        2 | 5 => Some(6),
        3 | 4 => Some(8),
        7 => Some(3),
        12 => Some(12),
        14 => Some(10),
        _ => None,
    }
}

fn frame(p: Param) -> Step {
    let mut items = Vec::new();
    let class = int(p, "FRAME_CLASS");
    match class {
        Some(0) | None => items.push(item(
            Status::Fail,
            "Frame class is not set",
            "Set FRAME_CLASS (1 = quad) and FRAME_TYPE, then reboot. Nothing will arm until it is set.",
        )),
        Some(c) => {
            let t = int(p, "FRAME_TYPE").unwrap_or(-1);
            let class_name = match c {
                1 => "Quad",
                2 => "Hexa",
                3 => "Octa",
                4 => "OctaQuad",
                5 => "Y6",
                7 => "Tri",
                _ => "Other",
            };
            items.push(item(
                Status::Pass,
                format!("{class_name}, {} layout (FRAME_CLASS {c}, FRAME_TYPE {t})", frame_type_name(t)),
                "Check the layout matches the motor order diagram for this frame type; ArduTune's Motor test confirms order and direction.",
            ));
            if c == 1 && t == 19 {
                items.push(item(
                    Status::Fail,
                    "FRAME_TYPE 19 is Y4, not an X quad",
                    "Y4 has two motors at the front and two stacked at the back. An earlier ArduTune offered 19 as \"BetaFlight X reversed\" by mistake: that is 18. Set the frame type that matches your quad (Motor test offers the switch), reboot, and test the motors again.",
                ));
            }
            if let Some(n) = motor_count(c) {
                let missing: Vec<usize> = (1..=n).filter(|m| !(1..=16).any(|ch| p(&format!("SERVO{ch}_FUNCTION")) == Some(32.0 + *m as f64))).collect();
                if missing.is_empty() {
                    items.push(item(Status::Pass, format!("All {n} motors have an output"), "SERVOn_FUNCTION 33…"));
                } else {
                    let list = missing.iter().map(|m| format!("motor {m}")).collect::<Vec<_>>().join(", ");
                    items.push(item(
                        Status::Fail,
                        format!("No output assigned to {list}"),
                        "Set SERVOn_FUNCTION on the outputs your ESCs are wired to: 33 for motor 1, 34 for motor 2 and so on.",
                    ));
                }
            }
        }
    }
    let pwm = int(p, "MOT_PWM_TYPE");
    match pwm {
        Some(v @ 4..=7) => items.push(item(
            Status::Pass,
            format!("DShot{} ESC protocol", [150, 300, 600, 1200][(v - 4) as usize]),
            "No ESC calibration needed, and it allows RPM telemetry.",
        )),
        Some(0..=2) => items.push(item(
            Status::Info,
            "Analog ESC protocol (PWM/OneShot)",
            "Needs ESC calibration. If your ESCs support DShot (most BLHeli_32/AM32/Bluejay ESCs do), DShot600 (MOT_PWM_TYPE = 6) is simpler and enables RPM telemetry.",
        )),
        Some(v) => items.push(item(Status::Info, format!("ESC protocol MOT_PWM_TYPE = {v}"), "")),
        None => {}
    }
    if let (Some(arm), Some(min)) = (p("MOT_SPIN_ARM"), p("MOT_SPIN_MIN")) {
        if arm >= min {
            items.push(item(
                Status::Warn,
                format!("Armed spin ({arm}) is not below minimum spin ({min})"),
                "MOT_SPIN_ARM must be lower than MOT_SPIN_MIN so motors idle slower when armed than the lowest flight throttle.",
            ));
        }
    }
    step("frame", "Frame and motors", "The flight controller must know the frame layout and which output drives each motor.", items, vec![])
}

// ---------- 2. ESC telemetry and RPM ----------

fn esc_telemetry(p: Param) -> Step {
    let mut items = Vec::new();
    let bdmask = int(p, "SERVO_BLH_BDMASK").unwrap_or(0);
    let dshot = matches!(int(p, "MOT_PWM_TYPE"), Some(4..=7));
    let serial_telem = ports_with(p, 16.0);
    let rpm = if bdmask != 0 && dshot {
        let outputs: Vec<String> = (0..16).filter(|b| bdmask >> b & 1 == 1).map(|b| (b + 1).to_string()).collect();
        items.push(item(
            Status::Pass,
            format!("Bidirectional DShot on outputs {}", outputs.join(", ")),
            "Each ESC reports its motor RPM: the best source for the harmonic notch. It needs a \"-bdshot\" firmware on some boards.",
        ));
        true
    } else if !serial_telem.is_empty() {
        items.push(item(
            Status::Pass,
            format!("Serial ESC telemetry on {}", port_list(&serial_telem)),
            "ESCs report RPM, temperature and current over a telemetry wire.",
        ));
        true
    } else {
        items.push(item(
            Status::Info,
            "No ESC RPM telemetry",
            if bdmask != 0 {
                "SERVO_BLH_BDMASK is set but the ESC protocol isn't DShot: bidirectional DShot needs MOT_PWM_TYPE 4-7."
            } else {
                "Without RPM the notch filter follows throttle, which is less precise. With DShot ESCs, bidirectional DShot (SERVO_BLH_BDMASK) gives RPM on the same wire."
            },
        ));
        false
    };
    if rpm {
        if let Some(poles) = p("SERVO_BLH_POLES") {
            items.push(item(
                Status::Info,
                format!("Motor poles: {}", format_value(poles)),
                "Must match your motors (count the magnets on the bell; 5\" motors usually have 14), or every RPM reading is wrong.",
            ));
        }
    }
    let mut changes = Vec::new();
    if p("INS_HNTCH_ENABLE") == Some(1.0) {
        let mode = int(p, "INS_HNTCH_MODE").unwrap_or(0);
        let freq = format_value(p("INS_HNTCH_FREQ").unwrap_or(80.0));
        if mode != 0 && p("INS_HNTCH_REF") == Some(0.0) {
            // ArduPilot skips tracking entirely when REF is 0.
            let (value, how) = match mode {
                1 => (p("MOT_THST_HOVER"), "the hover throttle (MOT_THST_HOVER)"),
                2 | 3 | 5 => (Some(1.0), "1"),
                _ => (None, "a positive value"),
            };
            items.push(item(
                Status::Warn,
                format!("The harmonic notch doesn't follow the motors: INS_HNTCH_REF is 0, so it stays at {freq} Hz"),
                format!("REF = 0 turns tracking off in every mode. Set INS_HNTCH_REF to {how} and reboot."),
            ));
            if let Some(v) = value {
                changes.extend(change(p, "INS_HNTCH_REF", v, "Let the harmonic notch follow motor speed (takes effect after a reboot)"));
            }
        }
        if rpm && mode != 3 && mode != 2 {
            items.push(item(
                Status::Warn,
                format!("The harmonic notch follows {} although RPM telemetry is available", if mode == 1 { "throttle" } else { "a fixed frequency" }),
                "INS_HNTCH_MODE = 3 makes it track ESC RPM. Re-check the filters with a flight log afterwards.",
            ));
            changes.extend(change(p, "INS_HNTCH_MODE", 3.0, "Track motor RPM from ESC telemetry"));
        } else {
            let note = notch_config(p);
            items.push(item(Status::Info, note.title, note.detail));
        }
    } else if p("INS_HNTCH_ENABLE").is_some() {
        items.push(item(
            Status::Info,
            "Harmonic notch filter is off",
            "Enable it (INS_HNTCH_ENABLE = 1, reboot) and set it up from a hover log with ArduTune's Noise and Filters analysis.",
        ));
    }
    step(
        "esc",
        "ESC telemetry and RPM",
        "Motor RPM lets the notch filter remove motor noise precisely, and ESC telemetry shows failing motors.",
        items,
        changes,
    )
}

// ---------- 3. Radio ----------

fn radio(p: Param, live: &Live) -> Step {
    let mut items = Vec::new();
    let mut changes = Vec::new();
    let rc_ports = ports_with(p, 23.0);
    let rc_options = int(p, "RC_OPTIONS").unwrap_or(0);
    let crsf = !rc_ports.is_empty() && (rc_options & (1 << 8) != 0 || rc_options & (1 << 13) != 0);
    match live.sensors {
        Some((present, _, healthy)) if present & SENSOR_RC != 0 && healthy & SENSOR_RC != 0 => {
            items.push(item(Status::Pass, "Receiver signal present", where_rc(&rc_ports)));
        }
        Some(_) => items.push(item(
            Status::Fail,
            "No receiver signal",
            format!(
                "{} Check the receiver has power (often only with the battery connected), is bound (solid LED) and its TX goes to the RX pad of that port.",
                where_rc(&rc_ports)
            ),
        )),
        None => items.push(item(Status::Info, "Receiver input", where_rc(&rc_ports))),
    }

    let defaults = (1..=4).all(|ch| p(&format!("RC{ch}_MIN")) == Some(1100.0) && p(&format!("RC{ch}_MAX")) == Some(1900.0));
    if p("RC1_MIN").is_none() {
        // Can't tell without the RC parameters.
    } else if defaults {
        items.push(item(Status::Fail, "The radio isn't calibrated", "RC1-4 still have the default 1100-1900 range. Calibrate the radio in a ground station."));
    } else {
        items.push(item(Status::Pass, "Radio calibrated", ""));
    }

    let rssi = int(p, "RSSI_TYPE");
    if crsf && rssi != Some(3) {
        items.push(item(
            Status::Warn,
            "RSSI isn't read from the CRSF/ELRS receiver",
            "RSSI_TYPE = 3 (receiver protocol) shows real link quality and lets a weak link be noticed.",
        ));
        changes.extend(change(p, "RSSI_TYPE", 3.0, "Read RSSI and link quality from the CRSF/ELRS receiver"));
    } else if rssi == Some(0) {
        items.push(item(Status::Info, "RSSI is off", "Set RSSI_TYPE to match your receiver if it reports signal strength."));
    }
    if crsf && rc_options & (1 << 8) != 0 {
        items.push(item(
            Status::Info,
            "CRSF passthrough telemetry is on (for the Yaapu script)",
            "It needs a generous ELRS telemetry ratio (1:2 to 1:8); otherwise the radio reports \"sensor lost\".",
        ));
    }

    let modes: Vec<u32> = (1..=6).filter_map(|i| p(&format!("FLTMODE{i}")).map(|m| m as u32)).collect();
    if !modes.is_empty() {
        let mut names: Vec<String> = Vec::new();
        for m in &modes {
            let n = mode_name(*m);
            if !names.contains(&n) {
                names.push(n);
            }
        }
        let has_basic = modes.iter().any(|m| !POSITION_MODES.contains(m));
        items.push(item(
            if has_basic { Status::Pass } else { Status::Warn },
            format!("Flight modes on channel {}: {}", format_value(p("FLTMODE_CH").unwrap_or(5.0)), names.join(", ")),
            if has_basic {
                "Includes a mode that flies without GPS."
            } else {
                "Every mode needs GPS: add Stabilize or AltHold so you can fly (and arm) without a position fix."
            },
        ));
    }

    let switch = (1..=16).find(|ch| matches!(p(&format!("RC{ch}_OPTION")), Some(41.0 | 153.0 | 154.0)));
    let rudder = int(p, "ARMING_RUDDER").unwrap_or(2);
    let how = match (switch, rudder) {
        (Some(ch), 0) => format!("arm switch on channel {ch}"),
        (Some(ch), _) => format!("arm switch on channel {ch}, or throttle down and yaw right"),
        (None, 0) => "ground station only".to_owned(),
        (None, _) => "throttle down and yaw right".to_owned(),
    };
    items.push(item(Status::Info, format!("Arming: {how}"), ""));
    for ch in (1..=16).filter(|ch| p(&format!("RC{ch}_OPTION")) == Some(crate::switch::GPS_DISABLE)) {
        let now = live.rc.and_then(|rc| rc.get(ch - 1).copied()).filter(|v| *v > 0);
        let position = match now {
            Some(v) if v > 1800 => " It is high now: the GPS is off.",
            Some(v) if v < 1200 => " It is low now: the GPS is on.",
            _ => "",
        };
        items.push(item(Status::Info, format!("GPS off switch on channel {ch}"), format!("High turns the GPS off (RC{ch}_OPTION = 65).{position}")));
    }
    step("radio", "Radio", "A calibrated, reliable radio link is the first safety system.", items, changes)
}

fn where_rc(ports: &[u32]) -> String {
    if ports.is_empty() {
        "Receiver on the RC input pad (no serial port set to RC input).".to_owned()
    } else {
        format!("Receiver on {} (SERIALn_PROTOCOL = 23).", port_list(ports))
    }
}

// ---------- 4. Failsafes ----------

fn failsafes(p: Param) -> Step {
    let mut items = Vec::new();
    let mut changes = Vec::new();
    match int(p, "FS_THR_ENABLE") {
        Some(0) => {
            items.push(item(
                Status::Fail,
                "Radio failsafe is off",
                "If the radio link is lost, the vehicle keeps doing the last thing it was told. FS_THR_ENABLE = 1 returns home (or lands without GPS).",
            ));
            changes.extend(change(p, "FS_THR_ENABLE", 1.0, "Return home or land when the radio link is lost"));
        }
        Some(_) => {
            let fs = p("FS_THR_VALUE");
            let min = p(&format!("RC{}_MIN", format_value(p("RCMAP_THROTTLE").unwrap_or(3.0))));
            match (fs, min) {
                (Some(fs), Some(min)) if fs >= min - 10.0 => items.push(item(
                    Status::Fail,
                    format!("Throttle failsafe value {fs:.0} is not below the lowest throttle {min:.0}"),
                    "FS_THR_VALUE must be at least 10 below RC3_MIN, or the failsafe triggers with the throttle down. Recalibrate the radio or lower FS_THR_VALUE.",
                )),
                _ => items.push(item(Status::Pass, "Radio failsafe is on", "")),
            }
        }
        None => {}
    }
    battery_failsafe(p, &mut items, &mut changes);
    if int(p, "FS_EKF_ACTION") == Some(0) {
        items.push(item(Status::Warn, "EKF failsafe is off", "FS_EKF_ACTION = 1 lands if the position estimate goes bad."));
    }
    if int(p, "FS_CRASH_CHECK") == Some(0) {
        items.push(item(Status::Warn, "Crash check is off", "FS_CRASH_CHECK = 1 disarms after a crash so the motors stop."));
        changes.extend(change(p, "FS_CRASH_CHECK", 1.0, "Disarm automatically after a crash"));
    }
    step("failsafes", "Failsafes", "What the vehicle does on its own when the radio, battery or position estimate fails.", items, changes)
}

/// Battery chemistry, from the per-cell voltages in MOT_BAT_VOLT_MAX/MIN
/// (as ArduPilot's initial tune setup sets them).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Chemistry {
    LiPo,
    LiPoHv,
    LiIon,
}

impl Chemistry {
    pub fn name(self) -> &'static str {
        match self {
            Chemistry::LiPo => "LiPo",
            Chemistry::LiPoHv => "LiPo HV",
            Chemistry::LiIon => "Li-ion",
        }
    }

    /// Low and critical failsafe voltages per cell, under load.
    fn thresholds(self) -> (f64, f64) {
        match self {
            Chemistry::LiPo | Chemistry::LiPoHv => (3.5, 3.3),
            Chemistry::LiIon => (3.1, 2.8),
        }
    }

    /// Full and empty voltage per cell, as ArduPilot's initial tune
    /// calculator uses them for MOT_BAT_VOLT_MAX/MIN.
    pub fn cell_range(self) -> (f64, f64) {
        match self {
            Chemistry::LiPo => (4.2, 3.3),
            Chemistry::LiPoHv => (4.35, 3.3),
            Chemistry::LiIon => (4.1, 2.8),
        }
    }
}

/// Cell count and chemistry from MOT_BAT_VOLT_MAX (4.05-4.45 V per full
/// cell; 4.3 V or more is high-voltage LiPo) and MOT_BAT_VOLT_MIN (below
/// 3.0 V per cell means Li-ion).
pub(crate) fn pack(p: Param) -> Option<(u32, Chemistry)> {
    let max = p("MOT_BAT_VOLT_MAX").filter(|v| *v > 0.0)?;
    let cells = (1..=14).find(|n| (4.05..=4.45).contains(&(max / f64::from(*n))))?;
    let min_cell = p("MOT_BAT_VOLT_MIN").filter(|v| *v > 0.0).map(|v| v / f64::from(cells));
    let chem = if min_cell.is_some_and(|v| v < 3.0) {
        Chemistry::LiIon
    } else if max / f64::from(cells) >= 4.3 {
        Chemistry::LiPoHv
    } else {
        Chemistry::LiPo
    };
    Some((cells, chem))
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

fn battery_failsafe(p: Param, items: &mut Vec<Item>, changes: &mut Vec<ParamChange>) {
    let Some(low_act) = int(p, "BATT_FS_LOW_ACT") else { return };
    let gps = p("GPS1_TYPE").or_else(|| p("GPS_TYPE")).is_some_and(|t| t != 0.0);
    let (rtl, rtl_name) = if gps { (2.0, "return home") } else { (1.0, "land") };
    let Some((cells, chem)) = pack(p) else {
        // Without the pack, only check that something is set.
        let none = p("BATT_LOW_VOLT").unwrap_or(0.0) == 0.0 && p("BATT_LOW_MAH").unwrap_or(0.0) == 0.0;
        if low_act == 0 || none {
            items.push(item(
                Status::Warn,
                if low_act == 0 { "No action on low battery" } else { "Low-battery action set but no threshold" },
                "Set BATT_FS_LOW_ACT (2 = return home, 1 = land) and BATT_LOW_VOLT: about 3.5 V per cell for LiPo, 3.1 V for Li-ion, under load. Set MOT_BAT_VOLT_MAX/MIN for your pack and ArduTune can suggest values.",
            ));
        } else {
            items.push(item(Status::Pass, "Low-battery failsafe set", ""));
        }
        return;
    };
    let n = f64::from(cells);
    let (low_cell, crit_cell) = chem.thresholds();
    let (low, crit) = (round1(low_cell * n), round1(crit_cell * n));
    items.push(item(
        Status::Info,
        format!("Battery: {cells}S {} (from MOT_BAT_VOLT_MAX/MIN)", chem.name()),
        format!(
            "Suggested failsafes for this pack: low {low} V ({low_cell} V per cell) → {rtl_name}, critical {crit} V ({crit_cell} V per cell) → land. If the pack is different, correct MOT_BAT_VOLT_MAX/MIN first."
        ),
    ));

    let low_level = Level { param: "BATT_LOW_VOLT", action: "BATT_FS_LOW_ACT", what: "low", value: low, bounds: (crit_cell, 3.8), act: (rtl, rtl_name) };
    check_level(p, n, chem, &low_level, items, changes);
    if p("BATT_CRT_VOLT").is_some() {
        let crit_level =
            Level { param: "BATT_CRT_VOLT", action: "BATT_FS_CRT_ACT", what: "critical", value: crit, bounds: (crit_cell - 0.3, low_cell), act: (1.0, "land") };
        check_level(p, n, chem, &crit_level, items, changes);
    }
}

/// One battery failsafe level: its voltage and action parameters, the
/// suggested voltage, acceptable volts per cell and the suggested action.
struct Level {
    param: &'static str,
    action: &'static str,
    what: &'static str,
    value: f64,
    bounds: (f64, f64),
    act: (f64, &'static str),
}

fn check_level(p: Param, cells: f64, chem: Chemistry, l: &Level, items: &mut Vec<Item>, changes: &mut Vec<ParamChange>) {
    // Vehicles store these as f32 (24.8 comes back as 24.799999).
    let now = (p(l.param).unwrap_or(0.0) * 1000.0).round() / 1000.0;
    let per_cell = now / cells;
    let what = l.what;
    if now == 0.0 || per_cell < l.bounds.0 || per_cell > l.bounds.1 {
        let why = if now == 0.0 {
            format!("{} isn't set", l.param)
        } else if per_cell < l.bounds.0 {
            format!("{} = {} V is {per_cell:.2} V per cell: too low for the {what} level on {}", l.param, format_value(now), chem.name())
        } else {
            format!("{} = {} V is {per_cell:.2} V per cell: it would trigger with a healthy pack", l.param, format_value(now))
        };
        items.push(item(Status::Warn, why, format!("{} V ({:.1} V per cell) is typical for the {what} level.", format_value(l.value), l.value / cells)));
        changes.extend(change(p, l.param, l.value, format!("{} failsafe at {:.1} V per cell", capitalise(what), l.value / cells)));
    } else {
        items.push(item(Status::Pass, format!("{} level at {} V ({per_cell:.2} V per cell)", capitalise(what), format_value(now)), ""));
    }
    if int(p, l.action) == Some(0) {
        let (value, name) = l.act;
        items.push(item(
            Status::Warn,
            format!("Nothing happens at the {what} level ({} = 0, warning only)", l.action),
            format!("{} = {} makes the vehicle {name}.", l.action, format_value(value)),
        ));
        changes.extend(change(p, l.action, value, format!("{} when the battery reaches the {what} level", capitalise(name))));
    }
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    c.next().map_or_else(String::new, |f| f.to_uppercase().collect::<String>() + c.as_str())
}

// ---------- 5. Battery monitor ----------

fn battery(p: Param, live: &Live) -> Step {
    let mut items = Vec::new();
    match int(p, "BATT_MONITOR") {
        Some(0) => items.push(item(
            Status::Fail,
            "No battery monitor",
            "Battery voltage isn't measured, so there is no low-battery failsafe. Most flight controllers use BATT_MONITOR = 4 (voltage and current).",
        )),
        Some(m) => {
            let kind = match m {
                3 => "voltage only",
                4 => "voltage and current",
                5 => "solder-on module",
                8 => "DroneCAN",
                9 => "ESC telemetry",
                _ => "configured",
            };
            items.push(item(Status::Pass, format!("Battery monitor: {kind} (BATT_MONITOR {m})"), ""));
            match live.battery_v {
                Some(v) if v < 1.0 => items.push(item(
                    Status::Warn,
                    format!("It reads {v:.1} V"),
                    "Connect the flight battery. If it is connected, the voltage pin or multiplier (BATT_VOLT_PIN, BATT_VOLT_MULT) is wrong.",
                )),
                Some(v) => items.push(item(
                    Status::Info,
                    format!("It reads {v:.2} V"),
                    "Compare with a voltmeter on the battery; adjust BATT_VOLT_MULT if they differ by more than 0.1 V.",
                )),
                None => {}
            }
            if m == 4 && p("BATT_CAPACITY") == Some(0.0) {
                items.push(item(Status::Info, "Battery capacity not set", "Set BATT_CAPACITY (mAh) so remaining capacity is shown."));
            }
        }
        None => {}
    }
    step("battery", "Battery monitor", "The battery voltage drives the low-battery failsafe and tells you when to land.", items, vec![])
}

// ---------- 6. Sensors ----------

const SENSORS_WHY: &str = "Calibrated accelerometers and compass give a correct attitude and heading.";

fn sensors(p: Param, live: &Live) -> Step {
    let mut items = Vec::new();
    let offs = ["X", "Y", "Z"].map(|a| p(&format!("INS_ACCOFFS_{a}")).unwrap_or(0.0));
    let scal = ["X", "Y", "Z"].map(|a| p(&format!("INS_ACCSCAL_{a}")).unwrap_or(1.0));
    if p("INS_ACCOFFS_X").is_none() {
        // Can't tell without the calibration parameters.
    } else if offs.iter().all(|v| *v == 0.0) && scal.iter().all(|v| *v == 1.0) {
        items.push(item(
            Status::Fail,
            "Accelerometer not calibrated",
            "Calibrate it in a ground station (six positions), with the vehicle level for the first one.",
        ));
    } else {
        items.push(item(Status::Pass, "Accelerometer calibrated", ""));
    }

    if p("COMPASS_ENABLE") == Some(0.0) {
        items.push(item(Status::Info, "Compass disabled", "Fine for flying without GPS modes; Loiter, RTL and other GPS modes need a compass."));
    } else {
        if p("COMPASS_DEV_ID").is_none() {
            return step("sensors", "Sensors and calibration", SENSORS_WHY, items, vec![]);
        }
        let found: Vec<&str> = ["", "2", "3"].into_iter().filter(|s| p(&format!("COMPASS_DEV_ID{s}")).is_some_and(|v| v != 0.0)).collect();
        if found.is_empty() {
            items.push(item(Status::Warn, "No compass found", "GPS modes need a compass. Check the GPS/compass module's I2C wires (SDA, SCL)."));
        }
        for s in found {
            let n = if s.is_empty() { "1".to_owned() } else { s.to_owned() };
            let ofs = ["X", "Y", "Z"].map(|a| p(&format!("COMPASS_OFS{s}_{a}")).unwrap_or(0.0));
            let external = p(&format!("COMPASS_EXTERN{}", if s.is_empty() { "AL" } else { s })) == Some(1.0);
            let used = p(&format!("COMPASS_USE{s}")) != Some(0.0);
            let kind = if external { "external" } else { "internal" };
            if !used {
                items.push(item(Status::Info, format!("Compass {n} ({kind}) is not used"), ""));
            } else if ofs.iter().all(|v| *v == 0.0) {
                items.push(item(
                    Status::Fail,
                    format!("Compass {n} ({kind}) not calibrated"),
                    "Calibrate outdoors with the battery mounted, away from metal (ArduTune's Compass calibration), then reboot.",
                ));
            } else {
                let size = ofs.iter().map(|v| v * v).sum::<f64>().sqrt();
                let max = p("COMPASS_OFFS_MAX").unwrap_or(1800.0);
                let orient = format_value(p(&format!("COMPASS_ORIENT{s}")).unwrap_or(0.0));
                if size > 0.8 * max {
                    items.push(item(
                        Status::Warn,
                        format!("Compass {n} ({kind}) has large offsets ({size:.0} of {max:.0} allowed)"),
                        "Strong magnetic interference near it: move it away from power wires and metal, then recalibrate.",
                    ));
                } else {
                    items.push(item(
                        Status::Pass,
                        format!("Compass {n} ({kind}) calibrated"),
                        if external {
                            format!("Orientation COMPASS_ORIENT{s} = {orient}: check it matches how the module is mounted (0 = arrow forward, flat).")
                        } else {
                            String::new()
                        },
                    ));
                }
            }
        }
    }

    if let Some((_, enabled, healthy)) = live.sensors {
        let bad: Vec<&str> = SENSOR_NAMES.iter().filter(|(b, _)| enabled & b != 0 && healthy & b == 0).map(|(_, n)| *n).collect();
        if !bad.is_empty() {
            items.push(item(Status::Fail, format!("Reported unhealthy: {}", bad.join(", ")), "Reboot with the vehicle still; recalibrate if it persists."));
        }
    }
    step("sensors", "Sensors and calibration", SENSORS_WHY, items, vec![])
}

// ---------- 7. Logging for tuning ----------

/// Whether pre- and post-filter gyro logging is set up (raw IMU or batch sampler).
pub(crate) fn gyro_logging(p: Param) -> (bool, String) {
    let raw = int(p, "INS_RAW_LOG_OPT");
    let bat_mask = int(p, "INS_LOG_BAT_MASK").unwrap_or(0);
    let bat_opt = int(p, "INS_LOG_BAT_OPT").unwrap_or(0);
    let raw_ok = raw.is_some_and(|o| o & (1 << RAW_LOG_OPT_PRE_POST_BIT) != 0 && o & 0b11 != 0);
    let bat_ok = bat_mask != 0 && bat_opt & (1 << BAT_OPT_PRE_POST_BIT) != 0;
    if raw_ok {
        (true, format!("Raw gyro logged before and after the filters (INS_RAW_LOG_OPT = {})", raw.unwrap_or(0)))
    } else if bat_ok {
        (true, format!("Batch sampler logs gyro before and after the filters (INS_LOG_BAT_MASK = {bat_mask}, INS_LOG_BAT_OPT = {bat_opt})"))
    } else if bat_mask != 0 {
        (false, "The batch sampler logs only after the filters: the noise the filters remove can't be analysed".to_owned())
    } else {
        (false, "High-rate gyro logging is off: no noise spectrum for filter tuning".to_owned())
    }
}

fn logging(p: Param, live: &Live) -> Step {
    let mut items = Vec::new();
    let backend = int(p, "LOG_BACKEND_TYPE").unwrap_or(1);
    let failed = live.messages.iter().any(|m| {
        let m = m.to_ascii_lowercase();
        m.contains("logging failed") || m.contains("no sd card") || m.contains("logging not started")
    });
    let health = live.sensors.and_then(|(_, enabled, healthy)| (enabled & SENSOR_LOGGING != 0).then_some(healthy & SENSOR_LOGGING != 0));
    if backend == 0 {
        items.push(item(Status::Fail, "Logging is off (LOG_BACKEND_TYPE = 0)", "Set LOG_BACKEND_TYPE = 1 to log to the SD card."));
    } else if backend & 1 == 0 {
        items.push(item(
            Status::Warn,
            "Not logging to an SD card",
            "Onboard flash or MAVLink logging is too small or slow for high-rate gyro logs. Use an SD card (LOG_BACKEND_TYPE = 1) if the board has a slot.",
        ));
    } else if failed || health == Some(false) {
        items.push(item(
            Status::Fail,
            "Logging to the SD card fails",
            "Insert an SD card formatted FAT32 (32 GB or smaller is safest) and reboot: the card is only detected at power-up. Try another card if it persists.",
        ));
    } else {
        items.push(item(Status::Pass, "Logging to the SD card", if health == Some(true) { "The vehicle reports logging as healthy." } else { "" }));
    }

    let mask = int(p, "LOG_BITMASK").unwrap_or(0) as u32;
    let missing: Vec<&str> = REQUIRED_LOG_BITS.iter().filter(|(b, _)| mask & (1 << b) == 0).map(|(_, n)| *n).collect();
    if missing.is_empty() {
        items.push(item(
            Status::Pass,
            "LOG_BITMASK has everything PID tuning needs",
            "Attitude at loop rate (ATTITUDE_FAST), PID, IMU, motor outputs and throttle.",
        ));
    } else {
        items.push(item(
            Status::Fail,
            format!("LOG_BITMASK is missing {}", missing.join(", ")),
            "Without these the step response and hover analysis can't run.",
        ));
    }

    let (gyro_ok, text) = gyro_logging(p);
    items.push(item(
        if gyro_ok { Status::Pass } else { Status::Fail },
        text,
        if gyro_ok { "Enough for the noise spectrum and filter simulation." } else { "Needed for the noise and filter analysis." },
    ));
    if p("INS_RAW_LOG_OPT").is_some_and(|v| v != 0.0) && p("INS_LOG_BAT_MASK").is_some_and(|v| v != 0.0) {
        items.push(item(Status::Warn, "Raw IMU logging and the batch sampler are both on", "One is enough; both make logs larger and can drop data."));
    }
    if gyro_ok {
        items.push(item(Status::Info, "Turn high-rate gyro logging off when tuning is finished", "It makes large logs (tens of MB per flight)."));
    }
    if p("LOG_DISARMED") == Some(1.0) {
        items.push(item(Status::Info, "Logging while disarmed is on", "Useful for setup; it fills the SD card faster."));
    }
    let changes = recommend_params(p, gyro_ok);
    step(
        "logging",
        "Logging for tuning",
        "Filter and PID tuning work from flight logs: they need gyro data before and after the filters and fast attitude/PID logging.",
        items,
        changes,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn params<S: AsRef<str>>(list: &[(S, f64)]) -> impl Fn(&str) -> Option<f64> {
        let map: HashMap<String, f64> = list.iter().map(|(k, v)| (k.as_ref().to_owned(), *v)).collect();
        move |n| map.get(n).copied()
    }

    /// The KakuteH7 quad from 2026-11-11, before RSSI_TYPE was fixed.
    fn kakute() -> Vec<(String, f64)> {
        let mut v: Vec<(String, f64)> = [
            ("FRAME_CLASS", 1.0),
            ("FRAME_TYPE", 18.0),
            ("MOT_PWM_TYPE", 6.0),
            ("MOT_SPIN_ARM", 0.1),
            ("MOT_SPIN_MIN", 0.15),
            ("SERVO_BLH_BDMASK", 15.0),
            ("SERVO_BLH_POLES", 14.0),
            ("SERIAL4_PROTOCOL", 5.0),
            ("SERIAL6_PROTOCOL", 23.0),
            ("SERIAL7_PROTOCOL", 16.0),
            ("RC_OPTIONS", 8480.0),
            ("RSSI_TYPE", 5.0),
            ("FLTMODE_CH", 6.0),
            ("RC5_OPTION", 153.0),
            ("ARMING_RUDDER", 2.0),
            ("FS_THR_ENABLE", 1.0),
            ("FS_THR_VALUE", 975.0),
            ("RCMAP_THROTTLE", 3.0),
            ("BATT_MONITOR", 4.0),
            ("BATT_FS_LOW_ACT", 0.0),
            ("BATT_LOW_VOLT", 10.5),
            ("BATT_CRT_VOLT", 0.0),
            ("BATT_FS_CRT_ACT", 0.0),
            ("MOT_BAT_VOLT_MAX", 32.8),
            ("MOT_BAT_VOLT_MIN", 22.4),
            ("GPS1_TYPE", 1.0),
            ("BATT_LOW_MAH", 0.0),
            ("BATT_CAPACITY", 1300.0),
            ("FS_EKF_ACTION", 1.0),
            ("FS_CRASH_CHECK", 1.0),
            ("INS_ACCOFFS_X", 0.017),
            ("INS_ACCOFFS_Y", -0.01),
            ("INS_ACCOFFS_Z", 0.19),
            ("COMPASS_ENABLE", 1.0),
            ("COMPASS_DEV_ID", 855297.0),
            ("COMPASS_EXTERNAL", 1.0),
            ("COMPASS_USE", 1.0),
            ("COMPASS_OFS_X", -38.6),
            ("COMPASS_OFS_Y", 117.8),
            ("COMPASS_OFS_Z", -129.7),
            ("COMPASS_ORIENT", 0.0),
            ("LOG_BACKEND_TYPE", 1.0),
            ("LOG_BITMASK", 2242303.0),
            ("LOG_DISARMED", 0.0),
            ("INS_RAW_LOG_OPT", 0.0),
            ("INS_LOG_BAT_MASK", 0.0),
            ("INS_LOG_BAT_OPT", 0.0),
            ("INS_HNTCH_ENABLE", 1.0),
            ("INS_HNTCH_MODE", 3.0),
            ("INS_HNTCH_FREQ", 80.0),
            ("INS_HNTCH_REF", 0.0),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_owned(), v))
        .collect();
        for m in 1..=4 {
            v.push((format!("SERVO{m}_FUNCTION"), 32.0 + m as f64));
            v.push((format!("RC{m}_MIN"), 988.0));
            v.push((format!("RC{m}_MAX"), 2011.0));
        }
        for (i, m) in [0.0, 0.0, 2.0, 2.0, 5.0, 5.0].into_iter().enumerate() {
            v.push((format!("FLTMODE{}", i + 1), m));
        }
        v
    }

    fn set(p: &mut Vec<(String, f64)>, name: &str, value: f64) {
        p.retain(|(n, _)| n != name);
        p.push((name.to_owned(), value));
    }

    fn step<'a>(g: &'a Guide, id: &str) -> &'a Step {
        g.steps.iter().find(|s| s.id == id).unwrap()
    }

    #[test]
    fn kakute_setup() {
        let live = Live {
            sensors: Some((1 << 16 | 1 << 24, 1 << 16 | 1 << 24, 1 << 16 | 1 << 24)),
            battery_v: Some(16.4),
            messages: vec![],
            rc: Some([1500, 1500, 988, 1500, 999, 999, 2011, 999]),
        };
        let mut p = kakute();
        p.push(("RC7_OPTION".to_owned(), 65.0));
        let g = assess(params(&p), &live);
        assert!(step(&g, "radio").items.iter().any(|i| i.title == "GPS off switch on channel 7" && i.detail.contains("the GPS is off")));
        let g = assess(params(&kakute()), &live);
        assert_eq!(g.steps.len(), 7);
        assert_eq!(step(&g, "frame").status, Status::Pass);
        let esc = step(&g, "esc");
        assert_eq!(esc.status, Status::Warn, "REF 0: the notch doesn't track");
        assert_eq!((esc.changes[0].name.as_str(), esc.changes[0].value), ("INS_HNTCH_REF", 1.0));

        let radio = step(&g, "radio");
        assert_eq!(radio.status, Status::Warn);
        assert_eq!(radio.changes.len(), 1);
        assert_eq!((radio.changes[0].name.as_str(), radio.changes[0].value), ("RSSI_TYPE", 3.0));
        assert!(radio.items.iter().any(|i| i.title.contains("Stabilize, AltHold, Loiter")));

        let fs = step(&g, "failsafes");
        assert_eq!(fs.status, Status::Warn, "no low-battery action");
        assert!(fs.items.iter().any(|i| i.title.contains("8S Li-ion")), "{:?}", fs.items);
        let got: Vec<(&str, f64)> = fs.changes.iter().map(|c| (c.name.as_str(), c.value)).collect();
        assert_eq!(got, [("BATT_LOW_VOLT", 24.8), ("BATT_FS_LOW_ACT", 2.0), ("BATT_CRT_VOLT", 22.4), ("BATT_FS_CRT_ACT", 1.0)]);

        let logging = step(&g, "logging");
        assert_eq!(logging.status, Status::Fail, "no gyro logging");
        let names: Vec<&str> = logging.changes.iter().map(|c| c.name.as_str()).collect();
        assert!(names.contains(&"INS_RAW_LOG_OPT"), "{names:?}");

        assert_eq!(step(&g, "sensors").status, Status::Pass, "{:?}", step(&g, "sensors").items);
    }

    #[test]
    fn battery_failsafe_by_pack() {
        // 8S Li-ion with the low level at 20 V (2.5 V per cell): too low.
        let mut p = kakute();
        set(&mut p, "BATT_LOW_VOLT", 20.0);
        set(&mut p, "BATT_FS_LOW_ACT", 2.0);
        let g = assess(params(&p), &Live::default());
        let fs = step(&g, "failsafes");
        assert!(fs.items.iter().any(|i| i.title.contains("2.50 V per cell: too low")), "{:?}", fs.items);
        assert!(fs.changes.iter().any(|c| c.name == "BATT_LOW_VOLT" && c.value == 24.8));
        assert!(!fs.changes.iter().any(|c| c.name == "BATT_FS_LOW_ACT"), "action already set");

        // A good setup passes.
        set(&mut p, "BATT_LOW_VOLT", 25.0);
        set(&mut p, "BATT_CRT_VOLT", 22.6);
        set(&mut p, "BATT_FS_CRT_ACT", 1.0);
        let g = assess(params(&p), &Live::default());
        assert!(step(&g, "failsafes").changes.is_empty(), "{:?}", step(&g, "failsafes").items);

        // 4S LiPo: 3.5 / 3.3 V per cell; land instead of RTL without GPS.
        let lipo = params(&[
            ("MOT_BAT_VOLT_MAX", 16.8),
            ("MOT_BAT_VOLT_MIN", 13.2),
            ("BATT_FS_LOW_ACT", 0.0),
            ("BATT_LOW_VOLT", 0.0),
            ("BATT_CRT_VOLT", 0.0),
            ("BATT_FS_CRT_ACT", 0.0),
            ("GPS1_TYPE", 0.0),
        ]);
        let g = assess(lipo, &Live::default());
        let got: Vec<(String, f64)> = step(&g, "failsafes").changes.iter().map(|c| (c.name.clone(), c.value)).collect();
        assert_eq!(
            got,
            [("BATT_LOW_VOLT".to_owned(), 14.0), ("BATT_FS_LOW_ACT".to_owned(), 1.0), ("BATT_CRT_VOLT".to_owned(), 13.2), ("BATT_FS_CRT_ACT".to_owned(), 1.0)]
        );

        // Unknown pack: advice only, no numbers.
        let unknown = params(&[("BATT_FS_LOW_ACT", 0.0), ("BATT_LOW_VOLT", 0.0), ("BATT_LOW_MAH", 0.0)]);
        let g = assess(unknown, &Live::default());
        assert!(step(&g, "failsafes").changes.is_empty());
        assert_eq!(step(&g, "failsafes").status, Status::Warn);
    }

    #[test]
    fn logging_ready_and_sd_failures() {
        let mut p = kakute();
        set(&mut p, "INS_RAW_LOG_OPT", 9.0);
        let ok = Live { sensors: Some((1 << 24, 1 << 24, 1 << 24)), ..Default::default() };
        let g = assess(params(&p), &ok);
        let logging = step(&g, "logging");
        assert!(logging.changes.is_empty(), "{:?}", logging.changes);
        assert_ne!(logging.status, Status::Fail, "{:?}", logging.items);

        let failing = Live { messages: vec!["PreArm: Logging failed".into()], ..Default::default() };
        let g = assess(params(&p), &failing);
        assert_eq!(step(&g, "logging").status, Status::Fail);
        assert!(step(&g, "logging").items[0].title.contains("SD card fails"));
    }

    #[test]
    fn finds_basic_setup_problems() {
        let p = params(&[
            ("FRAME_CLASS", 0.0),
            ("RC1_MIN", 1100.0),
            ("RC1_MAX", 1900.0),
            ("RC2_MIN", 1100.0),
            ("RC2_MAX", 1900.0),
            ("RC3_MIN", 1100.0),
            ("RC3_MAX", 1900.0),
            ("RC4_MIN", 1100.0),
            ("RC4_MAX", 1900.0),
            ("FS_THR_ENABLE", 0.0),
            ("BATT_MONITOR", 0.0),
            ("INS_ACCOFFS_X", 0.0),
            ("FS_CRASH_CHECK", 0.0),
        ]);
        let g = assess(p, &Live::default());
        assert_eq!(step(&g, "frame").status, Status::Fail);
        assert!(step(&g, "radio").items.iter().any(|i| i.title.contains("isn't calibrated")));
        let fs = step(&g, "failsafes");
        assert_eq!(fs.status, Status::Fail);
        assert_eq!(fs.changes.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["FS_THR_ENABLE", "FS_CRASH_CHECK"]);
        assert_eq!(step(&g, "battery").status, Status::Fail);
        assert_eq!(step(&g, "sensors").status, Status::Fail);
    }
}
