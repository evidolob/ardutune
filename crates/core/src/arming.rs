//! Why a vehicle won't arm: ArduPilot's pre-arm and arm messages grouped
//! by cause with advice, the GPS state and setup, and how this vehicle is
//! armed (switch or sticks). Input is what the vehicle reported live plus its
//! parameters; Copter only.

use serde::Serialize;

use crate::diagnose::mode_name;
use crate::readiness::Status;

/// What the vehicle reported while its pre-arm checks ran.
#[derive(Debug, Clone, Default)]
pub struct Snapshot {
    /// Status texts in the order received, without repeats.
    pub messages: Vec<String>,
    pub gps: Vec<Gps>,
    /// SYS_STATUS sensor bits: (present, enabled, healthy).
    pub sensors: Option<(u32, u32, u32)>,
    pub mode: Option<u32>,
    pub armed: bool,
    /// RC input channels 1-8 in µs (0 when not received).
    pub rc: Option<[u16; 8]>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Gps {
    pub instance: u8,
    /// MAVLink GPS_FIX_TYPE: 0 no GPS, 1 no fix, 2 2D, 3 3D, 4+ better.
    pub fix_type: u8,
    pub satellites: Option<u8>,
    pub hdop: Option<f32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Report {
    pub armed: bool,
    /// Nothing reported that blocks arming.
    pub ready: bool,
    pub mode: Option<String>,
    /// Most serious first.
    pub problems: Vec<Problem>,
    pub gps: Vec<GpsView>,
    /// How this vehicle is armed, with the current switch state.
    pub how_to_arm: Vec<String>,
    /// Every message the vehicle sent, as received.
    pub messages: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Problem {
    pub status: Status,
    pub area: &'static str,
    pub title: String,
    /// The vehicle's own messages behind this problem.
    pub messages: Vec<String>,
    pub advice: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct GpsView {
    pub instance: u8,
    pub fix: &'static str,
    pub satellites: Option<u8>,
    pub hdop: Option<f32>,
}

/// Copter modes that need a position estimate (GPS) to arm in.
const POSITION_MODES: [u32; 14] = [3, 4, 5, 6, 7, 11, 16, 17, 18, 19, 21, 23, 24, 27];
const SENSOR_NAMES: [(u32, &str); 14] = [
    (1, "gyro"),
    (1 << 1, "accelerometer"),
    (1 << 2, "compass"),
    (1 << 3, "barometer"),
    (1 << 5, "GPS"),
    (1 << 15, "motor outputs"),
    (1 << 16, "RC receiver"),
    (1 << 17, "second gyro"),
    (1 << 18, "second accelerometer"),
    (1 << 19, "second compass"),
    (1 << 21, "attitude estimate (AHRS)"),
    (1 << 24, "logging"),
    (1 << 25, "battery"),
    (1 << 26, "proximity sensor"),
];
const ARM_SWITCH_OPTIONS: [f64; 3] = [41.0, 153.0, 154.0];

/// Areas in the order they should be fixed: a bad compass can also upset
/// the attitude estimate, so it comes first.
const AREAS: [&str; 10] = ["Compass", "Attitude", "Radio", "Logging", "GPS", "Battery", "Safety switch", "Motors", "Throttle", "Other"];

pub fn assess(s: &Snapshot, param: impl Fn(&str) -> Option<f64>) -> Report {
    let mut problems: Vec<Problem> = Vec::new();

    // The vehicle's own reasons, grouped by area.
    let reasons: Vec<&str> = s.messages.iter().filter_map(|m| reason(m)).collect();
    for area in AREAS {
        let msgs: Vec<String> = latest_variants(reasons.iter().copied().filter(|r| area_of(r) == area));
        if msgs.is_empty() {
            continue;
        }
        problems.push(Problem {
            status: Status::Fail,
            area,
            title: title_for(area, &msgs),
            advice: advice_for(area, &msgs, &param),
            messages: msgs,
        });
    }

    let gps = s.gps.iter().map(|g| GpsView { instance: g.instance, fix: fix_name(g.fix_type), satellites: g.satellites, hdop: g.hdop }).collect();
    let mode_needs_gps = s.mode.is_some_and(|m| POSITION_MODES.contains(&m));
    if let Some(p) = gps_problem(s, &param, mode_needs_gps, problems.iter().any(|p| p.area == "GPS")) {
        match problems.iter_mut().find(|q| q.area == "GPS") {
            Some(q) => {
                q.title = p.title;
                q.advice = format!("{} {}", p.advice, q.advice);
            }
            None => problems.push(p),
        }
    }
    if let (Some(mode), true) = (s.mode, mode_needs_gps) {
        if !gps_good(s, &param) {
            problems.push(Problem {
                status: Status::Warn,
                area: "Mode",
                title: format!("Arming in {}, which needs a GPS position", mode_name(mode)),
                messages: vec![],
                advice: "Switch to Stabilize or AltHold to arm without a GPS fix.".to_owned(),
            });
        }
    }
    problems.extend(rc_problems(s, &param));
    if let Some(p) = unhealthy(s) {
        problems.push(p);
    }
    problems.sort_by_key(|p| std::cmp::Reverse(rank(p.status)));

    let ready = !s.armed && !problems.iter().any(|p| p.status == Status::Fail);
    Report {
        armed: s.armed,
        ready,
        mode: s.mode.map(mode_name),
        problems,
        gps,
        how_to_arm: how_to_arm(s, &param),
        messages: s.messages.clone(),
    }
}

fn rank(s: Status) -> u8 {
    match s {
        Status::Fail => 3,
        Status::Warn => 2,
        Status::Info => 1,
        Status::Pass => 0,
    }
}

/// The reason in a "PreArm: …" or "Arm: …" message.
fn reason(message: &str) -> Option<&str> {
    message.strip_prefix("PreArm: ").or_else(|| message.strip_prefix("Arm: ")).map(str::trim)
}

/// Keeps one message per kind, the latest: numbers in brackets change from
/// one report to the next ("xy diff:262>100", "xy diff:117>100").
fn latest_variants<'a>(reasons: impl Iterator<Item = &'a str>) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for r in reasons {
        let key = shape(r);
        match out.iter_mut().find(|o| shape(o) == key) {
            Some(o) => *o = r.to_owned(),
            None => out.push(r.to_owned()),
        }
    }
    out
}

fn shape(s: &str) -> String {
    s.chars().filter(|c| !c.is_ascii_digit() && *c != '.').collect::<String>().replace(" Wait or reboot", "")
}

fn area_of(reason: &str) -> &'static str {
    let r = reason.to_ascii_lowercase();
    let has = |words: &[&str]| words.iter().any(|w| r.contains(w));
    if has(&["mag field", "compass", "mag anomaly", "magnetometer"]) {
        "Compass"
    } else if has(&["ahrs", "dcm", "accels", "gyros", "ins:", "accel", "gyro", "imu"]) {
        "Attitude"
    } else if has(&["rc ", "rc:", "radio", "receiver", "throttle below failsafe"]) || r.starts_with("rc") {
        "Radio"
    } else if has(&["logging", "sd card", "log "]) {
        "Logging"
    } else if has(&["gps", "position", "hdop", "ekf", "fence", "home"]) {
        "GPS"
    } else if has(&["battery", "voltage", "batt"]) {
        "Battery"
    } else if has(&["safety"]) {
        "Safety switch"
    } else if has(&["motor", "frame", "servo", "esc"]) {
        "Motors"
    } else if has(&["throttle"]) {
        "Throttle"
    } else {
        "Other"
    }
}

fn title_for(area: &str, msgs: &[String]) -> String {
    let any = |w: &str| msgs.iter().any(|m| m.to_ascii_lowercase().contains(w));
    match area {
        "Compass" => "The compass reading doesn't match the Earth's field here".to_owned(),
        "Attitude" if any("inconsistent") && any("dcm") => "The two attitude estimates disagree".to_owned(),
        "Attitude" => "Accelerometer or gyro check failed".to_owned(),
        "Radio" if any("not found") => "No RC receiver signal".to_owned(),
        "Radio" if any("calibrating") => "The flight controller thinks a radio calibration is running".to_owned(),
        "Radio" => "Radio check failed".to_owned(),
        "Logging" => "Logging to the SD card failed".to_owned(),
        "GPS" => "No usable GPS position".to_owned(),
        "Battery" => "Battery check failed".to_owned(),
        "Safety switch" => "The safety switch is on".to_owned(),
        "Motors" => "Motor or frame setup check failed".to_owned(),
        "Throttle" => "Throttle check failed".to_owned(),
        _ => msgs.first().cloned().unwrap_or_default(),
    }
}

fn advice_for(area: &str, msgs: &[String], param: &impl Fn(&str) -> Option<f64>) -> String {
    let any = |w: &str| msgs.iter().any(|m| m.to_ascii_lowercase().contains(w));
    match area {
        "Compass" => {
            let orient = param("COMPASS_ORIENT").map_or(String::new(), |o| {
                if o == 0.0 {
                    " COMPASS_ORIENT is 0, so the GPS/compass arrow must point forward.".to_owned()
                } else {
                    format!(" COMPASS_ORIENT is {o}; check it matches how the compass is mounted.")
                }
            });
            format!(
                "Move away from metal, cars, laptops and phones and check again. If it still fails, calibrate the \
                 compass outdoors with the battery mounted as it flies, then reboot.{orient} If a good calibration \
                 still fails, the compass is too close to power wiring: raise or move it."
            )
        }
        "Attitude" if any("dcm") || any("ahrs") => "Reboot with the vehicle level and still. A bad compass can also tilt \
            the EKF's attitude, so fix any compass problem first. If it persists, calibrate the accelerometers."
            .to_owned(),
        "Attitude" => "Calibrate the accelerometers (with the vehicle level for the level step) and reboot. \
            \"Inconsistent\" errors that remain after that point to a faulty IMU or heavy vibration on the bench."
            .to_owned(),
        "Radio" if any("not found") => "Power the receiver (many boards only power it from the battery), check it's \
            bound (solid LED) and that its TX wire goes to the RX pad of the serial port set to RC input \
            (SERIALn_PROTOCOL = 23)."
            .to_owned(),
        "Radio" if any("calibrating") => "Close any ground station radio calibration page (Mission Planner, \
            QGroundControl), then reboot the flight controller."
            .to_owned(),
        "Radio" => "Calibrate the radio in a ground station and check the throttle failsafe value (FS_THR_VALUE) is \
            below the lowest throttle your transmitter sends."
            .to_owned(),
        "Logging" => "Insert an SD card formatted FAT32 (32 GB or smaller is safest) and reboot: the card is only \
            detected at power-up. If it still fails, try another card."
            .to_owned(),
        "GPS" => "".to_owned(),
        "Battery" => "Connect the flight battery. If it is connected and charged, check the battery monitor \
            (BATT_MONITOR, voltage calibration) and the arming voltage (BATT_ARM_VOLT)."
            .to_owned(),
        "Safety switch" => "Press the safety switch on the GPS until its LED is solid, or turn the switch off with \
            BRD_SAFETY_DEFLT = 0 if there is none."
            .to_owned(),
        "Motors" => "Check FRAME_CLASS and FRAME_TYPE, the motor outputs (SERVOn_FUNCTION) and the ESC protocol \
            (MOT_PWM_TYPE)."
            .to_owned(),
        "Throttle" => "Put the throttle stick all the way down before arming.".to_owned(),
        _ => "See ArduPilot's pre-arm check documentation for this message.".to_owned(),
    }
}

fn fix_name(fix: u8) -> &'static str {
    match fix {
        0 => "no GPS",
        1 => "no fix",
        2 => "2D fix",
        3 => "3D fix",
        4 => "DGPS",
        5 => "RTK float",
        6 => "RTK fixed",
        _ => "fix",
    }
}

fn gps_type(param: &impl Fn(&str) -> Option<f64>) -> Option<f64> {
    param("GPS1_TYPE").or_else(|| param("GPS_TYPE"))
}

fn gps_good(s: &Snapshot, param: &impl Fn(&str) -> Option<f64>) -> bool {
    let hdop_good = param("GPS_HDOP_GOOD").unwrap_or(140.0) / 100.0;
    s.gps.iter().any(|g| g.fix_type >= 3 && g.hdop.is_none_or(|h| f64::from(h) <= hdop_good))
}

/// Serial ports set to GPS, as "SERIAL4 (230400 baud)".
fn gps_ports(param: &impl Fn(&str) -> Option<f64>) -> Vec<String> {
    (0..10)
        .filter(|n| param(&format!("SERIAL{n}_PROTOCOL")) == Some(5.0))
        .map(|n| match param(&format!("SERIAL{n}_BAUD")) {
            Some(b) => format!("SERIAL{n} ({} baud)", baud(b)),
            None => format!("SERIAL{n}"),
        })
        .collect()
}

fn baud(code: f64) -> String {
    match code as i64 {
        1 => "1200".to_owned(),
        2 => "2400".to_owned(),
        4 => "4800".to_owned(),
        9 => "9600".to_owned(),
        19 => "19200".to_owned(),
        38 => "38400".to_owned(),
        57 => "57600".to_owned(),
        111 => "111100".to_owned(),
        115 => "115200".to_owned(),
        230 => "230400".to_owned(),
        256 => "256000".to_owned(),
        460 => "460800".to_owned(),
        500 => "500000".to_owned(),
        921 => "921600".to_owned(),
        1500 => "1500000".to_owned(),
        c => c.to_string(),
    }
}

/// GPS problems from its live state and setup. `blocking` is set when the
/// vehicle's own messages complain about GPS or position.
fn gps_problem(s: &Snapshot, param: &impl Fn(&str) -> Option<f64>, mode_needs_gps: bool, blocking: bool) -> Option<Problem> {
    let kind = gps_type(param);
    let needed = mode_needs_gps || blocking;
    let status = if needed { Status::Fail } else { Status::Warn };
    let problem = |status, title: String, advice: String| Some(Problem { status, area: "GPS", title, messages: vec![], advice });

    if kind == Some(0.0) {
        return problem(
            if needed { Status::Fail } else { Status::Info },
            "The GPS is turned off (GPS1_TYPE = 0)".to_owned(),
            "Set GPS1_TYPE (GPS_TYPE before 4.6) to 1 (auto) and reboot.".to_owned(),
        );
    }
    // DroneCAN, MAVLink, MSP and external AHRS GPS don't use a GPS serial port.
    let serial_gps = kind.is_none_or(|t| ![9.0, 14.0, 19.0, 21.0, 22.0, 23.0].contains(&t));
    let ports = gps_ports(param);
    if serial_gps && ports.is_empty() {
        return problem(
            status,
            "No serial port is set up for a GPS".to_owned(),
            "Find which SERIALn your GPS is wired to (the board's documentation maps UART pads to SERIAL numbers), \
             set SERIALn_PROTOCOL = 5 and reboot."
                .to_owned(),
        );
    }
    let gps = s.gps.iter().find(|g| g.instance == 0);
    // Fix type 0 (or no GPS message at all) means no GPS is talking.
    let detected = gps.is_some_and(|g| g.fix_type >= 1);
    let on = if ports.is_empty() { String::new() } else { format!(" on {}", ports.join(", ")) };
    if !detected {
        let advice = if kind == Some(9.0) {
            "Check the CAN wiring and that the CAN port is enabled for DroneCAN (CAN_P1_DRIVER = 1, \
             CAN_D1_PROTOCOL = 1), then reboot."
                .to_owned()
        } else {
            format!(
                "Check, in this order: the GPS has power (LED on; many boards power it only from the battery); its TX \
                 goes to the flight controller's RX and RX to TX; it is wired to the port set up for GPS{on}. With \
                 GPS1_TYPE = 1 the baud rate is found automatically."
            )
        };
        return problem(status, format!("No GPS detected{on}"), advice);
    }
    let g = gps.expect("detected");
    let sats = g.satellites.unwrap_or(0);
    let interference = "Common causes: indoors or under a roof, the module just powered up (the first fix can take \
        several minutes, longer after storage or a long trip), or interference from a video transmitter, USB 3 \
        devices or the flight controller itself: try with the VTX off and the GPS raised away from other electronics.";
    match g.fix_type {
        1 if sats == 0 => problem(
            status,
            "The GPS is connected but sees no satellites".to_owned(),
            format!("Go outside with a clear view of the sky and wait a few minutes. {interference}"),
        ),
        1 => problem(
            status,
            format!("The GPS sees {sats} satellite{} but has no fix yet", if sats == 1 { "" } else { "s" }),
            format!("Wait with a clear view of the sky. {interference}"),
        ),
        2 => problem(status, "Only a 2D fix".to_owned(), format!("A 3D fix needs more satellites. {interference}")),
        _ if !gps_good(s, param) => problem(
            status,
            format!("The fix is not accurate enough yet (HDOP {:.1})", g.hdop.unwrap_or(99.0)),
            format!(
                "ArduPilot wants HDOP below {:.1} (GPS_HDOP_GOOD). Wait a little longer with a clear sky view.",
                param("GPS_HDOP_GOOD").unwrap_or(140.0) / 100.0
            ),
        ),
        _ if blocking => problem(
            Status::Fail,
            "The GPS has a fix but the position estimate isn't ready".to_owned(),
            "Wait a minute for the EKF to settle. If a compass problem is listed, fix it first.".to_owned(),
        ),
        _ => None,
    }
}

fn rc_problems(s: &Snapshot, param: &impl Fn(&str) -> Option<f64>) -> Vec<Problem> {
    let Some(rc) = s.rc else { return vec![] };
    let mut out = Vec::new();
    let thr_ch = param("RCMAP_THROTTLE").unwrap_or(3.0) as usize;
    let Some(&thr) = rc.get(thr_ch.wrapping_sub(1)) else { return out };
    if thr == 0 {
        return out;
    }
    let thr = f64::from(thr);
    let min = param(&format!("RC{thr_ch}_MIN")).unwrap_or(1000.0);
    if param("FS_THR_ENABLE").unwrap_or(0.0) > 0.0 && param("FS_THR_VALUE").is_some_and(|fs| thr < fs) {
        out.push(Problem {
            status: Status::Fail,
            area: "Radio",
            title: format!("Throttle input ({thr:.0} µs) is below the failsafe value"),
            messages: vec![],
            advice: "Recalibrate the radio, or lower FS_THR_VALUE below your transmitter's lowest throttle.".to_owned(),
        });
    } else if thr > min + 50.0 {
        out.push(Problem {
            status: Status::Warn,
            area: "Throttle",
            title: format!("The throttle isn't at the bottom ({thr:.0} µs, minimum {min:.0})"),
            messages: vec![],
            advice: "Put the throttle stick all the way down to arm. If it is down, recalibrate the radio.".to_owned(),
        });
    }
    out
}

/// Sensors the vehicle uses that it reports as unhealthy.
fn unhealthy(s: &Snapshot) -> Option<Problem> {
    let (_, enabled, healthy) = s.sensors?;
    let bad: Vec<&str> =
        SENSOR_NAMES.iter().filter(|(bit, _)| enabled & bit != 0 && healthy & bit == 0).map(|(_, n)| *n).collect();
    if bad.is_empty() {
        return None;
    }
    Some(Problem {
        status: Status::Info,
        area: "Sensors",
        title: format!("Reported unhealthy: {}", bad.join(", ")),
        messages: vec![],
        advice: "These usually match the problems above; fixing those clears them.".to_owned(),
    })
}

fn how_to_arm(s: &Snapshot, param: &impl Fn(&str) -> Option<f64>) -> Vec<String> {
    let mut out = Vec::new();
    for ch in 1..=16 {
        let Some(opt) = param(&format!("RC{ch}_OPTION")) else { continue };
        if !ARM_SWITCH_OPTIONS.contains(&opt) {
            continue;
        }
        let now = s.rc.and_then(|rc| rc.get(ch - 1).copied()).filter(|v| *v > 0);
        let position = match now {
            Some(v) if v > 1800 => " It is on now: switch it off and on again, because arming needs the change.",
            Some(v) if v < 1200 => " It is off now.",
            _ => "",
        };
        out.push(format!("Arm switch on channel {ch}: flip it from low to high.{position}"));
    }
    match param("ARMING_RUDDER").unwrap_or(2.0) as i64 {
        1 => out.push("Sticks: throttle down and yaw right for 2 s (disarm by switch or ground station).".to_owned()),
        2 => out.push("Sticks: throttle down and yaw right for 2 s to arm, yaw left to disarm.".to_owned()),
        _ => {}
    }
    if out.is_empty() {
        out.push("No arm switch or stick arming is set up: arm from a ground station.".to_owned());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn params(list: &[(&str, f64)]) -> impl Fn(&str) -> Option<f64> {
        let map: HashMap<String, f64> = list.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect();
        move |n| map.get(n).copied()
    }

    /// The KakuteH7 from 2026-11-11: GPS indoors, then outside.
    fn kakute() -> Vec<(&'static str, f64)> {
        vec![
            ("GPS1_TYPE", 1.0),
            ("GPS_HDOP_GOOD", 140.0),
            ("SERIAL4_PROTOCOL", 5.0),
            ("SERIAL4_BAUD", 230.0),
            ("SERIAL6_PROTOCOL", 23.0),
            ("RC5_OPTION", 153.0),
            ("ARMING_RUDDER", 2.0),
            ("RCMAP_THROTTLE", 3.0),
            ("RC3_MIN", 988.0),
            ("FS_THR_ENABLE", 1.0),
            ("FS_THR_VALUE", 975.0),
            ("COMPASS_ORIENT", 0.0),
        ]
    }

    #[test]
    fn groups_arm_failures_by_cause_compass_first() {
        let s = Snapshot {
            messages: [
                "PreArm: Check mag field (xy diff:262>100)",
                "PreArm: AHRS: DCM Roll/Pitch inconsistent 20 deg. Wait or reboot",
                "PreArm: Logging failed",
                "PreArm: RC calibrating",
                "Arm: Check mag field (xy diff:117>100)",
            ]
            .map(str::to_owned)
            .to_vec(),
            gps: vec![Gps { instance: 0, fix_type: 3, satellites: Some(26), hdop: Some(0.55) }],
            mode: Some(0),
            rc: Some([1501, 1500, 988, 1499, 999, 999, 999, 999]),
            ..Default::default()
        };
        let r = assess(&s, params(&kakute()));
        assert!(!r.ready);
        let areas: Vec<&str> = r.problems.iter().map(|p| p.area).collect();
        assert_eq!(areas, ["Compass", "Attitude", "Radio", "Logging"]);
        assert_eq!(r.problems[0].messages, ["Check mag field (xy diff:117>100)"], "latest reading kept");
        assert!(r.problems[2].title.contains("radio calibration"));
        assert_eq!(r.mode.as_deref(), Some("Stabilize"));
        assert!(r.how_to_arm[0].starts_with("Arm switch on channel 5"));
        assert!(r.how_to_arm[0].contains("off now"));
    }

    #[test]
    fn explains_a_gps_without_satellites() {
        let s = Snapshot {
            gps: vec![Gps { instance: 0, fix_type: 1, satellites: Some(0), hdop: Some(99.99) }],
            mode: Some(5),
            ..Default::default()
        };
        let r = assess(&s, params(&kakute()));
        let gps = r.problems.iter().find(|p| p.area == "GPS").unwrap();
        assert_eq!(gps.status, Status::Fail, "Loiter needs GPS");
        assert!(gps.title.contains("no satellites"));
        assert!(r.problems.iter().any(|p| p.area == "Mode" && p.title.contains("Loiter")));

        // In Stabilize the same GPS state doesn't block arming.
        let r = assess(&Snapshot { mode: Some(0), ..s.clone() }, params(&kakute()));
        assert!(r.ready);
        assert_eq!(r.problems.iter().find(|p| p.area == "GPS").unwrap().status, Status::Warn);
    }

    #[test]
    fn finds_gps_setup_problems() {
        let missing = Snapshot { gps: vec![Gps { instance: 0, fix_type: 0, ..Default::default() }], mode: Some(5), ..Default::default() };
        let r = assess(&missing, params(&kakute()));
        assert!(r.problems[0].title.contains("No GPS detected on SERIAL4 (230400 baud)"), "{:?}", r.problems[0]);

        let mut p = kakute();
        p.retain(|(n, _)| *n != "SERIAL4_PROTOCOL");
        let r = assess(&missing, params(&p));
        assert!(r.problems[0].title.contains("No serial port"));

        let r = assess(&missing, params(&[("GPS1_TYPE", 0.0)]));
        assert!(r.problems[0].title.contains("turned off"));
    }

    #[test]
    fn ready_with_a_good_fix_and_no_messages() {
        let s = Snapshot {
            gps: vec![Gps { instance: 0, fix_type: 3, satellites: Some(29), hdop: Some(0.57) }],
            mode: Some(0),
            rc: Some([1500, 1500, 988, 1500, 1900, 999, 999, 999]),
            ..Default::default()
        };
        let r = assess(&s, params(&kakute()));
        assert!(r.ready);
        assert!(r.problems.is_empty(), "{:?}", r.problems);
        assert!(r.how_to_arm[0].contains("switch it off and on again"));
    }

    #[test]
    fn throttle_checks() {
        let base = Snapshot { mode: Some(0), ..Default::default() };
        let raised = Snapshot { rc: Some([1500, 1500, 1300, 1500, 0, 0, 0, 0]), ..base.clone() };
        assert!(assess(&raised, params(&kakute())).problems.iter().any(|p| p.area == "Throttle"));
        let low = Snapshot { rc: Some([1500, 1500, 950, 1500, 0, 0, 0, 0]), ..base };
        let r = assess(&low, params(&kakute()));
        assert!(r.problems.iter().any(|p| p.title.contains("below the failsafe")));
        assert!(!r.ready);
    }
}
