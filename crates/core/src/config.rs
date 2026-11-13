//! Basic vehicle configuration: frame, ESCs and motors, board orientation
//! and battery monitor on one page, with checks and a diagram of the
//! chosen quad layout.

use serde::Serialize;

use crate::form::{setting, Group};
use crate::motors::{self, Position};
use crate::readiness::Status;
use crate::setup::{frame_type_name, int, motor_count, Param};

#[derive(Debug, Clone, Serialize)]
pub struct Check {
    pub status: Status,
    pub title: String,
    pub detail: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct View {
    pub groups: Vec<Group>,
    pub checks: Vec<Check>,
    /// Quad layouts by FRAME_TYPE, for the diagram of the chosen one.
    pub previews: Vec<(i64, Vec<Position>)>,
    /// Bitmask of the outputs driving motors (SERVOn_FUNCTION 33…).
    pub motor_mask: u32,
}

fn check(status: Status, title: impl Into<String>, detail: impl Into<String>) -> Check {
    Check { status, title: title.into(), detail: detail.into() }
}

/// Outputs (bit n-1 for SERVOn) that drive motors.
fn motor_outputs(p: Param) -> u32 {
    (1..=16u32).filter(|ch| p(&format!("SERVO{ch}_FUNCTION")).is_some_and(|f| (33.0..=44.0).contains(&f))).fold(0, |m, ch| m | (1 << (ch - 1)))
}

fn outputs_list(mask: u32) -> String {
    let list: Vec<String> = (1..=16).filter(|ch| mask & (1 << (ch - 1)) != 0).map(|ch| ch.to_string()).collect();
    list.join(", ")
}

pub fn view(param: impl Fn(&str) -> Option<f64>) -> View {
    let p: Param = &param;
    let mask = motor_outputs(p);
    let class = int(p, "FRAME_CLASS").unwrap_or(0);
    let frame_type = int(p, "FRAME_TYPE").unwrap_or(-1);

    let mut frame = Vec::new();
    // Multirotor classes; helicopters and scripting frames are set up elsewhere.
    let classes: Vec<(f64, String)> = [
        (0.0, "Not set"),
        (1.0, "Quad"),
        (2.0, "Hexa"),
        (3.0, "Octa"),
        (4.0, "OctaQuad (X8)"),
        (5.0, "Y6"),
        (7.0, "Tri"),
        (12.0, "DodecaHexa"),
        (14.0, "Deca"),
    ]
    .iter()
    .map(|(v, n)| (*v, (*n).to_owned()))
    .collect();
    frame.extend(setting(p, "FRAME_CLASS", "Frame", "").map(|s| s.choices(classes)));
    frame.extend(setting(p, "FRAME_TYPE", "Layout", "").map(|s| {
        let named = s
            .options
            .iter()
            .map(|(v, n)| {
                let name = match *v as i64 {
                    1 => "X (props in)".to_owned(),
                    3 => "H (X, props out)".to_owned(),
                    12 => "BetaFlight X (props in)".to_owned(),
                    18 => "BetaFlight X reversed (props out)".to_owned(),
                    t if frame_type_name(t) != "other" => frame_type_name(t).to_owned(),
                    _ => n.clone(),
                };
                (*v, name)
            })
            .collect();
        s.choices(named).help("Props in: the front motors spin towards each other at the top. Props out: away from each other. The motor test confirms it.")
    }));

    let mut esc = Vec::new();
    esc.extend(
        setting(p, "MOT_PWM_TYPE", "ESC protocol", "").map(|s| s.help("DShot600 for current BLHeli_32, AM32 and Bluejay ESCs; Normal PWM for older ones.")),
    );
    esc.extend(
        setting(p, "SERVO_DSHOT_ESC", "ESC firmware", "").map(|s| s.help("Lets ArduPilot reverse motors in software and read extended telemetry (EDT).")),
    );
    if mask != 0 {
        let on = format!("On, motor outputs {}", outputs_list(mask));
        esc.extend(setting(p, "SERVO_BLH_BDMASK", "Bidirectional DShot", "").map(|s| {
            s.choices(vec![(0.0, "Off".to_owned()), (f64::from(mask), on)])
                .help("The ESCs report RPM on the signal wire, so the noise filter follows the motor speed. Needs ESC firmware that supports it (Bluejay, BLHeli_32, AM32).")
        }));
    }
    esc.extend(
        setting(p, "SERVO_BLH_POLES", "Motor poles", "magnets").map(|s| {
            s.help("Count the magnets in the bell: 14 on most 5-inch motors, 12 on many small ones, 24-28 on large ones. Wrong poles give wrong RPM.")
        }),
    );
    esc.extend(setting(p, "MOT_SPIN_ARM", "Spin when armed", "0-1").map(|s| s.help("Throttle the motors idle at once armed. Must be below Spin minimum.")));
    esc.extend(
        setting(p, "MOT_SPIN_MIN", "Spin minimum", "0-1")
            .map(|s| s.help("Lowest throttle in flight: high enough that no motor stops or stutters. About 0.03 above Spin when armed.")),
    );

    // ESC configuration passthrough: AM32, Bluejay and BLHeli tools reach the
    // ESCs through the flight controller's USB port.
    let mut passthrough = Vec::new();
    passthrough.extend(setting(p, "SERVO_BLH_AUTO", "ESC passthrough", "").map(|s| {
        s.choices(vec![(0.0, "Off".to_owned()), (1.0, "On for the motor outputs".to_owned())])
            .help("Lets ESC configuration tools (AM32 Configurator, esc-configurator.com for Bluejay, BLHeliSuite32) talk to the ESCs through the flight controller. Needs DShot.")
    }));
    passthrough.extend(
        setting(p, "SERVO_BLH_PORT", "Passthrough connection", "")
            .map(|s| s.help("Console is the USB port. A telemetry radio's MAVLink channel only if you configure the ESCs over the radio.")),
    );

    let mut board = Vec::new();
    board.extend(setting(p, "AHRS_ORIENTATION", "Board orientation", "").map(|s| {
        s.help("How the flight controller is mounted: None when its arrow points forward and it's the right way up. Recalibrate the accelerometer after changing it.")
    }));

    let mut battery = Vec::new();
    battery.extend(
        setting(p, "BATT_MONITOR", "Battery monitor", "").map(|s| s.help("Analog Voltage and Current for most flight controllers with a current sensor.")),
    );
    battery.extend(setting(p, "BATT_CAPACITY", "Capacity", "mAh"));

    let groups: Vec<Group> = [("Frame", frame), ("ESCs and motors", esc), ("ESC configuration", passthrough), ("Board", board), ("Battery", battery)]
        .into_iter()
        .filter(|(_, s)| !s.is_empty())
        .map(|(t, settings)| Group { title: t.to_owned(), settings })
        .collect();

    let mut checks = Vec::new();
    if class == 0 {
        checks.push(check(Status::Fail, "The frame isn't set", "Choose the frame and its layout, write them and reboot. Nothing arms until it's set."));
    }
    if class == 1 && frame_type == 19 {
        checks.push(check(
            Status::Fail,
            "Layout Y4 on a quad",
            "Y4 has two motors at the front and two stacked at the back. For a quad with four arms choose X or BetaFlight X (props in), or H or BetaFlight X reversed (props out).",
        ));
    }
    if let Some(n) = motor_count(class) {
        let have = mask.count_ones() as usize;
        if have < n {
            checks.push(check(
                Status::Fail,
                format!("Only {have} of {n} motors have an output"),
                "Set SERVOn_FUNCTION on the outputs your ESCs are wired to (33 for motor 1, 34 for motor 2…); the motor test then checks the order.",
            ));
        }
    }
    let pwm = int(p, "MOT_PWM_TYPE");
    let dshot = matches!(pwm, Some(4..=7));
    let bd = int(p, "SERVO_BLH_BDMASK").unwrap_or(0) as u32;
    if bd != 0 && !dshot {
        checks.push(check(
            Status::Warn,
            "Bidirectional DShot is on without DShot",
            "It only works with a DShot protocol: choose DShot300 or DShot600, or turn it off.",
        ));
    }
    if bd != 0 && mask != 0 && bd & mask != mask {
        checks.push(check(
            Status::Warn,
            format!("Bidirectional DShot covers outputs {} but motors are on {}", outputs_list(bd), outputs_list(mask)),
            "Every motor output should report RPM, or the noise filter follows only some motors.",
        ));
    }
    if dshot && p("SERVO_DSHOT_ESC") == Some(0.0) {
        checks.push(check(
            Status::Info,
            "ESC firmware not set",
            "With it set, ArduPilot can reverse motor directions in software (no re-soldering) and read extended telemetry.",
        ));
    }
    if p("SERVO_BLH_AUTO") == Some(1.0) && !dshot {
        checks.push(check(
            Status::Warn,
            "ESC passthrough is on without DShot",
            "ESC configuration tools only reach the ESCs when the flight controller drives them with DShot.",
        ));
    }
    if let (Some(arm), Some(min)) = (p("MOT_SPIN_ARM"), p("MOT_SPIN_MIN")) {
        if arm >= min {
            checks.push(check(
                Status::Fail,
                format!("Spin when armed ({arm:.2}) isn't below Spin minimum ({min:.2})"),
                "ArduPilot refuses to arm. Keep Spin minimum about 0.03 above Spin when armed.",
            ));
        }
    }
    match int(p, "BATT_MONITOR") {
        Some(0) => checks.push(check(
            Status::Warn,
            "No battery monitor",
            "Without it there's no battery voltage, no low-battery failsafe and no voltage compensation of the thrust. Most flight controllers measure voltage and current.",
        )),
        Some(4) if p("BATT_CAPACITY").is_some_and(|c| c <= 0.0) => checks.push(check(
            Status::Info,
            "Battery capacity not set",
            "Set the pack's capacity so the remaining percentage and the capacity failsafe work.",
        )),
        _ => {}
    }
    checks.sort_by_key(|c| match c.status {
        Status::Fail => 0,
        Status::Warn => 1,
        _ => 2,
    });

    let previews = motors::QUAD_TYPES.iter().filter_map(|t| motors::preview(1, *t).map(|ps| (*t, ps))).collect();
    View { groups, checks, previews, motor_mask: mask }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn params() -> HashMap<String, f64> {
        let mut m: HashMap<String, f64> = [
            ("FRAME_CLASS", 1.0),
            ("FRAME_TYPE", 18.0),
            ("MOT_PWM_TYPE", 6.0),
            ("SERVO_DSHOT_ESC", 1.0),
            ("SERVO_BLH_BDMASK", 15.0),
            ("SERVO_BLH_POLES", 14.0),
            ("MOT_SPIN_ARM", 0.1),
            ("MOT_SPIN_MIN", 0.13),
            ("AHRS_ORIENTATION", 0.0),
            ("BATT_MONITOR", 4.0),
            ("BATT_CAPACITY", 1500.0),
            ("SERVO_BLH_AUTO", 1.0),
            ("SERVO_BLH_PORT", 0.0),
        ]
        .iter()
        .map(|(k, v)| (k.to_string(), *v))
        .collect();
        for o in 1..=4 {
            m.insert(format!("SERVO{o}_FUNCTION"), 32.0 + o as f64);
        }
        m
    }

    #[test]
    fn healthy_build() {
        let m = params();
        let v = view(|n| m.get(n).copied());
        assert!(v.checks.is_empty(), "{:?}", v.checks);
        assert_eq!(v.motor_mask, 15);
        let titles: Vec<&str> = v.groups.iter().map(|g| g.title.as_str()).collect();
        assert_eq!(titles, ["Frame", "ESCs and motors", "ESC configuration", "Board", "Battery"]);
        let bd = v.groups[1].settings.iter().find(|s| s.name == "SERVO_BLH_BDMASK").unwrap();
        assert_eq!(bd.options[1], (15.0, "On, motor outputs 1, 2, 3, 4".to_owned()));
        assert!(v.groups[0].settings[0].reboot, "frame changes need a reboot");
        let bf_rev = v.previews.iter().find(|(t, _)| *t == 18).unwrap();
        assert_eq!((bf_rev.1[0].motor, bf_rev.1[0].spin), (2, Some(motors::Spin::Cw)));
    }

    #[test]
    fn problems() {
        let mut m = params();
        m.insert("MOT_PWM_TYPE".into(), 0.0);
        m.insert("SERVO_BLH_BDMASK".into(), 3.0);
        m.insert("MOT_SPIN_ARM".into(), 0.15);
        m.insert("BATT_MONITOR".into(), 0.0);
        m.insert("FRAME_TYPE".into(), 19.0);
        m.remove("SERVO4_FUNCTION");
        let v = view(|n| m.get(n).copied());
        let t: Vec<&str> = v.checks.iter().map(|c| c.title.as_str()).collect();
        assert!(t.contains(&"Layout Y4 on a quad"), "{t:?}");
        assert!(t.contains(&"Only 3 of 4 motors have an output"), "{t:?}");
        assert!(t.contains(&"Bidirectional DShot is on without DShot"), "{t:?}");
        assert!(t.iter().any(|t| t.starts_with("Spin when armed (0.15)")), "{t:?}");
        assert!(t.contains(&"No battery monitor"), "{t:?}");
        assert!(t.contains(&"ESC passthrough is on without DShot"), "{t:?}");
        assert_eq!(v.checks[0].status, Status::Fail);

        let mut m = params();
        m.insert("SERVO_BLH_BDMASK".into(), 3.0);
        let v = view(|n| m.get(n).copied());
        assert_eq!(v.checks[0].title, "Bidirectional DShot covers outputs 1, 2 but motors are on 1, 2, 3, 4");
    }
}
