//! Motor order and direction test. ArduCopter's motor test spins one motor
//! at a time by test letter: A is the front-right motor (or the front one on
//! a plus frame), then clockwise. The pilot reports which motor actually
//! spun and which way; from that come the output remapping
//! (`SERVOn_FUNCTION`) and the DShot direction reversals
//! (`SERVO_BLH_RVMASK`) that fix the layout. Quads only for the expected
//! layout; other frames can still spin motors one by one.

use serde::{Deserialize, Serialize};

use crate::params::format_value;
use crate::readiness::{ParamChange, Status};
use crate::setup::{change, frame_type_name, int, item, motor_count, Item, Param};

/// Spin direction, seen from above.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Spin {
    Cw,
    Ccw,
}

impl Spin {
    pub fn name(self) -> &'static str {
        match self {
            Spin::Cw => "clockwise",
            Spin::Ccw => "counter-clockwise",
        }
    }

    fn reversed(self) -> Spin {
        match self {
            Spin::Cw => Spin::Ccw,
            Spin::Ccw => Spin::Cw,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Position {
    /// Test letter: A, B, C…; the motor test's sequence number is its index + 1.
    pub letter: String,
    pub motor: u8,
    /// Where it sits, e.g. "front right"; empty when the layout isn't known.
    pub name: String,
    /// Degrees clockwise from the nose, for drawing.
    pub angle: Option<f64>,
    pub spin: Option<Spin>,
    /// The output (SERVOn) assigned to this motor.
    pub output: Option<u8>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Layout {
    pub frame: String,
    /// In test order.
    pub positions: Vec<Position>,
    /// Whether the expected positions and directions are known (quads).
    pub known: bool,
    /// Directions can be reversed in software (DShot ESCs).
    pub dshot: bool,
    pub frame_type: i64,
    /// The same layout with every motor reversed (props in vs props out),
    /// when ArduPilot has one: (FRAME_TYPE, name).
    pub reversed: Option<(i64, &'static str)>,
}

/// The FRAME_TYPE with every motor turning the other way.
pub fn reversed_frame(frame_type: i64) -> Option<(i64, &'static str)> {
    match frame_type {
        1 => Some((3, "H (X with every motor reversed)")),
        3 => Some((1, "X")),
        18 => Some((19, "BetaFlight X reversed (props out)")),
        19 => Some((18, "BetaFlight X (props in)")),
        _ => None,
    }
}

/// Switching to another FRAME_TYPE of the same class.
pub fn frame_change(param: impl Fn(&str) -> Option<f64>, frame_type: i64) -> Result<Vec<ParamChange>, String> {
    let p: Param = &param;
    let now = int(p, "FRAME_TYPE").ok_or("This vehicle has no FRAME_TYPE.")?;
    let Some((to, name)) = reversed_frame(now).filter(|(t, _)| *t == frame_type) else {
        return Err(format!("ArduTune only switches between a layout and its reversed version (FRAME_TYPE {now} has none as {frame_type})."));
    };
    Ok(change(p, "FRAME_TYPE", to as f64, format!("{name}: matches motors that turn the other way (takes effect after a reboot)")).into_iter().collect())
}

/// One quad layout: (motor, degrees from the nose, spin), in test order.
type Quad = [(u8, f64, Spin); 4];

const X: Quad = [(1, 45.0, Spin::Ccw), (4, 135.0, Spin::Cw), (2, -135.0, Spin::Ccw), (3, -45.0, Spin::Cw)];
const PLUS: Quad = [(3, 0.0, Spin::Cw), (1, 90.0, Spin::Ccw), (4, 180.0, Spin::Cw), (2, -90.0, Spin::Ccw)];
const DJI_X: Quad = [(1, 45.0, Spin::Ccw), (4, 135.0, Spin::Cw), (3, -135.0, Spin::Ccw), (2, -45.0, Spin::Cw)];
const CW_X: Quad = [(1, 45.0, Spin::Ccw), (2, 135.0, Spin::Cw), (3, -135.0, Spin::Ccw), (4, -45.0, Spin::Cw)];
const BF_X: Quad = [(2, 45.0, Spin::Ccw), (1, 135.0, Spin::Cw), (3, -135.0, Spin::Ccw), (4, -45.0, Spin::Cw)];

/// ArduPilot's quad layouts (AP_MotorsMatrix) by FRAME_TYPE; H and
/// BetaFlight X reversed are X and BetaFlight X with every motor reversed.
fn quad(frame_type: i64) -> Option<Quad> {
    let reverse = |q: Quad| q.map(|(m, a, s)| (m, a, s.reversed()));
    Some(match frame_type {
        0 => PLUS,
        1 => X,
        3 => reverse(X),
        12 => DJI_X,
        13 => CW_X,
        18 => BF_X,
        19 => reverse(BF_X),
        _ => return None,
    })
}

fn place(angle: f64) -> &'static str {
    match angle as i64 {
        0 => "front",
        45 => "front right",
        90 => "right",
        135 => "back right",
        180 => "back",
        -135 => "back left",
        -90 => "left",
        -45 => "front left",
        _ => "",
    }
}

fn output_of(p: Param, motor: u8) -> Option<u8> {
    (1..=16).find(|ch| p(&format!("SERVO{ch}_FUNCTION")) == Some(32.0 + f64::from(motor)))
}

fn letter(i: usize) -> String {
    char::from(b'A' + i as u8).to_string()
}

pub fn layout(param: impl Fn(&str) -> Option<f64>) -> Result<Layout, String> {
    let p: Param = &param;
    let class = int(p, "FRAME_CLASS").unwrap_or(0);
    let count = motor_count(class).ok_or("Set FRAME_CLASS (1 = quad) and FRAME_TYPE first, then reboot.")?;
    let frame_type = int(p, "FRAME_TYPE").unwrap_or(-1);
    let dshot = matches!(int(p, "MOT_PWM_TYPE"), Some(4..=7));
    let expected = if class == 1 { quad(frame_type) } else { None };
    let positions = match expected {
        Some(q) => q
            .iter()
            .enumerate()
            .map(|(i, &(motor, angle, spin))| Position {
                letter: letter(i),
                motor,
                name: place(angle).to_owned(),
                angle: Some(angle),
                spin: Some(spin),
                output: output_of(p, motor),
            })
            .collect(),
        // Without the layout, test letters still go clockwise from the
        // front, but the motor behind each isn't known here.
        None => (0..count)
            .map(|i| Position { letter: letter(i), motor: 0, name: String::new(), angle: None, spin: None, output: None })
            .collect(),
    };
    let class_name = if class == 1 { "Quad" } else { "Multirotor" };
    let reversed = if expected.is_some() { reversed_frame(frame_type) } else { None };
    Ok(Layout { frame: format!("{class_name} {}", frame_type_name(frame_type)), positions, known: expected.is_some(), dshot, frame_type, reversed })
}

/// What the pilot saw when test letter `index` ran: which position (index
/// into the layout) spun, if any, and which way.
#[derive(Debug, Clone, Deserialize)]
pub struct Observation {
    pub spun: Option<usize>,
    pub spin: Option<Spin>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Verdict {
    pub ok: bool,
    pub items: Vec<Item>,
    pub changes: Vec<ParamChange>,
    pub needs_reboot: bool,
}

pub fn check(param: impl Fn(&str) -> Option<f64>, observations: &[Observation]) -> Result<Verdict, String> {
    let p: Param = &param;
    let l = layout(p)?;
    if !l.known {
        return Err("ArduTune knows the expected motor layout for quads only.".to_owned());
    }
    let n = l.positions.len();
    if observations.len() != n {
        return Err(format!("Expected results for {n} motor tests."));
    }
    let mut items = Vec::new();
    let mut changes = Vec::new();

    let silent: Vec<&str> = observations.iter().zip(&l.positions).filter(|(o, _)| o.spun.is_none()).map(|(_, pos)| pos.letter.as_str()).collect();
    if !silent.is_empty() {
        items.push(item(
            Status::Fail,
            format!("Nothing spun for test {}", silent.join(", ")),
            "Check the battery is connected (ESCs need it), the ESC's signal wire and that the output has a motor function (SERVOn_FUNCTION). Then test again.",
        ));
        return Ok(Verdict { ok: false, items, changes, needs_reboot: false });
    }
    let mut seen = vec![false; n];
    for o in observations {
        let q = o.spun.filter(|q| *q < n).ok_or("A result names a motor position that doesn't exist.")?;
        if std::mem::replace(&mut seen[q], true) {
            items.push(item(
                Status::Fail,
                format!("Two tests spun the {} motor", l.positions[q].name),
                "Each test spins one motor, so one of the answers is wrong. Run the tests again and watch closely.",
            ));
            return Ok(Verdict { ok: false, items, changes, needs_reboot: false });
        }
    }

    // Test k drives the output assigned to the motor expected at position
    // k; it is wired to the motor at position q instead when they differ.
    let mut remap: Vec<(u8, u8)> = Vec::new();
    let mut reverse: Vec<u8> = Vec::new();
    for (k, o) in observations.iter().enumerate() {
        let tested = &l.positions[k];
        let q = o.spun.unwrap_or(k);
        let actual = &l.positions[q];
        let Some(output) = tested.output else {
            return Err(format!("Motor {} has no output assigned (SERVOn_FUNCTION {}).", tested.motor, 32 + tested.motor));
        };
        if q != k {
            items.push(item(
                Status::Fail,
                format!("Test {} spun the {} motor instead of the {} one", tested.letter, actual.name, tested.name),
                format!("Output {output} is wired to the {} motor: it should drive motor {} (SERVO{output}_FUNCTION = {}).", actual.name, actual.motor, 32 + actual.motor),
            ));
            remap.push((output, actual.motor));
        }
        let want = actual.spin.unwrap_or(Spin::Cw);
        match o.spin {
            Some(s) if s != want => {
                items.push(item(
                    Status::Fail,
                    format!("The {} motor spins {}; it should spin {}", actual.name, s.name(), want.name()),
                    if l.dshot {
                        format!("Reverse it in software (output {output} in SERVO_BLH_RVMASK), or swap any two of its three motor wires.")
                    } else {
                        "Swap any two of its three motor wires, or reverse it in the ESC configurator.".to_owned()
                    },
                ));
                reverse.push(output);
            }
            Some(_) => {}
            None => items.push(item(Status::Warn, format!("Direction of the {} motor not recorded", actual.name), "")),
        }
    }

    for (output, motor) in &remap {
        changes.extend(change(
            p,
            &format!("SERVO{output}_FUNCTION"),
            32.0 + f64::from(*motor),
            format!("Output {output} drives the motor at the motor {motor} position"),
        ));
    }
    let mut needs_reboot = false;
    // Every motor in its place but all turning the other way: the build is
    // the reversed layout (props in vs out), not four wiring mistakes. The
    // frame type decides which way ArduPilot steers yaw, so fix that.
    let all_reversed = remap.is_empty() && reverse.len() == n;
    if let (true, Some((to, name))) = (all_reversed, l.reversed) {
        items.retain(|i| i.status != Status::Fail);
        items.push(item(
            Status::Fail,
            format!("Every motor turns the other way: this build is {name}"),
            format!(
                "FRAME_TYPE is {} ({}), so ArduPilot would steer yaw backwards and the vehicle would spin on take-off. \
                 Set FRAME_TYPE = {to} and reboot. If you meant the other direction, reverse all four motors instead \
                 and turn the propellers over.",
                l.frame_type,
                frame_type_name(l.frame_type)
            ),
        ));
        changes.extend(change(p, "FRAME_TYPE", to as f64, format!("{name}: matches the way the motors turn (takes effect after a reboot)")));
        return Ok(Verdict { ok: false, items, changes, needs_reboot: true });
    }
    if !reverse.is_empty() && l.dshot {
        if let Some(mask) = int(p, "SERVO_BLH_RVMASK") {
            let new = reverse.iter().fold(mask, |m, o| m ^ (1 << (o - 1)));
            let list = reverse.iter().map(|o| o.to_string()).collect::<Vec<_>>().join(", ");
            changes.extend(change(p, "SERVO_BLH_RVMASK", new as f64, format!("Reverse the motor on output {list} (takes effect after a reboot)")));
            needs_reboot = true;
        }
    }
    let ok = remap.is_empty() && reverse.is_empty() && observations.iter().all(|o| o.spin.is_some());
    if ok {
        items.push(item(
            Status::Pass,
            "Every motor is in the right place and spins the right way",
            "Fit the propellers to match: each prop's leading edge (the higher side) leads in its motor's direction. A wrong prop pushes air up.",
        ));
    } else if !changes.is_empty() {
        let names = changes.iter().map(|c| format!("{} = {}", c.name, format_value(c.value))).collect::<Vec<_>>().join(", ");
        items.push(item(Status::Info, "ArduTune can fix this in software", format!("{names}. Run the motor test again afterwards to confirm.")));
    }
    Ok(Verdict { ok, items, changes, needs_reboot })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// The KakuteH7: BetaFlight X, motors 1-4 on outputs 1-4, DShot600.
    fn kakute() -> HashMap<String, f64> {
        let mut m: HashMap<String, f64> =
            [("FRAME_CLASS", 1.0), ("FRAME_TYPE", 18.0), ("MOT_PWM_TYPE", 6.0), ("SERVO_BLH_RVMASK", 0.0)].iter().map(|(k, v)| (k.to_string(), *v)).collect();
        for o in 1..=4 {
            m.insert(format!("SERVO{o}_FUNCTION"), 32.0 + o as f64);
        }
        m
    }

    fn obs(list: &[(usize, Spin)]) -> Vec<Observation> {
        list.iter().map(|(q, s)| Observation { spun: Some(*q), spin: Some(*s) }).collect()
    }

    #[test]
    fn betaflight_x_layout() {
        let m = kakute();
        let l = layout(|n| m.get(n).copied()).unwrap();
        assert!(l.known && l.dshot);
        let summary: Vec<String> =
            l.positions.iter().map(|p| format!("{} M{} {} {:?} out {:?}", p.letter, p.motor, p.name, p.spin.unwrap(), p.output.unwrap())).collect();
        assert_eq!(
            summary,
            ["A M2 front right Ccw out 2", "B M1 back right Cw out 1", "C M3 back left Ccw out 3", "D M4 front left Cw out 4"]
        );
    }

    #[test]
    fn correct_motors_pass() {
        let m = kakute();
        let v = check(|n| m.get(n).copied(), &obs(&[(0, Spin::Ccw), (1, Spin::Cw), (2, Spin::Ccw), (3, Spin::Cw)])).unwrap();
        assert!(v.ok && v.changes.is_empty(), "{v:?}");
    }

    #[test]
    fn swapped_outputs_and_a_reversed_motor() {
        let m = kakute();
        // Tests A and B spun each other's motors, and the back-left motor
        // spins the wrong way.
        let v = check(|n| m.get(n).copied(), &obs(&[(1, Spin::Cw), (0, Spin::Ccw), (2, Spin::Cw), (3, Spin::Cw)])).unwrap();
        assert!(!v.ok && v.needs_reboot);
        let changes: Vec<(&str, f64)> = v.changes.iter().map(|c| (c.name.as_str(), c.value)).collect();
        // Test A drove output 2 (motor 2) but the back-right motor spun:
        // output 2 must drive motor 1, and output 1 motor 2.
        assert_eq!(changes, vec![("SERVO2_FUNCTION", 33.0), ("SERVO1_FUNCTION", 34.0), ("SERVO_BLH_RVMASK", 4.0)]);
    }

    #[test]
    fn all_reversed_means_the_reversed_frame() {
        // The 15-inch quad: props out, but FRAME_TYPE says BetaFlight X.
        let m = kakute();
        let v = check(|n| m.get(n).copied(), &obs(&[(0, Spin::Cw), (1, Spin::Ccw), (2, Spin::Cw), (3, Spin::Ccw)])).unwrap();
        assert!(!v.ok && v.needs_reboot);
        let changes: Vec<(&str, f64)> = v.changes.iter().map(|c| (c.name.as_str(), c.value)).collect();
        assert_eq!(changes, [("FRAME_TYPE", 19.0)], "the frame, not four reversals");
        assert_eq!(v.items.iter().filter(|i| i.status == Status::Fail).count(), 1, "{:?}", v.items);
        let l = layout(|n| m.get(n).copied()).unwrap();
        assert_eq!(l.reversed.map(|r| r.0), Some(19));
        assert_eq!(frame_change(|n| m.get(n).copied(), 19).unwrap()[0].value, 19.0);
        assert!(frame_change(|n| m.get(n).copied(), 12).is_err());
    }

    #[test]
    fn wrong_answers_are_caught() {
        let m = kakute();
        let p = |n: &str| m.get(n).copied();
        let twice = check(p, &obs(&[(0, Spin::Ccw), (0, Spin::Cw), (2, Spin::Ccw), (3, Spin::Cw)])).unwrap();
        assert!(!twice.ok && twice.items[0].title.contains("Two tests"));
        let mut silent = obs(&[(0, Spin::Ccw), (1, Spin::Cw), (2, Spin::Ccw), (3, Spin::Cw)]);
        silent[2].spun = None;
        let v = check(p, &silent).unwrap();
        assert!(v.items[0].title.contains("Nothing spun for test C"));
        assert!(check(p, &silent[..2]).is_err());
    }

    #[test]
    fn plus_and_unknown_frames() {
        let mut m = kakute();
        m.insert("FRAME_TYPE".into(), 0.0);
        let l = layout(|n| m.get(n).copied()).unwrap();
        assert_eq!((l.positions[0].name.as_str(), l.positions[0].motor), ("front", 3));
        m.insert("FRAME_CLASS".into(), 2.0);
        let hexa = layout(|n| m.get(n).copied()).unwrap();
        assert!(!hexa.known);
        assert_eq!(hexa.positions.len(), 6);
    }
}
