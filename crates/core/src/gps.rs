//! The GPS page: what each GPS sees, whether it can be trusted (spoofing, a
//! position far from where the vehicle was last seen, glitches, the EKF's
//! view), what's wrong with its setup, and its settings.

use serde::{Deserialize, Serialize};

use crate::paramdoc;
use crate::readiness::{ParamChange, Status};
use crate::setup::{change, Param};

/// One GPS as the flight controller reports it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Fix {
    /// 0 for the first GPS.
    pub instance: u8,
    /// MAVLink GPS_FIX_TYPE: 0 no GPS, 1 no fix, 2 2D, 3 3D, 4+ better.
    pub fix_type: u8,
    pub satellites: Option<u8>,
    pub hdop: Option<f32>,
    /// Ground speed (m/s) and course over ground (degrees).
    pub speed: Option<f32>,
    pub course: Option<f32>,
    /// Latitude, longitude (degrees) and altitude (m).
    pub position: Option<(f64, f64, f32)>,
    /// Horizontal and vertical accuracy (m).
    pub h_acc: Option<f32>,
    pub v_acc: Option<f32>,
}

/// The EKF's status report: flags and innovation test ratios.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Ekf {
    pub flags: u16,
    pub velocity: f32,
    pub pos_horiz: f32,
    pub pos_vert: f32,
    pub compass: f32,
}

/// Where this vehicle's GPS was last trusted.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Known {
    pub lat: f64,
    pub lon: f64,
    /// When, for the pilot (e.g. "2026-11-10").
    pub when: String,
}

#[derive(Debug, Clone, Default)]
pub struct Live {
    pub gps: Vec<Fix>,
    pub ekf: Option<Ekf>,
    pub armed: bool,
    /// Seconds since power-up.
    pub boot_s: Option<f64>,
    /// Seconds after power-up the first 3D fix came, when seen.
    pub first_fix_s: Option<f64>,
    pub messages: Vec<String>,
    pub last_known: Option<Known>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Unit {
    /// 1 or 2.
    pub number: u8,
    /// The configured receiver type ("AUTO", "uBlox", "DroneCAN"…), `None` when off.
    pub kind: Option<String>,
    /// Where it's connected: "SERIAL3", "DroneCAN"…
    pub port: Option<String>,
    pub fix: &'static str,
    pub fix_type: u8,
    pub satellites: Option<u8>,
    pub hdop: Option<f32>,
    pub h_acc: Option<f32>,
    pub v_acc: Option<f32>,
    pub speed: Option<f32>,
    pub course: Option<f32>,
    pub position: Option<(f64, f64, f32)>,
    /// Constellations it's set to use; "receiver default" when not set.
    pub constellations: Vec<String>,
    pub rate_hz: Option<f64>,
    pub first_fix_s: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct EkfView {
    /// The EKF navigates with the GPS position.
    pub using_gps: bool,
    pub glitching: bool,
    /// Innovation test ratios by name; above 1 the EKF rejects that sensor.
    pub ratios: Vec<(&'static str, f32)>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Check {
    pub status: Status,
    pub title: String,
    pub detail: String,
    /// Changes that fix it, for review.
    pub changes: Vec<ParamChange>,
    /// Offer to put the GPS on a switch.
    pub gps_switch: bool,
    /// Offer "the vehicle really is here" (forget the last known position).
    pub accept_position: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct Setting {
    pub name: String,
    pub label: &'static str,
    pub value: Option<f64>,
    /// "select", "bits" or "number".
    pub kind: &'static str,
    pub options: Vec<(f64, String)>,
    pub unit: &'static str,
}

#[derive(Debug, Clone, Serialize)]
pub struct Group {
    pub title: String,
    pub settings: Vec<Setting>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Page {
    pub units: Vec<Unit>,
    pub ekf: Option<EkfView>,
    /// Most serious first.
    pub checks: Vec<Check>,
    pub settings: Vec<Group>,
    pub messages: Vec<String>,
    /// The position is good enough to remember as where the vehicle is.
    pub trusted: bool,
}

/// A disarmed vehicle doesn't move faster than someone walking with it.
const STILL_MAX_SPEED: f32 = 3.0;
/// Farther than this from the last trusted position looks like spoofing.
const FAR_KM: f64 = 50.0;
/// A cold start outdoors takes a minute or two; past this something's wrong.
const SLOW_FIX_S: f64 = 300.0;

const EKF_POS_HORIZ_ABS: u16 = 16;
const EKF_CONST_POS_MODE: u16 = 128;
const EKF_GPS_GLITCHING: u16 = 32768;

/// The 4.6+ parameter name, or the older one when the vehicle has that.
fn pick(p: Param, new: String, old: String) -> String {
    if p(&new).is_some() || p(&old).is_none() {
        new
    } else {
        old
    }
}

struct Names {
    kind: String,
    gnss: String,
    rate: String,
    pos: [String; 3],
}

fn names(p: Param, n: u8) -> Names {
    let old = |base: &str| if n == 1 { base.to_owned() } else { format!("{base}2") };
    Names {
        kind: pick(p, format!("GPS{n}_TYPE"), old("GPS_TYPE")),
        gnss: pick(p, format!("GPS{n}_GNSS_MODE"), old("GPS_GNSS_MODE")),
        rate: pick(p, format!("GPS{n}_RATE_MS"), old("GPS_RATE_MS")),
        pos: ["X", "Y", "Z"].map(|a| pick(p, format!("GPS{n}_POS_{a}"), format!("GPS_POS{n}_{a}"))),
    }
}

fn value_name(doc: &str, v: f64) -> String {
    paramdoc::doc(doc).and_then(|d| d.values.iter().find(|(x, _)| *x == v).map(|(_, n)| n.clone())).unwrap_or_else(|| format!("type {v}"))
}

pub fn fix_label(fix_type: u8) -> &'static str {
    match fix_type {
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

fn constellations(mode: Option<f64>) -> Vec<String> {
    match mode {
        None | Some(0.0) => vec!["receiver default".to_owned()],
        Some(m) => {
            let bits = paramdoc::doc("GPS1_GNSS_MODE").map(|d| d.bits.clone()).unwrap_or_default();
            bits.iter().filter(|(b, _)| (m as u32) & (1 << b) != 0).map(|(_, n)| n.clone()).collect()
        }
    }
}

/// How many of the four global systems (GPS, Galileo, BeiDou, GLONASS) a
/// GNSS mode enables; `None` for the receiver's default.
fn global_systems(mode: Option<f64>) -> Option<u32> {
    let m = mode.filter(|m| *m != 0.0)? as u32;
    Some([0, 2, 3, 6].iter().filter(|b| m & (1 << *b) != 0).count() as u32)
}

/// Serial ports set for a GPS, in order: ArduPilot gives the first to GPS 1.
fn gps_ports(p: Param) -> Vec<(u8, Option<f64>)> {
    (0..=9u8).filter(|n| p(&format!("SERIAL{n}_PROTOCOL")) == Some(5.0)).map(|n| (n, p(&format!("SERIAL{n}_BAUD")))).collect()
}

/// GPS types that don't use a serial port set to GPS.
fn needs_serial(kind: f64) -> bool {
    !matches!(kind as i64, 0 | 7 | 9 | 14 | 19 | 21 | 22 | 23)
}

/// Great-circle distance in km.
pub fn distance_km(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let dla = la2 - la1;
    let dlo = (b.1 - a.1).to_radians();
    let h = (dla / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlo / 2.0).sin().powi(2);
    6371.0 * 2.0 * h.sqrt().asin()
}

fn check(status: Status, title: impl Into<String>, detail: impl Into<String>) -> Check {
    Check { status, title: title.into(), detail: detail.into(), changes: vec![], gps_switch: false, accept_position: false }
}

pub fn page(param: impl Fn(&str) -> Option<f64>, live: &Live) -> Page {
    let p: Param = &param;
    let ports = gps_ports(p);
    let mut serial = ports.iter();
    let mut units = Vec::new();
    let mut checks = Vec::new();
    for n in 1..=2u8 {
        let names = names(p, n);
        let Some(kind) = p(&names.kind) else { continue };
        let fix = live.gps.iter().find(|g| g.instance == n - 1).copied().unwrap_or_default();
        if kind == 0.0 && fix.fix_type == 0 {
            if n == 1 {
                let mut c = check(
                    Status::Warn,
                    format!("The GPS is off ({} = 0)", names.kind),
                    "Without GPS only Stabilize, AltHold and other modes that don't need a position work, and there's no return home. \
                     If it was turned off because of spoofing, turn it back on where the GPS can be trusted.",
                );
                c.changes = change(p, &names.kind, 1.0, "Turn the GPS on, type detected automatically (after a reboot)").into_iter().collect();
                checks.push(c);
            }
            continue;
        }
        let port = if kind == 9.0 || kind == 22.0 || kind == 23.0 {
            Some("DroneCAN".to_owned())
        } else if needs_serial(kind) {
            serial.next().map(|(s, _)| format!("SERIAL{s}"))
        } else {
            None
        };
        if fix.fix_type == 0 {
            let detail = if needs_serial(kind) && port.is_none() {
                "No serial port is set for a GPS: set SERIALn_PROTOCOL = 5 (GPS) on the port the GPS is wired to, then reboot.".to_owned()
            } else {
                format!(
                    "Nothing answers{}. Check the wiring (GPS TX to the flight controller's RX, RX to TX, 5 V and ground) and that the GPS gets power: \
                     its LED usually blinks. ArduPilot finds the baud rate by itself.",
                    port.as_ref().map(|s| format!(" on {s}")).unwrap_or_default()
                )
            };
            checks.push(check(Status::Fail, format!("GPS {n} isn't detected"), detail));
        }
        let mode = p(&names.gnss);
        if fix.fix_type >= 3 && fix.satellites.is_some_and(|s| s < 10) && global_systems(mode).is_some_and(|g| g <= 1) {
            let mut c = check(
                Status::Warn,
                format!("GPS {n} uses only one satellite system ({} satellites)", fix.satellites.unwrap_or(0)),
                "More systems (Galileo, BeiDou, GLONASS) give more satellites and a steadier position. Leaving the choice to the receiver uses its default, which is several systems on current u-blox modules.",
            );
            c.changes = change(p, &names.gnss, 0.0, "Use the receiver's default satellite systems").into_iter().collect();
            checks.push(c);
        }
        units.push(Unit {
            number: n,
            kind: (kind != 0.0).then(|| value_name("GPS1_TYPE", kind)),
            port,
            fix: fix_label(fix.fix_type),
            fix_type: fix.fix_type,
            satellites: fix.satellites,
            hdop: fix.hdop,
            h_acc: fix.h_acc,
            v_acc: fix.v_acc,
            speed: fix.speed,
            course: fix.course,
            position: fix.position,
            constellations: constellations(mode),
            rate_hz: p(&names.rate).filter(|r| *r > 0.0).map(|r| (1000.0 / r * 10.0).round() / 10.0),
            first_fix_s: if n == 1 { live.first_fix_s } else { None },
        });
    }

    let best = live.gps.iter().filter(|g| g.fix_type >= 3).min_by_key(|g| g.instance).copied();
    let spoof_fixes = || crate::arming::spoofing_changes(&|n: &str| p(n));
    let mut far = false;
    let mut spoofed = false;
    if let Some(g) = best {
        if !live.armed && g.speed.is_some_and(|v| v > STILL_MAX_SPEED) {
            spoofed = true;
            let at = g.position.map_or(String::new(), |(la, lo, _)| format!(" and puts it at {la:.5}, {lo:.5}"));
            let mut c = check(
                Status::Fail,
                format!("The GPS says the vehicle moves at {:.0} m/s while it stands still: spoofed or faulty", g.speed.unwrap_or(0.0)),
                format!(
                    "The GPS reports {:.1} m/s{at}. If that isn't where the vehicle is, the receiver gets fake satellite signals. \
                     The EKF tilts its attitude to explain the false speed, in every flight mode: don't fly like this. \
                     To fly here, turn the GPS off (below), reboot and fly Stabilize or AltHold, or put the GPS on a switch.",
                    g.speed.unwrap_or(0.0)
                ),
            );
            c.changes = spoof_fixes();
            c.gps_switch = true;
            checks.push(c);
        }
        if let (Some((la, lo, _)), Some(k)) = (g.position, &live.last_known) {
            let km = distance_km((la, lo), (k.lat, k.lon));
            if km > FAR_KM {
                far = true;
                let mut c = check(
                    Status::Fail,
                    format!("The GPS puts the vehicle {km:.0} km from where it was last seen"),
                    format!(
                        "On {} the GPS was trusted at {:.4}, {:.4}; now it says {la:.4}, {lo:.4}. If you didn't travel there, the GPS is spoofed: \
                         turn it off or put it on a switch. If the vehicle really is here, say so and ArduTune remembers the new place.",
                        k.when, k.lat, k.lon
                    ),
                );
                if !spoofed {
                    c.changes = spoof_fixes();
                    c.gps_switch = true;
                }
                c.accept_position = true;
                checks.push(c);
            }
        }
        let hdop_good = p("GPS_HDOP_GOOD").unwrap_or(140.0) / 100.0;
        if g.hdop.is_some_and(|h| f64::from(h) > hdop_good) {
            checks.push(check(
                Status::Warn,
                format!("HDOP {:.1} is above the {hdop_good:.1} ArduPilot asks for", g.hdop.unwrap_or(0.0)),
                "Arming in position modes (Loiter, PosHold, RTL, Auto) is refused until it drops. Wait for more satellites, with a clear view of the sky.",
            ));
        }
        if g.h_acc.is_some_and(|a| a > 5.0) {
            checks.push(check(
                Status::Warn,
                format!("Position accuracy is {:.0} m", g.h_acc.unwrap_or(0.0)),
                "Position modes drift with an inaccurate GPS. Give it a few minutes and a clear view of the sky.",
            ));
        }
    } else if live.gps.iter().any(|g| g.fix_type >= 1) {
        let slow = live.boot_s.is_some_and(|b| b > SLOW_FIX_S);
        checks.push(check(
            if slow { Status::Warn } else { Status::Info },
            if slow { "No 3D fix after five minutes".to_owned() } else { "Waiting for a 3D fix".to_owned() },
            if slow {
                "Outdoors with a clear sky a fix takes a minute or two. Check the GPS has a view of the sky (not under carbon or near the video transmitter), and that the right satellite systems are on."
            } else {
                "Usually under a minute outdoors with a clear view of the sky; a cold start can take a few minutes. Indoors it won't get one."
            },
        ));
    }

    let ekf = live.ekf.map(|e| EkfView {
        using_gps: e.flags & EKF_POS_HORIZ_ABS != 0 && e.flags & EKF_CONST_POS_MODE == 0,
        glitching: e.flags & EKF_GPS_GLITCHING != 0,
        ratios: vec![("Velocity", e.velocity), ("Horizontal position", e.pos_horiz), ("Height", e.pos_vert), ("Compass", e.compass)],
    });
    let glitch_texts: Vec<String> = live.messages.iter().filter(|m| m.contains("Glitch") || m.contains("glitch")).cloned().collect();
    if ekf.as_ref().is_some_and(|e| e.glitching) || !glitch_texts.is_empty() {
        checks.push(check(
            Status::Warn,
            "The EKF reports a GPS glitch",
            "The GPS position jumped away from where the EKF expected it, so the EKF stopped trusting it for now. Often multipath near buildings or interference; repeated glitches on a still vehicle point to spoofing or a bad antenna position.",
        ));
    }
    if let Some(e) = live.ekf {
        let rejected: Vec<&str> = [("velocity", e.velocity), ("position", e.pos_horiz)].iter().filter(|(_, r)| *r >= 1.0).map(|(n, _)| *n).collect();
        if !rejected.is_empty() && best.is_some() {
            checks.push(check(
                Status::Warn,
                format!("The EKF rejects the GPS {}", rejected.join(" and ")),
                "Its test ratio is above 1: the GPS disagrees with what the other sensors say. On a still vehicle that means a bad or spoofed GPS.",
            ));
        }
    }
    if p("GPS_AUTO_CONFIG") == Some(0.0) && !units.is_empty() {
        checks.push(check(
            Status::Info,
            "ArduPilot doesn't configure the GPS (GPS_AUTO_CONFIG = 0)",
            "The update rate and satellite systems below then come from the receiver's own saved settings, not from these parameters.",
        ));
    }
    checks.sort_by_key(|c| match c.status {
        Status::Fail => 0,
        Status::Warn => 1,
        _ => 2,
    });

    let trusted = best.is_some_and(|g| g.satellites.is_some_and(|s| s >= 8) && g.hdop.is_some_and(|h| h <= 1.5) && (live.armed || g.speed.is_some_and(|v| v < 1.0)))
        && !spoofed
        && !far;
    Page { settings: settings(p), units, ekf, checks, messages: live.messages.clone(), trusted }
}

fn setting(p: Param, name: &str, label: &'static str, unit: &'static str) -> Option<Setting> {
    let value = p(name);
    value?;
    let doc = paramdoc::doc(name);
    let bits = doc.map(|d| d.bits.iter().map(|(b, n)| (f64::from(*b), n.clone())).collect::<Vec<_>>()).unwrap_or_default();
    let values = doc.map(|d| d.values.clone()).unwrap_or_default();
    let (kind, options) = if !bits.is_empty() {
        ("bits", bits)
    } else if !values.is_empty() && unit.is_empty() {
        ("select", values)
    } else {
        ("number", values)
    };
    Some(Setting { name: name.to_owned(), label, value, kind, options, unit })
}

fn settings(p: Param) -> Vec<Group> {
    let mut groups = Vec::new();
    for n in 1..=2u8 {
        let names = names(p, n);
        let on = p(&names.kind).is_some_and(|k| k != 0.0);
        let mut s: Vec<Setting> = setting(p, &names.kind, "Receiver type", "").into_iter().collect();
        if on {
            s.extend(setting(p, &names.gnss, "Satellite systems", ""));
            s.extend(setting(p, &names.rate, "Update rate", ""));
            for (name, axis) in names.pos.iter().zip(["Antenna offset forward", "Antenna offset right", "Antenna offset down"]) {
                s.extend(setting(p, name, axis, "m"));
            }
        }
        if !s.is_empty() {
            groups.push(Group { title: format!("GPS {n}"), settings: s });
        }
    }
    let two = p(&names(p, 2).kind).is_some_and(|k| k != 0.0);
    let mut general: Vec<Setting> = Vec::new();
    general.extend(setting(p, "GPS_AUTO_CONFIG", "ArduPilot configures the receiver", ""));
    general.extend(setting(p, "GPS_HDOP_GOOD", "HDOP to arm in position modes", "×100 (140 = 1.4)"));
    if two {
        general.extend(setting(p, "GPS_AUTO_SWITCH", "With two GPS", ""));
        general.extend(setting(p, "GPS_PRIMARY", "Main GPS", ""));
    }
    if !general.is_empty() {
        groups.push(Group { title: "General".to_owned(), settings: general });
    }
    groups
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn params() -> HashMap<String, f64> {
        [
            ("GPS1_TYPE", 1.0),
            ("GPS2_TYPE", 0.0),
            ("GPS1_GNSS_MODE", 0.0),
            ("GPS1_RATE_MS", 200.0),
            ("GPS1_POS_X", 0.0),
            ("GPS1_POS_Y", 0.0),
            ("GPS1_POS_Z", 0.0),
            ("GPS_AUTO_CONFIG", 1.0),
            ("GPS_HDOP_GOOD", 140.0),
            ("SERIAL3_PROTOCOL", 5.0),
            ("SERIAL3_BAUD", 230.0),
            ("SERIAL1_PROTOCOL", 2.0),
            ("FS_THR_ENABLE", 1.0),
        ]
        .iter()
        .map(|(k, v)| (k.to_string(), *v))
        .collect()
    }

    fn good() -> Fix {
        Fix { instance: 0, fix_type: 3, satellites: Some(16), hdop: Some(0.8), speed: Some(0.05), course: Some(0.0), position: Some((50.45, 30.52, 180.0)), h_acc: Some(1.2), v_acc: Some(2.0) }
    }

    #[test]
    fn healthy_gps_is_trusted() {
        let m = params();
        let live = Live { gps: vec![good()], ekf: Some(Ekf { flags: 0x3F, velocity: 0.1, pos_horiz: 0.1, pos_vert: 0.2, compass: 0.05 }), ..Default::default() };
        let pg = page(|n| m.get(n).copied(), &live);
        assert!(pg.checks.is_empty(), "{:?}", pg.checks);
        assert!(pg.trusted);
        let u = &pg.units[0];
        assert_eq!((u.port.as_deref(), u.kind.as_deref(), u.fix, u.rate_hz), (Some("SERIAL3"), Some("AUTO"), "3D fix", Some(5.0)));
        assert_eq!(u.constellations, ["receiver default"]);
        assert!(pg.ekf.unwrap().using_gps);
        assert_eq!(pg.units.len(), 1, "GPS 2 is off");
        let titles: Vec<&str> = pg.settings.iter().map(|g| g.title.as_str()).collect();
        assert_eq!(titles, ["GPS 1", "GPS 2", "General"]);
        assert_eq!(pg.settings[1].settings.len(), 1, "only the type of a GPS that is off");
        assert_eq!(pg.settings[0].settings[1].kind, "bits");
    }

    #[test]
    fn spoofed_far_away() {
        let m = params();
        let lima = Fix { speed: Some(56.0), position: Some((-12.05, -77.04, 150.0)), ..good() };
        let live = Live { gps: vec![lima], last_known: Some(Known { lat: 50.45, lon: 30.52, when: "2026-11-10".into() }), ..Default::default() };
        let pg = page(|n| m.get(n).copied(), &live);
        assert_eq!(pg.checks[0].status, Status::Fail);
        assert!(pg.checks[0].title.contains("56 m/s"));
        assert!(pg.checks[0].gps_switch);
        assert!(pg.checks[0].changes.iter().any(|c| c.name == "GPS1_TYPE" && c.value == 0.0));
        assert!(pg.checks[1].title.contains("km from where it was last seen"));
        assert!(pg.checks[1].accept_position && pg.checks[1].changes.is_empty(), "fixes offered once");
        assert!(!pg.trusted);
    }

    #[test]
    fn setup_problems() {
        let mut m = params();
        m.remove("SERIAL3_PROTOCOL");
        let live = Live { gps: vec![Fix::default()], boot_s: Some(60.0), ..Default::default() };
        let pg = page(|n| m.get(n).copied(), &live);
        assert!(pg.checks[0].title.contains("isn't detected") && pg.checks[0].detail.contains("SERIALn_PROTOCOL = 5"));

        let m = params();
        let waiting = Live { gps: vec![Fix { fix_type: 1, ..Default::default() }], boot_s: Some(400.0), ..Default::default() };
        let pg = page(|n| m.get(n).copied(), &waiting);
        assert_eq!((pg.checks[0].status, pg.checks[0].title.as_str()), (Status::Warn, "No 3D fix after five minutes"));

        let mut m = params();
        m.insert("GPS1_GNSS_MODE".into(), 1.0);
        let few = Live { gps: vec![Fix { satellites: Some(7), hdop: Some(1.9), ..good() }], ..Default::default() };
        let pg = page(|n| m.get(n).copied(), &few);
        let t: Vec<&str> = pg.checks.iter().map(|c| c.title.as_str()).collect();
        assert!(t.iter().any(|t| t.contains("only one satellite system")), "{t:?}");
        assert!(t.iter().any(|t| t.contains("HDOP 1.9")), "{t:?}");
        assert_eq!(pg.units[0].constellations, ["GPS"]);

        let mut m = params();
        m.insert("GPS1_TYPE".into(), 0.0);
        let pg = page(|n| m.get(n).copied(), &Live::default());
        assert!(pg.checks[0].title.contains("GPS is off") && pg.checks[0].changes[0].value == 1.0);
    }

    #[test]
    fn older_names() {
        let m: HashMap<String, f64> = [("GPS_TYPE", 2.0), ("GPS_GNSS_MODE", 69.0), ("GPS_RATE_MS", 100.0), ("GPS_POS1_X", 0.02), ("SERIAL4_PROTOCOL", 5.0)]
            .iter()
            .map(|(k, v)| (k.to_string(), *v))
            .collect();
        let pg = page(|n| m.get(n).copied(), &Live { gps: vec![good()], ..Default::default() });
        let u = &pg.units[0];
        assert_eq!((u.kind.as_deref(), u.port.as_deref(), u.rate_hz), (Some("uBlox"), Some("SERIAL4"), Some(10.0)));
        assert_eq!(u.constellations, ["GPS", "Galileo", "GLONASS"]);
        let names: Vec<&str> = pg.settings[0].settings.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["GPS_TYPE", "GPS_GNSS_MODE", "GPS_RATE_MS", "GPS_POS1_X"]);
    }

    #[test]
    fn distance() {
        let km = distance_km((50.45, 30.52), (-12.05, -77.04));
        assert!((km - 12_270.0).abs() < 100.0, "{km}");
    }
}
