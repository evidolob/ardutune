//! Serial ports: what each SERIALn does, what is live on it (from
//! ArduPilot's `@SYS/uarts.txt` traffic report), checks, and common setups
//! that set protocol, baud and options together.

use serde::Serialize;

use crate::paramdoc;
use crate::readiness::{ParamChange, Status};
use crate::setup::{change, Param};

/// One line of `@SYS/uarts.txt`.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Uart {
    pub serial: u8,
    /// Hardware name: "OTG1" (USB), "UART7", "USART2"…
    pub hw: String,
    /// Bytes per second sent and received since the previous read.
    pub tx_bps: u32,
    pub rx_bps: u32,
}

/// Parses ArduPilot's serial report, e.g.
/// `SERIAL3 USART3 TX*=      12 RX*=     140 TXBD=   120 RXBD=  1400 FlowCtrl=0`
/// where TXBD/RXBD are bytes per second times ten.
pub fn parse_uarts(text: &str) -> Vec<Uart> {
    text.lines()
        .filter_map(|line| {
            let mut words = line.split_whitespace();
            let serial = words.next()?.strip_prefix("SERIAL")?.parse().ok()?;
            let hw = words.next()?.to_owned();
            let field = |key: &str| -> Option<u32> {
                let at = line.find(key)? + key.len();
                line[at..].split_whitespace().next()?.parse().ok()
            };
            Some(Uart { serial, hw, tx_bps: field("TXBD=").map_or(0, |v| v / 10), rx_bps: field("RXBD=").map_or(0, |v| v / 10) })
        })
        .collect()
}

#[derive(Debug, Clone, Serialize)]
pub struct Port {
    pub n: u8,
    pub hw: Option<String>,
    /// The USB port: MAVLink only.
    pub usb: bool,
    pub protocol: f64,
    pub protocol_name: String,
    pub baud: Option<f64>,
    pub baud_name: Option<String>,
    /// Option bits set, by name.
    pub options: Vec<String>,
    pub rx_bps: Option<u32>,
    pub tx_bps: Option<u32>,
    /// What's live on it: "GPS 1: 3D fix", "receiver: signal"…
    pub live: Option<String>,
    /// The common setup it matches, by id.
    pub setup: Option<&'static str>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Setup {
    pub id: &'static str,
    pub name: &'static str,
    pub detail: &'static str,
}

#[derive(Debug, Clone, Serialize)]
pub struct Check {
    pub status: Status,
    pub title: String,
    pub detail: String,
    /// The port it's about, when one.
    pub port: Option<u8>,
    /// A setup that fixes it (on `port`, or a port the pilot picks).
    pub setup: Option<&'static str>,
}

#[derive(Debug, Clone, Serialize)]
pub struct View {
    pub ports: Vec<Port>,
    pub checks: Vec<Check>,
    pub setups: Vec<Setup>,
    /// Whether the traffic report was available.
    pub traffic: bool,
    /// Protocols, baud rates and option bits to choose from.
    pub protocols: Vec<(f64, String)>,
    pub bauds: Vec<(f64, String)>,
    pub option_bits: Vec<(u32, String)>,
}

/// What else the page knows: each GPS's fix (instance 0 first) and whether
/// the receiver has signal.
#[derive(Debug, Clone, Default)]
pub struct Live {
    pub gps_fix: Vec<(u8, &'static str)>,
    pub rc_signal: Option<bool>,
}

const NONE: f64 = -1.0;
const MAVLINK2: f64 = 2.0;
const GPS: f64 = 5.0;
const ESC_TELEMETRY: f64 = 16.0;
const RCIN: f64 = 23.0;
const DJI_FPV: f64 = 33.0;
const DISPLAYPORT: f64 = 42.0;

pub const SETUPS: [Setup; 8] = [
    Setup { id: "gps", name: "GPS", detail: "GPS receiver (ArduPilot finds its baud rate itself)" },
    Setup { id: "elrs", name: "ExpressLRS receiver", detail: "CRSF at 420 kbaud, link quality as RSSI, no flight-mode messages to the radio" },
    Setup { id: "crsf", name: "TBS Crossfire / Tracer receiver", detail: "CRSF, link quality as RSSI" },
    Setup { id: "osd", name: "Digital video OSD", detail: "DJI O3/O4, Walksnail or HDZero: MSP DisplayPort OSD" },
    Setup { id: "esc", name: "ESC telemetry", detail: "BLHeli/AM32 telemetry wire: RPM, current, temperature" },
    Setup { id: "radio", name: "Telemetry radio", detail: "SiK or RFD900 radio: MAVLink at 57600" },
    Setup { id: "mavlink", name: "MAVLink 115200", detail: "WiFi bridge, ELRS backpack or companion computer" },
    Setup { id: "off", name: "Nothing", detail: "Turn the port off" },
];

/// Protocol and baud each setup sets, to recognise a port's setup.
fn setup_of(protocol: f64, baud: Option<f64>) -> Option<&'static str> {
    Some(match (protocol as i64, baud.map(|b| b as i64)) {
        (5, _) => "gps",
        (42, _) => "osd",
        (16, _) => "esc",
        (2 | 1, Some(57)) => "radio",
        (2 | 1, Some(115)) => "mavlink",
        (-1, _) => "off",
        _ => return None,
    })
}

fn name(doc: &str, v: f64) -> String {
    paramdoc::doc(doc).and_then(|d| d.values.iter().find(|(x, _)| *x == v).map(|(_, n)| n.clone())).unwrap_or_else(|| format!("{v}"))
}

fn rate(bps: u32) -> String {
    if bps >= 1000 {
        format!("{:.1} kB/s", f64::from(bps) / 1000.0)
    } else {
        format!("{bps} B/s")
    }
}

/// GPS types that use a serial port set to GPS.
fn serial_gps(kind: f64) -> bool {
    !matches!(kind as i64, 0 | 7 | 9 | 14 | 19 | 21 | 22 | 23)
}

pub fn view(param: impl Fn(&str) -> Option<f64>, uarts: &[Uart], live: &Live) -> View {
    let p: Param = &param;
    let mut gps_seen = 0usize;
    let ports: Vec<Port> = (0..=9u8)
        .filter_map(|n| {
            let protocol = p(&format!("SERIAL{n}_PROTOCOL"))?;
            let baud = p(&format!("SERIAL{n}_BAUD"));
            let options = p(&format!("SERIAL{n}_OPTIONS")).unwrap_or(0.0) as u32;
            let uart = uarts.iter().find(|u| u.serial == n);
            let bits = paramdoc::doc("SERIAL1_OPTIONS").map(|d| d.bits.clone()).unwrap_or_default();
            let live_text = match protocol {
                GPS => {
                    let i = gps_seen;
                    gps_seen += 1;
                    live.gps_fix.get(i).map(|(_, fix)| format!("GPS {}: {fix}", i + 1))
                }
                RCIN => live.rc_signal.map(|s| if s { "receiver: signal".to_owned() } else { "receiver: no signal".to_owned() }),
                _ => None,
            }
            .or_else(|| uart.filter(|u| u.rx_bps > 0 || u.tx_bps > 0).map(|u| format!("receiving {}, sending {}", rate(u.rx_bps), rate(u.tx_bps))));
            Some(Port {
                n,
                hw: uart.map(|u| u.hw.clone()),
                usb: n == 0 || uart.is_some_and(|u| u.hw.starts_with("OTG")),
                protocol,
                protocol_name: match protocol {
                    NONE => "Nothing".to_owned(),
                    RCIN => "Receiver (RC input)".to_owned(),
                    _ => name("SERIAL1_PROTOCOL", protocol),
                },
                baud,
                baud_name: baud.map(|b| name("SERIAL1_BAUD", b)),
                options: bits.iter().filter(|(b, _)| options & (1 << b) != 0).map(|(_, n)| n.clone()).collect(),
                rx_bps: uart.map(|u| u.rx_bps),
                tx_bps: uart.map(|u| u.tx_bps),
                live: live_text,
                setup: match protocol {
                    _ if n == 0 => None,
                    // A receiver: ExpressLRS when set to 420 kbaud, otherwise not told apart.
                    RCIN if p("RC_OPTIONS").is_some_and(|o| (o as u32) & (1 << 13) != 0) => Some("elrs"),
                    RCIN => None,
                    _ => setup_of(protocol, baud),
                },
            })
        })
        .collect();

    let mut checks = Vec::new();
    for (protocol, what) in [(RCIN, "RC input"), (DISPLAYPORT, "DisplayPort OSD"), (DJI_FPV, "DJI FPV")] {
        let on: Vec<String> = ports.iter().filter(|x| x.protocol == protocol).map(|x| format!("SERIAL{}", x.n)).collect();
        if on.len() > 1 {
            checks.push(Check {
                status: Status::Warn,
                title: format!("{what} is set on {}", on.join(" and ")),
                detail: "Only one port can have it. Turn it off on the ports where nothing like it is connected.".to_owned(),
                port: None,
                setup: None,
            });
        }
    }
    let gps1 = p("GPS1_TYPE").or_else(|| p("GPS_TYPE"));
    if gps1.is_some_and(|k| k != 0.0 && serial_gps(k)) && !ports.iter().any(|x| x.protocol == GPS) {
        checks.push(Check {
            status: Status::Fail,
            title: "No port is set for the GPS".to_owned(),
            detail: "The GPS is turned on but no serial port listens to it. Choose the port it's wired to and set it up as GPS.".to_owned(),
            port: None,
            setup: Some("gps"),
        });
    }
    if !uarts.is_empty() {
        for x in ports.iter().filter(|x| !x.usb) {
            let rx = x.rx_bps.unwrap_or(0);
            let quiet_what = match x.protocol {
                GPS => Some("the GPS"),
                RCIN => Some("the receiver"),
                _ => None,
            };
            if let (Some(what), 0) = (quiet_what, rx) {
                checks.push(Check {
                    status: Status::Warn,
                    title: format!("Nothing arrives on SERIAL{} ({what})", x.n),
                    detail: format!(
                        "SERIAL{}{} receives no data. Check {what} has power and its TX wire goes to the flight controller's RX pad of this port (TX to RX, RX to TX), or that it isn't wired to another port.",
                        x.n,
                        x.hw.as_ref().map(|h| format!(" ({h})")).unwrap_or_default()
                    ),
                    port: Some(x.n),
                    setup: None,
                });
            }
            if x.protocol == NONE && rx > 0 {
                checks.push(Check {
                    status: Status::Info,
                    title: format!("Data arrives on SERIAL{}, which is off", x.n),
                    detail: format!("Something sends {} to it. If you connected a device there, set the port up for it.", rate(rx)),
                    port: Some(x.n),
                    setup: None,
                });
            }
        }
    }
    checks.sort_by_key(|c| match c.status {
        Status::Fail => 0,
        Status::Warn => 1,
        _ => 2,
    });
    let doc = |n: &str| paramdoc::doc(n);
    View {
        ports,
        checks,
        setups: SETUPS.to_vec(),
        traffic: !uarts.is_empty(),
        protocols: doc("SERIAL1_PROTOCOL").map(|d| d.values.clone()).unwrap_or_default(),
        bauds: doc("SERIAL1_BAUD").map(|d| d.values.clone()).unwrap_or_default(),
        option_bits: doc("SERIAL1_OPTIONS").map(|d| d.bits.iter().filter(|(b, _)| *b <= 3).cloned().collect()).unwrap_or_default(),
    }
}

/// The changes for a common setup on port `n`.
pub fn setup(param: impl Fn(&str) -> Option<f64>, n: u8, id: &str) -> Result<Vec<ParamChange>, String> {
    let p: Param = &param;
    let proto = format!("SERIAL{n}_PROTOCOL");
    if p(&proto).is_none() {
        return Err(format!("This board has no SERIAL{n}."));
    }
    if n == 0 {
        return Err("SERIAL0 is the USB port: it only does MAVLink.".to_owned());
    }
    let (protocol, baud, what) = match id {
        "gps" => (GPS, Some(230.0), "GPS"),
        "elrs" | "crsf" => (RCIN, Some(115.0), "RC input from the receiver"),
        "osd" => (DISPLAYPORT, Some(115.0), "DisplayPort OSD"),
        "esc" => (ESC_TELEMETRY, Some(115.0), "ESC telemetry"),
        "radio" => (MAVLINK2, Some(57.0), "MAVLink for the telemetry radio"),
        "mavlink" => (MAVLINK2, Some(115.0), "MAVLink"),
        "off" => (NONE, None, "nothing"),
        _ => return Err(format!("Unknown setup \"{id}\".")),
    };
    // One receiver and one OSD: refuse a second port.
    if matches!(protocol, RCIN | DISPLAYPORT) {
        if let Some(other) = (0..=9u8).find(|m| *m != n && p(&format!("SERIAL{m}_PROTOCOL")) == Some(protocol)) {
            return Err(format!("SERIAL{other} already has it: set that port to something else (or Nothing) first."));
        }
    }
    let mut out: Vec<ParamChange> = change(p, &proto, protocol, format!("SERIAL{n} does {what}")).into_iter().collect();
    if let Some(b) = baud {
        out.extend(change(p, &format!("SERIAL{n}_BAUD"), b, format!("SERIAL{n} at {}", name("SERIAL1_BAUD", b))));
    }
    if protocol != NONE {
        out.extend(change(p, &format!("SERIAL{n}_OPTIONS"), 0.0, "Standard wiring: not inverted, TX and RX not swapped"));
    }
    match id {
        "gps" => {
            let kind = if p("GPS1_TYPE").is_some() { "GPS1_TYPE" } else { "GPS_TYPE" };
            if p(kind) == Some(0.0) {
                out.extend(change(p, kind, 1.0, "Turn the GPS on, type detected automatically"));
            }
        }
        "elrs" | "crsf" => {
            out.extend(change(p, "RSSI_TYPE", 3.0, "Signal strength from the receiver's link quality"));
            if id == "elrs" {
                let now = p("RC_OPTIONS").unwrap_or(0.0) as u32;
                // Bit 9: no flight-mode messages to the radio; bit 13: 420 kbaud.
                out.extend(change(p, "RC_OPTIONS", f64::from(now | (1 << 9) | (1 << 13)), "ExpressLRS: 420 kbaud, no CRSF mode messages"));
            }
        }
        "osd" => out.extend(change(p, "OSD_TYPE", 5.0, "OSD drawn by the video system (MSP DisplayPort)")),
        _ => {}
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    const UARTS: &str = "UARTV1\nSERIAL0 OTG1  TX =     900 RX =      30 TXBD=  9000 RXBD=   300 FlowCtrl=0\nSERIAL1 UART7 TX*=       0 RX*=       0 TXBD=     0 RXBD=     0 FlowCtrl=0\nSERIAL3 USART3 TX*=      12 RX*=     140 TXBD=   120 RXBD=  1400 FlowCtrl=0\nSERIAL4 UART4 TX*=       0 RX*=      50 TXBD=     0 RXBD=   500 FlowCtrl=0\nSERIAL6 UART6 TX*=       0 RX*=       0 TXBD=     0 RXBD=     0 FlowCtrl=0\n";

    fn params() -> HashMap<String, f64> {
        let mut m: HashMap<String, f64> = HashMap::new();
        for (n, pr, b) in [(0, 2.0, 115.0), (1, 2.0, 57.0), (3, 5.0, 230.0), (4, -1.0, 57.0), (6, 23.0, 115.0)] {
            m.insert(format!("SERIAL{n}_PROTOCOL"), pr);
            m.insert(format!("SERIAL{n}_BAUD"), b);
            if n > 0 {
                m.insert(format!("SERIAL{n}_OPTIONS"), 0.0);
            }
        }
        m.extend([("GPS1_TYPE".to_string(), 1.0), ("RSSI_TYPE".into(), 0.0), ("RC_OPTIONS".into(), 32.0), ("OSD_TYPE".into(), 1.0)]);
        m
    }

    #[test]
    fn parses_the_report() {
        let u = parse_uarts(UARTS);
        assert_eq!(u.len(), 5);
        assert_eq!((u[2].serial, u[2].hw.as_str(), u[2].rx_bps, u[2].tx_bps), (3, "USART3", 140, 12));
        assert_eq!(u[0].hw, "OTG1");
    }

    #[test]
    fn ports_live_and_checks() {
        let m = params();
        let live = Live { gps_fix: vec![(0, "3D fix")], rc_signal: Some(false) };
        let v = view(|n| m.get(n).copied(), &parse_uarts(UARTS), &live);
        let row = |n: u8| v.ports.iter().find(|p| p.n == n).unwrap();
        assert!(row(0).usb && row(0).setup.is_none());
        assert_eq!((row(1).setup, row(1).baud_name.as_deref()), (Some("radio"), Some("57600")));
        assert_eq!((row(3).live.as_deref(), row(3).hw.as_deref()), (Some("GPS 1: 3D fix"), Some("USART3")));
        assert_eq!(row(6).live.as_deref(), Some("receiver: no signal"));
        let t: Vec<&str> = v.checks.iter().map(|c| c.title.as_str()).collect();
        assert_eq!(t, ["Nothing arrives on SERIAL6 (the receiver)", "Data arrives on SERIAL4, which is off"], "{t:?}");
        assert!(v.traffic);
    }

    #[test]
    fn missing_gps_port_and_duplicates() {
        let mut m = params();
        m.insert("SERIAL3_PROTOCOL".into(), 23.0);
        let v = view(|n| m.get(n).copied(), &[], &Live::default());
        let t: Vec<&str> = v.checks.iter().map(|c| c.title.as_str()).collect();
        assert_eq!(t, ["No port is set for the GPS", "RC input is set on SERIAL3 and SERIAL6"]);
        assert_eq!(v.checks[0].setup, Some("gps"));
        assert!(!v.traffic);
    }

    #[test]
    fn setups() {
        let m = params();
        let p = |n: &str| m.get(n).copied();
        let c = setup(p, 4, "gps").unwrap();
        let got: Vec<(&str, f64)> = c.iter().map(|c| (c.name.as_str(), c.value)).collect();
        assert_eq!(got, [("SERIAL4_PROTOCOL", 5.0), ("SERIAL4_BAUD", 230.0)], "options already 0");
        let c = setup(p, 1, "osd").unwrap();
        assert!(c.iter().any(|c| c.name == "OSD_TYPE" && c.value == 5.0));
        assert!(setup(p, 4, "elrs").unwrap_err().contains("SERIAL6 already has it"));
        let c = setup(p, 6, "elrs").unwrap();
        let get = |n: &str| c.iter().find(|x| x.name == n).map(|x| x.value);
        assert_eq!((get("RSSI_TYPE"), get("RC_OPTIONS")), (Some(3.0), Some(f64::from(32 | 512 | 8192))));
        assert!(setup(p, 0, "gps").unwrap_err().contains("USB"));
        assert_eq!(setup(p, 4, "off").unwrap(), vec![], "already off");
    }
}
