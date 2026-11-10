//! Serial port discovery: find the flight controller's USB port.

use serialport::SerialPortType;

/// USB vendor ids used by common ArduPilot flight controllers.
const AUTOPILOT_VIDS: [u16; 5] = [
    0x1209, // pid.codes, used by ArduPilot ChibiOS builds
    0x2DAE, // CubePilot
    0x26AC, // 3D Robotics / PX4 bootloaders
    0x3162, // Holybro
    0x0483, // STMicroelectronics, many boards
];

/// Baud rate for USB connections; ignored by USB CDC but required by the address format.
pub const USB_BAUD: u32 = 115_200;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    pub name: String,
    pub description: String,
    pub usb_id: Option<(u16, u16)>,
    pub likely_autopilot: bool,
}

impl PortInfo {
    pub fn address(&self) -> String {
        format!("serial:{}:{USB_BAUD}", self.name)
    }
}

/// Serial ports on this computer. On macOS only the `cu.*` device of each
/// pair is listed: opening `tty.*` blocks until carrier detect.
pub fn list() -> Vec<PortInfo> {
    let ports = serialport::available_ports().unwrap_or_default();
    let names: Vec<String> = ports.iter().map(|p| p.port_name.clone()).collect();
    ports
        .into_iter()
        .filter(|p| match p.port_name.strip_prefix("/dev/tty.") {
            Some(rest) => !names.contains(&format!("/dev/cu.{rest}")),
            None => true,
        })
        .map(|p| match p.port_type {
            SerialPortType::UsbPort(usb) => {
                let text = [usb.manufacturer.as_deref(), usb.product.as_deref()]
                    .into_iter()
                    .flatten()
                    .collect::<Vec<_>>()
                    .join(" ");
                let likely = AUTOPILOT_VIDS.contains(&usb.vid)
                    || ["ardupilot", "px4", "cube"].iter().any(|k| text.to_lowercase().contains(k));
                PortInfo {
                    name: p.port_name,
                    description: if text.is_empty() { "USB serial".to_owned() } else { text },
                    usb_id: Some((usb.vid, usb.pid)),
                    likely_autopilot: likely,
                }
            }
            other => PortInfo {
                name: p.port_name,
                description: format!("{other:?}"),
                usb_id: None,
                likely_autopilot: false,
            },
        })
        .collect()
}

/// Picks the connection address for a single attached flight controller.
pub fn auto_address() -> Result<String, String> {
    choose(&list())
}

fn choose(ports: &[PortInfo]) -> Result<String, String> {
    let likely: Vec<&PortInfo> = ports.iter().filter(|p| p.likely_autopilot).collect();
    let usb: Vec<&PortInfo> = ports.iter().filter(|p| p.usb_id.is_some()).collect();
    match (likely.as_slice(), usb.as_slice()) {
        ([one], _) => Ok(one.address()),
        ([], [one]) => Ok(one.address()),
        ([], []) => Err("no USB flight controller found. Plug it in, or pass --conn \
                         (e.g. udpin:0.0.0.0:14550 for a telemetry link)"
            .to_owned()),
        _ => {
            let names: Vec<&str> =
                (if likely.is_empty() { &usb } else { &likely }).iter().map(|p| p.name.as_str()).collect();
            Err(format!("several serial ports found ({}); choose one with --conn serial:<port>:{USB_BAUD}", names.join(", ")))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn port(name: &str, usb: bool, likely: bool) -> PortInfo {
        PortInfo {
            name: name.into(),
            description: String::new(),
            usb_id: usb.then_some((0x1209, 0x5741)),
            likely_autopilot: likely,
        }
    }

    #[test]
    fn chooses_single_autopilot() {
        let ports = [port("/dev/cu.Bluetooth", false, false), port("/dev/cu.usbmodem1", true, true)];
        assert_eq!(choose(&ports).unwrap(), "serial:/dev/cu.usbmodem1:115200");
        assert_eq!(choose(&[port("COM4", true, false)]).unwrap(), "serial:COM4:115200");
        assert!(choose(&[]).is_err());
        assert!(choose(&[port("a", true, true), port("b", true, true)]).unwrap_err().contains("a, b"));
    }
}
