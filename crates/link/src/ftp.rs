//! The part of MAVLink FTP (<https://mavlink.io/en/services/ftp.html>) used to
//! list the log directory, delete single log files (the log protocol can
//! only erase all logs at once) and read ArduPilot's packed parameter file
//! with defaults.

pub const TERMINATE_SESSION: u8 = 1;
pub const RESET_SESSIONS: u8 = 2;
pub const LIST_DIRECTORY: u8 = 3;
pub const OPEN_FILE_RO: u8 = 4;
pub const READ_FILE: u8 = 5;
pub const REMOVE_FILE: u8 = 8;
pub const ACK: u8 = 128;
pub const NAK: u8 = 129;

pub const ERR_FAIL: u8 = 1;
pub const ERR_EOF: u8 = 6;
pub const ERR_FILE_NOT_FOUND: u8 = 10;

/// Payload size of FILE_TRANSFER_PROTOCOL.
pub const PAYLOAD: usize = 251;
const HEADER: usize = 12;
pub const DATA_MAX: usize = PAYLOAD - HEADER;

/// One FTP request or reply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    pub seq: u16,
    pub session: u8,
    pub opcode: u8,
    pub req_opcode: u8,
    pub offset: u32,
    pub data: Vec<u8>,
}

impl Packet {
    pub fn new(seq: u16, opcode: u8, offset: u32, data: &[u8]) -> Self {
        Self { seq, session: 0, opcode, req_opcode: 0, offset, data: data[..data.len().min(DATA_MAX)].to_vec() }
    }

    /// The reply to `req`: the sequence number is the request's plus one.
    pub fn reply(req: &Packet, opcode: u8, data: &[u8]) -> Self {
        Self { req_opcode: req.opcode, offset: req.offset, ..Self::new(req.seq.wrapping_add(1), opcode, 0, data) }
    }

    pub fn encode(&self) -> [u8; PAYLOAD] {
        let mut p = [0u8; PAYLOAD];
        p[0..2].copy_from_slice(&self.seq.to_le_bytes());
        p[2] = self.session;
        p[3] = self.opcode;
        p[4] = self.data.len() as u8;
        p[5] = self.req_opcode;
        p[8..12].copy_from_slice(&self.offset.to_le_bytes());
        p[HEADER..HEADER + self.data.len()].copy_from_slice(&self.data);
        p
    }

    pub fn decode(p: &[u8; PAYLOAD]) -> Self {
        let size = (p[4] as usize).min(DATA_MAX);
        Self {
            seq: u16::from_le_bytes([p[0], p[1]]),
            session: p[2],
            opcode: p[3],
            req_opcode: p[5],
            offset: u32::from_le_bytes([p[8], p[9], p[10], p[11]]),
            data: p[HEADER..HEADER + size].to_vec(),
        }
    }
}

/// An entry of a ListDirectory reply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Entry {
    File { name: String, size: u64 },
    Dir(String),
    /// An entry the vehicle couldn't describe; still counts for the offset.
    Skip,
}

/// Parses `F<name>\t<size>`, `D<name>` and `S` entries, separated by NULs.
pub fn parse_entries(data: &[u8]) -> Vec<Entry> {
    data.split(|b| *b == 0)
        .filter(|e| !e.is_empty())
        .map(|e| {
            let text = String::from_utf8_lossy(&e[1..]);
            match e[0] {
                b'F' => {
                    let (name, size) = text.split_once('\t').unwrap_or((&text, "0"));
                    Entry::File { name: name.to_owned(), size: size.trim().parse().unwrap_or(0) }
                }
                b'D' => Entry::Dir(text.into_owned()),
                _ => Entry::Skip,
            }
        })
        .collect()
}

/// Encodes entries into ListDirectory replies' data, as many as fit.
pub fn encode_entries<'a>(entries: impl Iterator<Item = &'a Entry>) -> Vec<u8> {
    let mut out = Vec::new();
    for e in entries {
        let text = match e {
            Entry::File { name, size } => format!("F{name}\t{size}"),
            Entry::Dir(name) => format!("D{name}"),
            Entry::Skip => "S".to_owned(),
        };
        if out.len() + text.len() + 1 > DATA_MAX {
            break;
        }
        out.extend_from_slice(text.as_bytes());
        out.push(0);
    }
    out
}

/// One entry of ArduPilot's `@PARAM/param.pck`.
#[derive(Debug, Clone, PartialEq)]
pub struct PackedParam {
    pub name: String,
    pub value: f32,
    pub default: Option<f32>,
}

const PCK_MAGIC: u16 = 0x671b;
const PCK_MAGIC_DEFAULTS: u16 = 0x671c;

/// Parses `@PARAM/param.pck`: a header (magic, count, total), then per
/// parameter a type/flags byte, a prefix-length/name-length byte, the name
/// after the prefix it shares with the previous name, the value and (with
/// flag 1) the default. Zero bytes between entries are padding. In a file
/// with defaults, an entry without one is at its default.
pub fn parse_param_pck(data: &[u8]) -> Result<Vec<PackedParam>, String> {
    if data.len() < 6 {
        return Err("parameter file too short".to_owned());
    }
    let magic = u16::from_le_bytes([data[0], data[1]]);
    if magic != PCK_MAGIC && magic != PCK_MAGIC_DEFAULTS {
        return Err(format!("unexpected parameter file magic {magic:#06x}"));
    }
    let with_defaults = magic == PCK_MAGIC_DEFAULTS;
    let count = u16::from_le_bytes([data[2], data[3]]) as usize;
    let mut out = Vec::with_capacity(count);
    let mut last = String::new();
    let mut i = 6;
    while out.len() < count && i < data.len() {
        if data[i] == 0 {
            i += 1;
            continue;
        }
        let (kind, flags) = (data[i] & 0x0f, data[i] >> 4);
        let common = (data.get(i + 1).ok_or("truncated parameter file")? & 0x0f) as usize;
        let name_len = (data[i + 1] >> 4) as usize + 1;
        i += 2;
        let suffix = data.get(i..i + name_len).ok_or("truncated parameter file")?;
        let name = format!("{}{}", last.get(..common).ok_or("bad name prefix in parameter file")?, String::from_utf8_lossy(suffix));
        i += name_len;
        let size = match kind {
            1 => 1,
            2 => 2,
            3 | 4 => 4,
            _ => return Err(format!("unknown parameter type {kind} for {name}")),
        };
        let read = |at: usize| -> Result<f32, String> {
            let b = data.get(at..at + size).ok_or("truncated parameter file")?;
            Ok(match kind {
                1 => b[0] as i8 as f32,
                2 => i16::from_le_bytes([b[0], b[1]]) as f32,
                3 => i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f32,
                _ => f32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            })
        };
        let value = read(i)?;
        i += size;
        let default = if flags & 1 != 0 {
            let d = read(i)?;
            i += size;
            Some(d)
        } else {
            with_defaults.then_some(value)
        };
        last = name.clone();
        out.push(PackedParam { name, value, default });
    }
    Ok(out)
}

/// Writes a `param.pck` (with defaults), for the simulated vehicle. Values
/// are stored as floats except integer types, given as (name, value,
/// default, integer type 1-3 or 4 for float).
pub fn encode_param_pck(params: &[(String, f32, f32, u8)]) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&PCK_MAGIC_DEFAULTS.to_le_bytes());
    out.extend_from_slice(&(params.len() as u16).to_le_bytes());
    out.extend_from_slice(&(params.len() as u16).to_le_bytes());
    let mut last = "";
    for (name, value, default, kind) in params {
        let common = last.bytes().zip(name.bytes()).take_while(|(a, b)| a == b).count().min(15).min(name.len() - 1);
        let suffix = &name.as_bytes()[common..];
        // Like ArduPilot, only defaults that differ from the value are stored.
        let has_default = default != value;
        out.push(kind | (u8::from(has_default) << 4));
        out.push(common as u8 | (((suffix.len() - 1) as u8) << 4));
        out.extend_from_slice(suffix);
        for v in [value, default].into_iter().take(1 + usize::from(has_default)) {
            match kind {
                1 => out.push(*v as i8 as u8),
                2 => out.extend_from_slice(&(*v as i16).to_le_bytes()),
                3 => out.extend_from_slice(&(*v as i32).to_le_bytes()),
                _ => out.extend_from_slice(&v.to_le_bytes()),
            }
        }
        last = name;
    }
    out
}

/// Number of a log file: `00000012.BIN` → 12.
pub fn log_number(name: &str) -> Option<u32> {
    let stem = name.strip_suffix(".BIN").or_else(|| name.strip_suffix(".bin"))?;
    (!stem.is_empty() && stem.bytes().all(|b| b.is_ascii_digit())).then(|| stem.parse().ok())?
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packets_round_trip() {
        let p = Packet::new(41, LIST_DIRECTORY, 3, b"/APM/LOGS");
        let q = Packet::decode(&p.encode());
        assert_eq!(p, q);
        let r = Packet::reply(&q, ACK, b"x");
        assert_eq!((r.seq, r.req_opcode, r.opcode), (42, LIST_DIRECTORY, ACK));
    }

    #[test]
    fn entries_round_trip() {
        let entries = vec![
            Entry::File { name: "00000012.BIN".into(), size: 89_600_123 },
            Entry::Dir("..".into()),
            Entry::Skip,
            Entry::File { name: "LASTLOG.TXT".into(), size: 3 },
        ];
        assert_eq!(parse_entries(&encode_entries(entries.iter())), entries);
    }

    #[test]
    fn param_pck_round_trips_with_shared_prefixes() {
        let params = vec![
            ("ATC_RAT_PIT_P".to_owned(), 0.135, 0.135, 4),
            ("ATC_RAT_RLL_P".to_owned(), 0.08, 0.135, 4),
            ("ATC_RAT_RLL_SMAX".to_owned(), 50.0, 0.0, 4),
            ("LOG_BITMASK".to_owned(), 180223.0, 176126.0, 3),
            ("RSSI_TYPE".to_owned(), 3.0, 0.0, 1),
            ("SERIAL6_BAUD".to_owned(), -1.0, 57.0, 2),
        ];
        let mut bytes = encode_param_pck(&params);
        // Padding between entries is skipped.
        bytes.insert(6, 0);
        let parsed = parse_param_pck(&bytes).unwrap();
        let back: Vec<(String, f32, Option<f32>)> = parsed.into_iter().map(|p| (p.name, p.value, p.default)).collect();
        let want: Vec<(String, f32, Option<f32>)> = params.into_iter().map(|(n, v, d, _)| (n, v, Some(d))).collect();
        assert_eq!(back, want);
        assert!(parse_param_pck(&[1, 2, 3]).is_err());
    }

    #[test]
    fn log_numbers() {
        assert_eq!(log_number("00000012.BIN"), Some(12));
        assert_eq!(log_number("7.BIN"), Some(7));
        assert_eq!(log_number("LASTLOG.TXT"), None);
        assert_eq!(log_number(".BIN"), None);
    }
}
