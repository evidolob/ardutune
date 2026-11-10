//! The part of MAVLink FTP (<https://mavlink.io/en/services/ftp.html>) used to
//! list the log directory and delete single log files: the log protocol can
//! only erase all logs at once.

pub const LIST_DIRECTORY: u8 = 3;
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
    fn log_numbers() {
        assert_eq!(log_number("00000012.BIN"), Some(12));
        assert_eq!(log_number("7.BIN"), Some(7));
        assert_eq!(log_number("LASTLOG.TXT"), None);
        assert_eq!(log_number(".BIN"), None);
    }
}
