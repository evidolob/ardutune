//! Reader for ArduPilot DataFlash (`.bin`) logs.
//!
//! The format is self-describing: every message type is declared by an FMT
//! message earlier in the file, so no per-firmware message tables are needed.
//! Each message is `0xA3 0x95 <type>` followed by the fields packed
//! little-endian as described by its FMT format string.

use std::collections::{BTreeMap, HashMap};

use crate::log::{Column, Log, ParseStats, Table};

pub const HEAD1: u8 = 0xA3;
pub const HEAD2: u8 = 0x95;
pub const FMT_TYPE: u8 = 0x80;
/// Total length of an FMT message, header included.
pub const FMT_LEN: usize = 89;
pub const HEADER_LEN: usize = 3;

/// Storage type of a single field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prim {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F16,
    F32,
    F64,
    /// NUL-padded text of the given byte length.
    Text(usize),
    /// `int16[32]`, used by batch-sampler data (ISBD).
    I16x32,
}

impl Prim {
    pub fn size(self) -> usize {
        match self {
            Prim::I8 | Prim::U8 => 1,
            Prim::I16 | Prim::U16 | Prim::F16 => 2,
            Prim::I32 | Prim::U32 | Prim::F32 => 4,
            Prim::I64 | Prim::U64 | Prim::F64 => 8,
            Prim::Text(n) => n,
            Prim::I16x32 => 64,
        }
    }
}

/// A field's storage type plus the multiplier that converts it to real units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FieldKind {
    pub prim: Prim,
    pub scale: f64,
}

impl FieldKind {
    /// Decodes one character of an FMT format string.
    pub fn from_char(c: char) -> Option<Self> {
        use Prim::*;
        let (prim, scale) = match c {
            'b' => (I8, 1.0),
            'B' | 'M' => (U8, 1.0),
            'h' => (I16, 1.0),
            'H' => (U16, 1.0),
            'i' => (I32, 1.0),
            'I' => (U32, 1.0),
            'q' => (I64, 1.0),
            'Q' => (U64, 1.0),
            'g' => (F16, 1.0),
            'f' => (F32, 1.0),
            'd' => (F64, 1.0),
            'n' => (Text(4), 1.0),
            'N' => (Text(16), 1.0),
            'Z' => (Text(64), 1.0),
            'c' => (I16, 0.01),
            'C' => (U16, 0.01),
            'e' => (I32, 0.01),
            'E' => (U32, 0.01),
            'L' => (I32, 1e-7),
            'a' => (I16x32, 1.0),
            _ => return None,
        };
        Some(Self { prim, scale })
    }
}

/// A message type declared by an FMT message.
#[derive(Debug, Clone)]
pub struct Format {
    pub type_id: u8,
    /// Total message length, header included.
    pub length: usize,
    pub name: String,
    pub format: String,
    pub labels: Vec<String>,
    /// `None` when the format string is unknown or disagrees with `length`
    /// or `labels`; such messages are skipped by length.
    pub kinds: Option<Vec<FieldKind>>,
}

fn parse_fmt(p: &[u8]) -> Format {
    let type_id = p[0];
    let length = p[1] as usize;
    let name = decode_text(&p[2..6]);
    let format = decode_text(&p[6..22]);
    let labels: Vec<String> = decode_text(&p[22..86])
        .split(',')
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .collect();
    let kinds: Option<Vec<FieldKind>> = format.chars().map(FieldKind::from_char).collect();
    let kinds = kinds.filter(|k| {
        k.len() == labels.len()
            && HEADER_LEN + k.iter().map(|f| f.prim.size()).sum::<usize>() == length
    });
    Format { type_id, length, name, format, labels, kinds }
}

/// Parses a whole log held in memory. Never fails: corrupt regions are
/// skipped by scanning for the next message header, and counted in
/// [`ParseStats`]. A message is only accepted when the next one starts right
/// after it (or the file ends), which rejects header bytes that occur by
/// chance inside damaged data.
pub fn parse(bytes: &[u8]) -> Log {
    let mut formats: HashMap<u8, Format> = HashMap::new();
    let mut tables: BTreeMap<String, Table> = BTreeMap::new();
    let mut stats = ParseStats { bytes: bytes.len(), ..Default::default() };

    let mut i = 0;
    while i + HEADER_LEN <= bytes.len() {
        if bytes[i] != HEAD1 || bytes[i + 1] != HEAD2 {
            i += 1;
            stats.skipped_bytes += 1;
            continue;
        }
        let t = bytes[i + 2];

        if t == FMT_TYPE {
            if i + FMT_LEN > bytes.len() {
                stats.truncated = true;
                break;
            }
            if !next_is_header(bytes, i + FMT_LEN) {
                i += 1;
                stats.skipped_bytes += 1;
                continue;
            }
            let fmt = parse_fmt(&bytes[i + HEADER_LEN..i + FMT_LEN]);
            if fmt.type_id != FMT_TYPE {
                formats.insert(fmt.type_id, fmt);
            }
            stats.messages += 1;
            i += FMT_LEN;
            continue;
        }

        let Some(fmt) = formats.get(&t).filter(|f| f.length >= HEADER_LEN) else {
            i += 1;
            stats.skipped_bytes += 1;
            continue;
        };
        if i + fmt.length > bytes.len() {
            stats.truncated = true;
            break;
        }
        if !next_is_header(bytes, i + fmt.length) {
            i += 1;
            stats.skipped_bytes += 1;
            continue;
        }
        let payload = &bytes[i + HEADER_LEN..i + fmt.length];
        match &fmt.kinds {
            Some(kinds) => {
                let table = tables
                    .entry(fmt.name.clone())
                    .or_insert_with(|| Table::new(&fmt.name, &fmt.labels, kinds));
                // A name redefined with a different layout cannot share columns.
                if table.kinds == *kinds {
                    decode_into(table, payload);
                } else {
                    stats.undecodable += 1;
                }
            }
            None => stats.undecodable += 1,
        }
        stats.messages += 1;
        i += fmt.length;
    }

    Log::new(tables, stats)
}

/// True when a message header starts at `pos`, or the file ends before one could.
fn next_is_header(bytes: &[u8], pos: usize) -> bool {
    pos + 2 > bytes.len() || (bytes[pos] == HEAD1 && bytes[pos + 1] == HEAD2)
}

fn decode_into(table: &mut Table, mut p: &[u8]) {
    for (col, kind) in table.columns.iter_mut().zip(&table.kinds) {
        let (b, rest) = p.split_at(kind.prim.size());
        p = rest;
        match col {
            Column::Num(v) => v.push(decode_num(kind.prim, b) * kind.scale),
            Column::Text(v) => v.push(decode_text(b)),
            Column::I16x32(v) => {
                let mut a = [0i16; 32];
                for (dst, c) in a.iter_mut().zip(b.as_chunks::<2>().0) {
                    *dst = i16::from_le_bytes([c[0], c[1]]);
                }
                v.push(a);
            }
        }
    }
}

fn decode_num(prim: Prim, b: &[u8]) -> f64 {
    macro_rules! le {
        ($t:ty) => {
            <$t>::from_le_bytes(b.try_into().unwrap()) as f64
        };
    }
    match prim {
        Prim::I8 => b[0] as i8 as f64,
        Prim::U8 => b[0] as f64,
        Prim::I16 => le!(i16),
        Prim::U16 => le!(u16),
        Prim::I32 => le!(i32),
        Prim::U32 => le!(u32),
        Prim::I64 => le!(i64),
        Prim::U64 => le!(u64),
        Prim::F16 => f16_to_f32(u16::from_le_bytes([b[0], b[1]])) as f64,
        Prim::F32 => le!(f32),
        Prim::F64 => le!(f64),
        Prim::Text(_) | Prim::I16x32 => unreachable!("not a numeric field"),
    }
}

fn decode_text(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).trim().to_owned()
}

/// IEEE 754 half precision to single precision.
pub fn f16_to_f32(h: u16) -> f32 {
    let sign = if h & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = ((h >> 10) & 0x1f) as i32;
    let mant = (h & 0x3ff) as f32;
    let v = match exp {
        0 => mant * 2f32.powi(-24),
        31 if mant == 0.0 => f32::INFINITY,
        31 => f32::NAN,
        _ => (1.0 + mant / 1024.0) * 2f32.powi(exp - 15),
    };
    sign * v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{LogWriter, V};

    #[test]
    fn decodes_every_numeric_kind() {
        let mut w = LogWriter::new();
        let t = w.define("TST", "QbBhHiIqfdcCeEL", "TimeUS,b,B,h,H,i,I,q,f,d,c,C,e,E,L");
        w.write(
            t,
            &[
                V::I(1_000_000),
                V::I(-5),
                V::I(200),
                V::I(-30000),
                V::I(60000),
                V::I(-2_000_000_000),
                V::I(4_000_000_000),
                V::I(-9_000_000_000),
                V::F(1.5),
                V::F(-2.25),
                V::F(-12.34),
                V::F(567.89),
                V::F(-1234.56),
                V::F(9876.54),
                V::F(50.1234567),
            ],
        );
        let log = parse(&w.into_bytes());
        let t = log.table("TST").unwrap();
        let get = |f: &str| t.num(f).unwrap()[0];
        assert_eq!(get("TimeUS"), 1_000_000.0);
        assert_eq!(get("b"), -5.0);
        assert_eq!(get("B"), 200.0);
        assert_eq!(get("h"), -30000.0);
        assert_eq!(get("H"), 60000.0);
        assert_eq!(get("i"), -2_000_000_000.0);
        assert_eq!(get("I"), 4_000_000_000.0);
        assert_eq!(get("q"), -9_000_000_000.0);
        assert_eq!(get("f"), 1.5);
        assert_eq!(get("d"), -2.25);
        assert!((get("c") + 12.34).abs() < 1e-9);
        assert!((get("C") - 567.89).abs() < 1e-9);
        assert!((get("e") + 1234.56).abs() < 1e-9);
        assert!((get("E") - 9876.54).abs() < 1e-9);
        assert!((get("L") - 50.1234567).abs() < 1e-9);
    }

    #[test]
    fn decodes_text_and_arrays() {
        let mut w = LogWriter::new();
        let t = w.define("TXT", "QnNZa", "TimeUS,n,N,Z,a");
        let mut arr = [0i16; 32];
        arr[0] = -7;
        arr[31] = 1234;
        w.write(t, &[V::I(1), V::S("ab"), V::S("LOG_BITMASK"), V::S("hello world"), V::A(arr)]);
        let log = parse(&w.into_bytes());
        let t = log.table("TXT").unwrap();
        assert_eq!(t.text("n").unwrap()[0], "ab");
        assert_eq!(t.text("N").unwrap()[0], "LOG_BITMASK");
        assert_eq!(t.text("Z").unwrap()[0], "hello world");
        assert_eq!(t.arrays("a").unwrap()[0], arr);
    }

    #[test]
    fn half_floats() {
        assert_eq!(f16_to_f32(0x3C00), 1.0);
        assert_eq!(f16_to_f32(0xC000), -2.0);
        assert_eq!(f16_to_f32(0x0000), 0.0);
        assert!((f16_to_f32(0x3555) - 0.333).abs() < 1e-3);
    }

    #[test]
    fn resyncs_after_garbage_and_stops_at_truncation() {
        let mut w = LogWriter::new();
        let t = w.define("VAL", "Qf", "TimeUS,X");
        w.write(t, &[V::I(1), V::F(1.0)]);
        w.raw(&[0x00, 0xA3, 0x11, 0x95, 0xFF]);
        w.write(t, &[V::I(2), V::F(2.0)]);
        let mut bytes = w.into_bytes();
        // Append the first half of a copy of the last (15-byte) message.
        let tail = bytes[bytes.len() - 15..bytes.len() - 7].to_vec();
        bytes.extend_from_slice(&tail);

        let log = parse(&bytes);
        // The message right before the garbage is rejected too: nothing
        // proves where it ends.
        assert_eq!(log.table("VAL").unwrap().num("X").unwrap(), &[2.0]);
        assert_eq!(log.stats.skipped_bytes, 15 + 5);
        assert!(log.stats.truncated);
    }

    #[test]
    fn skips_unknown_format_chars_by_length() {
        let mut w = LogWriter::new();
        let good = w.define("OK", "Qf", "TimeUS,X");
        let bad = w.define_raw("BAD", "Q?", "TimeUS,Y", 3 + 8 + 2);
        w.write_raw(bad, &[0u8; 10]);
        w.write(good, &[V::I(5), V::F(3.0)]);
        let log = parse(&w.into_bytes());
        assert!(log.table("BAD").is_none());
        assert_eq!(log.stats.undecodable, 1);
        assert_eq!(log.table("OK").unwrap().num("X").unwrap(), &[3.0]);
    }
}
