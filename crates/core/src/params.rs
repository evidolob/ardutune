//! Parameter files: Mission Planner / MAVProxy (`NAME,VALUE`) and
//! QGroundControl (tab-separated `SYSID COMPID NAME VALUE TYPE`).

pub fn format_value(v: f64) -> String {
    if v.fract() == 0.0 && v.abs() < 1e15 {
        format!("{}", v as i64)
    } else {
        let s = format!("{v:.6}");
        s.trim_end_matches('0').trim_end_matches('.').to_owned()
    }
}

/// Parses a parameter file into (name, value) pairs, in file order.
pub fn parse(text: &str) -> Result<Vec<(String, f64)>, String> {
    let mut out = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let fields: Vec<&str> = if line.contains('\t') {
            line.split('\t').map(str::trim).collect()
        } else if line.contains(',') {
            line.split(',').map(str::trim).collect()
        } else {
            line.split_whitespace().collect()
        };
        let (name, value) = match fields.len() {
            2 | 3 => (fields[0], fields[1]),
            5 => (fields[2], fields[3]),
            _ => return Err(format!("line {}: expected NAME,VALUE but found {raw:?}", n + 1)),
        };
        let valid = !name.is_empty()
            && name.len() <= 16
            && name.chars().all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_');
        if !valid {
            return Err(format!("line {}: {name:?} is not a parameter name", n + 1));
        }
        let value: f64 = value.parse().map_err(|_| format!("line {}: {value:?} is not a number", n + 1))?;
        out.push((name.to_owned(), value));
    }
    Ok(out)
}

/// Writes a Mission Planner style file. `header` lines become comments.
/// Values are written with their `Display` form, which for `f32` read from
/// a vehicle is the shortest text that restores the exact value.
pub fn write<'a, V: std::fmt::Display>(header: &str, params: impl IntoIterator<Item = (&'a str, V)>) -> String {
    let mut out: String = header.lines().map(|l| format!("# {l}\n")).collect();
    for (name, value) in params {
        out += &format!("{name},{value}\n");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_common_formats() {
        let mp = "# comment\nLOG_BITMASK,180223\nATC_RAT_RLL_P,0.135 # inline\n\n";
        assert_eq!(parse(mp).unwrap(), vec![("LOG_BITMASK".into(), 180223.0), ("ATC_RAT_RLL_P".into(), 0.135)]);
        let qgc = "# Onboard parameters\n1\t1\tLOG_BITMASK\t180223\t6\n";
        assert_eq!(parse(qgc).unwrap(), vec![("LOG_BITMASK".into(), 180223.0)]);
        assert_eq!(parse("INS_RAW_LOG_OPT 9").unwrap(), vec![("INS_RAW_LOG_OPT".into(), 9.0)]);
        assert!(parse("lower,1").is_err());
        assert!(parse("LOG_BITMASK,abc").unwrap_err().contains("line 1"));
    }

    #[test]
    fn round_trips() {
        let text = write("backup\nsecond line", [("A_B", 1.0f32), ("C", 0.135), ("D", 1.5e-7)]);
        assert!(text.starts_with("# backup\n# second line\nA_B,1\nC,0.135\n"));
        let back = parse(&text).unwrap();
        assert_eq!(back.iter().map(|(_, v)| *v as f32).collect::<Vec<_>>(), vec![1.0f32, 0.135, 1.5e-7]);
    }

    #[test]
    fn formats_values() {
        assert_eq!(format_value(180223.0), "180223");
        assert_eq!(format_value(0.135), "0.135");
        assert_eq!(format_value(-2.5), "-2.5");
    }
}
