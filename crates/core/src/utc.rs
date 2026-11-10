//! UTC calendar formatting for file names and report headers, without a
//! date library.

/// `YYYYMMDD-HHMMSS` in UTC.
pub fn file_stamp(secs: u64) -> String {
    let (y, mo, d, h, mi, s) = civil(secs);
    format!("{y:04}{mo:02}{d:02}-{h:02}{mi:02}{s:02}")
}

/// `YYYY-MM-DD HH:MM:SS` in UTC.
pub fn date_time(secs: u64) -> String {
    let (y, mo, d, h, mi, s) = civil(secs);
    format!("{y:04}-{mo:02}-{d:02} {h:02}:{mi:02}:{s:02}")
}

/// Unix seconds to UTC calendar fields (Howard Hinnant's days-to-civil).
fn civil(secs: u64) -> (i64, u32, u32, u64, u64, u64) {
    let z = (secs / 86_400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe + era * 400 + i64::from(m <= 2);
    let rem = secs % 86_400;
    (y, m, d, rem / 3600, rem % 3600 / 60, rem % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_utc_times() {
        assert_eq!(date_time(0), "1970-01-01 00:00:00");
        assert_eq!(file_stamp(1_762_786_800), "20251110-150000");
        assert_eq!(date_time(951_782_400), "2000-02-29 00:00:00");
    }
}
