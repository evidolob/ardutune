//! Commands that talk to a vehicle over MAVLink.

use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context, Result};
use ardutune_core::params::{self, format_value};
use ardutune_link::{plan, ports, LogEntry, Link, Params};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

pub fn connect(conn: &str) -> Result<Link> {
    let address = if conn == "auto" { ports::auto_address().map_err(anyhow::Error::msg)? } else { conn.to_owned() };
    eprintln!("Connecting to {address} ...");
    let link = Link::connect(&address, CONNECT_TIMEOUT)?;
    let v = link.vehicle();
    eprintln!(
        "Connected: {} {} (system {}), {}",
        if v.is_ardupilot() { "ArduPilot" } else { "non-ArduPilot" },
        v.type_name(),
        v.system_id,
        if v.armed { "ARMED" } else { "disarmed" }
    );
    Ok(link)
}

pub fn print_ports() {
    let ports = ports::list();
    if ports.is_empty() {
        println!("No serial ports found.");
    }
    for p in ports {
        let usb = p.usb_id.map_or(String::new(), |(vid, pid)| format!(" [{vid:04x}:{pid:04x}]"));
        let mark = if p.likely_autopilot { "  <- flight controller?" } else { "" };
        println!("{}  {}{usb}{mark}", p.name, p.description);
    }
}

/// Rate-limits a redrawn progress line to about ten updates a second.
struct Throttle(Option<Instant>);

impl Throttle {
    fn ready(&mut self, done: usize, total: usize) -> bool {
        let due = self.0.is_none_or(|t| t.elapsed() >= Duration::from_millis(100)) || done >= total;
        if due {
            self.0 = Some(Instant::now());
        }
        due
    }
}

pub fn fetch_params(link: &mut Link) -> Result<Params> {
    let mut throttle = Throttle(None);
    let params = link.fetch_params(&mut |got, total| {
        if throttle.ready(got, total) {
            eprint!("\rReading parameters {got}/{total}");
        }
    })?;
    eprintln!();
    Ok(params)
}

/// Saves all parameters so a change can be undone with `ardutune apply`.
pub fn save_params(link: &Link, params: &Params, path: Option<&Path>) -> Result<PathBuf> {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let path = path.map_or_else(
        || PathBuf::from(format!("ardutune-backup-sys{}-{}.param", link.vehicle().system_id, file_stamp(now))),
        Path::to_path_buf,
    );
    let header = format!(
        "ArduTune parameter backup, {} UTC, system {}.\nRestore with: ardutune apply {}",
        date_time(now),
        link.vehicle().system_id,
        path.display()
    );
    let text = params::write(&header, params.iter().map(|(n, p)| (n.as_str(), p.value)));
    std::fs::write(&path, text).with_context(|| format!("writing {}", path.display()))?;
    Ok(path)
}

/// A parameter value to write, with an optional reason to show.
pub struct Wanted {
    pub name: String,
    pub value: f64,
    pub reason: Option<String>,
}

pub struct ApplyOptions {
    pub yes: bool,
    pub reboot: bool,
}

/// Shows the changes, asks for confirmation, backs up, writes and verifies.
pub fn apply(link: &mut Link, params: &Params, wanted: &[Wanted], opts: &ApplyOptions) -> Result<()> {
    if link.vehicle().armed {
        bail!("the vehicle is armed; disarm before changing parameters");
    }
    let pairs: Vec<(String, f64)> = wanted.iter().map(|w| (w.name.clone(), w.value)).collect();
    let plan = plan(params, &pairs);
    for name in &plan.unknown {
        eprintln!("Warning: the vehicle has no parameter {name}; skipping it.");
    }
    if plan.changes.is_empty() {
        println!("Nothing to change: the vehicle already has these values.");
        return Ok(());
    }

    println!("\nChanges to write:");
    for c in &plan.changes {
        println!("  {:<16} {} -> {}", c.name, c.current.value, format_value(c.value));
        if let Some(reason) = wanted.iter().find(|w| w.name == c.name).and_then(|w| w.reason.as_deref()) {
            println!("  {:<16} {reason}", "");
        }
    }
    if !opts.yes && !confirm(&format!("\nWrite {} parameter(s) to the vehicle?", plan.changes.len()), false)? {
        println!("Nothing written.");
        return Ok(());
    }

    let backup = save_params(link, params, None)?;
    println!("Saved current parameters to {}", backup.display());

    let results = link.apply(&plan.changes, &mut |c, r| match r {
        Ok(v) => println!("  OK    {} = {v}", c.name),
        Err(e) => println!("  FAIL  {e}"),
    })?;
    let ok = results.iter().filter(|r| r.is_ok()).count();
    println!("{ok} of {} parameter(s) written and verified.", results.len());
    if ok == 0 {
        return Ok(());
    }

    let reboot = opts.reboot || (!opts.yes && confirm("Reboot the flight controller now so all changes take effect?", true)?);
    if reboot {
        link.reboot()?;
        println!("Rebooting. Reconnect in a few seconds.");
    } else {
        println!("Reboot the flight controller before flying so all changes take effect.");
    }
    Ok(())
}

pub fn apply_file(conn: &str, file: &Path, opts: &ApplyOptions) -> Result<()> {
    let text = std::fs::read_to_string(file).with_context(|| format!("reading {}", file.display()))?;
    let wanted: Vec<Wanted> = params::parse(&text)
        .map_err(anyhow::Error::msg)?
        .into_iter()
        .map(|(name, value)| Wanted { name, value, reason: None })
        .collect();
    let mut link = connect(conn)?;
    let current = fetch_params(&mut link)?;
    apply(&mut link, &current, &wanted, opts)
}

fn confirm(question: &str, default_yes: bool) -> Result<bool> {
    print!("{question} {} ", if default_yes { "[Y/n]" } else { "[y/N]" });
    io::stdout().flush()?;
    let mut answer = String::new();
    io::stdin().lock().read_line(&mut answer)?;
    let a = answer.trim().to_lowercase();
    Ok(if a.is_empty() { default_yes } else { a == "y" || a == "yes" })
}

pub fn print_logs(entries: &[LogEntry]) {
    if entries.is_empty() {
        println!("No logs on the flight controller.");
        return;
    }
    println!("{:>5}  {:>9}  Date (UTC)", "ID", "Size");
    for e in entries {
        let date = if e.time_utc > 0 { date_time(e.time_utc as u64) } else { "no GPS time".to_owned() };
        println!("{:>5}  {:>6.1} MB  {date}", e.id, e.size as f64 / 1e6);
    }
}

/// Downloads one log (the latest when `id` is `None`) and returns where it was saved.
pub fn download(link: &mut Link, id: Option<u16>, out: Option<&Path>) -> Result<PathBuf> {
    let entries = link.list_logs()?;
    let entry = match id {
        Some(id) => entries.iter().find(|e| e.id == id).with_context(|| format!("no log with id {id}"))?,
        None => entries.iter().max_by_key(|e| e.id).context("no logs on the flight controller")?,
    };
    let path = out.map_or_else(
        || {
            PathBuf::from(if entry.time_utc > 0 {
                format!("log_{}_{}.bin", entry.id, file_stamp(entry.time_utc as u64))
            } else {
                format!("log_{}.bin", entry.id)
            })
        },
        Path::to_path_buf,
    );

    let start = Instant::now();
    let mut throttle = Throttle(None);
    let data = link.download_log(entry, &mut |done, total| {
        if !throttle.ready(done, total) {
            return;
        }
        let rate = done as f64 / start.elapsed().as_secs_f64().max(0.001) / 1e3;
        eprint!(
            "\rDownloading log {}: {:>3.0}% ({:.1}/{:.1} MB, {rate:.0} kB/s)",
            entry.id,
            done as f64 * 100.0 / total as f64,
            done as f64 / 1e6,
            total as f64 / 1e6
        );
    })?;
    eprintln!();
    std::fs::write(&path, &data).with_context(|| format!("writing {}", path.display()))?;
    println!("Saved log {} to {} in {:.0} s", entry.id, path.display(), start.elapsed().as_secs_f64());
    Ok(path)
}

/// `YYYYMMDD-HHMMSS` in UTC.
fn file_stamp(secs: u64) -> String {
    let (y, mo, d, h, mi, s) = civil(secs);
    format!("{y:04}{mo:02}{d:02}-{h:02}{mi:02}{s:02}")
}

/// `YYYY-MM-DD HH:MM:SS` in UTC.
fn date_time(secs: u64) -> String {
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
