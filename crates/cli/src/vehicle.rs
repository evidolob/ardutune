//! Commands that talk to a vehicle over MAVLink.

use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context, Result};
use ardutune_core::params::{self, format_value};
use ardutune_core::utc::{date_time, file_stamp};
use ardutune_link::{plan, ports, Link, LogEntry, Params};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// The address to open: `auto` becomes the flight controller's USB port.
pub fn address(conn: &str) -> Result<String> {
    if conn == "auto" {
        ports::auto_address().map_err(anyhow::Error::msg)
    } else {
        Ok(conn.to_owned())
    }
}

pub fn connect(conn: &str) -> Result<Link> {
    let address = address(conn)?;
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
pub fn save_params(link: &mut Link, params: &Params, path: Option<&Path>) -> Result<PathBuf> {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let path = path.map_or_else(|| PathBuf::from(format!("ardutune-backup-sys{}-{}.param", link.vehicle().system_id, file_stamp(now))), Path::to_path_buf);
    // Same vehicle line as the desktop app's backups, so they can tell
    // which flight controller a file came from.
    let id = link.identity().ok().filter(|i| i.uid.is_some());
    let controller =
        id.map_or(String::new(), |i| format!("Flight controller: {}, ID {}\n", i.board.as_deref().unwrap_or("unknown board"), i.uid.unwrap_or_default()));
    let header = format!(
        "ArduTune parameter backup, {} UTC, system {}.\n{controller}Restore with: ardutune apply {}",
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
    let wanted: Vec<Wanted> = params::parse(&text).map_err(anyhow::Error::msg)?.into_iter().map(|(name, value)| Wanted { name, value, reason: None }).collect();
    let mut link = connect(conn)?;
    let current = fetch_params(&mut link)?;
    apply(&mut link, &current, &wanted, opts)
}

/// Deletes a log; for the newest one (possibly still open on the vehicle)
/// offers to reboot first.
pub fn delete_log(conn: &str, id: u16) -> Result<Link> {
    let mut link = connect(conn)?;
    match link.delete_log(id, false) {
        Err(ardutune_link::Error::NewestLogOpen(_)) => {
            println!(
                "Log {id} is the newest log. The flight controller keeps it open after disarming, and deleting an open \
                 file can corrupt the SD card."
            );
            if !confirm("Reboot the flight controller now and then delete it?", false)? {
                bail!("nothing deleted");
            }
            let address = address(conn)?;
            eprintln!("Rebooting and reconnecting ...");
            let mut link = link.reboot_and_reconnect(&address, Duration::from_secs(40))?;
            link.delete_log(id, true)?;
            Ok(link)
        }
        r => r.map(|_| link).map_err(Into::into),
    }
}

pub fn confirm(question: &str, default_yes: bool) -> Result<bool> {
    print!("{question} {} ", if default_yes { "[Y/n]" } else { "[y/N]" });
    io::stdout().flush()?;
    let mut answer = String::new();
    io::stdin().lock().read_line(&mut answer)?;
    let a = answer.trim().to_lowercase();
    Ok(if a.is_empty() { default_yes } else { a == "y" || a == "yes" })
}

/// A line typed by the user, trimmed and lower-cased.
pub fn ask(question: &str) -> Result<String> {
    print!("{question} ");
    io::stdout().flush()?;
    let mut answer = String::new();
    io::stdin().lock().read_line(&mut answer)?;
    Ok(answer.trim().to_lowercase())
}

pub fn print_logs(entries: &[LogEntry]) {
    // Deleted files leave empty entries behind.
    let entries: Vec<&LogEntry> = entries.iter().filter(|e| e.size > 0).collect();
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
