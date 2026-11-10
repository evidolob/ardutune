mod vehicle;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use ardutune_core::params::format_value;
use ardutune_core::readiness::{Area, Status};
use ardutune_core::synth::{sample_flight, Preset};
use ardutune_core::{assess, Log, Report};
use ardutune_link::mock::{MockState, MockVehicle};
use clap::{Parser, Subcommand, ValueEnum};

use vehicle::{ApplyOptions, Wanted};

#[derive(Parser)]
#[command(name = "ardutune", version, about = "Diagnose and tune ArduPilot multicopters from flight logs")]
struct Cli {
    /// Vehicle connection: `auto` (USB), `serial:<port>:<baud>`, `udpin:0.0.0.0:14550`, `tcpout:<host>:<port>`
    #[arg(long, global = true, default_value = "auto", env = "ARDUTUNE_CONN")]
    conn: String,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Summarise a log: firmware, duration and message types
    Info {
        log: PathBuf,
        /// Also list each message's fields
        #[arg(long)]
        fields: bool,
    },
    /// Check whether a log has the data needed for filter and PID tuning
    Check {
        log: PathBuf,
        /// Print the report as JSON
        #[arg(long)]
        json: bool,
        /// Write recommended logging parameters to this .param file
        #[arg(long, value_name = "FILE")]
        param_out: Option<PathBuf>,
        /// Write the recommended parameters to the connected vehicle
        #[arg(long)]
        apply: bool,
        #[command(flatten)]
        apply_opts: ApplyArgs,
    },
    /// List serial ports and which look like flight controllers
    Ports,
    /// Connect to the vehicle and show what it is
    Connect,
    /// Save all of the vehicle's parameters to a .param file
    Params {
        #[arg(short, long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
    /// Write parameters from a .param file to the vehicle (also restores backups)
    Apply {
        file: PathBuf,
        #[command(flatten)]
        apply_opts: ApplyArgs,
    },
    /// List or download logs stored on the flight controller
    Logs {
        #[command(subcommand)]
        cmd: LogsCmd,
    },
    /// Write a synthetic sample log, to try ArduTune without a real flight
    Sample {
        out: PathBuf,
        #[arg(long, value_enum, default_value_t = SamplePreset::Ready)]
        preset: SamplePreset,
    },
    /// Run a simulated vehicle, to try the connection commands without hardware
    #[command(hide = true)]
    MockVehicle {
        #[arg(long, default_value = "udpin:127.0.0.1:14550")]
        listen: String,
    },
}

#[derive(Subcommand)]
enum LogsCmd {
    /// List logs on the flight controller
    List,
    /// Download a log (the latest by default)
    Download {
        /// Log id from `ardutune logs list`
        #[arg(long)]
        id: Option<u16>,
        #[arg(short, long, value_name = "FILE")]
        out: Option<PathBuf>,
        /// Run the readiness check on the downloaded log
        #[arg(long)]
        check: bool,
    },
}

#[derive(clap::Args)]
struct ApplyArgs {
    /// Don't ask for confirmation (and don't reboot unless --reboot)
    #[arg(long)]
    yes: bool,
    /// Reboot the flight controller after writing
    #[arg(long)]
    reboot: bool,
}

impl From<&ApplyArgs> for ApplyOptions {
    fn from(a: &ApplyArgs) -> Self {
        Self { yes: a.yes, reboot: a.reboot }
    }
}

#[derive(Clone, Copy, ValueEnum)]
enum SamplePreset {
    /// Logging set up for tuning, with hover and stick snaps
    Ready,
    /// Copter default logging: not enough data for tuning
    Default,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Info { log, fields } => info(&open(&log)?, fields),
        Cmd::Check { log, json, param_out, apply, apply_opts } => {
            let report = assess(&open(&log)?);
            if json {
                println!("{}", serde_json::to_string_pretty(&report)?);
            } else {
                print_report(&report);
            }
            if let Some(path) = param_out {
                if report.recommended_params.is_empty() {
                    eprintln!("No parameter changes needed; {} not written.", path.display());
                } else {
                    std::fs::write(&path, report.param_file()).with_context(|| format!("writing {}", path.display()))?;
                    eprintln!("Wrote {} parameter change(s) to {}", report.recommended_params.len(), path.display());
                }
            }
            if apply {
                apply_recommendations(&cli.conn, &report, &(&apply_opts).into())?;
            }
        }
        Cmd::Ports => vehicle::print_ports(),
        Cmd::Connect => {
            let mut link = vehicle::connect(&cli.conn)?;
            let params = vehicle::fetch_params(&mut link)?;
            println!("{} parameters", params.len());
        }
        Cmd::Params { out } => {
            let mut link = vehicle::connect(&cli.conn)?;
            let params = vehicle::fetch_params(&mut link)?;
            let path = vehicle::save_params(&link, &params, out.as_deref())?;
            println!("Saved {} parameters to {}", params.len(), path.display());
        }
        Cmd::Apply { file, apply_opts } => vehicle::apply_file(&cli.conn, &file, &(&apply_opts).into())?,
        Cmd::Logs { cmd: LogsCmd::List } => {
            let mut link = vehicle::connect(&cli.conn)?;
            vehicle::print_logs(&link.list_logs()?);
        }
        Cmd::Logs { cmd: LogsCmd::Download { id, out, check } } => {
            let mut link = vehicle::connect(&cli.conn)?;
            let path = vehicle::download(&mut link, id, out.as_deref())?;
            if check {
                println!();
                print_report(&assess(&open(&path)?));
            }
        }
        Cmd::Sample { out, preset } => {
            let preset = match preset {
                SamplePreset::Ready => Preset::Ready,
                SamplePreset::Default => Preset::DefaultLogging,
            };
            let bytes = sample_flight(preset);
            std::fs::write(&out, &bytes).with_context(|| format!("writing {}", out.display()))?;
            println!("Wrote {} ({:.1} MB)", out.display(), bytes.len() as f64 / 1e6);
        }
        Cmd::MockVehicle { listen } => mock_vehicle(&listen)?,
    }
    Ok(())
}

/// Recomputes the report's recommendations against the vehicle's current
/// parameters (they may have changed since the flight) and applies them.
fn apply_recommendations(conn: &str, report: &Report, opts: &ApplyOptions) -> Result<()> {
    println!();
    let mut link = vehicle::connect(conn)?;
    let params = vehicle::fetch_params(&mut link)?;
    let wanted: Vec<Wanted> = report
        .recommendations_for(|n| params.get(n).map(|p| p.value as f64))
        .into_iter()
        .map(|c| Wanted { name: c.name, value: c.value, reason: Some(c.reason) })
        .collect();
    vehicle::apply(&mut link, &params, &wanted, opts)
}

fn mock_vehicle(listen: &str) -> Result<()> {
    let client = listen.replacen("udpin", "udpout", 1).replacen("tcpin", "tcpout", 1);
    let logs = vec![sample_flight(Preset::DefaultLogging), sample_flight(Preset::Ready)];
    let _mock = MockVehicle::start_on(listen, &client, MockState::quad(logs))?;
    println!("Simulated quad listening on {listen}. Try:");
    println!("  ardutune --conn {client} logs list");
    println!("Press Ctrl-C to stop.");
    loop {
        std::thread::park();
    }
}

fn open(path: &Path) -> Result<Log> {
    Log::open(path).with_context(|| format!("reading {}", path.display()))
}

fn info(log: &Log, fields: bool) {
    match log.firmware() {
        Some(fw) if !fw.text.is_empty() => println!("Firmware: {}", fw.text),
        Some(fw) => println!("Firmware: {}.{}.{}", fw.major, fw.minor, fw.patch),
        None => println!("Firmware: unknown"),
    }
    if let Some(span) = log.time_span() {
        println!("Duration: {:.1} s", span.duration());
    }
    let armed = log.armed_intervals();
    println!("Armed:    {:.1} s in {} period(s)", ardutune_core::log::total_duration(&armed), armed.len());
    println!("Params:   {}", log.params().len());
    let s = &log.stats;
    println!("Messages: {} ({} skipped bytes, truncated: {})", s.messages, s.skipped_bytes, s.truncated);
    println!();
    for t in log.tables().filter(|t| !t.is_empty()) {
        if fields {
            println!("{:<6} {:>9}  {}", t.name, t.len(), t.labels.join(", "));
        } else {
            println!("{:<6} {:>9}", t.name, t.len());
        }
    }
}

fn print_report(r: &Report) {
    println!("ArduTune log readiness");
    println!("Firmware: {}", r.firmware.as_deref().unwrap_or("unknown"));
    println!("Log {:.0} s, armed {:.0} s", r.log_duration_s, r.armed_duration_s);

    for (area, heading) in [
        (Area::General, "General"),
        (Area::Vibration, "Vibration"),
        (Area::Filters, "Filters"),
        (Area::Pid, "PID"),
    ] {
        println!("\n{heading}");
        for c in r.checks.iter().filter(|c| c.area == area) {
            let tag = match c.status {
                Status::Pass => "PASS",
                Status::Info => "INFO",
                Status::Warn => "WARN",
                Status::Fail => "FAIL",
            };
            println!("  [{tag}] {}", c.title);
            if !c.detail.is_empty() {
                println!("         {}", c.detail);
            }
        }
    }

    let yes_no = |b: bool| if b { "yes" } else { "no" };
    println!("\nReady for filter tuning: {}", yes_no(r.filter_ready));
    println!("Ready for PID tuning:    {}", yes_no(r.pid_ready));

    if !r.recommended_params.is_empty() {
        println!("\nParameters to set before the next flight (write them with --apply)");
        for p in &r.recommended_params {
            let current = p.current.map_or("not in log".to_owned(), format_value);
            println!("  {} = {}  (now {current})\n      {}", p.name, format_value(p.value), p.reason);
        }
    }

    if r.flight_plan.is_empty() {
        return;
    }
    println!("\nNext test flight");
    for (i, step) in r.flight_plan.iter().enumerate() {
        println!("  {}. {step}", i + 1);
    }
}
