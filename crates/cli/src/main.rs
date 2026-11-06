use std::path::PathBuf;

use anyhow::{Context, Result};
use ardutune_core::readiness::{Area, Status};
use ardutune_core::synth::{sample_flight, Preset};
use ardutune_core::{assess, Log, Report};
use clap::{Parser, Subcommand, ValueEnum};

#[derive(Parser)]
#[command(name = "ardutune", version, about = "Diagnose and tune ArduPilot multicopters from flight logs")]
struct Cli {
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
    },
    /// Write a synthetic sample log, to try ArduTune without a real flight
    Sample {
        out: PathBuf,
        #[arg(long, value_enum, default_value_t = SamplePreset::Ready)]
        preset: SamplePreset,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum SamplePreset {
    /// Logging set up for tuning, with hover and stick snaps
    Ready,
    /// Copter default logging: not enough data for tuning
    Default,
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Info { log, fields } => info(&open(&log)?, fields),
        Cmd::Check { log, json, param_out } => {
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
    }
    Ok(())
}

fn open(path: &PathBuf) -> Result<Log> {
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
        println!("\nParameters to set before the next flight");
        for p in &r.recommended_params {
            let current = p.current.map_or("not in log".to_owned(), ardutune_core::readiness::format_value);
            println!(
                "  {} = {}  (now {current})\n      {}",
                p.name,
                ardutune_core::readiness::format_value(p.value),
                p.reason
            );
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
