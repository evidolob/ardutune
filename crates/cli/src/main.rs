mod vehicle;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use ardutune_core::diagnose::{diagnose, Confidence, Diagnosis, Finding};
use ardutune_core::filters::{self, Change, Evaluation, FilterReport};
use ardutune_core::noise::{self, NoiseReport, PeakClass, Source};
use ardutune_core::params::format_value;
use ardutune_core::pid::{self, PidReport};
use ardutune_core::plot;
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
    /// Find losses of control or crashes in a log and explain their likely cause
    Diagnose {
        log: PathBuf,
        /// Print the diagnosis as JSON
        #[arg(long)]
        json: bool,
    },
    /// Analyse gyro noise: spectrum before/after filters, noise peaks, notch coverage
    Noise {
        log: PathBuf,
        /// Print the analysis as JSON (includes full spectra and spectrogram)
        #[arg(long)]
        json: bool,
        /// Write a plot of the spectrum and spectrogram to this SVG file
        #[arg(long, value_name = "FILE")]
        svg: Option<PathBuf>,
    },
    /// Simulate gyro filter settings on a log's noise and recommend better ones
    Filters {
        log: PathBuf,
        /// Report filter delay at this frequency (Hz)
        #[arg(long, default_value_t = filters::DEFAULT_LAG_REF_HZ)]
        lag_at: f64,
        /// Print the result as JSON
        #[arg(long)]
        json: bool,
        /// Write a plot of the spectra and the noise/delay trade-off to this SVG file
        #[arg(long, value_name = "FILE")]
        svg: Option<PathBuf>,
        /// Write the chosen settings to the connected vehicle
        #[arg(long)]
        apply: bool,
        /// Which settings to apply
        #[arg(long, value_enum, default_value_t = FilterPick::Recommended)]
        pick: FilterPick,
        #[command(flatten)]
        apply_opts: ApplyArgs,
    },
    /// Analyse rate-loop PIDs: step response, oscillation, PID terms, AutoTune results
    Pid {
        log: PathBuf,
        /// Print the analysis as JSON
        #[arg(long)]
        json: bool,
        /// Write a plot of the step responses to this SVG file
        #[arg(long, value_name = "FILE")]
        svg: Option<PathBuf>,
        /// Write the chosen gains to the connected vehicle
        #[arg(long)]
        apply: bool,
        /// Which gains to apply
        #[arg(long, value_enum, default_value_t = PidPick::Suggestions)]
        pick: PidPick,
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
enum FilterPick {
    /// Same noise, less delay
    Recommended,
    /// Less noise, same delay
    Quieter,
}

#[derive(Clone, Copy, ValueEnum)]
enum PidPick {
    /// The cautious changes from this analysis
    Suggestions,
    /// Gains a finished AutoTune found but didn't save
    Autotune,
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
        Cmd::Diagnose { log, json } => {
            let d = diagnose(&open(&log)?);
            if json {
                println!("{}", serde_json::to_string_pretty(&d)?);
            } else {
                print_diagnosis(&d);
            }
        }
        Cmd::Noise { log, json, svg } => {
            let r = noise::analyse(&open(&log)?).map_err(anyhow::Error::msg)?;
            if json {
                println!("{}", serde_json::to_string_pretty(&r)?);
            } else {
                print_noise(&r);
            }
            if let Some(path) = svg {
                std::fs::write(&path, plot::render(&r)).with_context(|| format!("writing {}", path.display()))?;
                eprintln!("Wrote plot to {}", path.display());
            }
        }
        Cmd::Filters { log, lag_at, json, svg, apply, pick, apply_opts } => {
            let r = filters::simulate(&open(&log)?, lag_at).map_err(anyhow::Error::msg)?;
            if json {
                println!("{}", serde_json::to_string_pretty(&r)?);
            } else {
                print_filters(&r);
            }
            if let Some(path) = svg {
                std::fs::write(&path, plot::render_filters(&r)).with_context(|| format!("writing {}", path.display()))?;
                eprintln!("Wrote plot to {}", path.display());
            }
            if apply {
                let (changes, what) = match pick {
                    FilterPick::Recommended => (&r.changes, "recommended"),
                    FilterPick::Quieter => (&r.quieter_changes, "quieter"),
                };
                if changes.is_empty() {
                    println!("\nNo {what} settings to apply.");
                } else {
                    println!("\nApplying the {what} filter settings.");
                    let mut link = vehicle::connect(&cli.conn)?;
                    let params = vehicle::fetch_params(&mut link)?;
                    let wanted: Vec<Wanted> = changes
                        .iter()
                        .map(|c| Wanted { name: c.name.clone(), value: c.value, reason: None })
                        .collect();
                    vehicle::apply(&mut link, &params, &wanted, &(&apply_opts).into())?;
                }
            }
        }
        Cmd::Pid { log, json, svg, apply, pick, apply_opts } => {
            let r = pid::analyse(&open(&log)?).map_err(anyhow::Error::msg)?;
            if json {
                println!("{}", serde_json::to_string_pretty(&r)?);
            } else {
                print_pid(&r);
            }
            if let Some(path) = svg {
                std::fs::write(&path, plot::render_pid(&r)).with_context(|| format!("writing {}", path.display()))?;
                eprintln!("Wrote plot to {}", path.display());
            }
            if apply {
                let (chosen, what) = match pick {
                    PidPick::Suggestions => (&r.suggestions, "suggested"),
                    PidPick::Autotune => (&r.autotune, "AutoTune"),
                };
                if chosen.is_empty() {
                    println!("\nNo {what} gains to apply.");
                } else {
                    println!("\nApplying the {what} gains.");
                    let mut link = vehicle::connect(&cli.conn)?;
                    let params = vehicle::fetch_params(&mut link)?;
                    let wanted: Vec<Wanted> = chosen
                        .iter()
                        .map(|s| Wanted { name: s.name.clone(), value: s.value, reason: Some(s.reason.clone()) })
                        .collect();
                    vehicle::apply(&mut link, &params, &wanted, &(&apply_opts).into())?;
                }
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
    let (flight, source) = log.flight();
    println!("Flight:   {:.1} s in {} period(s), from {source:?}", ardutune_core::log::total_duration(&flight), flight.len());
    println!("Params:   {}", log.params().len());
    let s = &log.stats;
    println!(
        "Messages: {} ({} skipped bytes, {} corrupt dropped, truncated: {})",
        s.messages, s.skipped_bytes, s.bad_timestamps, s.truncated
    );
    println!();
    for t in log.tables().filter(|t| !t.is_empty()) {
        if fields {
            println!("{:<6} {:>9}  {}", t.name, t.len(), t.labels.join(", "));
        } else {
            println!("{:<6} {:>9}", t.name, t.len());
        }
    }
}

fn print_pid(r: &PidReport) {
    println!("ArduTune rate-loop PID analysis");
    println!("Rate data at {:.0} Hz.", r.sample_rate_hz);
    let opt = |v: Option<f64>| v.map_or("-".to_owned(), format_value);
    for a in &r.axes {
        let g = &a.gains;
        println!(
            "\n{} — P {} I {} D {} FF {}, D filter {}, angle P {}",
            a.axis.to_uppercase(),
            opt(g.p),
            opt(g.i),
            opt(g.d),
            opt(g.ff),
            g.fltd.map_or("-".to_owned(), |f| if f == 0.0 { "off".to_owned() } else { format!("{} Hz", format_value(f)) }),
            opt(g.angle_p)
        );
        match &a.step {
            Some(s) => {
                let ms = |v: Option<f64>| v.map_or("-".to_owned(), |v| format!("{v:.0} ms"));
                let caveat = if a.includes_autotune { ", includes AutoTune periods" } else { "" };
                println!(
                    "  Step response ({} windows{caveat}): overshoot {:.0}%, rise {}, delay {}, settles in {}, reaches {:.0}% of target",
                    s.windows,
                    s.overshoot_pct,
                    ms(s.rise_ms),
                    ms(s.delay_ms),
                    ms(s.settling_ms),
                    s.steady_state * 100.0
                );
            }
            None => println!("  Step response: not enough inputs on this axis."),
        }
        for f in &a.findings {
            println!("  - {f}");
        }
    }
    if !r.suggestions.is_empty() {
        println!("\nSuggested changes (apply with --apply):");
        for s in &r.suggestions {
            println!("  {:<38} {}", s.describe(), s.reason);
        }
    }
    if !r.autotune.is_empty() {
        println!("\nAutoTune finished these but they weren't saved (apply with --apply --pick autotune):");
        for s in &r.autotune {
            println!("  {}", s.describe());
        }
    }
    for n in &r.notes {
        println!("\nNote: {n}");
    }
    println!(
        "\nChange gains in small steps and test-hover after each change. Fix the filters (ardutune filters) before tuning PIDs."
    );
}

fn print_filters(r: &FilterReport) {
    println!("ArduTune gyro filter simulator");
    println!("Recorded noise: {:.0} Hz gyro data; delay measured at {:.0} Hz.", r.sample_rate_hz, r.lag_ref_hz);
    match &r.validation {
        Some(v) => println!(
            "Check: simulating your current filters reproduces the logged filtered gyro within {:.1} dB ({:.0}-{:.0} Hz).",
            v.rms_error_db, v.band_hz.0, v.band_hz.1
        ),
        None => println!("Check: no logged post-filter data to compare the simulation against."),
    }

    println!("\n  {:<13} {:>12}  {:>16}  Settings", "", "Noise >45 Hz", "Delay");
    let row = |label: &str, e: &Evaluation| {
        println!(
            "  {label:<13} {:>8.2} °/s  {:>5.0}° {:>5.1} ms  {}",
            e.noise_rms,
            e.lag_deg,
            e.delay_ms,
            e.settings.describe()
        )
    };
    row("Current", &r.current);
    if let Some(e) = &r.recommended {
        row("Recommended", e);
    }
    if let Some(e) = &r.quieter {
        row("Quieter", e);
    }

    let print_changes = |title: &str, changes: &[Change]| {
        if changes.is_empty() {
            return;
        }
        println!("\n{title}");
        for c in changes {
            let now = c.current.map_or("unset".to_owned(), format_value);
            println!("  {:<17} {now} -> {}", c.name, format_value(c.value));
        }
    };
    match &r.recommended {
        Some(_) => print_changes("Recommended changes (same noise, less delay; apply with --apply):", &r.changes),
        None => println!("\nYour current filters are already a good balance: nothing gives the same noise with clearly less delay."),
    }
    print_changes("Quieter option (less noise, same delay; apply with --apply --pick quieter):", &r.quieter_changes);

    if r.frontier.len() > 2 {
        println!("\nOther trade-offs, from least delay to least noise:");
        let n = r.frontier.len();
        let picks: Vec<usize> = (0..6).map(|i| i * (n - 1) / 5).collect::<Vec<_>>();
        let mut last = usize::MAX;
        for i in picks {
            if i == last {
                continue;
            }
            last = i;
            let e = &r.frontier[i];
            println!("  {:>6.2} °/s  {:>5.0}° {:>5.1} ms  {}", e.noise_rms, e.lag_deg, e.delay_ms, e.settings.describe());
        }
    }
    for n in &r.notes {
        println!("\nNote: {n}");
    }
    println!(
        "\nFilters change how the attitude loop responds. After changing them, hover carefully first, watch for \
         oscillation (lower the rate D or P if it appears) and check motor temperatures after a short flight."
    );
}

fn print_noise(r: &NoiseReport) {
    println!("ArduTune gyro noise analysis");
    let source = match r.source {
        Source::BatchSampler => "batch sampler",
        Source::RawImu => "raw IMU logging",
    };
    println!("Data: {source} at {:.0} Hz, {}", r.sample_rate_hz, r.selection);
    if let (Some(m), Some((lo, hi))) = (r.hover_motor_hz, r.hover_motor_range_hz) {
        let each: Vec<String> = r.hover_motor_hz_by_motor.iter().map(|(n, f)| format!("{n}: {f:.0}")).collect();
        println!("Motor speed: {m:.1} Hz on average ({lo:.0}-{hi:.0} Hz; per motor {})", each.join(", "));
    }
    match r.noise_rms_post {
        Some(post) => println!(
            "Gyro noise above {:.0} Hz: {:.1} °/s before filters, {post:.1} °/s after ({:.0} dB less)",
            noise::MIN_NOISE_HZ,
            r.noise_rms_pre,
            20.0 * (r.noise_rms_pre / post.max(1e-9)).log10()
        ),
        None => println!("Gyro noise above {:.0} Hz: {:.1} °/s", noise::MIN_NOISE_HZ, r.noise_rms_pre),
    }

    println!("\nNoise peaks");
    if r.peaks.is_empty() {
        println!("  None stand out.");
    } else {
        println!("  {:>7}  {:>9}  {:>6}  {:<5}  {:<26}  {:>14}  Notched", "Freq", "Above flr", "Level", "Axis", "What", "Filters remove");
    }
    for p in &r.peaks {
        let what = match p.class {
            PeakClass::MotorHarmonic { harmonic, tracks_rpm } => format!(
                "motor harmonic {harmonic}{}",
                match tracks_rpm {
                    Some(true) => " (tracks)",
                    _ => "",
                }
            ),
            PeakClass::FixedFrequency => "fixed frequency".to_owned(),
            PeakClass::ControlBand => "control-band motion".to_owned(),
            PeakClass::Unknown => "unclassified".to_owned(),
        };
        let removed = p.filter_reduction_db.map_or("-".to_owned(), |d| format!("{d:.0} dB"));
        let notched = match p.notch_covered {
            Some(true) => "yes",
            Some(false) => "no",
            None => "-",
        };
        println!(
            "  {:>4.0} Hz  {:>6.0} dB  {:>3.0} dB  {:<5}  {what:<26}  {removed:>14}  {notched}",
            p.freq_hz, p.prominence_db, p.level_db, p.axis
        );
    }

    println!("\nHarmonic notch");
    if r.notches.is_empty() {
        println!("  None enabled.");
    }
    for n in &r.notches {
        let centre = n.hover_center_hz.map_or("?".to_owned(), |c| format!("{c:.0} Hz"));
        let bw = n.bandwidth_hz.map_or("?".to_owned(), |b| format!("{b:.0} Hz"));
        let h: Vec<String> = n.harmonics.iter().map(u32::to_string).collect();
        println!("  {}: {}, centre ≈ {centre} at hover, bandwidth {bw}, harmonics {}", n.name, n.mode, h.join(", "));
    }

    if !r.findings.is_empty() {
        println!("\nFindings");
        for f in &r.findings {
            println!("  - {f}");
        }
    }
    for n in &r.notes {
        println!("\nNote: {n}");
    }
}

fn print_finding(prefix: &str, f: &Finding) {
    let tag = match f.confidence {
        Confidence::High => "HIGH",
        Confidence::Medium => "MEDIUM",
        Confidence::Low => "LOW",
    };
    println!("  {prefix}[{tag}] {}", f.title);
    for e in &f.evidence {
        println!("       - {e}");
    }
    if let Some(a) = &f.advice {
        println!("       What to do: {a}");
    }
}

fn print_diagnosis(d: &Diagnosis) {
    println!("ArduTune incident diagnosis");
    if d.incidents.is_empty() {
        println!("\nNo loss of control or crash found in this log.");
    }
    for (n, inc) in d.incidents.iter().enumerate() {
        println!("\nIncident {}: {}", n + 1, inc.summary);
        println!("\n  Timeline");
        for e in &inc.timeline {
            println!("    {:>8.2} s  {}", e.time_s, e.text);
        }
        if !inc.findings.is_empty() {
            println!("\n  Likely causes, most likely first");
            for (i, f) in inc.findings.iter().enumerate() {
                print_finding(&format!("{}. ", i + 1), f);
            }
        }
        if !inc.ruled_out.is_empty() {
            println!("\n  Ruled out");
            for r in &inc.ruled_out {
                println!("    - {r}");
            }
        }
    }
    if !d.warnings.is_empty() {
        println!("\nOther problems in this flight");
        for w in &d.warnings {
            print_finding("", w);
        }
    }
}

fn print_report(r: &Report) {
    println!("ArduTune log readiness");
    println!("Firmware: {}", r.firmware.as_deref().unwrap_or("unknown"));
    println!("Log {:.0} s, flight {:.0} s", r.log_duration_s, r.flight_duration_s);

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
