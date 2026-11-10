//! Log readiness: does this log contain what filter and PID tuning need?
//!
//! Each check reports a status with a plain-language explanation. When data
//! is missing, [`Report::recommended_params`] lists the logging parameters to
//! set before the next test flight and [`Report::flight_plan`] says what to fly.

use serde::Serialize;

use crate::log::{in_any, total_duration, Interval, Log, Table, Vehicle};
use crate::stats;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Status {
    Pass,
    Info,
    Warn,
    Fail,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Area {
    General,
    Vibration,
    Filters,
    Pid,
}

#[derive(Debug, Clone, Serialize)]
pub struct Check {
    pub id: &'static str,
    pub area: Area,
    pub status: Status,
    pub title: String,
    pub detail: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ParamChange {
    pub name: String,
    pub current: Option<f64>,
    pub value: f64,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct Report {
    pub firmware: Option<String>,
    pub log_duration_s: f64,
    pub armed_duration_s: f64,
    pub checks: Vec<Check>,
    /// The log has what gyro-noise / notch filter analysis needs.
    pub filter_ready: bool,
    /// The log has what rate-loop PID analysis needs.
    pub pid_ready: bool,
    pub recommended_params: Vec<ParamChange>,
    pub flight_plan: Vec<String>,
    #[serde(skip)]
    gyro_complete: bool,
}

impl Report {
    pub fn check(&self, id: &str) -> Option<&Check> {
        self.checks.iter().find(|c| c.id == id)
    }

    /// A Mission Planner style `.param` file with the recommended changes,
    /// also accepted by MAVProxy (`param load`).
    pub fn param_file(&self) -> String {
        let mut out = String::from(
            "# ArduTune: logging parameters for the next tuning test flight.\n\
             # Load with Mission Planner (Config > Full Parameter List > Load from file)\n\
             # or MAVProxy (param load <file>), then reboot the flight controller.\n",
        );
        for p in &self.recommended_params {
            let current = p.current.map_or("not in log".to_owned(), format_value);
            out += &format!("\n# {} (was {current})\n{},{}\n", p.reason, p.name, format_value(p.value));
        }
        out
    }

    /// The recommendations recomputed against other parameter values, such
    /// as those read from the vehicle just before applying them.
    pub fn recommendations_for(&self, param: impl Fn(&str) -> Option<f64>) -> Vec<ParamChange> {
        recommend_params(&param, self.gyro_complete)
    }
}

pub use crate::params::format_value;

// LOG_BITMASK bits (Copter 4.x).
const BIT_ATTITUDE_FAST: u32 = 0;
const BIT_ATTITUDE_MED: u32 = 1;
const BIT_CTUN: u32 = 4;
const BIT_IMU: u32 = 7;
const BIT_RCOUT: u32 = 10;
const BIT_PID: u32 = 12;
const BIT_IMU_RAW: u32 = 19;
const REQUIRED_LOG_BITS: [(u32, &str); 6] = [
    (BIT_ATTITUDE_FAST, "ATTITUDE_FAST"),
    (BIT_ATTITUDE_MED, "ATTITUDE_MED"),
    (BIT_CTUN, "CTUN"),
    (BIT_IMU, "IMU"),
    (BIT_RCOUT, "RCOUT"),
    (BIT_PID, "PID"),
];

/// INS_RAW_LOG_OPT: primary gyro only (bit 0) + pre and post filter (bit 3).
const RAW_LOG_OPT_RECOMMENDED: f64 = 9.0;
const RAW_LOG_OPT_PRE_POST_BIT: u32 = 3;
/// INS_LOG_BAT_OPT value for pre- and post-filter batch sampling.
const BAT_OPT_PRE_POST: f64 = 4.0;
const BAT_OPT_POST_ONLY: f64 = 2.0;

const MIN_ARMED_S: f64 = 30.0;
const VIBE_OK: f64 = 30.0;
const VIBE_BAD: f64 = 60.0;
/// Gyro sample rate that resolves noise up to 500 Hz.
const GYRO_RATE_OK_HZ: f64 = 1000.0;
const GYRO_RATE_MIN_HZ: f64 = 400.0;
const HOVER_OK_S: f64 = 30.0;
const HOVER_MIN_S: f64 = 10.0;
const HOVER_BLOCK_S: f64 = 2.0;
const HOVER_MIN_THROTTLE: f64 = 0.1;
const HOVER_MAX_THROTTLE_STD: f64 = 0.04;
const PID_RATE_OK_HZ: f64 = 100.0;
/// A 1 s window counts as a stick input when the rate target spans this much.
const SNAP_RANGE_DEG_S: [f64; 3] = [60.0, 60.0, 30.0];
const SNAPS_OK: [usize; 3] = [5, 5, 3];

struct Ctx<'a> {
    log: &'a Log,
    /// Armed periods, or the whole log when arming isn't recorded.
    flight: Vec<Interval>,
    flight_s: f64,
}

impl Ctx<'_> {
    /// Samples of `field` (with times) that fall inside the flight.
    fn flight_series(&self, table: &Table, field: &str) -> Option<(Vec<f64>, Vec<f64>)> {
        let t = table.time_s()?;
        let v = table.num(field)?;
        Some(t.into_iter().zip(v).filter(|(t, _)| in_any(&self.flight, *t)).map(|(t, v)| (t, *v)).unzip())
    }

    /// Message rate during the flight.
    fn flight_rate(&self, table: &Table) -> f64 {
        let Some(t) = table.time_s() else { return 0.0 };
        if self.flight_s <= 0.0 {
            return 0.0;
        }
        t.iter().filter(|t| in_any(&self.flight, **t)).count() as f64 / self.flight_s
    }

    /// Splits flight samples into consecutive windows of `len` seconds.
    fn windows(&self, t: &[f64], v: &[f64], len: f64) -> Vec<Vec<f64>> {
        let mut out = Vec::new();
        for iv in &self.flight {
            let n = (iv.duration() / len).floor() as usize;
            let mut w = vec![Vec::new(); n];
            for (t, v) in t.iter().zip(v) {
                if iv.contains(*t) {
                    let k = ((t - iv.start) / len) as usize;
                    if k < n {
                        w[k].push(*v);
                    }
                }
            }
            out.extend(w);
        }
        out
    }
}

/// Runs every readiness check on a parsed log.
pub fn assess(log: &Log) -> Report {
    let span = log.time_span();
    let armed = log.armed_intervals();
    let flight = if armed.is_empty() { span.into_iter().collect() } else { armed.clone() };
    let flight_s = total_duration(&flight);
    let ctx = Ctx { log, flight, flight_s };

    let mut checks = vec![
        check_firmware(log),
        check_armed(&armed, total_duration(&armed)),
        check_integrity(log),
        check_vibration(&ctx),
        check_clipping(&ctx),
    ];
    let gyro = check_gyro_data(&ctx);
    let gyro_complete = gyro.status == Status::Pass;
    checks.push(gyro);
    checks.push(check_hover(&ctx));
    checks.push(check_rpm_source(&ctx));
    checks.push(check_notch_config(log));
    checks.push(check_pid_logging(&ctx));
    checks.extend(check_excitation(&ctx));

    let not_failed = |id: &str| checks.iter().any(|c| c.id == id && c.status != Status::Fail);
    let filter_ready = ["vibration.levels", "filters.gyro_data", "filters.hover"].iter().all(|id| not_failed(id));
    let pid_ready = ["vibration.levels", "pid.logging", "pid.excitation.roll", "pid.excitation.pitch"]
        .iter()
        .all(|id| not_failed(id));

    let recommended_params = recommend_params(&|n| log.param(n), gyro_complete);
    let flight_plan = flight_plan(&recommended_params, filter_ready, pid_ready);

    Report {
        firmware: log.firmware().map(|f| f.text).filter(|t| !t.is_empty()),
        log_duration_s: span.map_or(0.0, |s| s.duration()),
        armed_duration_s: total_duration(&armed),
        checks,
        filter_ready,
        pid_ready,
        recommended_params,
        flight_plan,
        gyro_complete,
    }
}

fn check(id: &'static str, area: Area, status: Status, title: impl Into<String>, detail: impl Into<String>) -> Check {
    Check { id, area, status, title: title.into(), detail: detail.into() }
}

fn check_firmware(log: &Log) -> Check {
    let id = "general.firmware";
    match log.firmware() {
        Some(fw) if fw.vehicle == Vehicle::Copter => {
            let status = if fw.at_least(4, 3) { Status::Pass } else { Status::Warn };
            let detail = if status == Status::Pass {
                "Copter firmware detected.".to_owned()
            } else {
                "Firmware older than 4.3: some filter and logging options are missing. Updating is recommended.".to_owned()
            };
            check(id, Area::General, status, format!("Firmware: {}", fw.text), detail)
        }
        Some(fw) if fw.vehicle == Vehicle::Unknown => check(
            id,
            Area::General,
            Status::Warn,
            format!("Firmware {}.{}.{} (vehicle type unknown)", fw.major, fw.minor, fw.patch),
            "Could not confirm this is an ArduCopter log; results assume a multicopter.",
        ),
        Some(fw) => check(
            id,
            Area::General,
            Status::Warn,
            format!("Not a Copter log: {}", fw.text),
            "ArduTune targets multicopters; checks may not apply to this vehicle.",
        ),
        None => check(
            id,
            Area::General,
            Status::Warn,
            "Firmware version not found",
            "The log has no firmware banner (MSG) or VER message; parameter advice may not match your version.",
        ),
    }
}

fn check_armed(armed: &[Interval], armed_s: f64) -> Check {
    let id = "general.armed";
    if armed.is_empty() {
        return check(
            id,
            Area::General,
            Status::Warn,
            "Arming not recorded",
            "No ARM/EV arming events found; analysing the whole log as if it were flight.",
        );
    }
    let status = if armed_s >= MIN_ARMED_S { Status::Pass } else { Status::Warn };
    check(
        id,
        Area::General,
        status,
        format!("Armed for {armed_s:.0} s in {} period(s)", armed.len()),
        if status == Status::Pass { "Enough flight time to analyse." } else { "Short flight: aim for at least 60 s in the air." },
    )
}

fn check_integrity(log: &Log) -> Check {
    let id = "general.integrity";
    let dropped = log
        .table("DSF")
        .and_then(|t| t.num("Dp"))
        .and_then(|v| v.iter().copied().reduce(f64::max))
        .unwrap_or(0.0);
    let s = &log.stats;
    let mut problems = Vec::new();
    if dropped > 0.0 {
        problems.push(format!(
            "{dropped:.0} messages were dropped while logging (SD card too slow or log rate too high)"
        ));
    }
    if s.skipped_bytes > 0 {
        problems.push(format!("{} corrupt bytes skipped", s.skipped_bytes));
    }
    if s.undecodable > 0 {
        problems.push(format!("{} messages with unreadable formats", s.undecodable));
    }
    if s.truncated {
        problems.push("the file ends mid-message".to_owned());
    }
    if problems.is_empty() {
        check(id, Area::General, Status::Pass, "Log is intact", format!("{} messages read.", s.messages))
    } else {
        check(id, Area::General, Status::Warn, "Log has gaps or damage", problems.join("; ") + ".")
    }
}

fn primary_imu_filter(table: &Table, field: &str) -> Option<Vec<bool>> {
    let inst = table.num(field)?;
    let primary = inst.iter().copied().fold(f64::INFINITY, f64::min);
    Some(inst.iter().map(|i| *i == primary).collect())
}

fn check_vibration(ctx: &Ctx) -> Check {
    let id = "vibration.levels";
    let Some(vibe) = ctx.log.table("VIBE") else {
        return check(
            id,
            Area::Vibration,
            Status::Warn,
            "No vibration data (VIBE)",
            "Enable the IMU bit in LOG_BITMASK. Vibration must be checked before tuning.",
        );
    };
    let mut worst: Option<(&str, f64)> = None;
    for axis in ["VibeX", "VibeY", "VibeZ"] {
        let v = primary_flight_values(ctx, vibe, "IMU", axis);
        if let Some(p95) = stats::percentile(&v, 95.0) {
            if worst.is_none_or(|(_, w)| p95 > w) {
                worst = Some((axis, p95));
            }
        }
    }
    let Some((axis, level)) = worst else {
        return check(id, Area::Vibration, Status::Warn, "Vibration data empty", "VIBE has no samples during flight.");
    };
    let (status, detail) = if level < VIBE_OK {
        (Status::Pass, "Vibration is within limits.")
    } else if level <= VIBE_BAD {
        (Status::Warn, "Vibration is elevated: check prop balance and flight controller mounting before tuning.")
    } else {
        (Status::Fail, "Vibration is too high to tune reliably: fix props, motors or mounting first.")
    };
    check(id, Area::Vibration, status, format!("Vibration {level:.0} m/s² ({axis}, 95th percentile)"), detail)
}

/// Flight values of `field`, restricted to the lowest sensor instance when
/// the table has an instance column.
fn primary_flight_values(ctx: &Ctx, table: &Table, instance_field: &str, field: &str) -> Vec<f64> {
    let (Some(t), Some(v)) = (table.time_s(), table.num(field)) else { return Vec::new() };
    let keep = primary_imu_filter(table, instance_field);
    t.iter()
        .zip(v)
        .enumerate()
        .filter(|(i, (t, _))| keep.as_ref().is_none_or(|k| k[*i]) && in_any(&ctx.flight, **t))
        .map(|(_, (_, v))| *v)
        .collect()
}

fn check_clipping(ctx: &Ctx) -> Check {
    let id = "vibration.clipping";
    let Some(vibe) = ctx.log.table("VIBE") else {
        return check(id, Area::Vibration, Status::Info, "Clipping not checked", "No VIBE data.");
    };
    let field = ["Clip", "Clip0"].into_iter().find(|f| vibe.num(f).is_some());
    let clips = field.map(|f| primary_flight_values(ctx, vibe, "IMU", f)).unwrap_or_default();
    let (Some(min), Some(max)) =
        (clips.iter().copied().reduce(f64::min), clips.iter().copied().reduce(f64::max))
    else {
        return check(id, Area::Vibration, Status::Info, "Clipping not checked", "No clip counter in VIBE.");
    };
    let n = max - min;
    if n == 0.0 {
        check(id, Area::Vibration, Status::Pass, "No accelerometer clipping", "")
    } else {
        check(
            id,
            Area::Vibration,
            if n > 100.0 { Status::Fail } else { Status::Warn },
            format!("Accelerometer clipped {n:.0} times"),
            "The accelerometer hit its limit: vibration spikes or hard landings. Fix before tuning.",
        )
    }
}

fn check_gyro_data(ctx: &Ctx) -> Check {
    let id = "filters.gyro_data";
    let log = ctx.log;

    if let Some(gyr) = log.table("GYR") {
        let rate = raw_gyro_rate(gyr);
        let opt = log.param("INS_RAW_LOG_OPT").unwrap_or(0.0) as u32;
        let pre_post = opt & (1 << RAW_LOG_OPT_PRE_POST_BIT) != 0;
        let title = format!("Raw gyro logging at {}", rate.map_or("unknown rate".to_owned(), |r| format!("{r:.0} Hz")));
        return rate_status(id, rate, title, if pre_post {
            "Pre- and post-filter gyro data present: ready for noise analysis and filter verification."
        } else {
            "Pre-filter gyro only. Set INS_RAW_LOG_OPT bit 3 to also log post-filter data for verifying filters."
        }, pre_post);
    }

    if let Some(isbh) = log.table("ISBH") {
        let gyro_batches = isbh.num("type").map_or(isbh.len(), |t| t.iter().filter(|t| **t == 1.0).count());
        let rate = isbh.num_any(&["smp_rate", "SmpRate"]).and_then(stats::median);
        let opt = log.param("INS_LOG_BAT_OPT").unwrap_or(0.0);
        let (pre_post, note) = if opt == BAT_OPT_PRE_POST {
            (true, "Pre- and post-filter batches present.")
        } else if opt == BAT_OPT_POST_ONLY {
            (false, "Only post-filter batches: fine for verifying filters, but noise analysis needs pre-filter data (INS_LOG_BAT_OPT=4).")
        } else {
            (false, "Pre-filter batches only. INS_LOG_BAT_OPT=4 adds post-filter data for verifying filters.")
        };
        if gyro_batches == 0 {
            return check(id, Area::Filters, Status::Fail, "Batch sampler logged no gyro data", "Set INS_LOG_BAT_MASK=1.");
        }
        let title = format!(
            "Batch-sampled gyro: {gyro_batches} batches at {}",
            rate.map_or("unknown rate".to_owned(), |r| format!("{r:.0} Hz"))
        );
        return rate_status(id, rate, title, note, pre_post);
    }

    check(
        id,
        Area::Filters,
        Status::Fail,
        "No high-rate gyro data",
        "Noise analysis needs raw IMU logging (INS_RAW_LOG_OPT) or the batch sampler (INS_LOG_BAT_MASK). \
         See the recommended parameters.",
    )
}

/// Pass needs a good rate and both pre- and post-filter data.
fn rate_status(id: &'static str, rate: Option<f64>, title: String, note: &str, pre_post: bool) -> Check {
    match rate {
        Some(r) if r < GYRO_RATE_MIN_HZ => check(
            id,
            Area::Filters,
            Status::Fail,
            title,
            format!("Sample rate is too low: noise above {:.0} Hz is invisible or aliased.", r / 2.0),
        ),
        Some(r) if r < GYRO_RATE_OK_HZ => check(
            id,
            Area::Filters,
            Status::Warn,
            title,
            format!("Only noise below {:.0} Hz can be seen. {note}", r / 2.0),
        ),
        _ => check(id, Area::Filters, if pre_post { Status::Pass } else { Status::Warn }, title, note),
    }
}

/// Median sample rate of the primary gyro in GYR messages.
fn raw_gyro_rate(gyr: &Table) -> Option<f64> {
    let t_us = gyr.num_any(&["SampleUS", "TimeUS"])?;
    let keep = primary_imu_filter(gyr, "I");
    let ts: Vec<f64> = match &keep {
        Some(k) => t_us.iter().zip(k).filter(|(_, k)| **k).map(|(t, _)| *t).collect(),
        None => t_us.to_vec(),
    };
    let dt: Vec<f64> = ts.windows(2).map(|w| w[1] - w[0]).filter(|d| *d > 0.0).collect();
    stats::median(&dt).map(|d| 1e6 / d)
}

fn check_hover(ctx: &Ctx) -> Check {
    let id = "filters.hover";
    let Some((t, th)) = ctx.log.table("CTUN").and_then(|c| ctx.flight_series(c, "ThO")) else {
        return check(
            id,
            Area::Filters,
            Status::Warn,
            "Hover not checked (no CTUN.ThO)",
            "Enable the CTUN bit in LOG_BITMASK so steady hover can be found.",
        );
    };
    let hover_s = ctx
        .windows(&t, &th, HOVER_BLOCK_S)
        .iter()
        .filter(|w| {
            w.len() >= 5
                && stats::mean(w).is_some_and(|m| m > HOVER_MIN_THROTTLE)
                && stats::std_dev(w).is_some_and(|s| s < HOVER_MAX_THROTTLE_STD)
        })
        .count() as f64
        * HOVER_BLOCK_S;
    let status = if hover_s >= HOVER_OK_S {
        Status::Pass
    } else if hover_s >= HOVER_MIN_S {
        Status::Warn
    } else {
        Status::Fail
    };
    check(
        id,
        Area::Filters,
        status,
        format!("{hover_s:.0} s of steady hover"),
        match status {
            Status::Pass => "Enough steady hover for a clean noise spectrum.",
            Status::Warn => "Some hover found; 30-60 s of hands-off hover gives a cleaner spectrum.",
            _ => "No steady hover found. Hover hands-off for 30-60 s in AltHold.",
        },
    )
}

fn check_rpm_source(ctx: &Ctx) -> Check {
    let id = "filters.rpm_source";
    let any_spinning = |table: &str, fields: &[&str]| {
        ctx.log.table(table).is_some_and(|t| {
            fields.iter().any(|f| ctx.flight_series(t, f).is_some_and(|(_, v)| v.iter().any(|r| *r > 0.0)))
        })
    };
    if any_spinning("ESC", &["RPM"]) {
        check(
            id,
            Area::Filters,
            Status::Pass,
            "ESC RPM telemetry available",
            "Best source for the harmonic notch (INS_HNTCH_MODE=3, ESC telemetry): tracks each motor.",
        )
    } else if any_spinning("RPM", &["rpm1", "RPM"]) {
        check(
            id,
            Area::Filters,
            Status::Pass,
            "RPM sensor available",
            "Harmonic notch can track motor speed (INS_HNTCH_MODE=2, RPM sensor).",
        )
    } else {
        check(
            id,
            Area::Filters,
            Status::Info,
            "No motor RPM data",
            "The notch will follow throttle (INS_HNTCH_MODE=1). Bidirectional DShot ESCs give RPM tracking, \
             which filters noise more precisely.",
        )
    }
}

fn check_notch_config(log: &Log) -> Check {
    let id = "filters.notch_config";
    let p = |n: &str| log.param(n);
    let describe = |prefix: &str| -> Option<String> {
        if p(&format!("{prefix}_ENABLE"))? == 0.0 {
            return None;
        }
        let get = |s: &str| p(&format!("{prefix}_{s}")).map_or("?".to_owned(), format_value);
        Some(format!("{prefix}: mode {}, {} Hz, bandwidth {} Hz, ref {}", get("MODE"), get("FREQ"), get("BW"), get("REF")))
    };
    let notches: Vec<String> = ["INS_HNTCH", "INS_HNTC2"].iter().filter_map(|n| describe(n)).collect();
    let lpf = p("INS_GYRO_FILTER").map_or(String::new(), |v| format!(" Gyro low-pass INS_GYRO_FILTER = {} Hz.", format_value(v)));
    if notches.is_empty() {
        check(id, Area::Filters, Status::Info, "No harmonic notch enabled", format!("Current filters: low-pass only.{lpf}"))
    } else {
        check(id, Area::Filters, Status::Info, "Harmonic notch enabled", notches.join("; ") + "." + &lpf)
    }
}

fn check_pid_logging(ctx: &Ctx) -> Check {
    let id = "pid.logging";
    let tables: Vec<&Table> = ["PIDR", "PIDP", "PIDY"].iter().filter_map(|n| ctx.log.table(n)).collect();
    if tables.len() < 3 {
        return check(
            id,
            Area::Pid,
            Status::Fail,
            "Rate PID messages missing (PIDR/PIDP/PIDY)",
            "Enable the PID and ATTITUDE_FAST bits in LOG_BITMASK.",
        );
    }
    let rate = tables.iter().map(|t| ctx.flight_rate(t)).fold(f64::INFINITY, f64::min);
    if rate >= PID_RATE_OK_HZ {
        check(id, Area::Pid, Status::Pass, format!("Rate PIDs logged at {rate:.0} Hz"), "Fast enough for step-response analysis.")
    } else {
        check(
            id,
            Area::Pid,
            Status::Fail,
            format!("Rate PIDs logged at only {rate:.0} Hz"),
            "Step-response analysis needs loop-rate PID logging: enable ATTITUDE_FAST in LOG_BITMASK.",
        )
    }
}

fn check_excitation(ctx: &Ctx) -> Vec<Check> {
    const AXES: [(&str, &str, &str); 3] = [
        ("PIDR", "pid.excitation.roll", "roll"),
        ("PIDP", "pid.excitation.pitch", "pitch"),
        ("PIDY", "pid.excitation.yaw", "yaw"),
    ];
    AXES.iter()
        .enumerate()
        .map(|(axis, (table, id, name))| {
            let Some((t, tar)) = ctx.log.table(table).and_then(|t| ctx.flight_series(t, "Tar")) else {
                return check(id, Area::Pid, Status::Fail, format!("No {name} rate target logged"), "");
            };
            let snaps = ctx
                .windows(&t, &tar, 1.0)
                .iter()
                .filter(|w| stats::range(w).is_some_and(|r| r >= SNAP_RANGE_DEG_S[axis]))
                .count();
            let status = if snaps >= SNAPS_OK[axis] {
                Status::Pass
            } else if snaps > 0 {
                Status::Warn
            } else {
                Status::Fail
            };
            check(
                id,
                Area::Pid,
                status,
                format!("{snaps} sharp {name} inputs"),
                match status {
                    Status::Pass => "Enough stick inputs to estimate the step response.".to_owned(),
                    _ => format!("Do at least {} quick {name} stick snaps (centre, full deflection, back).", SNAPS_OK[axis]),
                },
            )
        })
        .collect()
}

fn recommend_params(param: &dyn Fn(&str) -> Option<f64>, gyro_complete: bool) -> Vec<ParamChange> {
    let mut out = Vec::new();
    let mut set = |name: &str, value: f64, reason: String| {
        let current = param(name);
        if current != Some(value) {
            out.push(ParamChange { name: name.to_owned(), current, value, reason });
        }
    };

    // Raw IMU logging exists from 4.5 on; older firmware uses the batch sampler.
    let raw_supported = param("INS_RAW_LOG_OPT").is_some();
    let current_mask = param("LOG_BITMASK").map_or(0, |m| m as u32);
    let mut mask = REQUIRED_LOG_BITS.iter().fold(current_mask, |m, (bit, _)| m | (1 << bit));
    if !gyro_complete && !raw_supported {
        // The batch sampler must not run alongside IMU_RAW logging.
        mask &= !(1 << BIT_IMU_RAW);
    }
    let missing: Vec<&str> =
        REQUIRED_LOG_BITS.iter().filter(|(bit, _)| current_mask & (1 << bit) == 0).map(|(_, n)| *n).collect();
    let reason = if missing.is_empty() {
        "Turn off IMU_RAW logging, which conflicts with the batch sampler".to_owned()
    } else {
        format!("Add {} logging (needed for PID and hover analysis)", missing.join(", "))
    };
    set("LOG_BITMASK", mask as f64, reason);

    if !gyro_complete {
        if raw_supported {
            let why = "raw IMU logging is used instead of the batch sampler";
            set(
                "INS_RAW_LOG_OPT",
                RAW_LOG_OPT_RECOMMENDED,
                "Log the primary gyro before and after filtering (4.5+, H7 boards; on F4/F7 boards use \
                 INS_LOG_BAT_MASK=1 and INS_LOG_BAT_OPT=4 instead)"
                    .to_owned(),
            );
            set("INS_LOG_BAT_MASK", 0.0, format!("Batch sampler off: {why}"));
            set("INS_LOG_BAT_OPT", 0.0, format!("Batch sampler off: {why}"));
        } else {
            set("INS_LOG_BAT_MASK", 1.0, "Batch-sample the first IMU for noise analysis".to_owned());
            set("INS_LOG_BAT_OPT", BAT_OPT_PRE_POST, "Sample gyro before and after filtering".to_owned());
        }
    }
    out
}

/// Steps for the next test flight; empty when this log already has everything.
fn flight_plan(params: &[ParamChange], filter_ready: bool, pid_ready: bool) -> Vec<String> {
    let mut plan = Vec::new();
    if params.is_empty() && filter_ready && pid_ready {
        return plan;
    }
    if !params.is_empty() {
        plan.push("Load the recommended parameters and reboot the flight controller.".to_owned());
    }
    plan.push("Check props and motors are undamaged and tight; use a fully charged battery.".to_owned());
    plan.push("Take off in AltHold and climb to 3-5 m, clear of obstacles.".to_owned());
    if !filter_ready || !pid_ready {
        plan.push("Hover hands-off for 30-60 s (for the noise spectrum).".to_owned());
    }
    plan.push(
        "Stick snaps, one axis at a time: quickly push roll to full and back to centre, wait 1 s, \
         then the other way. Repeat 5+ times, then do the same for pitch and yaw."
            .to_owned(),
    );
    plan.push("Land, disarm, and download the latest .bin log from the flight controller.".to_owned());
    if params.iter().any(|p| p.name == "INS_RAW_LOG_OPT" || p.name == "INS_LOG_BAT_MASK") {
        plan.push(
            "When tuning is finished, set INS_RAW_LOG_OPT=0 / INS_LOG_BAT_MASK=0 again: \
             high-rate gyro logging makes very large logs."
                .to_owned(),
        );
    }
    plan
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{sample_flight, Preset, DEFAULT_LOG_BITMASK};

    fn status(r: &Report, id: &str) -> Status {
        r.check(id).unwrap_or_else(|| panic!("missing check {id}")).status
    }

    #[test]
    fn ready_log_passes() {
        let log = Log::from_bytes(&sample_flight(Preset::Ready));
        let r = assess(&log);
        for c in &r.checks {
            assert!(c.status != Status::Fail && c.status != Status::Warn, "{c:?}");
        }
        assert!(r.filter_ready && r.pid_ready);
        assert!(r.recommended_params.is_empty(), "{:?}", r.recommended_params);
        assert!(r.flight_plan.is_empty());
        assert_eq!(r.firmware.as_deref(), Some("ArduCopter V4.6.2 (a1b2c3d4)"));
        assert!((r.armed_duration_s - 60.0).abs() < 1e-6);
        assert!(r.check("filters.gyro_data").unwrap().title.contains("2000 Hz"));
        assert!(r.check("pid.logging").unwrap().title.contains("400 Hz"));
    }

    #[test]
    fn default_logging_needs_changes() {
        let log = Log::from_bytes(&sample_flight(Preset::DefaultLogging));
        let r = assess(&log);
        assert!(!r.filter_ready && !r.pid_ready);
        assert_eq!(status(&r, "filters.gyro_data"), Status::Fail);
        assert_eq!(status(&r, "pid.logging"), Status::Fail);
        assert_eq!(status(&r, "pid.excitation.roll"), Status::Fail);
        assert_eq!(status(&r, "filters.hover"), Status::Pass);
        assert_eq!(status(&r, "vibration.levels"), Status::Pass);
        assert_eq!(status(&r, "filters.rpm_source"), Status::Info);

        let changes: Vec<(&str, f64)> = r.recommended_params.iter().map(|p| (p.name.as_str(), p.value)).collect();
        let mask = (DEFAULT_LOG_BITMASK | 1 | (1 << 12)) as f64;
        assert_eq!(changes, vec![("LOG_BITMASK", mask), ("INS_RAW_LOG_OPT", 9.0)]);
        let file = r.param_file();
        assert!(file.contains("LOG_BITMASK,180223\n"));
        assert!(file.contains("INS_RAW_LOG_OPT,9\n"));
        assert!(r.flight_plan.iter().any(|s| s.contains("INS_RAW_LOG_OPT=0")));
    }

    #[test]
    fn old_firmware_gets_batch_sampler() {
        // No INS_RAW_LOG_OPT parameter: firmware before 4.5.
        let mut w = crate::synth::LogWriter::new();
        let parm = w.define("PARM", "QNff", "TimeUS,Name,Value,Default");
        let mask = (DEFAULT_LOG_BITMASK | (1 << BIT_IMU_RAW)) as f64;
        w.write(parm, &[crate::synth::V::I(1), crate::synth::V::S("LOG_BITMASK"), crate::synth::V::F(mask), crate::synth::V::F(0.0)]);
        let r = assess(&Log::from_bytes(&w.into_bytes()));
        let get = |n: &str| r.recommended_params.iter().find(|p| p.name == n).map(|p| p.value);
        assert_eq!(get("INS_LOG_BAT_MASK"), Some(1.0));
        assert_eq!(get("INS_LOG_BAT_OPT"), Some(4.0));
        assert_eq!(get("INS_RAW_LOG_OPT"), None);
        let new_mask = get("LOG_BITMASK").unwrap() as u32;
        assert_eq!(new_mask & (1 << BIT_IMU_RAW), 0);
        assert_eq!(new_mask & 1, 1);
    }

    #[test]
    fn recomputes_against_live_params() {
        let r = assess(&Log::from_bytes(&sample_flight(Preset::DefaultLogging)));
        let live = |n: &str| match n {
            "LOG_BITMASK" => Some((DEFAULT_LOG_BITMASK | 1 | (1 << 12)) as f64),
            "INS_RAW_LOG_OPT" => Some(9.0),
            "INS_LOG_BAT_MASK" | "INS_LOG_BAT_OPT" => Some(0.0),
            _ => None,
        };
        assert!(r.recommendations_for(live).is_empty());
        // A vehicle whose LOG_BITMASK changed since the flight keeps its extra bits.
        let live = |n: &str| if n == "LOG_BITMASK" { Some(1.0 + 4.0) } else { live(n) };
        let changes = r.recommendations_for(live);
        let mask = changes[0].value as u32;
        assert_eq!(mask & 4, 4);
        assert_eq!(mask & (1 << 12), 1 << 12);
    }
}
