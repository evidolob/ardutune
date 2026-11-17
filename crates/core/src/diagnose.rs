//! Incident diagnosis: find losses of control and crashes in a log, then
//! rank likely causes with the evidence for each, and say what was ruled out.

use serde::Serialize;

use crate::log::Log;
use crate::stats;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Confidence {
    High,
    Medium,
    Low,
}

#[derive(Debug, Clone, Serialize)]
pub struct Finding {
    pub id: &'static str,
    pub confidence: Confidence,
    pub title: String,
    pub evidence: Vec<String>,
    pub advice: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Event {
    pub time_s: f64,
    pub text: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum IncidentKind {
    LossOfControl,
    LogEndsInFlight,
}

#[derive(Debug, Clone, Serialize)]
pub struct Incident {
    pub kind: IncidentKind,
    /// When things started to go wrong.
    pub onset_s: f64,
    pub mode: Option<String>,
    pub summary: String,
    pub timeline: Vec<Event>,
    /// Likely causes, most likely first.
    pub findings: Vec<Finding>,
    /// Causes the log argues against, with the reason.
    pub ruled_out: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Diagnosis {
    pub incidents: Vec<Incident>,
    /// Flight-wide problems not tied to one incident.
    pub warnings: Vec<Finding>,
}

/// Attitude beyond this (or this far from the target) counts as lost control.
const LOST_ANGLE_DEG: f64 = 60.0;
const LOST_ERROR_DEG: f64 = 30.0;
/// Walking back from the extreme, the onset is where the error was last below this.
const ONSET_ERROR_DEG: f64 = 5.0;
/// Window before the onset searched for causes.
const LOOKBACK_S: f64 = 3.0;
/// A log ending this soon after the onset means the flight ended there.
const END_AFTER_ONSET_S: f64 = 5.0;
/// ESC telemetry silent this long before the log ends counts as lost.
const ESC_SILENT_S: f64 = 0.3;
const ESC_ERR_WARN_PCT: f64 = 1.0;
const OSCILLATION_MIN_DEG_S: f64 = 20.0;
const OSCILLATION_GROWTH: f64 = 3.0;
const CELL_SAG_V: f64 = 3.3;
const BROWNOUT_VCC: f64 = 4.5;
const VIBE_BAD: f64 = 60.0;
const EKF_BAD: f64 = 1.0;

const COPTER_MODES: [(u32, &str); 25] = [
    (0, "Stabilize"),
    (1, "Acro"),
    (2, "AltHold"),
    (3, "Auto"),
    (4, "Guided"),
    (5, "Loiter"),
    (6, "RTL"),
    (7, "Circle"),
    (9, "Land"),
    (11, "Drift"),
    (13, "Sport"),
    (14, "Flip"),
    (15, "AutoTune"),
    (16, "PosHold"),
    (17, "Brake"),
    (18, "Throw"),
    (19, "Avoid_ADSB"),
    (20, "Guided_NoGPS"),
    (21, "Smart_RTL"),
    (22, "FlowHold"),
    (23, "Follow"),
    (24, "ZigZag"),
    (25, "SystemID"),
    (26, "Autorotate"),
    (28, "Turtle"),
];
/// Modes where large angles are intended.
const ACRO_MODES: [u32; 3] = [1, 14, 28];

struct Series {
    t: Vec<f64>,
    v: Vec<f64>,
}

impl Series {
    fn get(log: &Log, table: &str, field: &str) -> Option<Self> {
        let tb = log.table(table)?;
        Some(Self { t: tb.time_s()?, v: tb.num(field)?.to_vec() })
    }

    /// Like [`Self::get`], keeping only rows whose `key` column equals `value`.
    fn get_where(log: &Log, table: &str, field: &str, key: &str, value: f64) -> Option<Self> {
        let tb = log.table(table)?;
        let (t, v, k) = (tb.time_s()?, tb.num(field)?, tb.num(key)?);
        let (t, v) = t.iter().zip(v).zip(k).filter(|(_, k)| **k == value).map(|((t, v), _)| (*t, *v)).unzip();
        Some(Self { t, v })
    }

    /// Last value at or before `time`.
    fn at(&self, time: f64) -> Option<f64> {
        let n = self.t.partition_point(|t| *t <= time);
        n.checked_sub(1).map(|i| self.v[i])
    }

    fn window(&self, from: f64, to: f64) -> Vec<f64> {
        self.t.iter().zip(&self.v).filter(|(t, v)| **t >= from && **t <= to && v.is_finite()).map(|(_, v)| *v).collect()
    }

    fn last_time(&self) -> Option<f64> {
        self.t.last().copied()
    }
}

fn max_of(v: &[f64]) -> Option<f64> {
    v.iter().copied().reduce(f64::max)
}

fn min_of(v: &[f64]) -> Option<f64> {
    v.iter().copied().reduce(f64::min)
}

pub(crate) fn mode_name(num: u32) -> String {
    COPTER_MODES.iter().find(|(n, _)| *n == num).map_or(format!("mode {num}"), |(_, s)| (*s).to_owned())
}

fn mode_at(log: &Log, time: f64) -> Option<u32> {
    let s = Series::get(log, "MODE", "ModeNum").or_else(|| Series::get(log, "MODE", "Mode"))?;
    s.at(time).map(|m| m as u32)
}

/// Motor number driven by output channel `ch` (1-based), from SERVOn_FUNCTION.
fn motor_on_channel(log: &Log, ch: usize) -> Option<u32> {
    let f = log.param(&format!("SERVO{ch}_FUNCTION"))? as u32;
    (33..=40).contains(&f).then(|| f - 32)
}

/// Output channels (1-based) driving motors, with the motor number.
fn motor_channels(log: &Log) -> Vec<(usize, u32)> {
    (1..=16).filter_map(|ch| motor_on_channel(log, ch).map(|m| (ch, m))).collect()
}

/// ESC telemetry instances are numbered by output channel.
pub(crate) fn motor_for_esc(log: &Log, instance: u32) -> u32 {
    motor_on_channel(log, instance as usize + 1).unwrap_or(instance + 1)
}

fn join_motors(motors: &[u32]) -> String {
    let names: Vec<String> = motors.iter().map(u32::to_string).collect();
    match names.as_slice() {
        [one] => format!("Motor {one}"),
        [rest @ .., last] => format!("Motors {} and {last}", rest.join(", ")),
        [] => "No motor".to_owned(),
    }
}

fn esc_instances(log: &Log) -> Vec<u32> {
    let Some(inst) = log.table("ESC").and_then(|t| t.num("Instance")) else { return Vec::new() };
    let mut v: Vec<u32> = inst.iter().map(|i| *i as u32).collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// The last AutoTune progress message before `time`, e.g. "Pitch Rate D Down 25%".
fn autotune_step(log: &Log, time: f64) -> Option<String> {
    let msg = log.table("MSG")?;
    let (t, text) = (msg.time_s()?, msg.text("Message")?);
    t.iter()
        .zip(text)
        .rev()
        .find(|(t, m)| **t <= time && m.starts_with("AutoTune:") && !m.contains("pilot overrides"))
        .map(|(_, m)| m.trim_start_matches("AutoTune:").trim().to_owned())
}

pub fn diagnose(log: &Log) -> Diagnosis {
    let mut incidents: Vec<Incident> =
        find_losses_of_control(log).into_iter().map(|(onset, extreme)| build_incident(log, IncidentKind::LossOfControl, onset, Some(extreme))).collect();
    if incidents.is_empty() {
        if let Some(end) = ends_in_flight(log) {
            incidents.push(build_incident(log, IncidentKind::LogEndsInFlight, end, None));
        }
    }
    // Flight-wide checks stop at the first incident: its aftermath is not typical.
    let until = incidents.first().map_or(f64::INFINITY, |i| i.onset_s);
    let mut warnings = esc_error_warnings(log, until);
    warnings.extend(ekf_warnings(log, until));
    Diagnosis { incidents, warnings }
}

/// (onset, extreme) times of each loss of control during flight.
fn find_losses_of_control(log: &Log) -> Vec<(f64, f64)> {
    let table = if log.table("ANG").is_some() { "ANG" } else { "ATT" };
    let get = |f: &str| Series::get(log, table, f);
    let (Some(roll), Some(pitch), Some(des_r), Some(des_p)) = (get("Roll"), get("Pitch"), get("DesRoll"), get("DesPitch")) else {
        return Vec::new();
    };
    let (flight, _) = log.flight();
    let n = roll.t.len().min(pitch.v.len()).min(des_r.v.len()).min(des_p.v.len());
    let error = |i: usize| (roll.v[i] - des_r.v[i]).abs().max((pitch.v[i] - des_p.v[i]).abs());

    let mut out = Vec::new();
    let mut i = 0;
    while i < n {
        let t = roll.t[i];
        let acro = mode_at(log, t).is_some_and(|m| ACRO_MODES.contains(&m));
        let tilted = roll.v[i].abs() > LOST_ANGLE_DEG || pitch.v[i].abs() > LOST_ANGLE_DEG;
        let lost = (tilted && !acro) || error(i) > LOST_ERROR_DEG;
        if lost && flight.iter().any(|f| f.contains(t)) {
            let mut j = i;
            while j > 0 && error(j - 1) > ONSET_ERROR_DEG {
                j -= 1;
            }
            out.push((roll.t[j], t));
            // One incident per 10 s: what follows the first is its aftermath.
            let resume = t + 10.0;
            while i < n && roll.t[i] < resume {
                i += 1;
            }
            continue;
        }
        i += 1;
    }
    out
}

/// The log's last time, if it ends with motors running high above the ground.
fn ends_in_flight(log: &Log) -> Option<f64> {
    let (flight, _) = log.flight();
    let end = log.time_span()?.end;
    let last = flight.last()?;
    let alt = Series::get(log, "CTUN", "Alt")?;
    let ground = stats::percentile(&alt.v, 2.0)?;
    (end - last.end < 0.5 && alt.at(end)? - ground > 2.0).then_some(end)
}

fn build_incident(log: &Log, kind: IncidentKind, onset: f64, extreme: Option<f64>) -> Incident {
    let mode = mode_at(log, onset);
    let log_end = log.time_span().map_or(onset, |s| s.end);
    let mut timeline = Vec::new();
    let mut findings = Vec::new();
    let mut ruled_out = Vec::new();

    let mode_text = mode.map(|m| {
        let name = mode_name(m);
        match (m, autotune_step(log, onset)) {
            (15, Some(step)) => format!("{name} ({step})"),
            _ => name,
        }
    });
    esc_dropouts(log, onset, &mut findings, &mut timeline);
    thrust_loss_messages(log, onset, &mut findings);
    motor_rpm_collapse(log, onset, &mut findings);
    oscillation(log, onset, &mut findings, &mut ruled_out);
    power(log, onset, &mut findings, &mut ruled_out, &mut timeline);
    vibration(log, onset, &mut findings, &mut ruled_out);
    ekf(log, onset, &mut findings, &mut ruled_out);
    saturation(log, onset, &findings.is_empty(), &mut findings);

    match kind {
        IncidentKind::LossOfControl => {
            timeline.push(Event { time_s: onset, text: "Attitude starts to diverge from the target".to_owned() });
            if let Some(t) = extreme {
                timeline.push(Event { time_s: t, text: extreme_text(log, t) });
            }
            if let Some(rate) = peak_rate(log, onset, log_end) {
                timeline.push(Event { time_s: rate.0, text: format!("Peak rotation {:.0} °/s", rate.1) });
            }
        }
        IncidentKind::LogEndsInFlight => {}
    }
    for (t, m) in messages(log, onset - 2.0, log_end) {
        timeline.push(Event { time_s: t, text: format!("Message: {m}") });
    }
    if log_end - onset < END_AFTER_ONSET_S {
        let text = match kind {
            IncidentKind::LossOfControl => format!("Log ends {:.1} s after control was lost, consistent with impact or power loss", log_end - onset),
            IncidentKind::LogEndsInFlight => "Log ends while flying, with motors running".to_owned(),
        };
        timeline.push(Event { time_s: log_end, text });
    }
    timeline.sort_by(|a, b| a.time_s.total_cmp(&b.time_s));
    findings.sort_by_key(|f| f.confidence);

    let what = match kind {
        IncidentKind::LossOfControl => format!("Lost control at {onset:.2} s"),
        IncidentKind::LogEndsInFlight => format!("Log ends in flight at {onset:.2} s"),
    };
    let during = mode_text.map_or(String::new(), |m| format!(" in {m}"));
    let cause = findings.first().map_or(". No clear cause found in the log".to_owned(), |f| format!(". Most likely cause: {}", lowercase_first(&f.title)));
    Incident { kind, onset_s: onset, mode: mode.map(mode_name), summary: format!("{what}{during}{cause}."), timeline, findings, ruled_out }
}

fn capitalize(s: &str) -> String {
    let mut c = s.chars();
    c.next().map_or(String::new(), |f| f.to_uppercase().chain(c).collect())
}

fn lowercase_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        // Keep "ESC", "EKF" and similar acronyms.
        Some(f) if !s.chars().nth(1).is_some_and(|n| n.is_uppercase()) => f.to_lowercase().chain(c).collect(),
        _ => s.to_owned(),
    }
}

fn extreme_text(log: &Log, t: f64) -> String {
    let table = if log.table("ANG").is_some() { "ANG" } else { "ATT" };
    let r = Series::get(log, table, "Roll").and_then(|s| s.at(t)).unwrap_or(f64::NAN);
    let p = Series::get(log, table, "Pitch").and_then(|s| s.at(t)).unwrap_or(f64::NAN);
    format!("Attitude out of control: roll {r:.0}°, pitch {p:.0}°")
}

fn peak_rate(log: &Log, from: f64, to: f64) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for f in ["R", "P", "Y"] {
        let s = Series::get(log, "RATE", f)?;
        for (t, v) in s.t.iter().zip(&s.v) {
            // Beyond any gyro's range: a corrupt message.
            if *t >= from && *t <= to && v.abs() < 5000.0 && best.is_none_or(|b| v.abs() > b.1) {
                best = Some((*t, v.abs()));
            }
        }
    }
    best
}

fn messages(log: &Log, from: f64, to: f64) -> Vec<(f64, String)> {
    let Some(msg) = log.table("MSG") else { return Vec::new() };
    let (Some(t), Some(text)) = (msg.time_s(), msg.text("Message")) else { return Vec::new() };
    t.into_iter().zip(text).filter(|(t, m)| *t >= from && *t <= to && !m.contains("pilot overrides")).map(|(t, m)| (t, m.clone())).collect()
}

/// ESCs that stop sending telemetry around the onset while others carry on.
fn esc_dropouts(log: &Log, onset: f64, findings: &mut Vec<Finding>, timeline: &mut Vec<Event>) {
    let instances = esc_instances(log);
    let last: Vec<(u32, f64)> =
        instances.iter().filter_map(|i| Series::get_where(log, "ESC", "RPM", "Instance", *i as f64)?.last_time().map(|t| (*i, t))).collect();
    let Some(esc_end) = last.iter().map(|(_, t)| *t).reduce(f64::max) else { return };
    let lost: Vec<(u32, f64)> = last.iter().copied().filter(|(_, t)| *t < esc_end - ESC_SILENT_S && *t >= onset - LOOKBACK_S && *t <= onset + 0.5).collect();
    if lost.is_empty() {
        return;
    }
    let motors: Vec<u32> = lost.iter().map(|(i, _)| motor_for_esc(log, *i)).collect();
    let still: Vec<u32> = last.iter().filter(|(i, _)| !lost.iter().any(|(l, _)| l == i)).map(|(i, _)| motor_for_esc(log, *i)).collect();
    let first = lost.iter().map(|(_, t)| *t).fold(f64::INFINITY, f64::min);
    let together = lost.iter().all(|(_, t)| (t - first).abs() < 0.05);
    let before = first <= onset;
    let who = join_motors(&motors);
    timeline.push(Event { time_s: first, text: format!("{who}: ESC telemetry stops") });

    let lead = if before { format!("{:.2} s before control was lost", onset - first) } else { format!("{:.2} s after control was lost", first - onset) };
    let mut evidence = vec![
        format!("{who} sent no ESC telemetry after {first:.2} s ({lead})."),
        format!("{} kept reporting until {esc_end:.2} s, so the telemetry link itself was working.", join_motors(&still)),
    ];
    if motors.len() > 1 && together {
        evidence.push("They stopped within the same 50 ms, which points to a shared cause.".to_owned());
    }
    let advice = if motors.len() > 1 && together {
        "Several ESCs dropping out together usually share a cause: one half or the power feed of a 4-in-1 ESC, \
         a connector or ground, the DShot output group they are on, or motor wires chafed through by the \
         bells and shorting against each other or a carbon frame (look for bare copper; such a short is \
         often intermittent and gone on the bench). Inspect that wiring and solder joints, and bench-test the \
         motors at full throttle with props off. Lowering AutoTune aggressiveness (AUTOTUNE_AGGR) and \
         ATC_ACCEL_*_MAX (ATC_ACC_*_MAX from 4.7) reduces current spikes while testing."
    } else {
        "An ESC that stops reporting has usually reset, desynced or lost its signal. Check the ESC, its motor \
         and connectors, and the motor wires for insulation worn through by the bell; desyncs show up under \
         hard acceleration, so test with full-throttle punches."
    };
    findings.push(Finding {
        id: "esc_dropout",
        confidence: if before { Confidence::High } else { Confidence::Medium },
        title: format!("{who} stopped responding (ESC telemetry lost {lead})"),
        evidence,
        advice: Some(advice.to_owned()),
    });
}

/// ArduPilot's own thrust-loss detector.
fn thrust_loss_messages(log: &Log, onset: f64, findings: &mut Vec<Finding>) {
    let hits: Vec<(f64, String)> = messages(log, onset - 10.0, onset + 1.0).into_iter().filter(|(_, m)| m.contains("Thrust Loss")).collect();
    if let Some((t, m)) = hits.first() {
        findings.push(Finding {
            id: "thrust_loss",
            confidence: Confidence::High,
            title: "ArduPilot detected a loss of thrust".to_owned(),
            evidence: vec![format!("{t:.2} s: \"{m}\" (the number is the motor that hit full output).")],
            advice: Some("Check that motor, its prop and ESC; also check the vehicle isn't overloaded.".to_owned()),
        });
    }
}

/// A motor whose RPM falls far below the others just before the onset.
fn motor_rpm_collapse(log: &Log, onset: f64, findings: &mut Vec<Finding>) {
    let instances = esc_instances(log);
    let rpms: Vec<(u32, Vec<f64>)> = instances
        .iter()
        .filter_map(|i| Series::get_where(log, "ESC", "RPM", "Instance", *i as f64).map(|s| (*i, s)))
        .map(|(i, s)| (i, s.window(onset - 1.0, onset + 0.2)))
        .filter(|(_, w)| !w.is_empty())
        .collect();
    if rpms.len() < 3 {
        return;
    }
    let mins: Vec<(u32, f64)> = rpms.iter().map(|(i, w)| (*i, min_of(w).unwrap_or(0.0))).collect();
    let medians: Vec<f64> = rpms.iter().filter_map(|(_, w)| stats::median(w)).collect();
    let Some(typical) = stats::median(&medians) else { return };
    for (i, min) in mins {
        if typical > 1000.0 && min < 0.3 * typical {
            let m = motor_for_esc(log, i);
            findings.push(Finding {
                id: "rpm_collapse",
                confidence: Confidence::High,
                title: format!("Motor {m} slowed to {min:.0} RPM while the others ran near {typical:.0}"),
                evidence: vec![format!("Window {:.2}-{:.2} s around the onset.", onset - 1.0, onset + 0.2)],
                advice: Some(format!("Motor {m} lost thrust: check for a desync, loose prop, failing motor or ESC.")),
            });
        }
    }
}

/// Growing rate-loop oscillation before the onset means unstable gains.
fn oscillation(log: &Log, onset: f64, findings: &mut Vec<Finding>, ruled_out: &mut Vec<String>) {
    let rms = |v: &[f64]| (v.iter().map(|x| x * x).sum::<f64>() / v.len().max(1) as f64).sqrt();
    // The axis whose rate error grew the most.
    let mut worst: Option<(&str, f64, f64)> = None;
    for (axis, des, act) in [("roll", "RDes", "R"), ("pitch", "PDes", "P"), ("yaw", "YDes", "Y")] {
        let (Some(d), Some(a)) = (Series::get(log, "RATE", des), Series::get(log, "RATE", act)) else { continue };
        let err: Vec<f64> = d.v.iter().zip(&a.v).map(|(d, a)| d - a).collect();
        let e = Series { t: d.t, v: err };
        let pre = e.window(onset - 2.0, onset - 0.1);
        if pre.is_empty() {
            continue;
        }
        let (p, b) = (rms(&pre), rms(&e.window(onset - 32.0, onset - 2.0)));
        if worst.is_none_or(|w| p / b.max(1.0) > w.1 / w.2.max(1.0)) {
            worst = Some((axis, p, b));
        }
    }
    let Some((axis, pre_rms, base_rms)) = worst else { return };
    if pre_rms > OSCILLATION_MIN_DEG_S && pre_rms > OSCILLATION_GROWTH * base_rms {
        findings.push(Finding {
            id: "oscillation",
            confidence: Confidence::Medium,
            title: format!("{} rate control was oscillating before the loss", capitalize(axis)),
            evidence: vec![format!("{} rate error RMS {pre_rms:.0} °/s in the 2 s before, against {base_rms:.0} °/s earlier in the flight.", capitalize(axis))],
            advice: Some(
                "Gains too high or too little filtering. If AutoTune was running, the step it was testing pushed \
                 the gains past stability: restart AutoTune with lower AUTOTUNE_AGGR, or tune manually."
                    .to_owned(),
            ),
        });
    } else {
        ruled_out.push(format!(
            "Unstable gains: rate control was steady just before (worst axis {axis}: rate error RMS {pre_rms:.0} °/s, {base_rms:.0} °/s earlier)."
        ));
    }
}

fn power(log: &Log, onset: f64, findings: &mut Vec<Finding>, ruled_out: &mut Vec<String>, timeline: &mut Vec<Event>) {
    if let Some(vcc) = Series::get(log, "POWR", "Vcc") {
        if let Some(min) = min_of(&vcc.window(onset - LOOKBACK_S, onset + 0.2)) {
            if min < BROWNOUT_VCC {
                findings.push(Finding {
                    id: "brownout",
                    confidence: Confidence::High,
                    title: format!("Flight controller supply dropped to {min:.2} V"),
                    evidence: vec!["POWR.Vcc below 4.5 V can reset sensors and outputs.".to_owned()],
                    advice: Some("Check the BEC / power module and its wiring.".to_owned()),
                });
            }
        }
    }
    let Some(volt) = Series::get(log, "BAT", "Volt") else { return };
    let cells = log.param("MOT_BAT_VOLT_MAX").map(|v| (v / 4.2).round()).filter(|c| *c >= 1.0);
    // Stop short of the onset: a sample taken right at it already shows the
    // event's own current spike.
    let before = volt.window(onset - LOOKBACK_S, onset - 0.1);
    let after = volt.window(onset - 0.1, onset + 1.0);
    let (Some(min_before), Some(start)) = (min_of(&before), before.first().copied()) else { return };
    let per_cell = |v: f64| cells.map_or(String::new(), |c| format!(" ({:.2} V/cell)", v / c));
    if cells.is_some_and(|c| min_before / c < CELL_SAG_V) {
        findings.push(Finding {
            id: "battery_sag",
            confidence: Confidence::Medium,
            title: format!("Battery sagged to {min_before:.1} V{} before the loss", per_cell(min_before)),
            evidence: vec!["Low voltage under load can reset ESCs or leave too little thrust.".to_owned()],
            advice: Some("Use a healthier or higher-C battery; check connectors and battery wiring.".to_owned()),
        });
    } else {
        let mut text = format!("Battery: it held {start:.1}-{min_before:.1} V{} up to the event", per_cell(min_before));
        if let Some(min_after) = min_of(&after).filter(|m| *m < min_before - 1.0) {
            let t = volt.t.iter().zip(&volt.v).find(|(t, v)| **t >= onset && **v == min_after).map_or(onset, |(t, _)| *t);
            timeline.push(Event { time_s: t, text: format!("Battery sags to {min_after:.1} V{}", per_cell(min_after)) });
            text += &format!("; the sag to {min_after:.1} V came after, from the motors at full output");
        }
        ruled_out.push(text + ".");
    }
}

fn vibration(log: &Log, onset: f64, findings: &mut Vec<Finding>, ruled_out: &mut Vec<String>) {
    let mut worst: Option<f64> = None;
    for f in ["VibeX", "VibeY", "VibeZ"] {
        if let Some(m) = Series::get(log, "VIBE", f).and_then(|s| max_of(&s.window(onset - 5.0, onset))) {
            worst = Some(worst.map_or(m, |w| w.max(m)));
        }
    }
    let Some(worst) = worst else { return };
    if worst > VIBE_BAD {
        findings.push(Finding {
            id: "vibration",
            confidence: Confidence::Medium,
            title: format!("Vibration reached {worst:.0} m/s² before the loss"),
            evidence: vec!["Above 60 m/s² the attitude estimate can fail.".to_owned()],
            advice: Some("Look for a damaged prop, loose motor or broken soft-mount.".to_owned()),
        });
    } else {
        ruled_out.push(format!("Vibration: at most {worst:.0} m/s² in the 5 s before."));
    }
}

const EKF_FIELDS: [(&str, &str); 4] = [("SV", "velocity"), ("SP", "position"), ("SH", "height"), ("SM", "compass")];

/// EKF innovations high on average just before the onset (single spikes are
/// common and harmless).
fn ekf(log: &Log, onset: f64, findings: &mut Vec<Finding>, ruled_out: &mut Vec<String>) {
    let mut worst: Option<(&str, &str, f64)> = None;
    for (field, what) in EKF_FIELDS {
        let Some(s) = Series::get_where(log, "XKF4", field, "C", 0.0) else { continue };
        if let Some(m) = stats::mean(&s.window(onset - 5.0, onset)) {
            if worst.is_none_or(|w| m > w.2) {
                worst = Some((field, what, m));
            }
        }
    }
    let Some((field, what, mean)) = worst else { return };
    if mean > EKF_BAD {
        findings.push(Finding {
            id: "ekf",
            confidence: Confidence::Low,
            title: format!("The EKF's {what} estimate was struggling (XKF4.{field} averaged {mean:.2})"),
            evidence: vec!["EKF innovation ratios above 1 mean sensors disagree with the estimate.".to_owned()],
            advice: Some("Check GPS, compass and vibration.".to_owned()),
        });
    } else {
        ruled_out.push(format!("Navigation estimate: EKF innovations averaged below 1 in the 5 s before (highest: {what}, {mean:.2})."));
    }
}

/// Flight periods, cut off at `until`.
fn flight_until(log: &Log, until: f64) -> Vec<(f64, f64)> {
    let (flight, _) = log.flight();
    flight.iter().filter(|f| f.start < until).map(|f| (f.start, f.end.min(until))).collect()
}

/// EKF innovations that are high through the whole flight.
fn ekf_warnings(log: &Log, until: f64) -> Vec<Finding> {
    let flight = flight_until(log, until);
    let mut out = Vec::new();
    for (field, what) in EKF_FIELDS {
        let Some(s) = Series::get_where(log, "XKF4", field, "C", 0.0) else { continue };
        let v: Vec<f64> = flight.iter().flat_map(|(a, b)| s.window(*a, *b)).collect();
        let (Some(p95), Some(max)) = (stats::percentile(&v, 95.0), max_of(&v)) else { continue };
        if p95 <= EKF_BAD {
            continue;
        }
        let advice = if field == "SM" {
            "The compass disagrees with the other sensors much of the time, usually magnetic interference from \
             power wires, the ESC or motors. Move the compass away from them (or use an external one), run \
             compass/motor interference calibration or MagFit, and check COMPASS_ORIENT."
        } else {
            "Check the sensor behind this estimate (GPS for velocity/position, baro for height) and vibration."
        };
        out.push(Finding {
            id: "ekf_innovations",
            confidence: Confidence::Low,
            title: format!("The EKF's {what} innovations were high through the flight (XKF4.{field} 95th percentile {p95:.1}, peaks {max:.1})"),
            evidence: vec!["Values above 1 mean the sensor and the estimate disagree.".to_owned()],
            advice: Some(advice.to_owned()),
        });
    }
    out
}

/// Motor outputs pinned at their limits right after the onset show the
/// controller asked for everything and still couldn't recover.
fn saturation(log: &Log, onset: f64, no_cause_yet: &bool, findings: &mut Vec<Finding>) {
    let channels = motor_channels(log);
    let (Some(pmin), Some(pmax)) = (log.param("MOT_PWM_MIN"), log.param("MOT_PWM_MAX")) else { return };
    let spin_min = log.param("MOT_SPIN_MIN").unwrap_or(0.15);
    let low = pmin + (pmax - pmin) * spin_min + 10.0;
    let high = pmax - 10.0;
    let mut at_max = Vec::new();
    let mut at_min = Vec::new();
    for (ch, motor) in &channels {
        let Some(s) = Series::get(log, "RCOU", &format!("C{ch}")) else { continue };
        let w = s.window(onset - 0.2, onset + 0.3);
        if w.iter().any(|v| *v >= high) {
            at_max.push(*motor);
            continue;
        }
        if w.iter().any(|v| *v <= low) {
            at_min.push(*motor);
        }
    }
    if at_max.is_empty() {
        return;
    }
    let mut evidence = vec![format!("{} at full output around the onset.", join_motors(&at_max))];
    if !at_min.is_empty() {
        evidence.push(format!("{} at minimum output.", join_motors(&at_min)));
    }
    evidence.push("The attitude kept diverging anyway, so thrust or control authority was missing.".to_owned());
    findings.push(Finding {
        id: "saturation",
        // Supporting evidence once a cause is known; a lead on its own otherwise.
        confidence: if *no_cause_yet { Confidence::Medium } else { Confidence::Low },
        title: "Controller ran out of authority: motor outputs hit their limits".to_owned(),
        evidence,
        advice: if *no_cause_yet {
            Some("Without ESC telemetry to confirm, suspect a motor, prop or ESC on the side that couldn't hold.".to_owned())
        } else {
            None
        },
    });
}

/// ESCs whose telemetry error rate is high over the whole flight.
fn esc_error_warnings(log: &Log, until: f64) -> Vec<Finding> {
    let flight = flight_until(log, until);
    let rates: Vec<(u32, f64, f64)> = esc_instances(log)
        .into_iter()
        .filter_map(|i| {
            let s = Series::get_where(log, "ESC", "Err", "Instance", i as f64)?;
            let v: Vec<f64> = flight.iter().flat_map(|(a, b)| s.window(*a, *b)).collect();
            Some((i, stats::mean(&v)?, max_of(&v)?))
        })
        .collect();
    let mut out = Vec::new();
    for (i, mean, max) in &rates {
        let others: Vec<f64> = rates.iter().filter(|(j, _, _)| j != i).map(|(_, m, _)| *m).collect();
        let typical = stats::median(&others).unwrap_or(0.0);
        if *mean > ESC_ERR_WARN_PCT && *mean > 2.0 * typical {
            let m = motor_for_esc(log, *i);
            out.push(Finding {
                id: "esc_telemetry_errors",
                confidence: Confidence::Low,
                title: format!("Motor {m}: ESC telemetry error rate {mean:.1}% (peaks {max:.0}%)"),
                evidence: vec![format!("Other motors average {typical:.1}%.")],
                advice: Some(format!(
                    "Errors on one motor's bidirectional DShot line suggest a weak signal or ground on motor {m}'s \
                     wiring or ESC. Worth inspecting, especially if that ESC misbehaves."
                )),
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{LogWriter, V};

    const END_S: f64 = 30.0;

    struct Flight {
        /// ESC instance 0 stops reporting at this time.
        esc0_stops: Option<f64>,
        /// Rate error oscillation amplitude grows from this time.
        oscillation_from: Option<f64>,
        /// Attitude diverges from this time.
        diverge_at: Option<f64>,
    }

    fn write(f: &Flight) -> Log {
        let mut w = LogWriter::new();
        let parm = w.define("PARM", "QNff", "TimeUS,Name,Value,Default");
        let msg = w.define("MSG", "QZ", "TimeUS,Message");
        let mode = w.define("MODE", "QMBB", "TimeUS,Mode,ModeNum,Rsn");
        let ev = w.define("EV", "QB", "TimeUS,Id");
        let ang = w.define("ANG", "Qffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch");
        let rate = w.define("RATE", "Qffffff", "TimeUS,RDes,R,PDes,P,YDes,Y");
        let esc = w.define("ESC", "QBff", "TimeUS,Instance,RPM,Err");
        let rcou = w.define("RCOU", "QHHHH", "TimeUS,C1,C2,C3,C4");
        let bat = w.define("BAT", "Qff", "TimeUS,Volt,Curr");
        let us = |t: f64| V::I((t * 1e6) as i64);
        for (n, v) in [
            ("SERVO1_FUNCTION", 33.0),
            ("SERVO2_FUNCTION", 34.0),
            ("SERVO3_FUNCTION", 35.0),
            ("SERVO4_FUNCTION", 36.0),
            ("MOT_PWM_MIN", 1000.0),
            ("MOT_PWM_MAX", 2000.0),
            ("MOT_SPIN_MIN", 0.1),
            ("MOT_BAT_VOLT_MAX", 25.2),
        ] {
            w.write(parm, &[us(0.1), V::S(n), V::F(v), V::F(v)]);
        }
        w.write(ev, &[us(1.0), V::I(10)]);
        w.write(mode, &[us(1.0), V::I(15), V::I(15), V::I(0)]);
        w.write(msg, &[us(2.0), V::S("AutoTune: Pitch Rate D Down 25%")]);

        let diverging = |t: f64| f.diverge_at.is_some_and(|d| t >= d);
        for i in 0..(END_S * 400.0) as usize {
            let t = i as f64 / 400.0;
            let osc = f.oscillation_from.map_or(0.0, |o| if t >= o { 15.0 * (t - o) } else { 0.0 });
            let wobble = osc * (std::f64::consts::TAU * 12.0 * t).sin();
            let roll = if diverging(t) { -400.0 * (t - f.diverge_at.unwrap()) } else { 0.1 * wobble };
            w.write(ang, &[us(t), V::F(0.0), V::F(roll), V::F(0.0), V::F(0.0)]);
            let r = if diverging(t) { -400.0 } else { wobble };
            w.write(rate, &[us(t), V::F(0.0), V::F(r), V::F(0.0), V::F(0.5), V::F(0.0), V::F(-0.5)]);
        }
        for i in 0..(END_S * 100.0) as usize {
            let t = i as f64 / 100.0;
            for inst in 0..4 {
                if inst == 0 && f.esc0_stops.is_some_and(|s| t > s) {
                    continue;
                }
                w.write(esc, &[us(t), V::I(inst), V::F(3500.0), V::F(if inst == 0 { 2.0 } else { 0.2 })]);
            }
        }
        for i in 0..(END_S * 10.0) as usize {
            let t = i as f64 / 10.0;
            let pwm = |hi: bool| {
                if diverging(t) {
                    if hi {
                        2000
                    } else {
                        1100
                    }
                } else {
                    1400
                }
            };
            w.write(rcou, &[us(t), V::I(pwm(false)), V::I(pwm(true)), V::I(pwm(false)), V::I(pwm(true))]);
            let volt = if diverging(t) { 19.0 } else { 24.0 };
            w.write(bat, &[us(t), V::F(volt), V::F(20.0)]);
        }
        Log::from_bytes(&w.into_bytes())
    }

    #[test]
    fn blames_esc_dropout_before_loss_of_control() {
        let log = write(&Flight { esc0_stops: Some(28.9), oscillation_from: None, diverge_at: Some(29.0) });
        let d = diagnose(&log);
        assert_eq!(d.incidents.len(), 1, "{d:#?}");
        let inc = &d.incidents[0];
        assert_eq!(inc.kind, IncidentKind::LossOfControl);
        assert!((inc.onset_s - 29.0).abs() < 0.05, "{}", inc.onset_s);
        assert_eq!(inc.mode.as_deref(), Some("AutoTune"));
        let top = &inc.findings[0];
        assert_eq!((top.id, top.confidence), ("esc_dropout", Confidence::High), "{:#?}", inc.findings);
        assert!(top.title.starts_with("Motor 1 stopped responding"), "{}", top.title);
        assert!(inc.summary.contains("Pitch Rate D Down 25%"), "{}", inc.summary);
        assert!(inc.findings.iter().any(|f| f.id == "saturation" && f.confidence == Confidence::Low));
        assert!(inc.ruled_out.iter().any(|r| r.starts_with("Unstable gains")), "{:?}", inc.ruled_out);
        assert!(inc.ruled_out.iter().any(|r| r.starts_with("Battery")), "{:?}", inc.ruled_out);
        assert!(inc.timeline.iter().any(|e| e.text.starts_with("Log ends")));
        assert_eq!(d.warnings.len(), 1);
        assert!(d.warnings[0].title.starts_with("Motor 1: ESC telemetry error rate"));
    }

    #[test]
    fn blames_oscillation_when_motors_are_fine() {
        let log = write(&Flight { esc0_stops: None, oscillation_from: Some(26.0), diverge_at: Some(29.0) });
        let inc = &diagnose(&log).incidents[0];
        assert_eq!(inc.findings[0].id, "oscillation", "{:#?}", inc.findings);
        assert!(!inc.findings.iter().any(|f| f.id == "esc_dropout"));
    }

    #[test]
    fn calm_flight_has_no_incidents() {
        let log = write(&Flight { esc0_stops: None, oscillation_from: None, diverge_at: None });
        assert!(diagnose(&log).incidents.is_empty());
    }
}
