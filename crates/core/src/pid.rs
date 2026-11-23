//! Rate-loop PID analysis: step response per axis estimated from flight data
//! by Wiener deconvolution of target rate against actual rate, rate-error
//! oscillations, PID term health, cautious gain suggestions, and the results
//! of a finished AutoTune that weren't saved.

use std::f64::consts::PI;

use rustfft::num_complex::Complex;
use rustfft::FftPlanner;
use serde::Serialize;

use crate::log::{Interval, Log};
use crate::noise::{db, Fft, Series};
use crate::params::format_value;
use crate::stats;

/// Analysis window and step between windows.
const WIN_S: f64 = 2.0;
const HOP_S: f64 = 0.5;
/// Length of the step response shown.
pub const RESPONSE_S: f64 = 0.5;
/// Steady state is measured over this part of the response.
const STEADY_FROM_S: f64 = 0.25;
/// A window needs this much target-rate movement to excite the loop.
const MIN_INPUT_DEG_S: f64 = 20.0;
/// Above this the transfer-function estimate is rolled off (noise).
const CUT_HZ: f64 = 30.0;
const REGULARISATION: f64 = 1e-3;
/// Estimates whose steady state is outside this range are discarded.
const STEADY_RANGE: (f64, f64) = (0.5, 1.5);
pub const MIN_WINDOWS: usize = 5;
const SETTLE_BAND: f64 = 0.05;
const OSC_MIN_HZ: f64 = 8.0;
const OSC_MIN_PROMINENCE_DB: f64 = 10.0;
/// Smaller oscillations don't matter in flight.
const OSC_MIN_AMPLITUDE_DEG_S: f64 = 1.0;
/// No gyro measures beyond this; larger rates come from corrupt messages.
const MAX_RATE_DEG_S: f64 = 5000.0;
/// PID terms are fractions of full output; anything far beyond is corrupt.
const MAX_OUTPUT: f64 = 100.0;
const D_NOISE_HZ: f64 = 40.0;
const SATURATED: f64 = 0.9;
const MODE_AUTOTUNE: u32 = 15;
/// Gain changes suggested are capped at this fraction.
const STEP_FRACTION: f64 = 0.15;
const OVERSHOOT_HIGH_PCT: f64 = 20.0;
const OVERSHOOT_LOW_PCT: f64 = 5.0;
const SLOW_RISE_MS: f64 = 60.0;
const RECOMMENDED_SMAX: f64 = 50.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AxisDef {
    name: &'static str,
    des: &'static str,
    act: &'static str,
    out: &'static str,
    pid_table: &'static str,
    param: &'static str,
    accel: &'static str,
}

const AXES: [AxisDef; 3] = [
    AxisDef { name: "roll", des: "RDes", act: "R", out: "ROut", pid_table: "PIDR", param: "RLL", accel: "R" },
    AxisDef { name: "pitch", des: "PDes", act: "P", out: "POut", pid_table: "PIDP", param: "PIT", accel: "P" },
    AxisDef { name: "yaw", des: "YDes", act: "Y", out: "YOut", pid_table: "PIDY", param: "YAW", accel: "Y" },
];

#[derive(Debug, Clone, Serialize)]
pub struct StepResponse {
    pub time_s: Vec<f64>,
    /// Averaged response to a unit step in target rate.
    pub response: Vec<f64>,
    pub windows: usize,
    pub rise_ms: Option<f64>,
    pub delay_ms: Option<f64>,
    pub overshoot_pct: f64,
    pub settling_ms: Option<f64>,
    /// Final value of the response; 1 means the target is reached.
    pub steady_state: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct Oscillation {
    pub freq_hz: f64,
    pub prominence_db: f64,
    /// RMS rate error in the band around the peak, deg/s.
    pub amplitude_deg_s: f64,
    /// The peak is also in the measured rate, not only in the target.
    pub in_actual: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct Gains {
    pub p: Option<f64>,
    pub i: Option<f64>,
    pub d: Option<f64>,
    pub ff: Option<f64>,
    pub fltd: Option<f64>,
    pub fltt: Option<f64>,
    pub flte: Option<f64>,
    pub smax: Option<f64>,
    pub angle_p: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct TermStats {
    /// Mean I term in steady flight, as a fraction of full output.
    pub i_mean: Option<f64>,
    /// Share of D-term power above 40 Hz (D reacting to noise).
    pub d_noise_share: Option<f64>,
    /// Time the slew limiter reduced the gains, percent.
    pub slew_limited_pct: Option<f64>,
    /// Time the axis output was near its limit, percent.
    pub saturated_pct: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Suggestion {
    pub name: String,
    pub current: Option<f64>,
    pub value: f64,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct AxisReport {
    pub axis: &'static str,
    pub gains: Gains,
    pub step: Option<StepResponse>,
    /// The step response includes AutoTune periods, where gains vary.
    pub includes_autotune: bool,
    pub oscillation: Option<Oscillation>,
    pub terms: TermStats,
    pub findings: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PidReport {
    pub sample_rate_hz: f64,
    pub axes: Vec<AxisReport>,
    /// Cautious gain changes from the analysis.
    pub suggestions: Vec<Suggestion>,
    /// Gains a finished AutoTune found but didn't save.
    pub autotune: Vec<Suggestion>,
    pub notes: Vec<String>,
}

/// A signal resampled onto a uniform time grid.
struct Uniform {
    t0: f64,
    fs: f64,
    v: Vec<f64>,
}

/// Linear interpolation of a series onto a uniform grid over `iv`. Values
/// beyond `limit` are physically impossible (corrupt messages) and skipped.
fn resample(s: &Series, iv: Interval, fs: f64, limit: f64) -> Option<Uniform> {
    // Drop repeated timestamps: interpolation needs increasing time.
    let mut t = Vec::with_capacity(s.t.len());
    let mut v = Vec::with_capacity(s.v.len());
    for (ti, vi) in s.t.iter().zip(&s.v) {
        if t.last().is_none_or(|last| ti > last) && vi.is_finite() && vi.abs() <= limit {
            t.push(*ti);
            v.push(*vi);
        }
    }
    let start = iv.start.max(*t.first()?);
    let end = iv.end.min(*t.last()?);
    if end - start < WIN_S {
        return None;
    }
    let n = ((end - start) * fs) as usize;
    let mut out = Vec::with_capacity(n);
    let mut j = 0;
    for k in 0..n {
        let tk = start + k as f64 / fs;
        while j + 1 < t.len() && t[j + 1] < tk {
            j += 1;
        }
        let (t0, t1) = (t[j], t[(j + 1).min(t.len() - 1)]);
        let w = if t1 > t0 { ((tk - t0) / (t1 - t0)).clamp(0.0, 1.0) } else { 0.0 };
        out.push(v[j] + (v[(j + 1).min(v.len() - 1)] - v[j]) * w);
    }
    Some(Uniform { t0: start, fs, v: out })
}

fn tukey(n: usize, alpha: f64) -> Vec<f64> {
    let edge = (alpha * (n as f64 - 1.0) / 2.0).max(1.0);
    (0..n)
        .map(|i| {
            let i = i as f64;
            let from_end = (n as f64 - 1.0) - i;
            let d = i.min(from_end);
            if d < edge {
                0.5 * (1.0 - (PI * d / edge).cos())
            } else {
                1.0
            }
        })
        .collect()
}

/// Step response of one window: Wiener deconvolution of `y` by `x`, the
/// transfer function rolled off above [`CUT_HZ`], integrated impulse response.
fn window_step(x: &[f64], y: &[f64], fs: f64, planner: &mut FftPlanner<f64>) -> Vec<f64> {
    let n = x.len();
    let nfft = (2 * n).next_power_of_two();
    let w = tukey(n, 0.5);
    let mx = x.iter().sum::<f64>() / n as f64;
    let my = y.iter().sum::<f64>() / n as f64;
    let mut xs: Vec<Complex<f64>> = (0..nfft).map(|i| Complex::new(if i < n { (x[i] - mx) * w[i] } else { 0.0 }, 0.0)).collect();
    let mut ys: Vec<Complex<f64>> = (0..nfft).map(|i| Complex::new(if i < n { (y[i] - my) * w[i] } else { 0.0 }, 0.0)).collect();
    let fwd = planner.plan_fft_forward(nfft);
    fwd.process(&mut xs);
    fwd.process(&mut ys);

    let df = fs / nfft as f64;
    let freq = |k: usize| (k.min(nfft - k)) as f64 * df;
    let band_power = {
        let v: Vec<f64> = (0..nfft).filter(|k| freq(*k) < CUT_HZ).map(|k| xs[k].norm_sqr()).collect();
        stats::mean(&v).unwrap_or(1.0)
    };
    let lambda = REGULARISATION * band_power;
    let sigma = CUT_HZ / 3.0;
    let mut h: Vec<Complex<f64>> = (0..nfft)
        .map(|k| {
            let f = freq(k);
            let roll_off = if f <= CUT_HZ { 1.0 } else { (-((f - CUT_HZ) / sigma).powi(2)).exp() };
            ys[k] * xs[k].conj() / (xs[k].norm_sqr() + lambda) * roll_off
        })
        .collect();
    planner.plan_fft_inverse(nfft).process(&mut h);
    let m = (RESPONSE_S * fs) as usize;
    let mut acc = 0.0;
    h.iter()
        .take(m)
        .map(|c| {
            acc += c.re / nfft as f64;
            acc
        })
        .collect()
}

fn steady(step: &[f64], fs: f64) -> f64 {
    let from = (STEADY_FROM_S * fs) as usize;
    stats::mean(&step[from.min(step.len().saturating_sub(1))..]).unwrap_or(0.0)
}

/// Averaged step response over the excited windows of `segments`.
fn step_response(segments: &[(Uniform, Uniform)]) -> Option<StepResponse> {
    let fs = segments.first()?.0.fs;
    let n = (WIN_S * fs) as usize;
    let hop = (HOP_S * fs) as usize;
    let mut planner = FftPlanner::new();
    let mut kept: Vec<Vec<f64>> = Vec::new();
    for (x, y) in segments {
        let mut start = 0;
        while start + n <= x.v.len().min(y.v.len()) {
            let xw = &x.v[start..start + n];
            let active = xw.iter().map(|v| v.abs()).fold(0.0, f64::max) >= MIN_INPUT_DEG_S && stats::std_dev(xw).is_some_and(|s| s > MIN_INPUT_DEG_S / 4.0);
            if active {
                let step = window_step(xw, &y.v[start..start + n], fs, &mut planner);
                let ss = steady(&step, fs);
                if ss > STEADY_RANGE.0 && ss < STEADY_RANGE.1 {
                    kept.push(step);
                }
            }
            start += hop;
        }
    }
    if kept.len() < MIN_WINDOWS {
        return None;
    }
    let m = kept[0].len();
    let response: Vec<f64> = (0..m).map(|i| kept.iter().map(|s| s[i]).sum::<f64>() / kept.len() as f64).collect();
    let time_s: Vec<f64> = (0..m).map(|i| i as f64 / fs).collect();
    let ss = steady(&response, fs);
    let first_at = |level: f64| response.iter().position(|v| *v >= level * ss).map(|i| time_s[i] * 1000.0);
    let rise_ms = first_at(0.9).zip(first_at(0.1)).map(|(a, b)| a - b);
    let peak = response.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let outside = response.iter().rposition(|v| (v - ss).abs() > SETTLE_BAND * ss.abs());
    let settling_ms = match outside {
        Some(i) if i + 1 < m => Some(time_s[i + 1] * 1000.0),
        Some(_) => None,
        None => Some(0.0),
    };
    Some(StepResponse {
        windows: kept.len(),
        rise_ms,
        delay_ms: first_at(0.5),
        overshoot_pct: ((peak - ss) / ss * 100.0).max(0.0),
        settling_ms,
        steady_state: ss,
        time_s,
        response,
    })
}

/// Welch PSD of a set of uniform segments.
fn welch_1d(segments: &[&Uniform]) -> Option<(Vec<f64>, Vec<f64>)> {
    let fs = segments.first()?.fs;
    let n = ((WIN_S * fs) as usize).next_power_of_two() / 2;
    let fft = Fft::new(n, fs);
    let mut acc = vec![0.0; n / 2 + 1];
    let mut count = 0;
    for s in segments {
        let mut start = 0;
        while start + n <= s.v.len() {
            acc.iter_mut().zip(fft.psd(&s.v[start..start + n])).for_each(|(a, p)| *a += p);
            count += 1;
            start += n / 2;
        }
    }
    (count > 0).then(|| (fft.freqs(), acc.iter().map(|v| v / count as f64).collect()))
}

/// The most prominent peak above [`OSC_MIN_HZ`] against a lower-envelope floor.
fn strongest_peak(freq: &[f64], psd: &[f64]) -> Option<(usize, f64)> {
    let dbs: Vec<f64> = psd.iter().map(|v| db(*v)).collect();
    let df = freq.get(1).copied()?;
    let n = dbs.len();
    let mut best: Option<(usize, f64)> = None;
    for k in 2..n.saturating_sub(2) {
        if freq[k] < OSC_MIN_HZ || freq[k] > 0.45 * 2.0 * freq[n - 1] {
            continue;
        }
        if !(k - 2..=k + 2).all(|j| dbs[j] <= dbs[k]) {
            continue;
        }
        let half = ((10.0f64.max(0.5 * freq[k]) / df).round() as usize).max(3);
        let w = &dbs[k.saturating_sub(half)..(k + half + 1).min(n)];
        let prom = dbs[k] - stats::percentile(w, 20.0).unwrap_or(dbs[k]);
        if best.is_none_or(|b| prom > b.1) {
            best = Some((k, prom));
        }
    }
    best
}

fn band_rms(freq: &[f64], psd: &[f64], centre: f64) -> f64 {
    let df = freq.get(1).copied().unwrap_or(1.0);
    freq.iter().zip(psd).filter(|(f, _)| (**f - centre).abs() <= 0.15 * centre).map(|(_, p)| p * df).sum::<f64>().sqrt()
}

fn mode_intervals(log: &Log, mode: u32) -> Vec<Interval> {
    let Some(m) = Series::get(log, "MODE", "ModeNum").or_else(|| Series::get(log, "MODE", "Mode")) else { return Vec::new() };
    let end = log.time_span().map_or(0.0, |s| s.end);
    let mut out = Vec::new();
    for (i, (t, v)) in m.t.iter().zip(&m.v).enumerate() {
        if *v as u32 == mode {
            out.push(Interval { start: *t, end: m.t.get(i + 1).copied().unwrap_or(end) });
        }
    }
    out
}

/// `a` minus every interval in `cut`.
fn subtract(a: &[Interval], cut: &[Interval]) -> Vec<Interval> {
    let mut out = a.to_vec();
    for c in cut {
        out = out
            .into_iter()
            .flat_map(|i| {
                let mut parts = Vec::new();
                if c.start > i.start {
                    parts.push(Interval { start: i.start, end: c.start.min(i.end) });
                }
                if c.end < i.end {
                    parts.push(Interval { start: c.end.max(i.start), end: i.end });
                }
                parts.into_iter().filter(|p| p.duration() > 0.0)
            })
            .collect();
    }
    out
}

/// The rate and angle gains flown: `flown` holds earlier values where the
/// gains changed after the flight started.
fn gains(log: &Log, ax: &AxisDef, flown: &std::collections::HashMap<String, f64>) -> Gains {
    let get = |name: String| flown.get(&name).copied().or_else(|| log.param(&name));
    let p = |s: &str| get(format!("ATC_RAT_{}_{s}", ax.param));
    Gains {
        p: p("P"),
        i: p("I"),
        d: p("D"),
        ff: p("FF"),
        fltd: p("FLTD"),
        fltt: p("FLTT"),
        flte: p("FLTE"),
        smax: p("SMAX"),
        angle_p: get(format!("ATC_ANG_{}_P", ax.param)),
    }
}

fn is_gain(name: &str) -> bool {
    name.starts_with("ATC_RAT_") || name.starts_with("ATC_ANG_")
}

/// The first time at or after `from` that a rate or angle gain was changed
/// (AutoTune saving its gains at the disarm, or a change made in flight),
/// with the values the gains had before.
fn gain_change(log: &Log, from: f64) -> Option<(f64, std::collections::HashMap<String, f64>)> {
    let parm = log.table("PARM")?;
    let (names, values, times) = (parm.text("Name")?, parm.num("Value")?, parm.time_s()?);
    let mut before: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
    for ((name, value), t) in names.iter().zip(values).zip(&times) {
        if !is_gain(name) {
            continue;
        }
        match before.get(name) {
            // Compared as the vehicle stores them: 32-bit floats.
            Some(old) if *t >= from && *old as f32 != *value as f32 => return Some((*t, before)),
            Some(_) => {}
            None => {
                before.insert(name.clone(), *value);
            }
        }
    }
    None
}

pub fn analyse(log: &Log) -> Result<PidReport, String> {
    let rate = log.table("RATE").ok_or("this log has no RATE messages; enable ATTITUDE_FAST in LOG_BITMASK".to_owned())?;
    let t = rate.time_s().ok_or("RATE has no timestamps")?;
    let dt: Vec<f64> = t.windows(2).map(|w| w[1] - w[0]).filter(|d| *d > 0.0).collect();
    let fs = 1.0 / stats::median(&dt).ok_or("too few RATE messages")?;
    let fs = (fs / 10.0).round() * 10.0;
    if fs < 100.0 {
        return Err(format!("RATE is logged at only {fs:.0} Hz; step-response analysis needs loop-rate logging (ATTITUDE_FAST in LOG_BITMASK)"));
    }
    let (flight, _) = log.flight();
    // Gains changed after take-off: the flight flew the earlier ones, and
    // suggestions against the new ones would be wrong.
    let changed = flight.first().and_then(|f| gain_change(log, f.start));
    let flown = changed.as_ref().map(|(_, before)| before.clone()).unwrap_or_default();
    let autotune = mode_intervals(log, MODE_AUTOTUNE);
    let normal = subtract(&flight, &autotune);
    let mut notes = Vec::new();

    let mut axes = Vec::new();
    let mut suggestions = Vec::new();
    for ax in AXES {
        let (Some(des), Some(act)) = (Series::get(log, "RATE", ax.des), Series::get(log, "RATE", ax.act)) else { continue };
        let pairs = |ivs: &[Interval]| -> Vec<(Uniform, Uniform)> {
            ivs.iter().filter_map(|iv| Some((resample(&des, *iv, fs, MAX_RATE_DEG_S)?, resample(&act, *iv, fs, MAX_RATE_DEG_S)?))).collect()
        };
        let normal_pairs = pairs(&normal);
        let (step, includes_autotune, used) = match step_response(&normal_pairs) {
            Some(s) => (Some(s), false, normal_pairs),
            None => {
                let all = pairs(&flight);
                (step_response(&all), !autotune.is_empty(), all)
            }
        };

        // Oscillation in the rate error, confirmed in the measured rate.
        let errors: Vec<Uniform> = used.iter().map(|(x, y)| Uniform { t0: x.t0, fs, v: x.v.iter().zip(&y.v).map(|(a, b)| a - b).collect() }).collect();
        let oscillation = welch_1d(&errors.iter().collect::<Vec<_>>()).and_then(|(f, p)| {
            let (k, prom) = strongest_peak(&f, &p)?;
            let amplitude = band_rms(&f, &p, f[k]);
            if prom < OSC_MIN_PROMINENCE_DB || amplitude < OSC_MIN_AMPLITUDE_DEG_S {
                return None;
            }
            let actual = welch_1d(&used.iter().map(|(_, y)| y).collect::<Vec<_>>());
            let in_actual = actual.is_some_and(|(fa, pa)| {
                let ka = fa.iter().position(|x| *x >= f[k]).unwrap_or(0);
                let lo = ka.saturating_sub(10);
                let floor = stats::percentile(&pa[lo..(ka + 10).min(pa.len())].iter().map(|v| db(*v)).collect::<Vec<_>>(), 20.0);
                floor.is_some_and(|fl| db(pa[ka]) - fl > 6.0)
            });
            Some(Oscillation { freq_hz: f[k], prominence_db: prom, amplitude_deg_s: amplitude, in_actual })
        });

        let terms = term_stats(log, &ax, &normal, &flight, fs);
        let g = gains(log, &ax, &flown);
        let mut findings = Vec::new();
        axis_findings(&ax, &g, step.as_ref(), includes_autotune, oscillation.as_ref(), &terms, &mut findings);
        if !includes_autotune && changed.is_none() {
            if let Some(s) = &step {
                suggest(&ax, &g, s, oscillation.as_ref(), &mut suggestions);
            }
        }
        if g.smax == Some(0.0) && ax.name != "yaw" {
            suggestions.push(Suggestion {
                name: format!("ATC_RAT_{}_SMAX", ax.param),
                current: Some(0.0),
                value: RECOMMENDED_SMAX,
                reason: format!(
                    "Turn the {} slew limiter on: it automatically cuts P and D if the loop starts to oscillate, \
                     a safety net ArduPilot recommends.",
                    ax.name
                ),
            });
        }
        axes.push(AxisReport { axis: ax.name, gains: g, step, includes_autotune, oscillation, terms, findings });
    }
    if axes.iter().any(|a| a.includes_autotune) {
        notes.push(
            "Some axes had too little normal flight with stick inputs, so their step response includes AutoTune \
             periods, where AutoTune keeps changing the gains. Gain suggestions are skipped for those axes; fly a \
             test with stick snaps in AltHold for a clean measurement."
                .to_owned(),
        );
    }
    if let Some((t, _)) = &changed {
        let landed = flight.last().is_some_and(|f| *t >= f.end - 1.0);
        notes.push(if landed {
            format!(
                "The gains were changed at {t:.0} s, after the flight (AutoTune saves its gains at the disarm): the \
                 flight flew the earlier gains, shown here. Its step response says nothing about the new gains, so \
                 there are no suggestions. Fly the new gains (outside AutoTune) and open that log."
            )
        } else {
            format!(
                "The gains were changed at {t:.0} s, during the flight: the step response mixes the earlier gains, \
                 shown here, with the new ones, so there are no suggestions. Fly the new gains and open that log."
            )
        });
    }
    let (autotune, tuned) = autotune_results(log);
    if !autotune.is_empty() && !tuned.is_empty() && tuned.len() < 3 {
        notes.push(format!("AutoTune only finished {}; the other axes keep their current gains if you apply its results.", tuned.join(" and ")));
    }
    Ok(PidReport { sample_rate_hz: fs, axes, suggestions, autotune, notes })
}

fn term_stats(log: &Log, ax: &AxisDef, normal: &[Interval], flight: &[Interval], fs: f64) -> TermStats {
    let steady_ivs = if normal.iter().map(Interval::duration).sum::<f64>() > 5.0 { normal } else { flight };
    let values = |s: &Series, ivs: &[Interval]| -> Vec<f64> {
        ivs.iter().flat_map(|i| s.window(i.start, i.end).to_vec()).filter(|v| v.is_finite() && v.abs() <= MAX_OUTPUT).collect()
    };
    let pid = |f: &str| Series::get(log, ax.pid_table, f);
    let i_mean = pid("I").and_then(|s| stats::mean(&values(&s, steady_ivs)));
    let slew_limited_pct = pid("Dmod").and_then(|s| {
        let v = values(&s, flight);
        (!v.is_empty()).then(|| v.iter().filter(|d| **d < 0.95).count() as f64 / v.len() as f64 * 100.0)
    });
    let d_noise_share = pid("D").and_then(|s| {
        let segs: Vec<Uniform> = flight.iter().filter_map(|iv| resample(&s, *iv, fs, MAX_OUTPUT)).collect();
        let (f, p) = welch_1d(&segs.iter().collect::<Vec<_>>())?;
        let total: f64 = p.iter().sum();
        let high: f64 = f.iter().zip(&p).filter(|(f, _)| **f >= D_NOISE_HZ).map(|(_, p)| *p).sum();
        (total > 0.0).then(|| high / total)
    });
    let saturated_pct = Series::get(log, "RATE", ax.out).and_then(|s| {
        let v = values(&s, flight);
        (!v.is_empty()).then(|| v.iter().filter(|o| o.abs() >= SATURATED).count() as f64 / v.len() as f64 * 100.0)
    });
    TermStats { i_mean, d_noise_share, slew_limited_pct, saturated_pct }
}

fn axis_findings(
    ax: &AxisDef,
    g: &Gains,
    step: Option<&StepResponse>,
    includes_autotune: bool,
    osc: Option<&Oscillation>,
    terms: &TermStats,
    out: &mut Vec<String>,
) {
    let axis = ax.name;
    match step {
        None => out.push(format!("Not enough sharp {axis} inputs to measure the step response; fly stick snaps on this axis.")),
        Some(s) => {
            let caveat = if includes_autotune { " (includes AutoTune periods)" } else { "" };
            if s.overshoot_pct > OVERSHOOT_HIGH_PCT {
                out.push(format!("The {axis} response overshoots by {:.0}%{caveat}: the loop is under-damped.", s.overshoot_pct));
            }
            if s.steady_state < 0.85 {
                out.push(format!("The {axis} rate settles at {:.0}% of the target{caveat}: tracking is weak (I or FF low).", s.steady_state * 100.0));
            }
            if s.overshoot_pct < OVERSHOOT_LOW_PCT && s.rise_ms.is_some_and(|r| r > SLOW_RISE_MS) {
                out.push(format!("The {axis} response is slow (rise time {:.0} ms) with no overshoot{caveat}: there is room for more P.", s.rise_ms.unwrap()));
            }
        }
    }
    if let Some(o) = osc {
        let real = if o.in_actual { "visible in the measured rate, so the vehicle is really moving" } else { "in the error only" };
        out.push(format!("The {axis} rate error has a {:.0} Hz oscillation ({:.1} °/s RMS, {real}).", o.freq_hz, o.amplitude_deg_s));
    }
    if terms.d_noise_share.is_some_and(|d| d > 0.5) {
        out.push(format!(
            "Most of the {axis} D term is above 40 Hz ({:.0}%): D is mostly amplifying noise. Improve gyro \
             filtering or lower ATC_RAT_{}_D / ATC_RAT_{}_FLTD.",
            terms.d_noise_share.unwrap() * 100.0,
            ax.param,
            ax.param
        ));
    }
    if terms.saturated_pct.is_some_and(|p| p > 2.0) {
        out.push(format!("The {axis} output was at its limit {:.1}% of the time: the vehicle may lack control authority.", terms.saturated_pct.unwrap()));
    }
    if terms.i_mean.is_some_and(|i| i.abs() > 0.1) {
        out.push(format!(
            "The {axis} I term holds {:+.2} of output in steady flight: that's a constant imbalance (centre of \
             gravity, a weak motor, or a twisted motor for yaw), not something to tune away.",
            terms.i_mean.unwrap()
        ));
    }
    if g.smax == Some(0.0) && axis != "yaw" {
        out.push(format!("The {axis} slew limiter (ATC_RAT_{}_SMAX) is off.", ax.param));
    }
}

/// Bounded gain changes from a clean step response.
fn suggest(ax: &AxisDef, g: &Gains, s: &StepResponse, osc: Option<&Oscillation>, out: &mut Vec<Suggestion>) {
    let mut push = |field: &str, current: Option<f64>, factor: f64, reason: String| {
        if let Some(c) = current.filter(|c| *c > 0.0) {
            let value = (c * factor * 10000.0).round() / 10000.0;
            out.push(Suggestion { name: format!("ATC_RAT_{}_{field}", ax.param), current: Some(c), value, reason });
        }
    };
    let axis = ax.name;
    if s.overshoot_pct > OVERSHOOT_HIGH_PCT {
        push("P", g.p, 1.0 - STEP_FRACTION, format!("{axis} overshoots {:.0}%: less P damps it", s.overshoot_pct));
        if osc.is_some_and(|o| o.in_actual && o.freq_hz > 15.0) {
            push("D", g.d, 1.0 - STEP_FRACTION, format!("{axis} rings at {:.0} Hz: less D", osc.unwrap().freq_hz));
        }
    } else if s.overshoot_pct < OVERSHOOT_LOW_PCT && s.rise_ms.is_some_and(|r| r > SLOW_RISE_MS) {
        push("P", g.p, 1.0 + STEP_FRACTION, format!("{axis} is slow with no overshoot: more P sharpens it"));
    }
    if s.steady_state < 0.85 {
        push("I", g.i, 1.0 + STEP_FRACTION, format!("{axis} settles short of the target: more I"));
    }
}

/// Above this, a maximum acceleration is in centidegrees/s², not degrees/s².
const MAX_ACCEL_DEG: f64 = 10_000.0;

/// Gains printed by AutoTune for axes it finished and the vehicle doesn't
/// have yet, e.g. "AutoTune: Roll Rate: P:0.042, I:0.042, D:0.0031" and
/// "AutoTune: Roll Angle P:19.298, Max Accel:114972".
///
/// The messages round the gains to a few digits, and the value saved can
/// differ by one unit of the last digit printed (0.0543 printed as 0.054,
/// 0.04575 as 0.045), so a gain within that of the log's parameter counts
/// as saved.
/// Also returns the axes AutoTune finished.
fn autotune_results(log: &Log) -> (Vec<Suggestion>, Vec<&'static str>) {
    // The number after `key`, and one unit of its last printed digit.
    let number_after = |s: &str, key: &str| -> Option<(f64, f64)> {
        let rest = &s[s.find(key)? + key.len()..];
        let text = rest.trim_start().split(|c: char| c == ',' || c.is_whitespace()).next()?;
        let decimals = text.split_once('.').map_or(0, |(_, frac)| frac.len());
        Some((text.parse().ok()?, 10f64.powi(-(decimals as i32))))
    };
    let mut found: Vec<Suggestion> = Vec::new();
    let mut finished: Vec<&'static str> = Vec::new();
    for msg in log.text_messages() {
        let Some(body) = msg.strip_prefix("AutoTune:").map(str::trim) else { continue };
        let Some(ax) = AXES.iter().find(|a| body.to_lowercase().starts_with(a.name)) else { continue };
        let mut add = |name: String, printed: Option<(f64, f64)>, what: &str| {
            let Some((value, unit)) = printed else { return };
            found.retain(|s| s.name != name);
            let current = log.param(&name);
            // A little slack for the 32-bit float the vehicle stores.
            let saved = current.is_some_and(|c| (c - value).abs() <= unit * (1.0 + 1e-6) + 1e-9);
            if !saved {
                found.push(Suggestion { current, name, value, reason: format!("{} {what} found by AutoTune", ax.name) });
            }
        };
        if body.contains("Rate:") {
            if !finished.contains(&ax.name) {
                finished.push(ax.name);
            }
            for (key, field) in [("P:", "P"), ("I:", "I"), ("D:", "D")] {
                add(format!("ATC_RAT_{}_{field}", ax.param), number_after(body, key), &format!("rate {field}"));
            }
        } else if body.contains("Angle P:") {
            add(format!("ATC_ANG_{}_P", ax.param), number_after(body, "Angle P:"), "angle P");
            // 4.7 renamed ATC_ACCEL_x_MAX (centidegrees/s²) to ATC_ACC_x_MAX (degrees/s²).
            let accel = number_after(body, "Max Accel:");
            let new_name = format!("ATC_ACC_{}_MAX", ax.accel);
            if log.param(&new_name).is_some() {
                // Values this large are centidegrees, as older AutoTune prints them.
                add(new_name, accel.map(|(v, unit)| if v >= MAX_ACCEL_DEG { (v / 100.0, unit / 100.0) } else { (v, unit) }), "max acceleration");
            } else {
                add(format!("ATC_ACCEL_{}_MAX", ax.accel), accel, "max acceleration");
            }
        }
    }
    (found, finished)
}

impl Suggestion {
    pub fn describe(&self) -> String {
        let now = self.current.map_or("unset".to_owned(), format_value);
        format!("{} {now} -> {}", self.name, format_value(self.value))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{LogWriter, V};

    /// Flight where the rate loop behaves as a second-order system with
    /// damping `zeta`, natural frequency `fn_hz` and a delay, driven by steps.
    fn second_order_log(zeta: f64, fn_hz: f64, delay_s: f64, autotune: bool) -> Log {
        second_order_log_with(zeta, fn_hz, delay_s, autotune, &[])
    }

    /// The same, with parameter changes written at the given times.
    fn second_order_log_with(zeta: f64, fn_hz: f64, delay_s: f64, autotune: bool, later: &[(f64, &str, f64)]) -> Log {
        let fs = 400.0;
        let dur = 60.0;
        let mut w = LogWriter::new();
        let ev = w.define("EV", "QB", "TimeUS,Id");
        let mode = w.define("MODE", "QMBB", "TimeUS,Mode,ModeNum,Rsn");
        let parm = w.define("PARM", "QNff", "TimeUS,Name,Value,Default");
        let rate = w.define("RATE", "Qfffffffff", "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut");
        let msg = w.define("MSG", "QZ", "TimeUS,Message");
        let us = |t: f64| V::I((t * 1e6) as i64);
        w.write(ev, &[us(1.0), V::I(10)]);
        let m = if autotune { 15 } else { 2 };
        w.write(mode, &[us(1.0), V::I(m), V::I(m), V::I(0)]);
        for (n, v) in [("ATC_RAT_RLL_P", 0.1), ("ATC_RAT_RLL_I", 0.1), ("ATC_RAT_RLL_D", 0.004), ("ATC_RAT_RLL_SMAX", 0.0)] {
            w.write(parm, &[us(0.5), V::S(n), V::F(v), V::F(v)]);
        }
        w.write(msg, &[us(50.0), V::S("AutoTune: Roll Rate: P:0.042, I:0.042, D:0.0031")]);
        w.write(msg, &[us(50.0), V::S("AutoTune: Roll Angle P:19.298, Max Accel:114972")]);

        let wn = 2.0 * PI * fn_hz;
        let dt = 1.0 / fs;
        let delay = (delay_s * fs) as usize;
        let (mut y, mut yd) = (0.0f64, 0.0f64);
        let mut hist = vec![0.0; delay + 1];
        let mut seed = 12345u32;
        let mut target = 0.0;
        let mut next_change = 1.0;
        for i in 0..(dur * fs) as usize {
            let t = 1.0 + i as f64 * dt;
            if t >= next_change {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let r = (seed >> 8) as f64 / (1u32 << 24) as f64;
                target = if target == 0.0 { (r - 0.5) * 300.0 } else { 0.0 };
                next_change = t + 0.3 + r * 0.5;
            }
            hist.rotate_right(1);
            hist[0] = target;
            let u = hist[delay];
            // Semi-implicit Euler, sub-stepped for accuracy.
            for _ in 0..10 {
                let h = dt / 10.0;
                let ydd = wn * wn * (u - y) - 2.0 * zeta * wn * yd;
                yd += ydd * h;
                y += yd * h;
            }
            w.write(rate, &[us(t), V::F(target), V::F(y), V::F(0.1), V::F(0.0), V::F(0.0), V::F(0.0), V::F(0.0), V::F(0.0), V::F(0.0)]);
        }
        w.write(ev, &[us(1.0 + dur), V::I(11)]);
        for (t, n, v) in later {
            w.write(parm, &[us(*t), V::S(n), V::F(*v), V::F(*v)]);
        }
        Log::from_bytes(&w.into_bytes())
    }

    #[test]
    fn gains_saved_after_the_flight_get_no_suggestions() {
        // AutoTune saves at the disarm (61 s): the flight flew P 0.1, and its
        // overshoot says nothing about the new 0.05.
        let log = second_order_log_with(0.4, 8.0, 0.01, false, &[(61.5, "ATC_RAT_RLL_P", 0.05), (61.5, "ATC_RAT_RLL_D", 0.002)]);
        let r = analyse(&log).unwrap();
        let roll = r.axes.iter().find(|a| a.axis == "roll").unwrap();
        let f32_of = |v: Option<f64>| v.map(|x| x as f32);
        assert_eq!((f32_of(roll.gains.p), f32_of(roll.gains.d)), (Some(0.1), Some(0.004)), "the gains flown");
        assert!(roll.findings.iter().any(|f| f.contains("overshoots")), "{:?}", roll.findings);
        assert!(r.suggestions.iter().all(|s| s.name != "ATC_RAT_RLL_P"), "{:?}", r.suggestions);
        assert!(r.notes.iter().any(|n| n.contains("after the flight")), "{:?}", r.notes);

        // The same values written again change nothing.
        let log = second_order_log_with(0.4, 8.0, 0.01, false, &[(61.5, "ATC_RAT_RLL_P", 0.1)]);
        assert!(analyse(&log).unwrap().suggestions.iter().any(|s| s.name == "ATC_RAT_RLL_P"));
    }

    #[test]
    fn recovers_overshoot_and_delay_of_a_known_system() {
        // zeta 0.4: theoretical overshoot exp(-pi*z/sqrt(1-z^2)) = 25.4%.
        let r = analyse(&second_order_log(0.4, 8.0, 0.01, false)).unwrap();
        let roll = r.axes.iter().find(|a| a.axis == "roll").unwrap();
        let s = roll.step.as_ref().expect("step response");
        assert!((s.overshoot_pct - 25.4).abs() < 6.0, "overshoot {}", s.overshoot_pct);
        assert!((s.steady_state - 1.0).abs() < 0.1, "steady {}", s.steady_state);
        // Delay to 50%: 10 ms transport plus about 30 ms of the 8 Hz system.
        let d = s.delay_ms.unwrap();
        assert!(d > 25.0 && d < 60.0, "delay {d}");
        assert!(!roll.includes_autotune);
        assert!(roll.findings.iter().any(|f| f.contains("overshoots")), "{:?}", roll.findings);
        let names: Vec<&str> = r.suggestions.iter().map(|s| s.name.as_str()).collect();
        assert!(names.contains(&"ATC_RAT_RLL_P"), "{names:?}");
        let p = r.suggestions.iter().find(|s| s.name == "ATC_RAT_RLL_P").unwrap();
        assert!((p.value - 0.085).abs() < 1e-9, "{p:?}");
        assert!(names.contains(&"ATC_RAT_RLL_SMAX"));
        // Pitch and yaw targets never move.
        assert!(r.axes.iter().find(|a| a.axis == "pitch").unwrap().step.is_none());
    }

    #[test]
    fn well_damped_system_gets_no_gain_change() {
        let r = analyse(&second_order_log(0.9, 8.0, 0.01, false)).unwrap();
        let s = r.axes[0].step.as_ref().unwrap();
        assert!(s.overshoot_pct < 5.0, "{}", s.overshoot_pct);
        assert!(!r.suggestions.iter().any(|s| s.name == "ATC_RAT_RLL_P"), "{:?}", r.suggestions);
    }

    #[test]
    fn autotune_flight_reports_but_does_not_suggest() {
        let r = analyse(&second_order_log(0.4, 8.0, 0.01, true)).unwrap();
        assert!(r.axes[0].includes_autotune);
        assert!(!r.suggestions.iter().any(|s| s.name == "ATC_RAT_RLL_P"));
        assert!(!r.notes.is_empty());
        let at: Vec<(String, f64)> = r.autotune.iter().map(|s| (s.name.clone(), s.value)).collect();
        assert_eq!(
            at,
            vec![
                ("ATC_RAT_RLL_P".to_owned(), 0.042),
                ("ATC_RAT_RLL_I".to_owned(), 0.042),
                ("ATC_RAT_RLL_D".to_owned(), 0.0031),
                ("ATC_ANG_RLL_P".to_owned(), 19.298),
                ("ATC_ACCEL_R_MAX".to_owned(), 114972.0),
            ]
        );
        assert_eq!(r.autotune[0].current, Some(0.1f32 as f64));
    }

    #[test]
    fn autotune_gains_saved_to_the_printed_precision_are_not_listed() {
        // A real 4.7 tune: the saved values against the rounded messages.
        let mut w = LogWriter::new();
        let parm = w.define("PARM", "QNff", "TimeUS,Name,Value,Default");
        let msg = w.define("MSG", "QZ", "TimeUS,Message");
        for (name, value) in [
            ("ATC_RAT_PIT_P", 0.045_748_636_f32),
            ("ATC_RAT_PIT_I", 0.045_748_636),
            ("ATC_RAT_PIT_D", 0.002_740_561_7),
            ("ATC_ANG_PIT_P", 1.967_559_8),
            ("ATC_ACC_P_MAX", 258.742_95),
            ("ATC_RAT_RLL_P", 0.054_299_87),
            ("ATC_RAT_RLL_I", 0.054_299_87),
            ("ATC_RAT_RLL_D", 0.001_5),
        ] {
            w.write(parm, &[V::I(1), V::S(name), V::F(f64::from(value)), V::F(0.0)]);
        }
        w.write(msg, &[V::I(2), V::S("AutoTune: Pitch Rate: P:0.045, I:0.045, D:0.0027")]);
        w.write(msg, &[V::I(3), V::S("AutoTune: Pitch Angle P:1.968, Max Accel:258.74")]);
        w.write(msg, &[V::I(4), V::S("AutoTune: Roll Rate: P:0.054, I:0.054, D:0.0032")]);
        let (found, finished) = autotune_results(&Log::from_bytes(&w.into_bytes()));
        assert_eq!(finished, ["pitch", "roll"]);
        // Only roll D wasn't saved (0.0015 on the vehicle).
        let names: Vec<&str> = found.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["ATC_RAT_RLL_D"]);
    }

    #[test]
    fn autotune_accel_uses_the_4_7_name_and_units() {
        let mut w = LogWriter::new();
        let parm = w.define("PARM", "QNff", "TimeUS,Name,Value,Default");
        let msg = w.define("MSG", "QZ", "TimeUS,Message");
        w.write(parm, &[V::I(1), V::S("ATC_ACC_R_MAX"), V::F(1100.0), V::F(1100.0)]);
        w.write(msg, &[V::I(2), V::S("AutoTune: Roll Angle P:19.298, Max Accel:114972")]);
        let log = Log::from_bytes(&w.into_bytes());
        let (found, _) = autotune_results(&log);
        let accel = found.iter().find(|s| s.name.starts_with("ATC_ACC")).unwrap();
        assert_eq!((accel.name.as_str(), accel.value, accel.current), ("ATC_ACC_R_MAX", 1149.72, Some(1100.0)));
    }
}
