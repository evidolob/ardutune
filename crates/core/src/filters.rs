//! Gyro filter simulator: ArduPilot's harmonic notch and low-pass filters
//! applied to recorded pre-filter gyro noise, to compare settings by the
//! noise they leave and the delay they add before flying them.
//!
//! Each gyro batch is short (about 0.5 s), so the notch centre is taken as
//! constant within a batch and filtering is applied as |H(f)|² to its
//! spectrum. The simulation of the current settings is checked against the
//! post-filter data the flight controller logged.

use std::f64::consts::PI;

use rustfft::num_complex::Complex;
use serde::Serialize;

use crate::log::Log;
use crate::noise::{self, Series, CONTROL_BAND_HZ, MIN_NOISE_HZ, NO_GYRO_DATA};
use crate::params::format_value;
use crate::stats;

/// Notches at or above this fraction of the sample rate are not applied.
const NYQUIST_LIMIT: f64 = 0.45;
/// Batches used per simulation, spread over the steady flight.
const MAX_SIM_BATCHES: usize = 40;
/// Frequency at which filter delay is reported unless asked otherwise.
pub const DEFAULT_LAG_REF_HZ: f64 = 20.0;
/// A recommendation may leave at most this much more noise than now.
const NOISE_TOLERANCE: f64 = 1.05;
/// ...and must cut at least this much delay to be worth changing.
const MIN_LAG_GAIN_DEG: f64 = 1.0;
const LPF_CANDIDATES_HZ: [f64; 10] = [20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 100.0, 120.0];
/// Notch bandwidths tried, as a fraction of INS_HNTCH_FREQ. Narrower notches
/// look good in steady flight but miss when RPM tracking lags in manoeuvres.
const BW_RATIOS: [f64; 3] = [0.5, 0.75, 1.0];
/// Validation band upper limit.
const VALIDATE_MAX_HZ: f64 = 500.0;

/// INS_HNTCH_OPTS bits.
const OPT_DOUBLE: u32 = 1 << 0;
const OPT_PER_MOTOR: u32 = 1 << 1;
const OPT_LOOP_RATE: u32 = 1 << 2;
const OPT_TRIPLE: u32 = 1 << 4;
const MODE_FIXED: u32 = 0;
const MODE_THROTTLE: u32 = 1;
const MODE_RPM: u32 = 2;
const MODE_ESC: u32 = 3;
const MODE_FFT: u32 = 4;
const MODE_RPM2: u32 = 5;

/// A normalised biquad: H(z) = (b0 + b1 z⁻¹ + b2 z⁻²) / (1 + a1 z⁻¹ + a2 z⁻²).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Biquad {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
}

impl Biquad {
    /// ArduPilot's LowPassFilter2p: a 2nd-order Butterworth biquad.
    pub fn low_pass(fs: f64, cutoff: f64) -> Option<Self> {
        if cutoff <= 0.0 || cutoff >= fs / 2.0 {
            return None;
        }
        let ohm = (PI * cutoff / fs).tan();
        let k = 2.0 * (PI / 4.0).cos();
        let c = 1.0 + k * ohm + ohm * ohm;
        let b0 = ohm * ohm / c;
        Some(Self { b0, b1: 2.0 * b0, b2: b0, a1: 2.0 * (ohm * ohm - 1.0) / c, a2: (1.0 - k * ohm + ohm * ohm) / c })
    }

    /// ArduPilot's NotchFilter::init_with_A_and_Q.
    pub fn notch(fs: f64, center: f64, a: f64, q: f64) -> Option<Self> {
        if center <= 2.0 || center >= 0.5 * fs || q <= 0.0 {
            return None;
        }
        let omega = 2.0 * PI * center / fs;
        let alpha = omega.sin() / (2.0 * q);
        let a0 = 1.0 + alpha;
        Some(Self {
            b0: (1.0 + alpha * a * a) / a0,
            b1: -2.0 * omega.cos() / a0,
            b2: (1.0 - alpha * a * a) / a0,
            a1: -2.0 * omega.cos() / a0,
            a2: (1.0 - alpha) / a0,
        })
    }

    pub fn response(&self, f: f64, fs: f64) -> Complex<f64> {
        let z1 = Complex::from_polar(1.0, -2.0 * PI * f / fs);
        let z2 = z1 * z1;
        (self.b0 + self.b1 * z1 + self.b2 * z2) / (1.0 + self.a1 * z1 + self.a2 * z2)
    }
}

/// ArduPilot's NotchFilter::calculate_A_and_Q: depth from attenuation, Q
/// from the bandwidth at INS_HNTCH_FREQ. Q is then shared by every centre
/// and harmonic, so the bandwidth scales with frequency.
pub fn notch_a_q(center: f64, bandwidth: f64, attenuation_db: f64) -> (f64, f64) {
    let a = 10f64.powf(-attenuation_db / 40.0);
    if center <= 0.5 * bandwidth {
        return (a, 0.0);
    }
    let octaves = (center / (center - bandwidth / 2.0)).log2() * 2.0;
    let p = 2f64.powf(octaves);
    (a, p.sqrt() / (p - 1.0))
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NotchSettings {
    /// Parameter prefix: INS_HNTCH or INS_HNTC2.
    pub prefix: &'static str,
    pub mode: u32,
    pub freq_hz: f64,
    pub bandwidth_hz: f64,
    pub attenuation_db: f64,
    /// HMNCS bitmask: bit 0 = fundamental.
    pub harmonics: u32,
    pub options: u32,
    /// FM_RAT: the notch never goes below FREQ × this.
    pub min_ratio: f64,
    pub reference: f64,
}

impl NotchSettings {
    fn harmonic_list(&self) -> Vec<u32> {
        (0..16).filter(|b| self.harmonics & (1 << b) != 0).map(|b| b + 1).collect()
    }

    fn describe(&self) -> String {
        let mode = match self.mode {
            MODE_FIXED => "fixed",
            MODE_THROTTLE => "throttle",
            MODE_RPM | MODE_RPM2 => "RPM sensor",
            MODE_ESC if self.options & OPT_PER_MOTOR != 0 => "ESC telemetry, per motor",
            MODE_ESC => "ESC telemetry",
            MODE_FFT => "in-flight FFT",
            _ => "other",
        };
        let h: Vec<String> = self.harmonic_list().iter().map(u32::to_string).collect();
        format!(
            "notch {mode}, harmonics {}, bandwidth {} Hz at {} Hz",
            h.join("+"),
            format_value(self.bandwidth_hz),
            format_value(self.freq_hz)
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FilterSettings {
    pub gyro_lpf_hz: f64,
    pub notches: Vec<NotchSettings>,
}

impl FilterSettings {
    pub fn from_log(log: &Log) -> Self {
        let mut notches = Vec::new();
        for prefix in ["INS_HNTCH", "INS_HNTC2"] {
            let p = |s: &str, default: f64| log.param(&format!("{prefix}_{s}")).unwrap_or(default);
            if p("ENABLE", 0.0) == 0.0 {
                continue;
            }
            notches.push(NotchSettings {
                prefix,
                mode: p("MODE", 0.0) as u32,
                freq_hz: p("FREQ", 80.0),
                bandwidth_hz: p("BW", 40.0),
                attenuation_db: p("ATT", 40.0),
                harmonics: p("HMNCS", 1.0) as u32,
                options: p("OPTS", 0.0) as u32,
                min_ratio: p("FM_RAT", 1.0),
                reference: p("REF", 0.0),
            });
        }
        Self { gyro_lpf_hz: log.param("INS_GYRO_FILTER").unwrap_or(20.0), notches }
    }

    /// The parameters these settings correspond to.
    pub fn params(&self) -> Vec<(String, f64)> {
        let mut out = vec![("INS_GYRO_FILTER".to_owned(), self.gyro_lpf_hz)];
        for n in &self.notches {
            let p = |s: &str| format!("{}_{s}", n.prefix);
            out.extend([
                (p("ENABLE"), 1.0),
                (p("MODE"), n.mode as f64),
                (p("FREQ"), n.freq_hz),
                (p("BW"), n.bandwidth_hz),
                (p("ATT"), n.attenuation_db),
                (p("HMNCS"), n.harmonics as f64),
                (p("OPTS"), n.options as f64),
                (p("FM_RAT"), n.min_ratio),
                (p("REF"), n.reference),
            ]);
        }
        out
    }

    pub fn describe(&self) -> String {
        let mut parts = vec![format!("low-pass {} Hz", format_value(self.gyro_lpf_hz))];
        parts.extend(self.notches.iter().map(NotchSettings::describe));
        if self.notches.is_empty() {
            parts.push("no notch".to_owned());
        }
        parts.join("; ")
    }
}

/// What the notch can see during one batch.
#[derive(Debug, Clone, Default)]
struct Context {
    motors_hz: Vec<f64>,
    rpm_sensor_hz: Option<f64>,
    throttle: Option<f64>,
}

/// Notch centre frequencies (before harmonics) for the conditions in `ctx`,
/// following how ArduPilot feeds each mode (AP_Vehicle::update_dynamic_notch):
/// REF = 0 turns tracking off, RPM and ESC frequencies are scaled by REF,
/// per-motor (multi-source) notches are not.
fn base_centers(n: &NotchSettings, ctx: &Context) -> Vec<f64> {
    if n.mode == MODE_FIXED || n.reference <= 0.0 {
        return vec![n.freq_hz];
    }
    let floor = n.freq_hz * n.min_ratio.max(0.0);
    let average = stats::mean(&ctx.motors_hz);
    let centers = match n.mode {
        MODE_THROTTLE => vec![match ctx.throttle {
            Some(t) => n.freq_hz * (t.max(0.0) / n.reference).sqrt(),
            None => n.freq_hz,
        }],
        MODE_RPM | MODE_RPM2 => vec![ctx.rpm_sensor_hz.or(average).map_or(n.freq_hz, |f| f * n.reference)],
        MODE_ESC if n.options & OPT_PER_MOTOR != 0 && !ctx.motors_hz.is_empty() => ctx.motors_hz.clone(),
        _ => vec![average.map_or(n.freq_hz, |f| f * n.reference)],
    };
    centers.into_iter().map(|c| c.max(floor)).collect()
}

fn notch_filters(n: &NotchSettings, ctx: &Context, fs: f64) -> Vec<Biquad> {
    let (a, q) = notch_a_q(n.freq_hz, n.bandwidth_hz, n.attenuation_db);
    let harmonics = n.harmonic_list();
    base_centers(n, ctx)
        .into_iter()
        .flat_map(|c| harmonics.iter().map(move |k| c * *k as f64))
        .filter(|f| *f < NYQUIST_LIMIT * fs)
        .filter_map(|f| Biquad::notch(fs, f, a, q))
        .collect()
}

/// Recorded pre-filter noise, ready to filter.
struct Prepared {
    fs: f64,
    freq_hz: Vec<f64>,
    /// Per batch: spectrum (all axes summed, linear) and notch inputs.
    batches: Vec<(Vec<f64>, Context)>,
    hover: Context,
    /// Mean pre-filter spectrum.
    pre: Vec<f64>,
}

impl Prepared {
    fn df(&self) -> f64 {
        self.freq_hz.get(1).copied().unwrap_or(1.0)
    }

    /// Mean spectrum after the notches only.
    fn notched(&self, notches: &[NotchSettings]) -> Vec<f64> {
        let mut out = vec![0.0; self.freq_hz.len()];
        for (psd, ctx) in &self.batches {
            let filters: Vec<Biquad> = notches.iter().flat_map(|n| notch_filters(n, ctx, self.fs)).collect();
            for (k, (o, p)) in out.iter_mut().zip(psd).enumerate() {
                let f = self.freq_hz[k];
                let gain: f64 = filters.iter().map(|b| b.response(f, self.fs).norm_sqr()).product();
                *o += p * gain;
            }
        }
        out.iter_mut().for_each(|v| *v /= self.batches.len().max(1) as f64);
        out
    }

    fn low_pass_gain(&self, cutoff: f64) -> Vec<f64> {
        let Some(lp) = Biquad::low_pass(self.fs, cutoff) else { return vec![1.0; self.freq_hz.len()] };
        self.freq_hz.iter().map(|f| lp.response(*f, self.fs).norm_sqr()).collect()
    }

    /// Phase lag (degrees) of the whole filter chain at `f`, at hover.
    fn lag_deg(&self, s: &FilterSettings, f: f64) -> f64 {
        let mut filters: Vec<Biquad> = s.notches.iter().flat_map(|n| notch_filters(n, &self.hover, self.fs)).collect();
        filters.extend(Biquad::low_pass(self.fs, s.gyro_lpf_hz));
        // Summing each biquad's phase avoids wrapping a cascade's total.
        -filters.iter().map(|b| b.response(f, self.fs).arg()).sum::<f64>().to_degrees()
    }

    fn evaluate(&self, s: &FilterSettings, notched: &[f64], lag_ref: f64) -> Evaluation {
        let lp = self.low_pass_gain(s.gyro_lpf_hz);
        let post: Vec<f64> = notched.iter().zip(&lp).map(|(n, g)| n * g).collect();
        let df = self.df();
        let noise_rms =
            self.freq_hz.iter().zip(&post).filter(|(f, _)| **f >= CONTROL_BAND_HZ).map(|(_, p)| p * df).sum::<f64>().sqrt();
        let lag_deg = self.lag_deg(s, lag_ref);
        Evaluation {
            settings: s.clone(),
            noise_rms,
            lag_deg,
            delay_ms: lag_deg / 360.0 / lag_ref * 1000.0,
            spectrum_db: post.iter().map(|v| noise::db(*v)).collect(),
        }
    }
}

/// Values of `field` from rows where `key` equals `value`.
fn series_where(log: &Log, table: &str, field: &str, key: &str, value: f64) -> Option<Series> {
    let tb = log.table(table)?;
    let (t, v) = (tb.time_s()?, tb.num(field)?);
    let Some(k) = tb.num(key) else { return Some(Series { t, v: v.to_vec() }) };
    let (t, v) = t.iter().zip(v).zip(k).filter(|(_, k)| **k == value).map(|((t, v), _)| (*t, *v)).unzip();
    Some(Series { t, v })
}

fn mean_in(s: Option<&Series>, from: f64, to: f64) -> Option<f64> {
    s.and_then(|s| stats::mean(s.window(from, to)))
}

#[derive(Debug, Clone, Serialize)]
pub struct Evaluation {
    pub settings: FilterSettings,
    /// RMS gyro noise left above the control band, deg/s.
    pub noise_rms: f64,
    /// Phase lag the filters add at the reference frequency.
    pub lag_deg: f64,
    pub delay_ms: f64,
    /// Simulated after-filter spectrum, all axes, dB re (deg/s)²/Hz.
    pub spectrum_db: Vec<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Validation {
    /// RMS difference between simulated and logged post-filter spectra.
    pub rms_error_db: f64,
    pub band_hz: (f64, f64),
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Change {
    pub name: String,
    pub current: Option<f64>,
    pub value: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct FilterReport {
    pub sample_rate_hz: f64,
    pub lag_ref_hz: f64,
    pub freq_hz: Vec<f64>,
    pub pre_db: Vec<f64>,
    pub logged_post_db: Option<Vec<f64>>,
    pub validation: Option<Validation>,
    pub current: Evaluation,
    /// Same noise or less, less delay.
    pub recommended: Option<Evaluation>,
    /// Much less noise for about the same delay.
    pub quieter: Option<Evaluation>,
    /// Best trade-offs found, by increasing delay.
    pub frontier: Vec<Evaluation>,
    /// Parameter changes for `recommended`.
    pub changes: Vec<Change>,
    /// Parameter changes for `quieter`.
    pub quieter_changes: Vec<Change>,
    pub notes: Vec<String>,
}

pub fn simulate(log: &Log, lag_ref_hz: f64) -> Result<FilterReport, String> {
    let (_, batches) = noise::gyro_batches(log).ok_or(NO_GYRO_DATA.to_owned())?;
    let motors = noise::MotorSpeed::new(log);
    let sel = noise::select(log, &batches, &motors)?;
    if sel.pre.is_empty() {
        return Err("this log only has post-filter gyro data; simulating filters needs the noise before them \
                    (INS_LOG_BAT_OPT=4 or INS_RAW_LOG_OPT=9)"
            .to_owned());
    }
    let mut notes = sel.notes.clone();
    let step = sel.pre.len().div_ceil(MAX_SIM_BATCHES).max(1);
    let chosen: Vec<&noise::GyroBatch> = sel.pre.iter().copied().step_by(step).collect();
    let fft = noise::fft_for(&chosen).ok_or("gyro batches are too short to analyse".to_owned())?;
    let rpm_sensor = series_where(log, "RPM", "RPM", "I", 0.0);
    let throttle = noise::Series::get(log, "CTUN", "ThO");

    let context = |windows: &[(f64, f64)]| Context {
        motors_hz: motors.per_motor(windows).into_iter().map(|(_, f)| f).collect(),
        rpm_sensor_hz: stats::median(
            &windows.iter().filter_map(|(a, b)| mean_in(rpm_sensor.as_ref(), *a, *b)).collect::<Vec<_>>(),
        )
        .filter(|v| *v > 0.0)
        .map(|rpm| rpm / 60.0),
        throttle: stats::median(&windows.iter().filter_map(|(a, b)| mean_in(throttle.as_ref(), *a, *b)).collect::<Vec<_>>()),
    };
    let prepared_batches: Vec<(Vec<f64>, Context)> = chosen
        .iter()
        .map(|b| {
            let mut psd = vec![0.0; fft.n / 2 + 1];
            for x in &b.xyz {
                psd.iter_mut().zip(fft.psd(x)).for_each(|(a, p)| *a += p);
            }
            (psd, context(&[(b.start_s, b.end_s())]))
        })
        .collect();
    let all_windows: Vec<(f64, f64)> = sel.pre.iter().map(|b| (b.start_s, b.end_s())).collect();
    let mut pre = vec![0.0; fft.n / 2 + 1];
    for (psd, _) in &prepared_batches {
        pre.iter_mut().zip(psd).for_each(|(a, p)| *a += p / prepared_batches.len() as f64);
    }
    let prep = Prepared { fs: fft.rate, freq_hz: fft.freqs(), batches: prepared_batches, hover: context(&all_windows), pre };

    let current_settings = FilterSettings::from_log(log);
    if current_settings.notches.iter().any(|n| n.options & (OPT_DOUBLE | OPT_TRIPLE) != 0) {
        notes.push("Double and triple notches are simulated as single notches, so their effect is underestimated.".to_owned());
    }
    let current = prep.evaluate(&current_settings, &prep.notched(&current_settings.notches), lag_ref_hz);

    let logged_post = noise::welch(&sel.post);
    // Logged integers carry rounding noise (step²/12 per axis, spread over
    // half the sample rate), which hides anything quieter in the log.
    let quantum = stats::median(&sel.post.iter().filter_map(|b| b.quantum).collect::<Vec<_>>());
    let rounding = quantum.map_or(0.0, |q| 3.0 * q * q / 12.0 / (prep.fs / 2.0));
    let validation = logged_post.as_ref().map(|post| {
        let total = post.total_db();
        let hi = VALIDATE_MAX_HZ.min(NYQUIST_LIMIT * prep.fs);
        let diffs: Vec<f64> = prep
            .freq_hz
            .iter()
            .enumerate()
            .filter(|(_, f)| **f >= MIN_NOISE_HZ && **f <= hi)
            .map(|(k, f)| {
                let simulated = 10f64.powf(current.spectrum_db[k] / 10.0) + rounding;
                noise::db(simulated) - total[post.bin(*f)]
            })
            .collect();
        let rms = (diffs.iter().map(|d| d * d).sum::<f64>() / diffs.len().max(1) as f64).sqrt();
        Validation { rms_error_db: rms, band_hz: (MIN_NOISE_HZ, hi) }
    });
    if validation.as_ref().is_some_and(|v| v.rms_error_db > 6.0) {
        notes.push(
            "The simulation of your current settings doesn't match the logged filtered data well, so treat \
             the recommendation with care."
                .to_owned(),
        );
    }

    let evaluations = search(&prep, &current_settings, motors.known(), lag_ref_hz, &mut notes);
    let frontier = pareto(&evaluations);
    let recommended = evaluations
        .iter()
        .filter(|e| e.noise_rms <= current.noise_rms * NOISE_TOLERANCE)
        .min_by(|a, b| a.lag_deg.total_cmp(&b.lag_deg))
        .filter(|e| e.lag_deg < current.lag_deg - MIN_LAG_GAIN_DEG)
        .cloned();
    let quieter = evaluations
        .iter()
        .filter(|e| e.lag_deg <= current.lag_deg + 0.5)
        .min_by(|a, b| a.noise_rms.total_cmp(&b.noise_rms))
        .filter(|e| e.noise_rms < 0.8 * current.noise_rms)
        .cloned();
    let changes = recommended.as_ref().map_or_else(Vec::new, |r| diff(&current_settings, &r.settings, log));
    let quieter_changes = quieter.as_ref().map_or_else(Vec::new, |q| diff(&current_settings, &q.settings, log));

    Ok(FilterReport {
        sample_rate_hz: prep.fs,
        lag_ref_hz,
        pre_db: prep.pre.iter().map(|v| noise::db(*v)).collect(),
        freq_hz: prep.freq_hz.clone(),
        logged_post_db: logged_post.map(|p| prep.freq_hz.iter().map(|f| p.total_db()[p.bin(*f)]).collect()),
        validation,
        current,
        recommended,
        quieter,
        frontier,
        changes,
        quieter_changes,
        notes,
    })
}

/// Candidate settings around the current ones: the primary notch's mode,
/// harmonics and bandwidth, and the low-pass cutoff. A second notch is kept.
fn search(prep: &Prepared, current: &FilterSettings, motors_known: bool, lag_ref: f64, notes: &mut Vec<String>) -> Vec<Evaluation> {
    let base = match current.notches.first() {
        Some(n) => Some(n.clone()),
        None if motors_known => {
            let slowest = prep.hover.motors_hz.iter().copied().reduce(f64::min).unwrap_or(50.0);
            let freq = (0.8 * slowest).round();
            notes.push("No harmonic notch is enabled; the search includes an ESC-telemetry notch.".to_owned());
            Some(NotchSettings {
                prefix: "INS_HNTCH",
                mode: MODE_ESC,
                freq_hz: freq,
                bandwidth_hz: (freq / 2.0).round(),
                attenuation_db: 40.0,
                harmonics: 0b11,
                options: OPT_PER_MOTOR | OPT_LOOP_RATE,
                min_ratio: 1.0,
                reference: 1.0,
            })
        }
        None => {
            notes.push(
                "No harmonic notch and no motor RPM data: only the low-pass filter is searched. Bidirectional \
                 DShot (ESC RPM telemetry) would allow a notch that tracks the motors."
                    .to_owned(),
            );
            None
        }
    };
    for n in &current.notches {
        if n.mode != MODE_FIXED && n.reference <= 0.0 {
            notes.push(format!(
                "{}_REF is 0, so the notch doesn't track: it stays at {} Hz. Tracking modes need REF = 1 (RPM, ESC \
                 telemetry) or the hover throttle (throttle mode); the suggested settings include it.",
                n.prefix,
                format_value(n.freq_hz)
            ));
        }
    }
    let rest: Vec<NotchSettings> = current.notches.iter().skip(1).cloned().collect();
    let mut notch_sets: Vec<Vec<NotchSettings>> = vec![current.notches.clone()];
    if let Some(base) = base {
        let mut modes = vec![(base.mode, base.options)];
        let per_motor = (MODE_ESC, (base.options | OPT_PER_MOTOR | OPT_LOOP_RATE) & !(OPT_DOUBLE | OPT_TRIPLE));
        if motors_known && prep.hover.motors_hz.len() > 1 && (base.mode, base.options) != per_motor {
            modes.push(per_motor);
        }
        let mut bandwidths: Vec<f64> = BW_RATIOS.iter().map(|r| (r * base.freq_hz).round()).collect();
        bandwidths.push(base.bandwidth_hz);
        bandwidths.sort_by(f64::total_cmp);
        bandwidths.dedup();
        for (mode, options) in modes {
            for harmonics in (1u32..16).filter(|h| h & 1 == 1) {
                for &bw in &bandwidths {
                    let reference = tracking_reference(mode, base.reference);
                    let mut set = vec![NotchSettings { mode, options, harmonics, bandwidth_hz: bw, reference, ..base.clone() }];
                    set.extend(rest.iter().cloned());
                    notch_sets.push(set);
                }
            }
        }
    }
    let mut cutoffs: Vec<f64> = LPF_CANDIDATES_HZ.iter().copied().filter(|c| *c < NYQUIST_LIMIT * prep.fs).collect();
    cutoffs.push(current.gyro_lpf_hz);
    cutoffs.sort_by(f64::total_cmp);
    cutoffs.dedup();

    let mut out = Vec::new();
    for set in notch_sets {
        let notched = prep.notched(&set);
        for &c in &cutoffs {
            let s = FilterSettings { gyro_lpf_hz: c, notches: set.clone() };
            out.push(prep.evaluate(&s, &notched, lag_ref));
        }
    }
    out
}

/// REF for a candidate notch: RPM and ESC modes only track with a positive
/// REF, and 1 uses the measured frequency as it is. Other modes keep theirs.
fn tracking_reference(mode: u32, current: f64) -> f64 {
    match mode {
        MODE_RPM | MODE_RPM2 | MODE_ESC if current <= 0.0 => 1.0,
        _ => current,
    }
}

/// Settings no other beats on both noise and delay, by increasing delay.
fn pareto(evals: &[Evaluation]) -> Vec<Evaluation> {
    let mut sorted: Vec<&Evaluation> = evals.iter().collect();
    sorted.sort_by(|a, b| a.lag_deg.total_cmp(&b.lag_deg).then(a.noise_rms.total_cmp(&b.noise_rms)));
    let mut out: Vec<Evaluation> = Vec::new();
    for e in sorted {
        if out.last().is_none_or(|l| e.noise_rms < l.noise_rms * 0.97) {
            out.push(e.clone());
        }
    }
    out
}

/// Parameter changes from `from` to `to`, against the log's values.
fn diff(from: &FilterSettings, to: &FilterSettings, log: &Log) -> Vec<Change> {
    let before: Vec<(String, f64)> = from.params();
    to.params()
        .into_iter()
        .filter(|(name, value)| before.iter().find(|(n, _)| n == name).is_none_or(|(_, v)| (v - value).abs() > 1e-6))
        .map(|(name, value)| Change { current: log.param(&name), name, value })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{sample_flight, Preset};

    const FS: f64 = 2000.0;

    #[test]
    fn low_pass_is_minus_3_db_at_cutoff_and_lags_90_degrees() {
        let lp = Biquad::low_pass(FS, 50.0).unwrap();
        let h = lp.response(50.0, FS);
        assert!((20.0 * h.norm().log10() + 3.01).abs() < 0.05, "{}", 20.0 * h.norm().log10());
        assert!((h.arg().to_degrees() + 90.0).abs() < 1.0, "{}", h.arg().to_degrees());
        assert!((lp.response(0.0, FS).norm() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn notch_depth_and_bandwidth_match_ardupilot() {
        // 40 dB notch, 40 Hz wide at 80 Hz.
        let (a, q) = notch_a_q(80.0, 40.0, 40.0);
        let n = Biquad::notch(FS, 80.0, a, q).unwrap();
        let db = |f: f64| 20.0 * n.response(f, FS).norm().log10();
        assert!((db(80.0) + 40.0).abs() < 0.1, "{}", db(80.0));
        // About -3 dB at the band edges, flat far away.
        assert!(db(60.0) > -6.0 && db(60.0) < -1.0, "{}", db(60.0));
        assert!(db(10.0).abs() < 0.2 && db(400.0).abs() < 0.2);
    }

    #[test]
    fn per_motor_notch_follows_each_motor() {
        let n = NotchSettings {
            prefix: "INS_HNTCH",
            mode: MODE_ESC,
            freq_hz: 40.0,
            bandwidth_hz: 20.0,
            attenuation_db: 40.0,
            harmonics: 0b11,
            options: OPT_PER_MOTOR,
            min_ratio: 1.0,
            reference: 1.0,
        };
        let ctx = Context { motors_hz: vec![50.0, 60.0, 35.0], ..Default::default() };
        // 35 Hz is floored at FREQ × FM_RAT = 40.
        assert_eq!(base_centers(&n, &ctx), vec![50.0, 60.0, 40.0]);
        assert_eq!(notch_filters(&n, &ctx, FS).len(), 6);
        let avg = NotchSettings { options: 0, ..n.clone() };
        assert_eq!(base_centers(&avg, &ctx), vec![48.333333333333336]);
        // REF scales the average; REF = 0 turns tracking off altogether.
        assert_eq!(base_centers(&NotchSettings { reference: 2.0, ..avg.clone() }, &ctx), vec![96.66666666666667]);
        assert_eq!(base_centers(&NotchSettings { reference: 0.0, ..n }, &ctx), vec![40.0]);
    }

    #[test]
    fn sample_flight_recommends_a_notch_and_less_delay() {
        let log = Log::from_bytes(&sample_flight(Preset::Ready));
        let r = simulate(&log, DEFAULT_LAG_REF_HZ).unwrap();
        // The sample flies a 20 Hz low-pass and no notch.
        assert!(r.current.settings.notches.is_empty());
        let rec = r.recommended.expect("a better setting");
        assert!(!rec.settings.notches.is_empty(), "{}", rec.settings.describe());
        assert!(rec.settings.gyro_lpf_hz > 20.0);
        assert!(rec.lag_deg < r.current.lag_deg && rec.noise_rms <= r.current.noise_rms * NOISE_TOLERANCE);
        assert!(r.changes.iter().any(|c| c.name == "INS_HNTCH_ENABLE" && c.value == 1.0));
        assert!(r.changes.iter().any(|c| c.name == "INS_GYRO_FILTER"));
        // Frontier is ordered by delay with falling noise.
        assert!(r.frontier.windows(2).all(|w| w[0].lag_deg <= w[1].lag_deg && w[0].noise_rms > w[1].noise_rms));
    }
}
