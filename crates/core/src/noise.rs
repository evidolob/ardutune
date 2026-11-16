//! Gyro noise analysis: averaged spectra before and after filtering, a
//! spectrogram over the flight, and noise peaks classified by whether they
//! follow motor speed, checked against the harmonic notch configuration.

use std::collections::BTreeMap;
use std::f64::consts::PI;

use rustfft::num_complex::Complex;
use rustfft::FftPlanner;
use serde::Serialize;

use crate::log::{in_any, Log};
use crate::stats;

const RAD_TO_DEG: f64 = 180.0 / PI;
pub(crate) const NO_GYRO_DATA: &str = "this log has no high-rate gyro data; run `ardutune check` for the logging parameters to set";
const ISB_GYRO: f64 = 1.0;
/// INS_LOG_BAT_OPT bits.
const BAT_OPT_POST_ONLY_BIT: u32 = 1;
const BAT_OPT_PRE_POST_BIT: u32 = 2;
/// INS_RAW_LOG_OPT bit for logging both pre- and post-filter gyro data.
const RAW_OPT_PRE_POST_BIT: u32 = 3;
/// Raw gyro streams are cut into segments of this many samples.
const RAW_SEGMENT: usize = 1024;
const MIN_FFT: usize = 256;
/// Below this, spectra show flight manoeuvres rather than noise.
pub const MIN_NOISE_HZ: f64 = 30.0;
const PEAK_MIN_PROMINENCE_DB: f64 = 6.0;
const PEAK_MERGE_HZ: f64 = 8.0;
const PEAK_MERGE_FRACTION: f64 = 0.1;
const FLOOR_MIN_HALF_HZ: f64 = 20.0;
const FLOOR_HALF_FRACTION: f64 = 0.4;
const FLOOR_PERCENTILE: f64 = 20.0;
const MAX_PEAKS: usize = 8;
const MAX_HARMONIC: u32 = 6;
/// Harmonic tolerance never exceeds this fraction of the motor frequency.
const HARMONIC_SPACING_FRACTION: f64 = 0.2;
/// Peaks below this overlap the attitude control loop.
pub const CONTROL_BAND_HZ: f64 = 45.0;
/// A peak within this fraction of k × motor frequency matches harmonic k.
const HARMONIC_TOLERANCE: f64 = 0.08;
/// Hover batches must have rate demands below this.
const CALM_RATE_DEG_S: f64 = 30.0;
const MIN_HOVER_BATCHES: usize = 4;
/// Motor speed must vary at least this much to tell tracking from fixed peaks.
const MIN_RPM_SPREAD: f64 = 1.15;
const TRACK_DECISION_DB: f64 = 2.0;
const SPECTROGRAM_MAX_BINS: usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    BatchSampler,
    RawImu,
}

/// A contiguous run of gyro samples, in deg/s.
#[derive(Debug, Clone)]
pub struct GyroBatch {
    pub start_s: f64,
    pub instance: u32,
    pub post_filter: bool,
    pub rate_hz: f64,
    /// Size of one stored step (deg/s) when samples are stored as integers.
    pub quantum: Option<f64>,
    pub xyz: [Vec<f64>; 3],
}

impl GyroBatch {
    pub fn end_s(&self) -> f64 {
        self.start_s + self.xyz[0].len() as f64 / self.rate_hz
    }
}

/// Gyro batches from the batch sampler, or else from raw IMU logging.
pub fn gyro_batches(log: &Log) -> Option<(Source, Vec<GyroBatch>)> {
    if let Some(b) = batch_sampler(log).filter(|b| !b.is_empty()) {
        return Some((Source::BatchSampler, b));
    }
    raw_gyro(log).filter(|b| !b.is_empty()).map(|b| (Source::RawImu, b))
}

fn batch_sampler(log: &Log) -> Option<Vec<GyroBatch>> {
    let h = log.table("ISBH")?;
    let d = log.table("ISBD")?;
    let (hn, kind, inst, mul) = (h.num("N")?, h.num("type")?, h.num("instance")?, h.num("mul")?);
    let (count, rate, start) = (h.num("smp_cnt")?, h.num("smp_rate")?, h.num("SampleUS")?);
    let (dn, seq) = (d.num("N")?, d.num("seqno")?);
    let (x, y, z) = (d.arrays("x")?, d.arrays("y")?, d.arrays("z")?);

    // Data rows per batch number, split into runs where seqno restarts:
    // the 16-bit batch number wraps on long flights.
    let mut runs: BTreeMap<u32, Vec<Vec<usize>>> = BTreeMap::new();
    for (i, (n, s)) in dn.iter().zip(seq).enumerate() {
        let list = runs.entry(*n as u32).or_default();
        let restart = list.last().and_then(|r| r.last()).is_none_or(|&last| *s <= seq[last]);
        if restart {
            list.push(Vec::new());
        }
        list.last_mut().unwrap().push(i);
    }
    let mut next_run: BTreeMap<u32, usize> = BTreeMap::new();

    let opt = log.param("INS_LOG_BAT_OPT").unwrap_or(0.0) as u32;
    let sampled = (log.param("INS_LOG_BAT_MASK").unwrap_or(1.0) as u32).count_ones().max(1);
    let mut out = Vec::new();
    for i in 0..h.len() {
        let n = hn[i] as u32;
        let k = next_run.entry(n).or_default();
        let Some(rows) = runs.get(&n).and_then(|r| r.get(*k)) else { continue };
        *k += 1;
        if kind[i] != ISB_GYRO || mul[i] <= 0.0 || rate[i] <= 0.0 {
            continue;
        }
        let mut rows = rows.clone();
        rows.sort_by(|a, b| seq[*a].total_cmp(&seq[*b]));
        let contiguous = rows.iter().enumerate().all(|(j, r)| seq[*r] as usize == j);
        let want = count[i] as usize;
        if !contiguous || rows.len() * 32 < want {
            continue;
        }
        let scale = RAD_TO_DEG / mul[i];
        let axis = |a: &[[i16; 32]]| -> Vec<f64> { rows.iter().flat_map(|r| a[*r].iter().map(|v| *v as f64 * scale)).take(want).collect() };
        let post_filter = if opt & (1 << BAT_OPT_PRE_POST_BIT) != 0 {
            // Post-filter batches are logged as extra instances after the sampled IMUs.
            inst[i] as u32 >= sampled
        } else {
            opt & (1 << BAT_OPT_POST_ONLY_BIT) != 0
        };
        out.push(GyroBatch {
            start_s: start[i] * 1e-6,
            instance: inst[i] as u32,
            post_filter,
            rate_hz: rate[i],
            quantum: Some(scale),
            xyz: [axis(x), axis(y), axis(z)],
        });
    }
    Some(out)
}

fn raw_gyro(log: &Log) -> Option<Vec<GyroBatch>> {
    let g = log.table("GYR")?;
    let t_us = g.num_any(&["SampleUS", "TimeUS"])?;
    let inst = g.num("I")?;
    let (gx, gy, gz) = (g.num("GyrX")?, g.num("GyrY")?, g.num("GyrZ")?);
    let pre_post = (log.param("INS_RAW_LOG_OPT").unwrap_or(0.0) as u32) & (1 << RAW_OPT_PRE_POST_BIT) != 0;
    let imus = log.table("IMU").and_then(|t| t.num("I")).map_or(1, |i| {
        let mut v: Vec<i64> = i.iter().map(|x| *x as i64).collect();
        v.sort_unstable();
        v.dedup();
        v.len()
    }) as u32;

    let mut by_inst: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (i, k) in inst.iter().enumerate() {
        by_inst.entry(*k as u32).or_default().push(i);
    }
    let mut out = Vec::new();
    for (k, rows) in by_inst {
        let dt: Vec<f64> = rows.windows(2).map(|w| t_us[w[1]] - t_us[w[0]]).filter(|d| *d > 0.0).collect();
        let Some(step) = stats::median(&dt) else { continue };
        let rate = 1e6 / step;
        let mut seg: Vec<usize> = Vec::new();
        for &r in &rows {
            if seg.last().is_some_and(|&p| t_us[r] - t_us[p] > 3.0 * step) {
                seg.clear();
            }
            seg.push(r);
            if seg.len() == RAW_SEGMENT {
                let axis = |a: &[f64]| seg.iter().map(|i| a[*i] * RAD_TO_DEG).collect();
                out.push(GyroBatch {
                    start_s: t_us[seg[0]] * 1e-6,
                    instance: k,
                    post_filter: pre_post && k >= imus,
                    rate_hz: rate,
                    quantum: None,
                    xyz: [axis(gx), axis(gy), axis(gz)],
                });
                seg.clear();
            }
        }
    }
    Some(out)
}

/// Power spectral density per axis, in (deg/s)²/Hz.
#[derive(Debug, Clone, Serialize)]
pub struct Spectrum {
    pub freq_hz: Vec<f64>,
    pub psd: [Vec<f64>; 3],
    pub batches: usize,
}

impl Spectrum {
    pub(crate) fn df(&self) -> f64 {
        self.freq_hz.get(1).copied().unwrap_or(1.0)
    }

    /// Sum over axes, in dB.
    pub fn total_db(&self) -> Vec<f64> {
        (0..self.freq_hz.len()).map(|k| db(self.psd.iter().map(|a| a[k]).sum())).collect()
    }

    pub(crate) fn bin(&self, f: f64) -> usize {
        ((f / self.df()).round() as usize).min(self.freq_hz.len() - 1)
    }

    /// RMS noise between `from` Hz and Nyquist, all axes combined, in deg/s.
    pub fn rms_above(&self, from: f64) -> f64 {
        let df = self.df();
        let total: f64 = (self.bin(from)..self.freq_hz.len()).map(|k| self.psd.iter().map(|a| a[k]).sum::<f64>() * df).sum();
        total.sqrt()
    }
}

pub(crate) fn db(v: f64) -> f64 {
    10.0 * v.max(1e-12).log10()
}

pub(crate) struct Fft {
    pub(crate) n: usize,
    pub(crate) rate: f64,
    window: Vec<f64>,
    norm: f64,
    plan: std::sync::Arc<dyn rustfft::Fft<f64>>,
}

impl Fft {
    pub(crate) fn new(n: usize, rate: f64) -> Self {
        let window: Vec<f64> = (0..n).map(|i| 0.5 - 0.5 * (2.0 * PI * i as f64 / n as f64).cos()).collect();
        let norm = rate * window.iter().map(|w| w * w).sum::<f64>();
        Self { n, rate, window, norm, plan: FftPlanner::new().plan_fft_forward(n) }
    }

    pub(crate) fn freqs(&self) -> Vec<f64> {
        (0..=self.n / 2).map(|k| k as f64 * self.rate / self.n as f64).collect()
    }

    /// One-sided PSD of the first `n` samples, mean removed, Hann window.
    pub(crate) fn psd(&self, x: &[f64]) -> Vec<f64> {
        let x = &x[..self.n];
        let mean = x.iter().sum::<f64>() / self.n as f64;
        let mut buf: Vec<Complex<f64>> = x.iter().zip(&self.window).map(|(v, w)| Complex::new((v - mean) * w, 0.0)).collect();
        self.plan.process(&mut buf);
        (0..=self.n / 2)
            .map(|k| {
                let p = buf[k].norm_sqr() / self.norm;
                if k == 0 || k == self.n / 2 {
                    p
                } else {
                    2.0 * p
                }
            })
            .collect()
    }
}

/// Analysis length: the largest power of two that fits every batch.
pub(crate) fn fft_for(batches: &[&GyroBatch]) -> Option<Fft> {
    let shortest = batches.iter().map(|b| b.xyz[0].len()).min()?;
    if shortest < MIN_FFT {
        return None;
    }
    let n = 1 << (usize::BITS - 1 - shortest.leading_zeros());
    let rate = stats::median(&batches.iter().map(|b| b.rate_hz).collect::<Vec<_>>())?;
    Some(Fft::new(n, rate))
}

pub(crate) fn welch(batches: &[&GyroBatch]) -> Option<Spectrum> {
    let fft = fft_for(batches)?;
    let bins = fft.n / 2 + 1;
    let mut psd = [vec![0.0; bins], vec![0.0; bins], vec![0.0; bins]];
    for b in batches {
        for (acc, x) in psd.iter_mut().zip(&b.xyz) {
            for (a, p) in acc.iter_mut().zip(fft.psd(x)) {
                *a += p;
            }
        }
    }
    for acc in &mut psd {
        acc.iter_mut().for_each(|v| *v /= batches.len() as f64);
    }
    Some(Spectrum { freq_hz: fft.freqs(), psd, batches: batches.len() })
}

/// Pre-filter noise over time; rows are batches.
#[derive(Debug, Clone, Serialize)]
pub struct Spectrogram {
    pub time_s: Vec<f64>,
    pub freq_hz: Vec<f64>,
    /// dB re (deg/s)²/Hz, all axes summed; `power_db[row][bin]`.
    pub power_db: Vec<Vec<f64>>,
    /// Motor rotation frequency during each batch, when known.
    pub motor_hz: Vec<Option<f64>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PeakClass {
    /// k × motor rotation frequency. `tracks_rpm` is `None` when motor speed
    /// barely changed, so tracking couldn't be tested.
    MotorHarmonic {
        harmonic: u32,
        tracks_rpm: Option<bool>,
    },
    /// Stays put while motor speed changes: a frame or mount resonance.
    FixedFrequency,
    /// Below [`CONTROL_BAND_HZ`] and not a motor harmonic: real body motion
    /// such as a control-loop oscillation or prop wash, more than sensor noise.
    ControlBand,
    Unknown,
}

#[derive(Debug, Clone, Serialize)]
pub struct Peak {
    pub freq_hz: f64,
    pub level_db: f64,
    /// Height above the surrounding noise floor.
    pub prominence_db: f64,
    pub axis: &'static str,
    pub class: PeakClass,
    /// How much the filters removed at this frequency.
    pub filter_reduction_db: Option<f64>,
    /// Whether a configured harmonic notch targets this harmonic.
    pub notch_covered: Option<bool>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Notch {
    /// Parameter prefix, `INS_HNTCH` or `INS_HNTC2`.
    pub name: &'static str,
    pub mode: String,
    /// Approximate centre frequency at hover.
    pub hover_center_hz: Option<f64>,
    /// For notches that follow motor speed: the lowest they go (FREQ × FM_RAT).
    pub min_hz: Option<f64>,
    pub bandwidth_hz: Option<f64>,
    /// Harmonics of the centre frequency it filters (1 = fundamental).
    pub harmonics: Vec<u32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct NoiseReport {
    pub source: Source,
    pub sample_rate_hz: f64,
    /// Which part of the flight the spectra come from.
    pub selection: String,
    pub hover_motor_hz: Option<f64>,
    /// Slowest and fastest motor at hover, Hz.
    pub hover_motor_range_hz: Option<(f64, f64)>,
    /// (motor number, Hz) at hover.
    pub hover_motor_hz_by_motor: Vec<(u32, f64)>,
    pub pre: Spectrum,
    pub post: Option<Spectrum>,
    /// RMS gyro noise above [`MIN_NOISE_HZ`], deg/s.
    pub noise_rms_pre: f64,
    pub noise_rms_post: Option<f64>,
    pub peaks: Vec<Peak>,
    pub notches: Vec<Notch>,
    pub spectrogram: Spectrogram,
    /// Plain-language conclusions, most important first.
    pub findings: Vec<String>,
    /// Caveats about the data.
    pub notes: Vec<String>,
}

/// Times and values of a log field, for lookups by time.
pub(crate) struct Series {
    pub(crate) t: Vec<f64>,
    pub(crate) v: Vec<f64>,
}

impl Series {
    pub(crate) fn get(log: &Log, table: &str, field: &str) -> Option<Self> {
        let tb = log.table(table)?;
        Some(Self { t: tb.time_s()?, v: tb.num(field)?.to_vec() })
    }

    pub(crate) fn window(&self, from: f64, to: f64) -> &[f64] {
        let a = self.t.partition_point(|t| *t < from);
        let b = self.t.partition_point(|t| *t <= to);
        &self.v[a..b.max(a)]
    }
}

/// Motor rotation frequencies (Hz) per motor, from ESC or RPM telemetry.
pub(crate) struct MotorSpeed(Vec<(u32, Series)>);

impl MotorSpeed {
    pub(crate) fn new(log: &Log) -> Self {
        let split = |table: &str, field: &str, key: &str| -> Option<Vec<(u32, Series)>> {
            let tb = log.table(table)?;
            let (t, v) = (tb.time_s()?, tb.num(field)?);
            let Some(k) = tb.num(key) else { return Some(vec![(0, Series { t, v: v.to_vec() })]) };
            let mut by: BTreeMap<u32, Series> = BTreeMap::new();
            for ((t, v), k) in t.iter().zip(v).zip(k) {
                let s = by.entry(*k as u32).or_insert(Series { t: Vec::new(), v: Vec::new() });
                s.t.push(*t);
                s.v.push(*v);
            }
            Some(by.into_iter().collect())
        };
        // ESC instances are numbered by output channel; map them to motor numbers.
        let esc = split("ESC", "RPM", "Instance").map(|v| v.into_iter().map(|(i, s)| (crate::diagnose::motor_for_esc(log, i), s)).collect::<Vec<_>>());
        let sensor = || split("RPM", "RPM", "I").or_else(|| split("RPM", "rpm1", "I"));
        let motors = esc.or_else(|| sensor().map(|v| v.into_iter().map(|(i, s)| (i + 1, s)).collect())).unwrap_or_default();
        Self(motors.into_iter().filter(|(_, s)| s.v.iter().any(|v| *v > 0.0)).collect())
    }

    pub(crate) fn known(&self) -> bool {
        !self.0.is_empty()
    }

    /// Each motor's typical (median) frequency over the windows, by motor number.
    pub(crate) fn per_motor(&self, windows: &[(f64, f64)]) -> Vec<(u32, f64)> {
        self.0
            .iter()
            .filter_map(|(m, s)| {
                let w: Vec<f64> = windows.iter().flat_map(|(a, b)| s.window(*a, *b).iter().copied()).filter(|v| *v > 0.0).collect();
                stats::median(&w).map(|rpm| (*m, rpm / 60.0))
            })
            .collect()
    }

    /// Mean over all motors.
    pub(crate) fn hz(&self, from: f64, to: f64) -> Option<f64> {
        stats::mean(&self.per_motor(&[(from, to)]).iter().map(|(_, f)| *f).collect::<Vec<_>>())
    }
}

/// Gyro batches chosen for analysis: steady flight when there is enough.
pub(crate) struct Selection<'a> {
    pub(crate) pre: Vec<&'a GyroBatch>,
    pub(crate) post: Vec<&'a GyroBatch>,
    /// All in-flight batches, for the spectrogram.
    pub(crate) pre_all: Vec<&'a GyroBatch>,
    pub(crate) post_all: Vec<&'a GyroBatch>,
    pub(crate) label: String,
    pub(crate) notes: Vec<String>,
}

pub(crate) fn select<'a>(log: &Log, batches: &'a [GyroBatch], motors: &MotorSpeed) -> Result<Selection<'a>, String> {
    let (flight, _) = log.flight();
    let rates: Vec<Series> = ["RDes", "PDes"].iter().filter_map(|f| Series::get(log, "RATE", f)).collect();
    let calm = |b: &GyroBatch| {
        rates.iter().all(|s| {
            let abs: Vec<f64> = s.window(b.start_s, b.end_s()).iter().map(|v| v.abs()).collect();
            stats::percentile(&abs, 95.0).is_none_or(|p| p < CALM_RATE_DEG_S)
        })
    };
    let flying = |b: &GyroBatch| in_any(&flight, b.start_s) && in_any(&flight, b.end_s()) && (!motors.known() || motors.hz(b.start_s, b.end_s()).is_some());
    let in_flight: Vec<&GyroBatch> = batches.iter().filter(|b| flying(b)).collect();
    if in_flight.is_empty() {
        return Err("no gyro batches were logged while flying".to_owned());
    }
    let pre_all: Vec<&GyroBatch> = in_flight.iter().copied().filter(|b| !b.post_filter).collect();
    let post_all: Vec<&GyroBatch> = in_flight.iter().copied().filter(|b| b.post_filter).collect();
    let mut notes = Vec::new();

    let mut pre: Vec<&GyroBatch> = pre_all.iter().copied().filter(|b| calm(b)).collect();
    let mut post: Vec<&GyroBatch> = post_all.iter().copied().filter(|b| calm(b)).collect();
    let label = if pre.len() >= MIN_HOVER_BATCHES || (pre_all.is_empty() && post.len() >= MIN_HOVER_BATCHES) {
        format!("steady flight ({} of {} batches)", pre.len().max(post.len()), pre_all.len().max(post_all.len()))
    } else {
        notes.push("Too little steady flight; spectra include manoeuvres.".to_owned());
        pre = pre_all.clone();
        post = post_all.clone();
        format!("whole flight ({} batches)", pre.len().max(post.len()))
    };
    if !motors.known() {
        notes.push("No motor RPM data, so peaks can't be matched to motor speed.".to_owned());
    }
    Ok(Selection { pre, post, pre_all, post_all, label, notes })
}

pub fn analyse(log: &Log) -> Result<NoiseReport, String> {
    let (source, batches) = gyro_batches(log).ok_or(NO_GYRO_DATA.to_owned())?;
    let motors = MotorSpeed::new(log);
    let sel = select(log, &batches, &motors)?;
    let Selection { pre_all, post_all, label: selection, mut notes, .. } = sel;
    let (mut pre_sel, post_sel) = (sel.pre, sel.post);
    if pre_sel.is_empty() {
        notes.push("Only post-filter data was logged; peaks are found in filtered noise.".to_owned());
        pre_sel = post_sel.clone();
    }

    let pre = welch(&pre_sel).ok_or("gyro batches are too short to analyse".to_owned())?;
    let post = (!post_sel.is_empty() && !std::ptr::eq(pre_sel[0], post_sel[0])).then(|| welch(&post_sel)).flatten();
    let windows: Vec<(f64, f64)> = pre_sel.iter().map(|b| (b.start_s, b.end_s())).collect();
    let per_motor = motors.per_motor(&windows);
    let speeds: Vec<f64> = per_motor.iter().map(|(_, f)| *f).collect();
    let hover_motor_hz = stats::mean(&speeds);
    let motor_range = speeds.iter().copied().reduce(f64::min).zip(speeds.iter().copied().reduce(f64::max));

    let spectro_batches: Vec<&GyroBatch> = if pre_all.is_empty() { post_all.clone() } else { pre_all.clone() };
    let rows = batch_spectra(&spectro_batches, &motors);
    let notches = notch_config(log, hover_motor_hz);

    let mut peaks = find_peaks(&pre);
    for p in &mut peaks {
        p.class = classify(p.freq_hz, motor_range, &rows, pre.df());
        if let Some(post) = &post {
            let k = post.bin(p.freq_hz);
            let post_db = db(post.psd.iter().map(|a| a[k]).sum());
            p.filter_reduction_db = Some(p.level_db - post_db);
        }
        if let PeakClass::MotorHarmonic { harmonic, .. } = p.class {
            p.notch_covered = Some(notches.iter().any(|n| n.harmonics.contains(&harmonic)));
        }
    }

    Ok(NoiseReport {
        source,
        sample_rate_hz: pre.freq_hz.last().copied().unwrap_or(0.0) * 2.0,
        selection,
        hover_motor_hz,
        hover_motor_range_hz: motor_range,
        findings: findings(&peaks, &per_motor, &notches),
        hover_motor_hz_by_motor: per_motor,
        noise_rms_pre: pre.rms_above(MIN_NOISE_HZ),
        noise_rms_post: post.as_ref().map(|p| p.rms_above(MIN_NOISE_HZ)),
        spectrogram: spectrogram(&rows),
        pre,
        post,
        peaks,
        notches,
        notes,
    })
}

/// Per-batch spectrum (all axes summed, linear) with time and motor frequency.
struct BatchSpectrum {
    time_s: f64,
    motor_hz: Option<f64>,
    freq_hz: Vec<f64>,
    psd: Vec<f64>,
}

fn batch_spectra(batches: &[&GyroBatch], motors: &MotorSpeed) -> Vec<BatchSpectrum> {
    let Some(fft) = fft_for(batches) else { return Vec::new() };
    let freqs = fft.freqs();
    let mut out: Vec<BatchSpectrum> = batches
        .iter()
        .map(|b| {
            let mut psd = vec![0.0; freqs.len()];
            for x in &b.xyz {
                for (a, p) in psd.iter_mut().zip(fft.psd(x)) {
                    *a += p;
                }
            }
            BatchSpectrum { time_s: b.start_s, motor_hz: motors.hz(b.start_s, b.end_s()), freq_hz: freqs.clone(), psd }
        })
        .collect();
    out.sort_by(|a, b| a.time_s.total_cmp(&b.time_s));
    out
}

fn spectrogram(rows: &[BatchSpectrum]) -> Spectrogram {
    let Some(first) = rows.first() else {
        return Spectrogram { time_s: Vec::new(), freq_hz: Vec::new(), power_db: Vec::new(), motor_hz: Vec::new() };
    };
    // Merge neighbouring bins to keep the matrix small.
    let group = first.freq_hz.len().div_ceil(SPECTROGRAM_MAX_BINS);
    let freq_hz = first.freq_hz.chunks(group).map(|c| c[0]).collect();
    let power_db = rows.iter().map(|r| r.psd.chunks(group).map(|c| db(c.iter().sum::<f64>() / c.len() as f64)).collect()).collect();
    Spectrogram { time_s: rows.iter().map(|r| r.time_s).collect(), freq_hz, power_db, motor_hz: rows.iter().map(|r| r.motor_hz).collect() }
}

fn find_peaks(spec: &Spectrum) -> Vec<Peak> {
    let total = spec.total_db();
    let df = spec.df();
    let n = total.len();
    // Floor: the lower envelope over a window that widens with frequency, so a
    // broad motor hump (four motors at slightly different speeds) still
    // stands out against it.
    let floor: Vec<f64> = (0..n)
        .map(|k| {
            let half = ((FLOOR_MIN_HALF_HZ.max(FLOOR_HALF_FRACTION * spec.freq_hz[k]) / df).round() as usize).max(5);
            let w = &total[k.saturating_sub(half)..(k + half + 1).min(n)];
            stats::percentile(w, FLOOR_PERCENTILE).unwrap_or(total[k])
        })
        .collect();
    let lo = spec.bin(MIN_NOISE_HZ);
    let hi = n.saturating_sub(3);
    let mut found: Vec<Peak> = Vec::new();
    for k in lo.max(2)..hi {
        let local_max = (k - 2..=k + 2).all(|j| total[j] <= total[k]);
        let prominence = total[k] - floor[k];
        if !local_max || prominence < PEAK_MIN_PROMINENCE_DB {
            continue;
        }
        let axis = ["roll", "pitch", "yaw"][(0..3).max_by(|a, b| spec.psd[*a][k].total_cmp(&spec.psd[*b][k])).unwrap()];
        let peak = Peak {
            freq_hz: spec.freq_hz[k],
            level_db: total[k],
            prominence_db: prominence,
            axis,
            class: PeakClass::Unknown,
            filter_reduction_db: None,
            notch_covered: None,
        };
        match found.last_mut() {
            Some(last) if peak.freq_hz - last.freq_hz < PEAK_MERGE_HZ.max(PEAK_MERGE_FRACTION * last.freq_hz) => {
                if peak.prominence_db > last.prominence_db {
                    *last = peak;
                }
            }
            _ => found.push(peak),
        }
    }
    found.sort_by(|a, b| b.prominence_db.total_cmp(&a.prominence_db));
    found.truncate(MAX_PEAKS);
    found.sort_by(|a, b| a.freq_hz.total_cmp(&b.freq_hz));
    found
}

/// Is a peak at `f` (found at hover) a motor harmonic, and does it move with
/// motor speed? `motor_range` is the slowest and fastest motor at hover.
fn classify(f: f64, motor_range: Option<(f64, f64)>, rows: &[BatchSpectrum], df: f64) -> PeakClass {
    let Some((lo, hi)) = motor_range.filter(|(lo, _)| *lo > 0.0) else { return PeakClass::Unknown };
    let m = (lo + hi) / 2.0;
    // Harmonic k when f falls in k × [slowest, fastest] motor, give or take a
    // tolerance capped well below the harmonic spacing.
    let tol = (HARMONIC_TOLERANCE * f).min(HARMONIC_SPACING_FRACTION * m).max(2.0 * df);
    let matching = (1..=MAX_HARMONIC).find(|k| {
        let k = *k as f64;
        f >= k * lo - tol && f <= k * hi + tol
    });
    let harmonic = matching.is_some();
    let k = matching.unwrap_or_else(|| (f / m).round().max(1.0) as u32);

    // Compare how strongly each batch shows energy where a tracking peak
    // would be (k_eff × its motor speed) versus at the fixed hover frequency.
    let k_eff = f / m;
    let usable: Vec<&BatchSpectrum> = rows.iter().filter(|r| r.motor_hz.is_some()).collect();
    let speeds: Vec<f64> = usable.iter().filter_map(|r| r.motor_hz).collect();
    let spread = match (stats::percentile(&speeds, 90.0), stats::percentile(&speeds, 10.0)) {
        (Some(hi), Some(lo)) if lo > 0.0 => hi / lo,
        _ => 1.0,
    };
    let tracks = if spread < MIN_RPM_SPREAD {
        None
    } else {
        let band_db = |r: &BatchSpectrum, centre: f64| -> Option<f64> {
            let w = (0.06 * centre).max(2.0 * df);
            let vals: Vec<f64> = r.freq_hz.iter().zip(&r.psd).filter(|(fr, _)| (**fr - centre).abs() <= w).map(|(_, p)| *p).collect();
            vals.iter().copied().reduce(f64::max).map(db)
        };
        let mut track = Vec::new();
        let mut fixed = Vec::new();
        for r in &usable {
            let moving = r.motor_hz.unwrap() * k_eff;
            // Only batches where the two hypotheses point at different frequencies.
            if (moving - f).abs() < 0.15 * f {
                continue;
            }
            if let (Some(a), Some(b)) = (band_db(r, moving), band_db(r, f)) {
                track.push(a);
                fixed.push(b);
            }
        }
        match (stats::mean(&track), stats::mean(&fixed)) {
            (Some(a), Some(b)) if track.len() >= 3 => {
                if a > b + TRACK_DECISION_DB {
                    Some(true)
                } else if b > a + TRACK_DECISION_DB {
                    Some(false)
                } else {
                    None
                }
            }
            _ => None,
        }
    };
    match (harmonic, tracks) {
        (_, Some(false)) => PeakClass::FixedFrequency,
        (true, t) => PeakClass::MotorHarmonic { harmonic: k, tracks_rpm: t },
        (false, Some(true)) => PeakClass::MotorHarmonic { harmonic: k, tracks_rpm: Some(true) },
        (false, None) if f < CONTROL_BAND_HZ => PeakClass::ControlBand,
        (false, None) => PeakClass::Unknown,
    }
}

/// Motors more than this far apart in hover point to an imbalance.
const MOTOR_SPREAD_WARN: f64 = 0.15;
/// Peaks this far above the floor are worth acting on.
const SIGNIFICANT_DB: f64 = 10.0;
/// Filters removing at least this much make a peak harmless.
const WELL_FILTERED_DB: f64 = 20.0;

fn findings(peaks: &[Peak], per_motor: &[(u32, f64)], notches: &[Notch]) -> Vec<String> {
    let mut out = Vec::new();
    let fmt_db = |v: Option<f64>| v.map_or("an unknown amount".to_owned(), |d| format!("{d:.0} dB"));
    for p in peaks.iter().filter(|p| p.prominence_db >= SIGNIFICANT_DB) {
        match p.class {
            PeakClass::ControlBand => out.push(format!(
                "Strong {} motion at {:.0} Hz ({:.0} dB above the floor). It is too close to the control loop for \
                 filters to remove without adding delay (they take off {}), so it is likely real movement: an \
                 oscillation, prop wash or a flexible frame. Check it in PID tuning.",
                p.axis,
                p.freq_hz,
                p.prominence_db,
                fmt_db(p.filter_reduction_db)
            )),
            PeakClass::MotorHarmonic { harmonic, .. } if p.notch_covered == Some(false) => {
                if p.filter_reduction_db.is_some_and(|r| r >= WELL_FILTERED_DB) {
                    continue;
                }
                let target = notches
                    .first()
                    .map_or("enable the harmonic notch (INS_HNTCH_ENABLE)".to_owned(), |n| format!("add harmonic {harmonic} to {}_HMNCS", n.name));
                out.push(format!(
                    "Motor harmonic {harmonic} at {:.0} Hz is not targeted by a notch and the filters only remove {}: {target}.",
                    p.freq_hz,
                    fmt_db(p.filter_reduction_db)
                ));
            }
            PeakClass::MotorHarmonic { harmonic, .. } if p.filter_reduction_db.is_some_and(|r| r < 15.0) => out.push(format!(
                "The notch covers motor harmonic {harmonic} at {:.0} Hz but only removes {}: its centre or bandwidth may not match.",
                p.freq_hz,
                fmt_db(p.filter_reduction_db)
            )),
            PeakClass::FixedFrequency => out.push(format!(
                "Fixed-frequency noise at {:.0} Hz on {} that doesn't follow motor speed: a frame or mount resonance. \
                 Filters remove {}; a static notch (INS_HNTC2 in mode 0) can target it.",
                p.freq_hz,
                p.axis,
                fmt_db(p.filter_reduction_db)
            )),
            _ => {}
        }
    }
    // A tracking notch whose floor is above the motors never reaches them.
    if let Some(slowest) = per_motor.iter().map(|(_, f)| *f).reduce(f64::min) {
        for n in notches {
            if let Some(min) = n.min_hz.filter(|m| *m > slowest * 0.95) {
                out.insert(
                    0,
                    format!(
                        "{name} can't reach the motors: it never goes below {name}_FREQ × {name}_FM_RAT = {min:.0} Hz, but the slowest \
                         motor runs at {slowest:.0} Hz in steady flight, so the notch sits above the motor noise. Set {name}_FREQ to about \
                         {low:.0} Hz (a little below the slowest motor); the Filters page proposes it.",
                        name = n.name,
                        low = (0.8 * slowest).round()
                    ),
                );
            }
        }
    }
    if let (Some(lo), Some(hi)) = (per_motor.iter().map(|(_, f)| *f).reduce(f64::min), per_motor.iter().map(|(_, f)| *f).reduce(f64::max)) {
        if per_motor.len() > 1 && hi / lo - 1.0 > MOTOR_SPREAD_WARN {
            let list: Vec<String> = per_motor.iter().map(|(m, f)| format!("motor {m} {:.0} RPM", f * 60.0)).collect();
            out.push(format!(
                "Motors run at quite different speeds in steady flight ({}; {:.0}% spread). The fast ones are \
                 working harder: the centre of gravity may be off, a motor or prop may be weaker, or a motor may be \
                 twisted (yaw imbalance).",
                list.join(", "),
                (hi / lo - 1.0) * 100.0
            ));
        }
    }
    out
}

fn notch_config(log: &Log, hover_motor_hz: Option<f64>) -> Vec<Notch> {
    let mut out = Vec::new();
    for name in ["INS_HNTCH", "INS_HNTC2"] {
        let p = |s: &str| log.param(&format!("{name}_{s}"));
        if p("ENABLE").unwrap_or(0.0) == 0.0 {
            continue;
        }
        let mode = p("MODE").unwrap_or(0.0) as u32;
        let freq = p("FREQ");
        // REF = 0 turns tracking off in every mode (AP_Vehicle::update_dynamic_notch).
        let reference = p("REF").unwrap_or(0.0);
        let tracks_motors = matches!(mode, 2 | 3 | 5) && reference > 0.0;
        let min_hz = freq.map(|f| f * p("FM_RAT").unwrap_or(1.0)).filter(|_| tracks_motors);
        let hover_center_hz = match mode {
            _ if reference <= 0.0 => freq,
            0 => freq,
            1 => match (freq, p("REF"), log.param("MOT_THST_HOVER")) {
                (Some(f), Some(r), Some(h)) if r > 0.0 => Some(f * (h / r).sqrt()),
                (f, _, _) => f,
            },
            // RPM-based modes follow motor speed × REF, floored at FREQ × FM_RAT.
            _ => hover_motor_hz.map(|m| m * reference).map(|m| min_hz.map_or(m, |f| m.max(f))).or(freq),
        };
        let mode_name = match mode {
            0 => "fixed frequency",
            1 => "throttle",
            2 => "RPM sensor",
            3 => "ESC telemetry",
            4 => "in-flight FFT",
            5 => "second RPM sensor",
            _ => "other",
        };
        let hmncs = p("HMNCS").unwrap_or(1.0) as u32;
        out.push(Notch {
            name: if name == "INS_HNTCH" { "INS_HNTCH" } else { "INS_HNTC2" },
            mode: format!("{mode_name} (mode {mode})"),
            hover_center_hz,
            min_hz,
            bandwidth_hz: p("BW"),
            harmonics: (0..16).filter(|b| hmncs & (1 << b) != 0).map(|b| b + 1).collect(),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{sample_flight, Preset};

    #[test]
    fn notch_that_cant_reach_the_motors_is_reported_first() {
        let notch = Notch {
            name: "INS_HNTCH",
            mode: "ESC telemetry (mode 3)".into(),
            hover_center_hz: Some(80.0),
            min_hz: Some(80.0),
            bandwidth_hz: Some(80.0),
            harmonics: vec![1],
        };
        let found = findings(&[], &[(1, 58.0), (2, 56.0)], std::slice::from_ref(&notch));
        assert!(found[0].contains("can't reach the motors") && found[0].contains("about 45 Hz"), "{found:?}");
        let fine = Notch { min_hz: Some(45.0), ..notch };
        assert!(findings(&[], &[(1, 58.0), (2, 56.0)], &[fine]).is_empty());
    }

    #[test]
    fn sine_lands_in_the_right_bin_with_the_right_power() {
        // 5 deg/s amplitude sine at 120 Hz: power 12.5 (deg/s)².
        let rate = 1000.0;
        let x: Vec<f64> = (0..1024).map(|i| 5.0 * (2.0 * PI * 120.0 * i as f64 / rate).sin()).collect();
        let b = GyroBatch { start_s: 0.0, instance: 0, post_filter: false, rate_hz: rate, quantum: None, xyz: [x.clone(), x.clone(), x] };
        let s = welch(&[&b]).unwrap();
        let k = (0..s.freq_hz.len()).max_by(|a, b| s.psd[0][*a].total_cmp(&s.psd[0][*b])).unwrap();
        assert!((s.freq_hz[k] - 120.0).abs() < 1.0, "{}", s.freq_hz[k]);
        let power: f64 = s.psd[0].iter().sum::<f64>() * s.df();
        assert!((power - 12.5).abs() < 0.5, "{power}");
    }

    #[test]
    fn finds_motor_harmonics_in_sample_flight() {
        let log = Log::from_bytes(&sample_flight(Preset::Ready));
        let r = analyse(&log).unwrap();
        assert_eq!(r.source, Source::RawImu);
        assert!((r.hover_motor_hz.unwrap() - 150.0).abs() < 3.0, "{:?}", r.hover_motor_hz);
        let classes: Vec<(u32, PeakClass)> = r.peaks.iter().map(|p| (p.freq_hz.round() as u32, p.class)).collect();
        let harmonic = |k: u32| r.peaks.iter().any(|p| matches!(p.class, PeakClass::MotorHarmonic { harmonic, .. } if harmonic == k));
        assert!(harmonic(1) && harmonic(2), "{classes:?}");
        // The sample's notch is disabled.
        assert!(r.notches.is_empty());
        assert!(r.peaks.iter().all(|p| p.notch_covered != Some(true)));
        assert!(!r.spectrogram.time_s.is_empty());
    }

    #[test]
    fn no_gyro_data_is_an_error() {
        let log = Log::from_bytes(&sample_flight(Preset::DefaultLogging));
        assert!(analyse(&log).unwrap_err().contains("no high-rate gyro data"));
    }
}
