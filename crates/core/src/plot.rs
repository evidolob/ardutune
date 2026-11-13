//! SVG plots of the noise, filter and PID reports, shared by the CLI and the
//! desktop app.

use std::fmt::Write;

use crate::filters::FilterReport;
use crate::noise::{NoiseReport, PeakClass};
use crate::pid::PidReport;

const W: f64 = 960.0;
const LEFT: f64 = 64.0;
const RIGHT: f64 = 24.0;
const SPEC_TOP: f64 = 48.0;
const SPEC_H: f64 = 320.0;
const GRAM_TOP: f64 = SPEC_TOP + SPEC_H + 72.0;
const GRAM_H: f64 = 260.0;
const H: f64 = GRAM_TOP + GRAM_H + 56.0;
const PRE: &str = "#c2410c";
const POST: &str = "#1d4ed8";
const MAX_HARMONIC_BANDS: u32 = 4;

/// Viridis, sampled.
const COLORMAP: [(f64, f64, f64); 6] =
    [(68.0, 1.0, 84.0), (65.0, 68.0, 135.0), (42.0, 120.0, 142.0), (34.0, 168.0, 132.0), (122.0, 209.0, 81.0), (253.0, 231.0, 37.0)];

fn color(x: f64) -> String {
    let x = x.clamp(0.0, 1.0) * (COLORMAP.len() - 1) as f64;
    let i = (x.floor() as usize).min(COLORMAP.len() - 2);
    let t = x - i as f64;
    let (a, b) = (COLORMAP[i], COLORMAP[i + 1]);
    let mix = |p: f64, q: f64| (p + (q - p) * t).round() as u8;
    format!("#{:02x}{:02x}{:02x}", mix(a.0, b.0), mix(a.1, b.1), mix(a.2, b.2))
}

fn nice_step(range: f64, target_ticks: f64) -> f64 {
    let raw = range / target_ticks;
    let mag = 10f64.powf(raw.log10().floor());
    [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * mag).find(|s| *s >= raw).unwrap_or(10.0 * mag)
}

pub fn render(r: &NoiseReport) -> String {
    let fmax = r.pre.freq_hz.last().copied().unwrap_or(500.0);
    let plot_w = W - LEFT - RIGHT;
    let x_of = |f: f64| LEFT + f / fmax * plot_w;
    let pre = r.pre.total_db();
    let post = r.post.as_ref().map(|p| p.total_db());

    let visible = |v: &[f64]| -> Vec<f64> { r.pre.freq_hz.iter().zip(v).filter(|(f, _)| **f >= 5.0).map(|(_, d)| *d).collect() };
    let mut all = visible(&pre);
    if let Some(p) = &post {
        all.extend(visible(p));
    }
    let top = all.iter().copied().fold(f64::NEG_INFINITY, f64::max) + 5.0;
    let bottom = all.iter().copied().fold(f64::INFINITY, f64::min).max(top - 80.0) - 2.0;
    let y_of = |d: f64| SPEC_TOP + (top - d.clamp(bottom, top)) / (top - bottom) * SPEC_H;

    let mut s = String::new();
    let _ = writeln!(
        s,
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="system-ui, sans-serif" font-size="12">
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="{LEFT}" y="24" font-size="16" font-weight="600" fill="#111827">Gyro noise spectrum (all axes, {})</text>"##,
        r.selection
    );

    // Motor harmonic bands, green when a notch targets them.
    if let Some((lo, hi)) = r.hover_motor_range_hz {
        for k in 1..=MAX_HARMONIC_BANDS {
            let (a, b) = (lo * k as f64, hi * k as f64);
            if a > fmax {
                break;
            }
            let covered = r.notches.iter().any(|n| n.harmonics.contains(&k));
            let fill = if covered { "#dcfce7" } else { "#f3f4f6" };
            let _ = writeln!(
                s,
                r##"<rect x="{:.1}" y="{SPEC_TOP}" width="{:.1}" height="{SPEC_H}" fill="{fill}"/><text x="{:.1}" y="{:.1}" fill="#6b7280" text-anchor="middle">{k}×</text>"##,
                x_of(a),
                (x_of(b.min(fmax)) - x_of(a)).max(1.0),
                (x_of(a) + x_of(b.min(fmax))) / 2.0,
                SPEC_TOP + 14.0
            );
        }
    }

    // Grid and axes.
    let fstep = nice_step(fmax, 10.0);
    let mut f = 0.0;
    while f <= fmax + 1e-9 {
        let x = x_of(f);
        let _ = writeln!(
            s,
            r##"<line x1="{x:.1}" y1="{SPEC_TOP}" x2="{x:.1}" y2="{:.1}" stroke="#e5e7eb"/><text x="{x:.1}" y="{:.1}" fill="#374151" text-anchor="middle">{f:.0}</text>"##,
            SPEC_TOP + SPEC_H,
            SPEC_TOP + SPEC_H + 16.0
        );
        f += fstep;
    }
    let dstep = nice_step(top - bottom, 6.0);
    let mut d = (bottom / dstep).ceil() * dstep;
    while d <= top {
        let y = y_of(d);
        let _ = writeln!(
            s,
            r##"<line x1="{LEFT}" y1="{y:.1}" x2="{:.1}" y2="{y:.1}" stroke="#e5e7eb"/><text x="{:.1}" y="{:.1}" fill="#374151" text-anchor="end">{d:.0}</text>"##,
            W - RIGHT,
            LEFT - 6.0,
            y + 4.0
        );
        d += dstep;
    }
    let _ = writeln!(
        s,
        r##"<text x="{:.1}" y="{:.1}" fill="#374151" text-anchor="middle">Frequency (Hz)</text>
<text x="16" y="{:.1}" fill="#374151" text-anchor="middle" transform="rotate(-90 16 {:.1})">dB re (°/s)²/Hz</text>"##,
        LEFT + plot_w / 2.0,
        SPEC_TOP + SPEC_H + 34.0,
        SPEC_TOP + SPEC_H / 2.0,
        SPEC_TOP + SPEC_H / 2.0
    );

    let line = |vals: &[f64], colour: &str| -> String {
        let pts: Vec<String> = r.pre.freq_hz.iter().zip(vals).map(|(f, d)| format!("{:.1},{:.1}", x_of(*f), y_of(*d))).collect();
        format!(r##"<polyline fill="none" stroke="{colour}" stroke-width="1.5" points="{}"/>"##, pts.join(" ")) + "\n"
    };
    s += &line(&pre, PRE);
    if let Some(p) = &post {
        s += &line(p, POST);
    }

    // Labels go in rows along the top of the plot, each in the first row
    // with room, joined to its peak by a leader line.
    let mut rows: Vec<f64> = Vec::new();
    for p in &r.peaks {
        let what = match p.class {
            PeakClass::MotorHarmonic { harmonic, .. } => format!("{harmonic}× motor"),
            PeakClass::FixedFrequency => "fixed".to_owned(),
            PeakClass::ControlBand => "control band".to_owned(),
            PeakClass::Unknown => String::new(),
        };
        let (x, y) = (x_of(p.freq_hz), y_of(p.level_db));
        let row = rows.iter().position(|end| x > *end).unwrap_or(rows.len());
        let label = format!("{:.0} Hz {what}", p.freq_hz);
        let end = x + 7.0 * label.chars().count() as f64 + 8.0;
        if row == rows.len() {
            rows.push(end);
        } else {
            rows[row] = end;
        }
        let ly = SPEC_TOP + 30.0 + row as f64 * 15.0;
        let _ = writeln!(
            s,
            r##"<line x1="{x:.1}" y1="{y:.1}" x2="{x:.1}" y2="{:.1}" stroke="#9ca3af" stroke-width="0.6"/><circle cx="{x:.1}" cy="{y:.1}" r="3" fill="{PRE}"/><text x="{:.1}" y="{ly:.1}" fill="#111827">{label}</text>"##,
            ly + 3.0,
            x + 3.0
        );
    }

    // Legend.
    let lx = W - RIGHT - 230.0;
    let _ = writeln!(
        s,
        r##"<rect x="{lx}" y="{}" width="226" height="{}" fill="#ffffff" stroke="#e5e7eb"/>
<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{PRE}" stroke-width="2"/><text x="{}" y="{}" fill="#111827">Before filters</text>"##,
        SPEC_TOP + 22.0,
        if post.is_some() { 64.0 } else { 46.0 },
        lx + 8.0,
        SPEC_TOP + 36.0,
        lx + 32.0,
        SPEC_TOP + 36.0,
        lx + 38.0,
        SPEC_TOP + 40.0
    );
    let mut ly = SPEC_TOP + 54.0;
    if post.is_some() {
        let _ = writeln!(
            s,
            r##"<line x1="{}" y1="{ly}" x2="{}" y2="{ly}" stroke="{POST}" stroke-width="2"/><text x="{}" y="{}" fill="#111827">After filters</text>"##,
            lx + 8.0,
            lx + 32.0,
            lx + 38.0,
            ly + 4.0
        );
        ly += 18.0;
    }
    let _ = writeln!(
        s,
        r##"<rect x="{}" y="{}" width="24" height="10" fill="#dcfce7"/><text x="{}" y="{}" fill="#111827">Motor harmonic (notched)</text>"##,
        lx + 8.0,
        ly - 6.0,
        lx + 38.0,
        ly + 4.0
    );

    // Most of what matters sits below a few motor harmonics.
    let gram_max = r.hover_motor_hz.map_or(fmax, |m| (8.0 * m).max(400.0)).min(fmax);
    spectrogram(&mut s, r, gram_max);
    s += "</svg>\n";
    s
}

fn spectrogram(s: &mut String, r: &NoiseReport, fmax: f64) {
    let g = &r.spectrogram;
    let (Some(&t0), Some(&t1)) = (g.time_s.first(), g.time_s.last()) else { return };
    if g.freq_hz.len() < 2 {
        return;
    }
    let plot_w = W - LEFT - RIGHT;
    let span = (t1 - t0).max(1.0);
    let x_of = |t: f64| LEFT + (t - t0) / span * plot_w;
    let y_of = |f: f64| GRAM_TOP + GRAM_H - f / fmax * GRAM_H;
    let all: Vec<f64> = g.power_db.iter().flatten().copied().collect();
    let lo = crate::stats::percentile(&all, 5.0).unwrap_or(-60.0);
    let hi = crate::stats::percentile(&all, 99.5).unwrap_or(0.0);
    let df = g.freq_hz[1] - g.freq_hz[0];
    let typical_dt = crate::stats::median(&g.time_s.windows(2).map(|w| w[1] - w[0]).collect::<Vec<_>>()).unwrap_or(1.0);

    let _ = writeln!(
        s,
        r##"<text x="{LEFT}" y="{:.1}" font-size="16" font-weight="600" fill="#111827">Spectrogram before filters (line: motor speed)</text>"##,
        GRAM_TOP - 16.0
    );
    for (i, (t, row)) in g.time_s.iter().zip(&g.power_db).enumerate() {
        let next = g.time_s.get(i + 1).copied().unwrap_or(t + typical_dt);
        let w = (x_of(next.min(t + 2.0 * typical_dt)) - x_of(*t)).max(0.5);
        for (f, d) in g.freq_hz.iter().zip(row).filter(|(f, _)| **f < fmax) {
            let _ = writeln!(
                s,
                r##"<rect x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}" fill="{}"/>"##,
                x_of(*t),
                y_of(f + df),
                w + 0.3,
                (y_of(*f) - y_of(f + df)) + 0.3,
                color((d - lo) / (hi - lo).max(1e-6))
            );
        }
        s.push('\n');
    }
    for k in [1.0, 2.0] {
        let pts: Vec<String> =
            g.time_s.iter().zip(&g.motor_hz).filter_map(|(t, m)| m.filter(|m| m * k <= fmax).map(|m| format!("{:.1},{:.1}", x_of(*t), y_of(m * k)))).collect();
        if pts.len() > 1 {
            let dash = if k > 1.0 { r##" stroke-dasharray="4 3""## } else { "" };
            let _ = writeln!(s, r##"<polyline fill="none" stroke="#ffffff" stroke-width="1.2"{dash} points="{}"/>"##, pts.join(" "));
        }
    }
    let fstep = nice_step(fmax, 5.0);
    let mut f = 0.0;
    while f <= fmax + 1e-9 {
        let _ = writeln!(s, r##"<text x="{:.1}" y="{:.1}" fill="#374151" text-anchor="end">{f:.0}</text>"##, LEFT - 6.0, y_of(f) + 4.0);
        f += fstep;
    }
    let tstep = nice_step(span, 8.0);
    let mut t = (t0 / tstep).ceil() * tstep;
    while t <= t1 {
        let _ = writeln!(s, r##"<text x="{:.1}" y="{:.1}" fill="#374151" text-anchor="middle">{t:.0}</text>"##, x_of(t), GRAM_TOP + GRAM_H + 16.0);
        t += tstep;
    }
    let _ = write!(
        s,
        r##"<text x="{:.1}" y="{:.1}" fill="#374151" text-anchor="middle">Time (s)</text>
<text x="16" y="{:.1}" fill="#374151" text-anchor="middle" transform="rotate(-90 16 {:.1})">Frequency (Hz)</text>"##,
        LEFT + plot_w / 2.0,
        GRAM_TOP + GRAM_H + 34.0,
        GRAM_TOP + GRAM_H / 2.0,
        GRAM_TOP + GRAM_H / 2.0
    );
}

const SIM: &str = "#15803d";
const QUIET: &str = "#7c3aed";
const GREY: &str = "#9ca3af";

/// Spectra (recorded, logged after filters, simulated) and the noise/delay
/// trade-off of the settings tried.
pub fn render_filters(r: &FilterReport) -> String {
    const TOP: f64 = 48.0;
    const PH: f64 = 320.0;
    const TRADE_TOP: f64 = TOP + PH + 80.0;
    const TH: f64 = 260.0;
    const HEIGHT: f64 = TRADE_TOP + TH + 56.0;
    let plot_w = W - LEFT - RIGHT;
    let fmax = r.freq_hz.last().copied().unwrap_or(500.0).min(500.0);
    let x_of = |f: f64| LEFT + f / fmax * plot_w;
    let mut lines: Vec<(&[f64], &str, &str, bool)> = vec![(&r.pre_db, GREY, "Recorded noise (before filters)", false)];
    if let Some(p) = &r.logged_post_db {
        lines.push((p, POST, "Logged after filters", false));
    }
    lines.push((&r.current.spectrum_db, POST, "Simulated: current settings", true));
    if let Some(e) = &r.recommended {
        lines.push((&e.spectrum_db, SIM, "Simulated: recommended", false));
    }
    if let Some(e) = &r.quieter {
        lines.push((&e.spectrum_db, QUIET, "Simulated: quieter", false));
    }
    let visible: Vec<f64> =
        lines.iter().flat_map(|(v, ..)| r.freq_hz.iter().zip(v.iter()).filter(|(f, _)| **f >= 5.0 && **f <= fmax).map(|(_, d)| *d)).collect();
    let top = visible.iter().copied().fold(f64::NEG_INFINITY, f64::max) + 5.0;
    let bottom = visible.iter().copied().fold(f64::INFINITY, f64::min).max(top - 90.0) - 2.0;
    let y_of = |d: f64| TOP + (top - d.clamp(bottom, top)) / (top - bottom) * PH;

    let mut s = String::new();
    let _ = writeln!(
        s,
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {HEIGHT}" width="{W}" height="{HEIGHT}" font-family="system-ui, sans-serif" font-size="12">
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="{LEFT}" y="24" font-size="16" font-weight="600" fill="#111827">Gyro noise through the filters (all axes)</text>"##
    );
    grid(&mut s, LEFT, TOP, plot_w, PH, 0.0, fmax, bottom, top, &x_of, &y_of, "Frequency (Hz)", "dB re (°/s)²/Hz");
    for (vals, colour, _, dashed) in &lines {
        let pts: Vec<String> =
            r.freq_hz.iter().zip(vals.iter()).filter(|(f, _)| **f <= fmax).map(|(f, d)| format!("{:.1},{:.1}", x_of(*f), y_of(*d))).collect();
        let dash = if *dashed { r##" stroke-dasharray="5 3""## } else { "" };
        let _ = writeln!(s, r##"<polyline fill="none" stroke="{colour}" stroke-width="1.5"{dash} points="{}"/>"##, pts.join(" "));
    }
    let lx = W - RIGHT - 250.0;
    let _ = writeln!(s, r##"<rect x="{lx}" y="{}" width="246" height="{}" fill="#ffffff" stroke="#e5e7eb"/>"##, TOP + 8.0, 12.0 + 18.0 * lines.len() as f64);
    for (i, (_, colour, label, dashed)) in lines.iter().enumerate() {
        let y = TOP + 24.0 + 18.0 * i as f64;
        let dash = if *dashed { r##" stroke-dasharray="5 3""## } else { "" };
        let _ = writeln!(
            s,
            r##"<line x1="{}" y1="{y}" x2="{}" y2="{y}" stroke="{colour}" stroke-width="2"{dash}/><text x="{}" y="{}" fill="#111827">{label}</text>"##,
            lx + 8.0,
            lx + 32.0,
            lx + 38.0,
            y + 4.0
        );
    }

    // Trade-off: delay vs noise for every setting on the frontier.
    // Beyond twice today's delay the options are only of academic interest.
    let lag_cap = 2.0 * r.current.lag_deg.max(10.0);
    let pts: Vec<(f64, f64)> = r.frontier.iter().filter(|e| e.lag_deg <= lag_cap).map(|e| (e.lag_deg, e.noise_rms)).collect();
    let mut all = pts.clone();
    all.push((r.current.lag_deg, r.current.noise_rms));
    let lag_max = all.iter().map(|p| p.0).fold(0.0, f64::max) * 1.05;
    let noise_max = all.iter().map(|p| p.1).fold(0.0, f64::max) * 1.1;
    let tx = |lag: f64| LEFT + lag / lag_max * plot_w;
    let ty = |n: f64| TRADE_TOP + TH - n / noise_max * TH;
    let _ = writeln!(
        s,
        r##"<text x="{LEFT}" y="{:.1}" font-size="16" font-weight="600" fill="#111827">Trade-off: delay at {:.0} Hz vs noise left above 45 Hz</text>"##,
        TRADE_TOP - 16.0,
        r.lag_ref_hz
    );
    grid(&mut s, LEFT, TRADE_TOP, plot_w, TH, 0.0, lag_max, 0.0, noise_max, &tx, &ty, "Delay (degrees of phase)", "Noise (°/s RMS)");
    let curve: Vec<String> = pts.iter().map(|(l, n)| format!("{:.1},{:.1}", tx(*l), ty(*n))).collect();
    let _ = writeln!(s, r##"<polyline fill="none" stroke="{GREY}" stroke-width="1.2" points="{}"/>"##, curve.join(" "));
    for (l, n) in &pts {
        let _ = writeln!(s, r##"<circle cx="{:.1}" cy="{:.1}" r="2.5" fill="{GREY}"/>"##, tx(*l), ty(*n));
    }
    // Label offsets: current below-right, recommended above, quieter right.
    let mut mark = |e: &crate::filters::Evaluation, colour: &str, label: &str, dx: f64, dy: f64| {
        let (x, y) = (tx(e.lag_deg), ty(e.noise_rms));
        let _ = writeln!(
            s,
            r##"<circle cx="{x:.1}" cy="{y:.1}" r="6" fill="{colour}"/><text x="{:.1}" y="{:.1}" fill="{colour}" font-weight="600">{label}</text>"##,
            x + dx,
            y + dy
        );
    };
    mark(&r.current, POST, "Current", 10.0, 20.0);
    if let Some(e) = &r.recommended {
        mark(e, SIM, "Recommended", -40.0, -12.0);
    }
    if let Some(e) = &r.quieter {
        mark(e, QUIET, "Quieter", 10.0, 4.0);
    }
    s += "</svg>\n";
    s
}

/// Axes with gridlines and tick labels for a plot area.
#[allow(clippy::too_many_arguments)]
fn grid(
    s: &mut String,
    left: f64,
    top: f64,
    width: f64,
    height: f64,
    x0: f64,
    x1: f64,
    y0: f64,
    y1: f64,
    x_of: &dyn Fn(f64) -> f64,
    y_of: &dyn Fn(f64) -> f64,
    x_label: &str,
    y_label: &str,
) {
    let xs = nice_step(x1 - x0, 10.0);
    let mut x = (x0 / xs).ceil() * xs;
    while x <= x1 + 1e-9 {
        let px = x_of(x);
        let _ = writeln!(
            s,
            r##"<line x1="{px:.1}" y1="{top}" x2="{px:.1}" y2="{:.1}" stroke="#e5e7eb"/><text x="{px:.1}" y="{:.1}" fill="#374151" text-anchor="middle">{}</text>"##,
            top + height,
            top + height + 16.0,
            crate::params::format_value((x * 100.0).round() / 100.0)
        );
        x += xs;
    }
    let ys = nice_step(y1 - y0, 6.0);
    let mut y = (y0 / ys).ceil() * ys;
    while y <= y1 + 1e-9 {
        let py = y_of(y);
        let _ = writeln!(
            s,
            r##"<line x1="{left}" y1="{py:.1}" x2="{:.1}" y2="{py:.1}" stroke="#e5e7eb"/><text x="{:.1}" y="{:.1}" fill="#374151" text-anchor="end">{}</text>"##,
            left + width,
            left - 6.0,
            py + 4.0,
            crate::params::format_value((y * 100.0).round() / 100.0)
        );
        y += ys;
    }
    let _ = writeln!(
        s,
        r##"<text x="{:.1}" y="{:.1}" fill="#374151" text-anchor="middle">{x_label}</text>
<text x="16" y="{:.1}" fill="#374151" text-anchor="middle" transform="rotate(-90 16 {:.1})">{y_label}</text>"##,
        left + width / 2.0,
        top + height + 34.0,
        top + height / 2.0,
        top + height / 2.0
    );
}

const AXIS_COLOURS: [&str; 3] = ["#c2410c", "#1d4ed8", "#15803d"];

/// Step responses of the three axes against the ideal unit step.
pub fn render_pid(r: &PidReport) -> String {
    const TOP: f64 = 48.0;
    const PH: f64 = 380.0;
    const HEIGHT: f64 = TOP + PH + 60.0;
    let plot_w = W - LEFT - RIGHT;
    let tmax = crate::pid::RESPONSE_S * 1000.0;
    let peak = r.axes.iter().filter_map(|a| a.step.as_ref()).flat_map(|s| s.response.iter().copied()).fold(1.2, f64::max).min(2.5);
    let (y0, y1) = (0.0, (peak * 10.0).ceil() / 10.0 + 0.1);
    let x_of = |ms: f64| LEFT + ms / tmax * plot_w;
    let y_of = |v: f64| TOP + (y1 - v.clamp(y0, y1)) / (y1 - y0) * PH;
    let mut s = String::new();
    let _ = writeln!(
        s,
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {HEIGHT}" width="{W}" height="{HEIGHT}" font-family="system-ui, sans-serif" font-size="12">
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="{LEFT}" y="24" font-size="16" font-weight="600" fill="#111827">Rate-loop step response (response to a unit step in target rate)</text>"##
    );
    grid(&mut s, LEFT, TOP, plot_w, PH, 0.0, tmax, y0, y1, &x_of, &y_of, "Time (ms)", "Rate / target");
    let _ = writeln!(s, r##"<line x1="{LEFT}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#111827" stroke-dasharray="4 3"/>"##, y_of(1.0), W - RIGHT, y_of(1.0));
    let mut legend = 0;
    for (a, colour) in r.axes.iter().zip(AXIS_COLOURS) {
        let Some(step) = &a.step else { continue };
        let pts: Vec<String> = step.time_s.iter().zip(&step.response).map(|(t, v)| format!("{:.1},{:.1}", x_of(t * 1000.0), y_of(*v))).collect();
        let dash = if a.includes_autotune { r##" stroke-dasharray="6 2""## } else { "" };
        let _ = writeln!(s, r##"<polyline fill="none" stroke="{colour}" stroke-width="2"{dash} points="{}"/>"##, pts.join(" "));
        let ly = TOP + 24.0 + 18.0 * legend as f64;
        let note = if a.includes_autotune { " (incl. AutoTune)" } else { "" };
        let _ = writeln!(
            s,
            r##"<line x1="{:.1}" y1="{ly}" x2="{:.1}" y2="{ly}" stroke="{colour}" stroke-width="2"{dash}/><text x="{:.1}" y="{:.1}" fill="#111827">{}: overshoot {:.0}%{note}</text>"##,
            W - RIGHT - 300.0,
            W - RIGHT - 276.0,
            W - RIGHT - 270.0,
            ly + 4.0,
            a.axis,
            step.overshoot_pct
        );
        legend += 1;
    }
    s += "</svg>\n";
    s
}
