//! Small statistics helpers.

pub fn mean(v: &[f64]) -> Option<f64> {
    (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
}

pub fn std_dev(v: &[f64]) -> Option<f64> {
    let m = mean(v)?;
    Some((v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt())
}

/// Linear-interpolated percentile, `p` in 0..=100.
pub fn percentile(v: &[f64], p: f64) -> Option<f64> {
    let mut s: Vec<f64> = v.iter().copied().filter(|x| x.is_finite()).collect();
    if s.is_empty() {
        return None;
    }
    s.sort_by(f64::total_cmp);
    let rank = (p / 100.0).clamp(0.0, 1.0) * (s.len() - 1) as f64;
    let (lo, hi) = (rank.floor() as usize, rank.ceil() as usize);
    Some(s[lo] + (s[hi] - s[lo]) * (rank - lo as f64))
}

pub fn median(v: &[f64]) -> Option<f64> {
    percentile(v, 50.0)
}

pub fn range(v: &[f64]) -> Option<f64> {
    let max = v.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min = v.iter().copied().fold(f64::INFINITY, f64::min);
    (!v.is_empty()).then_some(max - min)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basics() {
        let v = [1.0, 2.0, 3.0, 4.0];
        assert_eq!(mean(&v), Some(2.5));
        assert_eq!(median(&v), Some(2.5));
        assert_eq!(percentile(&v, 100.0), Some(4.0));
        assert_eq!(range(&v), Some(3.0));
        assert!((std_dev(&v).unwrap() - 1.118034).abs() < 1e-6);
        assert_eq!(mean(&[]), None);
    }
}
