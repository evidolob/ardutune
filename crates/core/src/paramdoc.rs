//! ArduPilot's Copter parameter metadata (descriptions, units, ranges,
//! allowed values, bitmask bits), bundled so it works offline. Regenerate
//! with `tools/update-param-docs.py`.

use std::collections::HashMap;
use std::io::Read;
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};

use crate::params::format_value;

const BUNDLED: &[u8] = include_bytes!("../data/copter-params.json.gz");

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Doc {
    /// Short display name.
    pub name: String,
    pub desc: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub units: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub range: Option<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub step: Option<f64>,
    /// Allowed values and their names.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub values: Vec<(f64, String)>,
    /// Bit numbers and their names, for bitmask parameters.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bits: Vec<(u32, String)>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub reboot: bool,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub readonly: bool,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub advanced: bool,
}

#[derive(Debug, Deserialize)]
pub struct Docs {
    /// Where the metadata came from, and when.
    pub source: String,
    pub fetched: String,
    pub params: HashMap<String, Doc>,
}

/// The bundled metadata, parsed on first use.
pub fn docs() -> &'static Docs {
    static DOCS: OnceLock<Docs> = OnceLock::new();
    DOCS.get_or_init(|| {
        let mut json = String::new();
        flate2::read::GzDecoder::new(BUNDLED).read_to_string(&mut json).expect("bundled parameter metadata is gzip");
        serde_json::from_str(&json).expect("bundled parameter metadata is valid")
    })
}

pub fn doc(name: &str) -> Option<&'static Doc> {
    docs().params.get(name)
}

/// Reasons to double-check a value before writing it. ArduPilot's ranges
/// are advice (the firmware may accept more), so these warn rather than block.
pub fn warnings(name: &str, value: f64) -> Vec<String> {
    let Some(d) = doc(name) else { return vec![] };
    let mut out = Vec::new();
    if d.readonly {
        out.push(format!("{name} is read-only: the vehicle will ignore the change"));
    }
    if !d.values.is_empty() && d.bits.is_empty() {
        if !d.values.iter().any(|(v, _)| *v == value) {
            out.push(format!("{} is not one of the documented values for {name}", format_value(value)));
        }
    } else if let Some([lo, hi]) = d.range {
        if value < lo || value > hi {
            out.push(format!(
                "{} is outside the documented range {} to {}",
                format_value(value),
                format_value(lo),
                format_value(hi)
            ));
        }
    }
    if !d.bits.is_empty() {
        let known: u64 = d.bits.iter().fold(0, |m, (b, _)| m | 1u64.checked_shl(*b).unwrap_or(0));
        if value < 0.0 || value.fract() != 0.0 || (value as u64) & !known != 0 {
            out.push(format!("{} sets bits {name} doesn't document", format_value(value)));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_metadata_loads() {
        let d = docs();
        assert!(d.params.len() > 5000);
        let mode = doc("INS_HNTCH_MODE").unwrap();
        assert!(mode.values.iter().any(|(v, n)| *v == 3.0 && n == "ESC Telemetry"));
        let ref_doc = doc("INS_HNTCH_REF").unwrap();
        assert!(ref_doc.reboot);
        assert!(doc("RC_OPTIONS").unwrap().bits.iter().any(|(b, n)| *b == 8 && n.contains("passthrough")));
        assert_eq!(doc("ATC_RAT_RLL_P").unwrap().units.as_deref(), Some("s/rad"));
    }

    #[test]
    fn warns_about_suspicious_values() {
        assert!(warnings("INS_HNTCH_MODE", 3.0).is_empty());
        assert_eq!(warnings("INS_HNTCH_MODE", 9.0).len(), 1);
        assert!(warnings("ATC_RAT_RLL_P", 0.1).is_empty());
        assert!(warnings("ATC_RAT_RLL_P", 2.0)[0].contains("outside the documented range"));
        assert!(warnings("RC_OPTIONS", 8480.0).is_empty());
        assert!(!warnings("RC_OPTIONS", 1.5).is_empty());
        assert!(warnings("NOT_A_PARAM", 1.0).is_empty());
    }
}
