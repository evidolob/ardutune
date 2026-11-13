//! Settings shown as form controls (a list, ticks for a bitmask, or a
//! number), named and described from ArduPilot's parameter docs.

use serde::Serialize;

use crate::paramdoc;
use crate::setup::Param;

#[derive(Debug, Clone, Serialize)]
pub struct Setting {
    pub name: String,
    pub label: &'static str,
    pub value: Option<f64>,
    /// "select", "bits" or "number".
    pub kind: &'static str,
    pub options: Vec<(f64, String)>,
    pub unit: &'static str,
    /// A line of advice under the control; empty for none.
    pub help: &'static str,
    /// Takes effect after a reboot.
    pub reboot: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct Group {
    pub title: String,
    pub settings: Vec<Setting>,
}

/// The setting for parameter `name`, when the vehicle has it: a list when
/// the docs name its values (and there's no unit), ticks for a bitmask, a
/// number otherwise (the named values become hints).
pub fn setting(p: Param, name: &str, label: &'static str, unit: &'static str) -> Option<Setting> {
    let value = p(name);
    value?;
    let doc = paramdoc::doc(name);
    let bits = doc.map(|d| d.bits.iter().map(|(b, n)| (f64::from(*b), n.clone())).collect::<Vec<_>>()).unwrap_or_default();
    let values = doc.map(|d| d.values.clone()).unwrap_or_default();
    let (kind, options) = if !bits.is_empty() {
        ("bits", bits)
    } else if !values.is_empty() && unit.is_empty() {
        ("select", values)
    } else {
        ("number", values)
    };
    Some(Setting { name: name.to_owned(), label, value, kind, options, unit, help: "", reboot: doc.is_some_and(|d| d.reboot) })
}

impl Setting {
    pub fn help(mut self, help: &'static str) -> Self {
        self.help = help;
        self
    }

    /// A list of these choices instead of the docs' values.
    pub fn choices(mut self, options: Vec<(f64, String)>) -> Self {
        self.kind = "select";
        self.options = options;
        self
    }
}
