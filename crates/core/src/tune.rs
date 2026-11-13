//! Starting values for a new build, from its prop size and battery: the same
//! formulas as ArduPilot's initial tune parameters calculator (Mission
//! Planner's ConfigInitialParams), so a first hover and AutoTune start from
//! sensible filters and acceleration limits.

use serde::Serialize;

use crate::readiness::ParamChange;
use crate::setup::{change, pack, Chemistry, Param};

#[derive(Debug, Clone, Serialize)]
pub struct InitialTune {
    pub prop_in: f64,
    pub cells: u32,
    pub chemistry: Chemistry,
    /// Whether cells and chemistry came from MOT_BAT_VOLT_MAX/MIN.
    pub pack_from_vehicle: bool,
    /// The rate gains differ from the defaults: values from a tune would be
    /// overwritten.
    pub looks_tuned: bool,
    pub changes: Vec<ParamChange>,
    /// What to do after the first hover.
    pub after: Vec<String>,
}

/// The pack ArduTune would assume: from the vehicle, else 4S LiPo.
pub fn vehicle_pack(param: impl Fn(&str) -> Option<f64>) -> (u32, Chemistry, bool) {
    let p: Param = &param;
    pack(p).map_or((4, Chemistry::LiPo, false), |(c, k)| (c, k, true))
}

/// Mission Planner's RoundTo(value, -2): to the nearest hundred.
fn hundreds(v: f64) -> f64 {
    (v / 100.0).round() * 100.0
}

fn round_to(v: f64, places: i32) -> f64 {
    let f = 10f64.powi(places);
    (v * f).round() / f
}

pub fn initial_tune(param: impl Fn(&str) -> Option<f64>, prop_in: f64, cells: Option<u32>, chemistry: Option<Chemistry>) -> Result<InitialTune, String> {
    if !(2.0..=40.0).contains(&prop_in) {
        return Err("Prop size must be between 2 and 40 inches.".to_owned());
    }
    let p: Param = &param;
    let (vcells, vchem, known) = vehicle_pack(p);
    let cells = cells.unwrap_or(vcells);
    if !(1..=14).contains(&cells) {
        return Err("Cell count must be between 1 and 14.".to_owned());
    }
    let chemistry = chemistry.unwrap_or(vchem);
    let pack_from_vehicle = known && cells == vcells && chemistry == vchem;

    let accel_rp = hundreds(-2.613267 * prop_in.powi(3) + 343.39216 * prop_in.powi(2) - 15083.7121 * prop_in + 235771.0).max(10000.0);
    let accel_y = hundreds(-900.0 * prop_in + 36000.0).max(8000.0);
    let gyro = (289.22 * prop_in.powf(-0.838)).round().max(20.0);
    let rate_filter = (gyro / 2.0).max(10.0);
    let expo = round_to(0.15686 * prop_in.ln() + 0.23693, 2).min(0.8);
    let (cell_max, cell_min) = chemistry.cell_range();
    let n = f64::from(cells);

    let size = format!("{}\" props", crate::params::format_value(prop_in));
    let mut changes = Vec::new();
    let mut add = |name: &str, value: f64, reason: String| changes.extend(change(p, name, value, reason));
    add("INS_GYRO_FILTER", gyro, format!("Gyro low-pass for {size}: larger props are slower and noisier at lower frequencies"));
    // ArduPilot's AutoTune page asks for 10 Hz roll and pitch filters on
    // props of 13 inches and more; the calculator's half-gyro value is higher.
    let (rp_filter, why) =
        if prop_in >= 13.0 { (10.0, "10 Hz for props of 13 inches and more (ArduPilot's AutoTune advice)") } else { (rate_filter, "half the gyro filter") };
    for axis in ["RLL", "PIT"] {
        add(&format!("ATC_RAT_{axis}_FLTD"), rp_filter, format!("D-term filter: {why}"));
        add(&format!("ATC_RAT_{axis}_FLTT"), rp_filter, format!("Target filter: {why}"));
        add(&format!("ATC_RAT_{axis}_FLTE"), 0.0, "No error filter on roll and pitch".to_owned());
    }
    add("ATC_RAT_YAW_FLTD", 0.0, "No D-term filter on yaw".to_owned());
    add("ATC_RAT_YAW_FLTT", rate_filter, "Yaw target filter at half the gyro filter".to_owned());
    add("ATC_RAT_YAW_FLTE", 2.0, "Smooths the yaw error: yaw has little authority".to_owned());
    // ArduCopter 4.7 renamed the limits and changed them from cdeg/s² to deg/s².
    let accel = if p("ATC_ACCEL_R_MAX").is_some() {
        [("ATC_ACCEL_R_MAX", accel_rp, "cdeg/s²"), ("ATC_ACCEL_P_MAX", accel_rp, "cdeg/s²"), ("ATC_ACCEL_Y_MAX", accel_y, "cdeg/s²")]
    } else {
        [("ATC_ACC_R_MAX", accel_rp / 100.0, "deg/s²"), ("ATC_ACC_P_MAX", accel_rp / 100.0, "deg/s²"), ("ATC_ACC_Y_MAX", accel_y / 100.0, "deg/s²")]
    };
    for (name, value, units) in accel {
        add(name, value, format!("Acceleration limit for {size} ({} {units}); AutoTune refines it", crate::params::format_value(value)));
    }
    let acro_yaw_p = 0.5 * accel_y / 4500.0;
    if p("ACRO_YAW_P").is_some() {
        add("ACRO_YAW_P", round_to(acro_yaw_p, 2), "Acro yaw rate to match the yaw acceleration limit".to_owned());
    } else {
        // ACRO_Y_RATE replaced it: 4.5 was 202.5 deg/s.
        add("ACRO_Y_RATE", round_to(acro_yaw_p * 45.0, 1), "Acro yaw rate to match the yaw acceleration limit".to_owned());
    }
    add("ATC_THR_MIX_MAN", 0.1, "Low throttle-mix priority for the first flights; raise to 0.5 after a good hover".to_owned());
    add("INS_ACCEL_FILTER", 10.0, "Accelerometer filter for altitude hold".to_owned());
    add("MOT_THST_EXPO", expo, format!("Thrust curve typical for {size}"));
    let (max, min) = (round_to(cell_max * n, 2), round_to(cell_min * n, 2));
    add("MOT_BAT_VOLT_MAX", max, format!("{cells}S {} full ({cell_max} V per cell): keeps the response the same as the pack drains", chemistry.name()));
    add("MOT_BAT_VOLT_MIN", min, format!("{cells}S {} empty ({cell_min} V per cell)", chemistry.name()));

    let looks_tuned = p("ATC_RAT_RLL_P").is_some_and(|v| (v - 0.135).abs() > 1e-4) || p("ATC_RAT_PIT_P").is_some_and(|v| (v - 0.135).abs() > 1e-4);
    let (accz_p, accz_i) = if p("PSC_D_ACC_P").is_some() {
        ("PSC_D_ACC_P to 0.1 × MOT_THST_HOVER", "PSC_D_ACC_I to 0.2 × MOT_THST_HOVER")
    } else {
        ("PSC_ACCZ_P to MOT_THST_HOVER", "PSC_ACCZ_I to 2 × MOT_THST_HOVER")
    };
    let after = vec![
        "Make a first short hover in AltHold and check the vibration and noise with ArduTune's analysis before going further.".to_owned(),
        format!("Once MOT_THST_HOVER has learned the hover throttle, set {accz_p} and {accz_i}."),
        "Set ATC_THR_MIX_MAN to 0.5, then tune the filters from a hover log and run AutoTune.".to_owned(),
    ];
    Ok(InitialTune { prop_in, cells, chemistry, pack_from_vehicle, looks_tuned, changes, after })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn vehicle(extra: &[(&str, f64)]) -> HashMap<String, f64> {
        let mut m: HashMap<String, f64> = [
            "INS_GYRO_FILTER",
            "ATC_RAT_RLL_FLTD",
            "ATC_RAT_RLL_FLTT",
            "ATC_RAT_RLL_FLTE",
            "ATC_RAT_PIT_FLTD",
            "ATC_RAT_PIT_FLTT",
            "ATC_RAT_PIT_FLTE",
            "ATC_RAT_YAW_FLTD",
            "ATC_RAT_YAW_FLTT",
            "ATC_RAT_YAW_FLTE",
            "ATC_THR_MIX_MAN",
            "INS_ACCEL_FILTER",
            "MOT_THST_EXPO",
        ]
        .iter()
        .map(|n| (n.to_string(), -1.0))
        .collect();
        m.insert("ATC_RAT_RLL_P".into(), 0.135);
        m.insert("ATC_RAT_PIT_P".into(), 0.135);
        for (k, v) in extra {
            m.insert(k.to_string(), *v);
        }
        m
    }

    fn value(t: &InitialTune, name: &str) -> f64 {
        t.changes.iter().find(|c| c.name == name).unwrap_or_else(|| panic!("{name} not suggested")).value
    }

    #[test]
    fn large_props_get_10_hz_roll_pitch_filters() {
        let m = vehicle(&[]);
        let t = initial_tune(|n| m.get(n).copied(), 15.0, Some(8), Some(Chemistry::LiIon)).unwrap();
        for n in ["ATC_RAT_RLL_FLTD", "ATC_RAT_RLL_FLTT", "ATC_RAT_PIT_FLTD", "ATC_RAT_PIT_FLTT"] {
            assert_eq!(value(&t, n), 10.0, "{n}");
        }
        // Re-reading the written values as 32-bit floats proposes nothing.
        let written: std::collections::HashMap<String, f64> =
            m.iter().map(|(k, v)| (k.clone(), *v)).chain(t.changes.iter().map(|c| (c.name.clone(), f64::from(c.value as f32)))).collect();
        let again = initial_tune(|n| written.get(n).copied(), 15.0, Some(8), Some(Chemistry::LiIon)).unwrap();
        assert!(again.changes.is_empty(), "{:?}", again.changes);
    }

    #[test]
    fn nine_inch_4s_matches_mission_planner() {
        let m = vehicle(&[
            ("ATC_ACCEL_R_MAX", 0.0),
            ("ATC_ACCEL_P_MAX", 0.0),
            ("ATC_ACCEL_Y_MAX", 0.0),
            ("ACRO_YAW_P", 4.5),
            ("MOT_BAT_VOLT_MAX", 0.0),
            ("MOT_BAT_VOLT_MIN", 0.0),
        ]);
        let t = initial_tune(|n| m.get(n).copied(), 9.0, Some(4), Some(Chemistry::LiPo)).unwrap();
        assert_eq!(value(&t, "INS_GYRO_FILTER"), 46.0);
        assert_eq!(value(&t, "ATC_RAT_RLL_FLTD"), 23.0);
        assert_eq!(value(&t, "ATC_RAT_YAW_FLTE"), 2.0);
        assert_eq!(value(&t, "ATC_ACCEL_Y_MAX"), 27900.0);
        assert_eq!(value(&t, "ATC_ACCEL_R_MAX"), 125900.0);
        assert_eq!(value(&t, "ACRO_YAW_P"), 3.1);
        assert_eq!(value(&t, "MOT_THST_EXPO"), 0.58);
        assert_eq!(value(&t, "MOT_BAT_VOLT_MAX"), 16.8);
        assert_eq!(value(&t, "MOT_BAT_VOLT_MIN"), 13.2);
        assert!(!t.looks_tuned && !t.pack_from_vehicle);
    }

    #[test]
    fn copter_4_7_names_and_pack_from_vehicle() {
        // The KakuteH7: 8S Li-ion, ArduCopter 4.7 names, a tuned roll gain.
        let m = vehicle(&[
            ("ATC_ACC_R_MAX", 1100.0),
            ("ATC_ACC_P_MAX", 1100.0),
            ("ATC_ACC_Y_MAX", 270.0),
            ("ACRO_Y_RATE", 202.5),
            ("MOT_BAT_VOLT_MAX", 32.8),
            ("MOT_BAT_VOLT_MIN", 22.4),
            ("ATC_RAT_RLL_P", 0.08),
        ]);
        let t = initial_tune(|n| m.get(n).copied(), 5.0, None, None).unwrap();
        assert_eq!((t.cells, t.chemistry, t.pack_from_vehicle, t.looks_tuned), (8, Chemistry::LiIon, true, true));
        assert_eq!(value(&t, "INS_GYRO_FILTER"), 75.0);
        assert_eq!(value(&t, "ATC_ACC_R_MAX"), 1686.0);
        assert_eq!(value(&t, "ATC_ACC_Y_MAX"), 315.0);
        assert_eq!(value(&t, "ACRO_Y_RATE"), 157.5);
        // The pack is already right.
        assert!(t.changes.iter().all(|c| !c.name.starts_with("MOT_BAT")));
        assert!(t.after[1].contains("PSC_ACCZ_P"));
        assert!(initial_tune(|n| m.get(n).copied(), 0.0, None, None).is_err());
    }
}
