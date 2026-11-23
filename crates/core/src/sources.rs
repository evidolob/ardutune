//! Changes proposed by name ("autotune:recommended:15", "serial:4:gps"…),
//! worked out from the vehicle's live parameters. Shared by the desktop app
//! and the web app; sources that need a log or a file are theirs.

use crate::{arming, autotune, gps, logging, motors, rc, serial, setup, switch};

/// A parameter, the value to write and why.
pub type Wanted = (String, f64, Option<String>);

/// What the pages last showed, for sources that pick one of their problems.
#[derive(Default)]
pub struct Context<'a> {
    pub last_arming: Option<&'a arming::Report>,
    pub last_gps: Option<&'a gps::Page>,
}

fn wanted(changes: Vec<crate::readiness::ParamChange>) -> Vec<Wanted> {
    changes.into_iter().map(|c| (c.name, c.value, Some(c.reason))).collect()
}

fn numbers(s: &str) -> Result<Vec<f64>, String> {
    s.split(':').map(|x| x.parse::<f64>().map_err(|e| e.to_string())).collect()
}

/// The changes for `source`, or `None` when it isn't worked out from
/// parameters alone.
pub fn from_params(param: impl Fn(&str) -> Option<f64>, source: &str, ctx: &Context) -> Option<Result<Vec<Wanted>, String>> {
    let p = &param;
    let (kind, rest) = source.split_once(':')?;
    Some(match kind {
        // Fixes from the latest pre-arm check, for one area ("prearm:GPS").
        "prearm" => ctx.last_arming.ok_or_else(|| "Run the pre-arm check first.".to_owned()).map(|report| {
            report
                .problems
                .iter()
                .filter(|pr| pr.area == rest)
                .flat_map(|pr| pr.changes.iter().map(|ch| (ch.name.clone(), ch.value, Some(ch.reason.clone()))))
                .collect()
        }),
        // Fixes from the GPS page, by check: "gps:<index>".
        "gps" => ctx.last_gps.ok_or_else(|| "Open the GPS page first.".to_owned()).and_then(|page| {
            let check = rest.parse::<usize>().ok().and_then(|i| page.checks.get(i)).ok_or("That GPS problem is gone: look at the page again.")?;
            Ok(check.changes.iter().map(|ch| (ch.name.clone(), ch.value, Some(ch.reason.clone()))).collect())
        }),
        // "autotune:<part>:<prop inches>", part being a check id,
        // "recommended", "quiktune" or "after".
        "autotune" => {
            let (part, prop) = rest.split_once(':').unwrap_or((rest, ""));
            let v = autotune::view(p, prop.parse().ok(), None);
            match part {
                "recommended" => Ok(wanted(v.recommended)),
                "quiktune" => Ok(wanted(v.quiktune.changes)),
                "after" => Ok(wanted(v.after)),
                id => v
                    .checks
                    .into_iter()
                    .find(|c| c.id == id)
                    .map(|c| wanted(c.changes))
                    .ok_or("That AutoTune check no longer applies: look at the page again.".to_owned()),
            }
        }
        // A common serial port setup: "serial:<n>:<setup>".
        "serial" => (|| {
            let (n, id) = rest.split_once(':').ok_or("bad serial source")?;
            let n = n.parse::<u8>().map_err(|e| e.to_string())?;
            serial::setup(p, n, id).map(wanted)
        })(),
        // The reversed version of the frame layout: "frame:<FRAME_TYPE>".
        "frame" => rest.parse::<i64>().map_err(|e| e.to_string()).and_then(|t| motors::frame_change(p, t)).map(wanted),
        // A function on a switch: "switch:<channel>:<RCn_OPTION>".
        "switch" => numbers(rest).and_then(|n| match n[..] {
            [ch, option] => switch::assign(p, ch as u8, option).map(wanted),
            _ => Err("bad switch source".to_owned()),
        }),
        // Two functions on one three-position switch: "split:<a>:<up>:<b>:<down>".
        "split" => numbers(rest).and_then(|n| match n[..] {
            [a, up, b, down] => switch::split(p, (a as u8, up), (b as u8, down)).map(wanted),
            _ => Err("bad split source".to_owned()),
        }),
        // A tuning knob: "knob:<channel>:<TUNE>:<min>:<max>".
        "knob" => numbers(rest).and_then(|n| match n[..] {
            [ch, tune, min, max] => rc::tuning_knob(p, ch as u8, tune, min, max).map(wanted),
            _ => Err("bad knob source".to_owned()),
        }),
        // A logging preset: "logging:tuning" or "logging:normal".
        "logging" => {
            let v = logging::view(p);
            match rest {
                "tuning" => Ok(wanted(v.tuning)),
                "normal" => Ok(wanted(v.normal)),
                _ => Err(format!("unknown logging preset {rest}")),
            }
        }
        // A setup guide step's changes, from the live values.
        "setup" => {
            let guide = setup::assess(p, &setup::Live::default());
            guide.steps.into_iter().find(|s| s.id == rest).map(|s| wanted(s.changes)).ok_or(format!("unknown setup step {rest}"))
        }
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn param_sources() {
        let m: HashMap<String, f64> =
            [("SERIAL4_PROTOCOL", -1.0), ("SERIAL4_BAUD", 57.0), ("SERIAL4_OPTIONS", 0.0), ("AUTOTUNE_AGGR", 0.1), ("AUTOTUNE_AXES", 7.0)]
                .iter()
                .map(|(k, v)| (k.to_string(), *v))
                .collect();
        let p = |n: &str| m.get(n).copied();
        let ctx = Context::default();
        let serial = from_params(p, "serial:4:esc", &ctx).unwrap().unwrap();
        assert_eq!((serial[0].0.as_str(), serial[0].1), ("SERIAL4_PROTOCOL", 16.0));
        let at = from_params(p, "autotune:recommended:15", &ctx).unwrap().unwrap();
        assert_eq!((at[0].0.as_str(), at[0].1), ("AUTOTUNE_AGGR", 0.075));
        assert!(from_params(p, "prearm:GPS", &ctx).unwrap().unwrap_err().contains("pre-arm check"));
        assert!(from_params(p, "filters", &ctx).is_none(), "log sources aren't parameter sources");
        assert!(from_params(p, "file:x", &ctx).is_none());
    }
}
