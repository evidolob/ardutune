//! The web app's vehicle commands: the desktop app's commands (same names,
//! same JSON shapes) over a [`Link`] whose transport is Web Serial. What
//! needs the computer's files (backups, drone names, downloaded logs) isn't
//! here yet.

use std::collections::{BTreeMap, HashMap};
use std::time::Duration;

use ardutune_core::sources::{self, Wanted};
use ardutune_core::{arming, autotune, config, gps, motors, paramdoc, rc, serial, setup, switch, tune};
use ardutune_link::{plan, Link, Params};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::Loaded;

const SETUP_LISTEN: Duration = Duration::from_secs(3);

/// A connected vehicle.
pub struct Session {
    pub link: Link,
    pub params: Params,
    defaults: Option<HashMap<String, f32>>,
    uid: Option<String>,
    board: Option<String>,
    last_arming: Option<arming::Report>,
    last_gps: Option<gps::Page>,
    /// Changes shown for review, written by `apply_changes`.
    pending: Option<Vec<(String, f64)>>,
    gps_saw_no_fix: bool,
    gps_first_fix_s: Option<f64>,
}

impl Session {
    pub fn new(mut link: Link, params: Params) -> Self {
        let id = link.identity().ok();
        Session {
            link,
            params,
            defaults: None,
            uid: id.as_ref().and_then(|i| i.uid.clone()),
            board: id.and_then(|i| i.board),
            last_arming: None,
            last_gps: None,
            pending: None,
            gps_saw_no_fix: false,
            gps_first_fix_s: None,
        }
    }

    fn p(&self) -> impl Fn(&str) -> Option<f64> + '_ {
        |n| self.params.get(n).map(|p| p.value as f64)
    }

    pub fn info(&self) -> Value {
        let v = self.link.vehicle();
        let short = self.uid.as_deref().map(|u| &u[u.len().saturating_sub(6)..]);
        json!({
            "address": "webserial",
            "kind": v.type_name(),
            "ardupilot": v.is_ardupilot(),
            "system_id": v.system_id,
            "armed": v.armed,
            "params": self.params.len(),
            "uid": self.uid,
            "board": self.board,
            "name": Value::Null,
            "label": match (&self.board, short) {
                (Some(b), Some(s)) => Some(format!("{b} …{s}")),
                (Some(b), None) => Some(b.clone()),
                _ => None,
            },
        })
    }
}

fn to_value(v: impl Serialize) -> Result<Value, String> {
    serde_json::to_value(v).map_err(|e| e.to_string())
}

fn arg<T: for<'de> Deserialize<'de>>(args: &Value, name: &str) -> Result<T, String> {
    serde_json::from_value(args.get(name).cloned().unwrap_or(Value::Null)).map_err(|e| format!("{name}: {e}"))
}

#[derive(Deserialize, Default)]
struct Inputs {
    tune: Option<TuneInput>,
    observations: Option<Vec<motors::Observation>>,
    captured: Option<Vec<(u8, u16, u16, u16)>>,
}

#[derive(Deserialize)]
struct TuneInput {
    prop_in: f64,
    cells: Option<u32>,
    chemistry: Option<setup::Chemistry>,
}

/// Runs vehicle command `cmd` with its JSON `args`. `session` is `None`
/// while disconnected (and is cleared when the link drops).
pub fn call(session: &mut Option<Session>, cmd: &str, args: &Value, log: Option<&Loaded>) -> Result<Value, String> {
    match cmd {
        "list_ports" => return Ok(json!([])),
        "rc_choices" => {
            let (modes, options, tune) = rc::choices();
            let tune: Vec<_> = tune.into_iter().map(|(v, n)| (v, n, rc::tune_target(v))).collect();
            return Ok(json!({ "modes": modes, "options": options, "tune": tune, "knob_options": rc::KNOB_OPTIONS }));
        }
        "disconnect" => {
            *session = None;
            return Ok(Value::Null);
        }
        "vehicle_status" => {
            let Some(s) = session.as_mut() else { return Ok(Value::Null) };
            if let Err(e) = s.link.refresh_armed() {
                return gone(session, e);
            }
            return Ok(s.info());
        }
        _ => {}
    }
    let s = session.as_mut().ok_or("Not connected to a vehicle.")?;
    match run(s, cmd, args, log) {
        Err(Failure::Link(e)) => gone(session, e),
        Err(Failure::Other(e)) => Err(e),
        Ok(v) => Ok(v),
    }
}

/// A link error: a vanished connection forgets the session.
fn gone(session: &mut Option<Session>, e: ardutune_link::Error) -> Result<Value, String> {
    if matches!(e, ardutune_link::Error::Disconnected) {
        *session = None;
    }
    Err(e.to_string())
}

enum Failure {
    Link(ardutune_link::Error),
    Other(String),
}

impl From<ardutune_link::Error> for Failure {
    fn from(e: ardutune_link::Error) -> Self {
        Failure::Link(e)
    }
}

impl From<String> for Failure {
    fn from(e: String) -> Self {
        Failure::Other(e)
    }
}

impl From<&str> for Failure {
    fn from(e: &str) -> Self {
        Failure::Other(e.to_owned())
    }
}

fn run(s: &mut Session, cmd: &str, args: &Value, log: Option<&Loaded>) -> Result<Value, Failure> {
    Ok(match cmd {
        "param_table" => {
            if let Some(packed) = s.link.param_defaults()? {
                for p in &packed {
                    if let Some(param) = s.params.get_mut(&p.name) {
                        param.value = p.value;
                    }
                }
                s.defaults = Some(packed.into_iter().filter_map(|p| Some((p.name, p.default?))).collect());
            }
            let rows: Vec<Value> = s
                .params
                .iter()
                .map(|(name, p)| {
                    json!({
                        "name": name,
                        "value": p.value as f64,
                        "default": s.defaults.as_ref().and_then(|d| d.get(name)).map(|v| *v as f64),
                        "integer": p.is_integer(),
                        "doc": paramdoc::doc(name),
                    })
                })
                .collect();
            let docs = paramdoc::docs();
            json!({ "rows": rows, "has_defaults": s.defaults.is_some(), "docs_source": docs.source, "docs_fetched": docs.fetched })
        }
        "prepare_changes" => prepare(s, args, log)?,
        "apply_changes" => {
            let wanted = s.pending.take().ok_or("Nothing to write.")?;
            let names: Option<Vec<String>> = arg(args, "names")?;
            let mut changes = plan(&s.params, &wanted).changes;
            if let Some(names) = &names {
                changes.retain(|ch| names.contains(&ch.name));
            }
            let outcomes = if changes.is_empty() { vec![] } else { s.link.apply(&changes, &mut |_, _| {})? };
            let mut results = Vec::new();
            for (ch, r) in changes.iter().zip(outcomes) {
                match r {
                    Ok(v) => {
                        if let Some(p) = s.params.get_mut(&ch.name) {
                            p.value = v;
                        }
                        results.push(json!({ "name": ch.name, "value": v, "error": Value::Null }));
                    }
                    Err(e) => results.push(json!({ "name": ch.name, "value": Value::Null, "error": e.to_string() })),
                }
            }
            json!({ "backup": Value::Null, "results": results })
        }
        "reboot" => {
            s.link.reboot()?;
            return Err(Failure::Link(ardutune_link::Error::Disconnected));
        }
        "vehicle_config" => to_value(config::view(s.p()))?,
        "autotune_status" => to_value(autotune::view(s.p(), arg(args, "propIn")?, log.map(|l| &l.log)))?,
        "vehicle_pack" => {
            let (cells, chemistry, from_vehicle) = tune::vehicle_pack(s.p());
            json!({ "cells": cells, "chemistry": chemistry, "from_vehicle": from_vehicle })
        }
        "initial_tune" => {
            let t: TuneInput = arg(args, "tune")?;
            to_value(tune::initial_tune(s.p(), t.prop_in, t.cells, t.chemistry)?)?
        }
        "motor_layout" => to_value(motors::layout(s.p())?)?,
        "motor_test" => {
            let (seq, throttle, seconds): (u8, f32, f32) = (arg(args, "seq")?, arg(args, "throttle")?, arg(args, "seconds")?);
            s.link.motor_test(seq, throttle.clamp(3.0, 20.0), seconds.clamp(0.5, 3.0))?;
            Value::Null
        }
        "motor_check" => to_value(motors::check(s.p(), &arg::<Vec<motors::Observation>>(args, "observations")?)?)?,
        "rc_status" => {
            let sample = s.link.rc_sample(Duration::from_millis(250))?;
            let view = rc::view(s.p(), &sample.last);
            let rssi_pct = sample.rssi.map(|r| ((u32::from(r) * 100 + 127) / 254) as u8);
            let names = ["TUNE", "TUNE_MIN", "TUNE_MAX"].into_iter().chain((1..=61).filter_map(|t| rc::tune_target(f64::from(t))));
            let tuning: BTreeMap<String, f64> = names.filter_map(|n| s.params.get(n).map(|p| (n.to_owned(), p.value as f64))).collect();
            json!({ "view": view, "ranges": sample.ranges, "rssi_pct": rssi_pct, "tuning": tuning })
        }
        "detect_switch" => {
            let listen = Duration::from_secs_f64(arg::<Option<f64>>(args, "listenS")?.unwrap_or(6.0).clamp(1.0, 30.0));
            let moves = s.link.rc_ranges(listen)?;
            to_value(switch::flipped(s.p(), &moves, arg(args, "option")?)?)?
        }
        "gps_status" => {
            let sample = s.link.gps_sample(Duration::from_millis(600))?;
            let fixes: Vec<gps::Fix> = sample
                .gps
                .iter()
                .map(|g| gps::Fix {
                    instance: g.instance,
                    fix_type: g.fix_type,
                    satellites: g.satellites,
                    hdop: g.hdop,
                    speed: g.speed,
                    course: g.course,
                    position: g.position,
                    h_acc: g.h_acc,
                    v_acc: g.v_acc,
                })
                .collect();
            if !fixes.iter().any(|g| g.instance == 0 && g.fix_type >= 3) {
                s.gps_saw_no_fix = true;
            } else if s.gps_saw_no_fix && s.gps_first_fix_s.is_none() {
                s.gps_first_fix_s = sample.boot_s;
            }
            let live = gps::Live {
                gps: fixes,
                ekf: sample.ekf.map(|e| gps::Ekf { flags: e.flags, velocity: e.velocity, pos_horiz: e.pos_horiz, pos_vert: e.pos_vert, compass: e.compass }),
                armed: s.link.vehicle().armed,
                boot_s: sample.boot_s,
                first_fix_s: s.gps_first_fix_s,
                messages: sample.messages,
                last_known: None,
            };
            let page = gps::page(s.p(), &live);
            s.last_gps = Some(page.clone());
            to_value(page)?
        }
        "serial_status" => {
            let report = s.link.uart_info()?;
            let gps = s.link.gps_sample(Duration::from_millis(300))?;
            let rc = s.link.rc_sample(Duration::from_millis(450))?;
            let uarts = report.map(|t| serial::parse_uarts(&t)).unwrap_or_default();
            let live = serial::Live {
                gps_fix: gps.gps.iter().map(|g| (g.instance, gps::fix_label(g.fix_type))).collect(),
                rc_signal: Some(rc.last.iter().take(4).any(|v| *v > 0 && *v != u16::MAX)),
            };
            to_value(serial::view(s.p(), &uarts, &live))?
        }
        "arming_check" => {
            let listen = Duration::from_secs_f64(arg::<Option<f64>>(args, "listenS")?.unwrap_or(6.0).clamp(1.0, 120.0));
            let status = s.link.arming_status(listen)?;
            let report = arming::assess(&status.snapshot(), s.p());
            s.last_arming = Some(report.clone());
            to_value(report)?
        }
        "setup_guide" => {
            let status = s.link.arming_status(SETUP_LISTEN)?;
            to_value(setup::assess(s.p(), &status.setup_live()))?
        }
        other => return Err(Failure::Other(format!("{other} needs the ArduTune desktop app for now."))),
    })
}

/// Works out what writing a set of recommendations would change and keeps
/// it for `apply_changes`.
fn prepare(s: &mut Session, args: &Value, log: Option<&Loaded>) -> Result<Value, Failure> {
    let source: String = arg(args, "source")?;
    let inputs: Inputs = arg::<Option<Inputs>>(args, "inputs")?.unwrap_or_default();
    let values: Option<Vec<(String, f64)>> = arg(args, "values")?;
    let p = s.p();
    let wanted: Vec<Wanted> = match values {
        Some(values) if source == "edits" => values
            .into_iter()
            .map(|(n, v)| {
                let warnings = paramdoc::warnings(&n, v);
                (n, v, (!warnings.is_empty()).then(|| warnings.join("; ")))
            })
            .collect(),
        _ if source == "initial_tune" => {
            let t = inputs.tune.ok_or("No prop size given.")?;
            reasons(tune::initial_tune(&p, t.prop_in, t.cells, t.chemistry)?.changes)
        }
        _ if source == "rc_calibration" => reasons(rc::calibration(&p, &inputs.captured.ok_or("No radio calibration captured.")?)?),
        _ if source == "motors" => reasons(motors::check(&p, &inputs.observations.ok_or("No motor test results given.")?)?.changes),
        _ => {
            let ctx = sources::Context { last_arming: s.last_arming.as_ref(), last_gps: s.last_gps.as_ref() };
            match sources::from_params(&p, &source, &ctx) {
                Some(r) => r?,
                None => log_source(&p, &source, log)?,
            }
        }
    };
    drop(p);
    let pairs: Vec<(String, f64)> = wanted.iter().map(|(n, v, _)| (n.clone(), *v)).collect();
    let pl = plan(&s.params, &pairs);
    let rows: Vec<Value> = pl
        .changes
        .iter()
        .map(|ch| {
            let reason = wanted.iter().find(|(n, ..)| *n == ch.name).and_then(|(.., r)| r.clone());
            json!({ "name": ch.name, "current": ch.current.value as f64, "value": ch.value, "reason": reason })
        })
        .collect();
    s.pending = Some(pairs);
    Ok(json!({ "rows": rows, "unchanged": pl.unchanged, "unknown": pl.unknown, "skipped": [], "armed": s.link.vehicle().armed, "warning": Value::Null }))
}

fn reasons(changes: Vec<ardutune_core::readiness::ParamChange>) -> Vec<Wanted> {
    changes.into_iter().map(|c| (c.name, c.value, Some(c.reason))).collect()
}

/// Changes recommended by the open log's analyses.
fn log_source(p: &impl Fn(&str) -> Option<f64>, source: &str, log: Option<&Loaded>) -> Result<Vec<Wanted>, String> {
    let loaded = log.ok_or("Open a log first.")?;
    let not_run = |what: &str| format!("Run the {what} analysis first.");
    Ok(match source {
        "readiness" => reasons(loaded.readiness.recommendations_for(p)),
        "filters" | "filters_quieter" => {
            let r = loaded.filters.as_ref().ok_or_else(|| not_run("filter"))?;
            let changes = if source == "filters" { &r.changes } else { &r.quieter_changes };
            changes.iter().map(|c| (c.name.clone(), c.value, None)).collect()
        }
        "pid" | "pid_autotune" => {
            let r = loaded.pid.as_ref().ok_or_else(|| not_run("PID"))?;
            let chosen = if source == "pid" { &r.suggestions } else { &r.autotune };
            chosen.iter().map(|s| (s.name.clone(), s.value, Some(s.reason.clone()))).collect()
        }
        other => return Err(format!("{other} needs the ArduTune desktop app for now.")),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ardutune_link::mock::{MockState, MockVehicle};

    /// The web commands against the simulated vehicle, over the desktop
    /// transport: the same session code the browser runs.
    #[test]
    fn commands_against_the_simulated_vehicle() {
        let mock = MockVehicle::start(MockState::quad(Vec::new())).unwrap();
        let mut link = Link::connect(mock.client_address(), Duration::from_secs(5)).unwrap();
        let params = link.fetch_params(&mut |_, _| {}).unwrap();
        let mut session = Some(Session::new(link, params));
        let none = json!({});
        let info = call(&mut session, "vehicle_status", &none, None).unwrap();
        assert_eq!(info["board"], json!("KakuteH7-bdshot"), "{info}");
        let cfg = call(&mut session, "vehicle_config", &none, None).unwrap();
        assert_eq!(cfg["groups"][0]["title"], json!("Frame"));
        let plan = call(&mut session, "prepare_changes", &json!({ "source": "serial:4:esc" }), None).unwrap();
        assert_eq!(plan["rows"][0]["name"], json!("SERIAL4_PROTOCOL"), "{plan}");
        let applied = call(&mut session, "apply_changes", &none, None).unwrap();
        assert_eq!(applied["results"][0]["value"], json!(16.0), "{applied}");
        assert_eq!(mock.state().params["SERIAL4_PROTOCOL"].0, 16.0);
        let table = call(&mut session, "param_table", &none, None).unwrap();
        assert!(table["rows"].as_array().unwrap().len() > 100);
        assert!(call(&mut session, "list_vehicle_logs", &none, None).unwrap_err().contains("desktop app"));
        call(&mut session, "disconnect", &none, None).unwrap();
        assert!(call(&mut session, "vehicle_status", &none, None).unwrap().is_null());
    }
}
