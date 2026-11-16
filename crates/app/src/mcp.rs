//! An MCP (Model Context Protocol) server in the app, so an AI assistant
//! (Claude or any MCP client) can read the vehicle, its logs and ArduTune's
//! analyses, and propose changes that open the app's usual review. Nothing
//! is written, armed, spun or flashed from the assistant's side: the user
//! confirms every write in the app.
//!
//! Streamable HTTP transport on 127.0.0.1 only, with a bearer token; off
//! until turned on in the app. JSON-RPC requests get JSON responses (no
//! server-sent events).

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use ardutune_core::{paramdoc, rawlog};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tauri::{AppHandle, Emitter, Manager, Runtime};

use crate::{analysis, data_dir, history, lock, vehicle, AppState, CmdResult};

pub const DEFAULT_PORT: u16 = 47653;
const PROTOCOL_VERSIONS: [&str; 3] = ["2025-06-18", "2025-03-26", "2024-11-05"];
const MAX_REQUEST: usize = 4 << 20;
/// Recent tool calls shown in the app.
const ACTIVITY: usize = 40;

#[derive(Serialize, Deserialize, Clone)]
struct Settings {
    enabled: bool,
    token: String,
    port: u16,
}

/// The server's state in `AppState`.
#[derive(Default)]
pub struct State {
    stop: Option<Arc<AtomicBool>>,
    port: Option<u16>,
    activity: VecDeque<Activity>,
    /// The result of the latest write, for `last_write_result`.
    pub last_write: Option<Value>,
}

#[derive(Serialize, Clone)]
struct Activity {
    time: String,
    tool: String,
    ok: bool,
}

#[derive(Serialize)]
pub struct Status {
    enabled: bool,
    running: bool,
    url: Option<String>,
    token: String,
    activity: Vec<Activity>,
}

fn settings_path<R: Runtime>(app: &AppHandle<R>) -> CmdResult<std::path::PathBuf> {
    Ok(data_dir(app, "")?.join("mcp.json"))
}

fn new_token() -> String {
    let mut b = [0u8; 16];
    getrandom::getrandom(&mut b).expect("system random numbers");
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn load<R: Runtime>(app: &AppHandle<R>) -> Settings {
    settings_path(app).ok().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|t| serde_json::from_str(&t).ok()).unwrap_or_else(|| Settings {
        enabled: false,
        token: new_token(),
        port: DEFAULT_PORT,
    })
}

fn store<R: Runtime>(app: &AppHandle<R>, s: &Settings) -> CmdResult<()> {
    let path = settings_path(app)?;
    std::fs::write(&path, serde_json::to_string_pretty(s).map_err(|e| e.to_string())?).map_err(|e| format!("writing {}: {e}", path.display()))
}

fn status<R: Runtime>(app: &AppHandle<R>) -> Status {
    let s = load(app);
    let state = app.state::<AppState>();
    let st = lock(&state.mcp);
    Status {
        enabled: s.enabled,
        running: st.stop.is_some(),
        url: st.port.map(|p| format!("http://127.0.0.1:{p}/mcp")),
        token: s.token,
        activity: st.activity.iter().rev().cloned().collect(),
    }
}

/// Starts the server when the app starts, if it was left on.
pub fn start_if_enabled<R: Runtime>(app: &AppHandle<R>) {
    let s = load(app);
    if s.enabled {
        let _ = start(app, &s);
    }
}

fn start<R: Runtime>(app: &AppHandle<R>, s: &Settings) -> CmdResult<()> {
    stop(app);
    let (listener, port) = (s.port..s.port.saturating_add(10))
        .find_map(|p| TcpListener::bind(("127.0.0.1", p)).ok().map(|l| (l, p)))
        .ok_or(format!("No free port from {} for the AI assistant server.", s.port))?;
    listener.set_nonblocking(true).map_err(|e| e.to_string())?;
    let flag = Arc::new(AtomicBool::new(false));
    {
        let state = app.state::<AppState>();
        let mut st = lock(&state.mcp);
        st.stop = Some(flag.clone());
        st.port = Some(port);
    }
    let app = app.clone();
    let token = s.token.clone();
    std::thread::spawn(move || {
        while !flag.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, _)) => {
                    let (app, token) = (app.clone(), token.clone());
                    std::thread::spawn(move || {
                        let _ = serve(&app, &token, stream);
                    });
                }
                Err(_) => std::thread::sleep(Duration::from_millis(100)),
            }
        }
    });
    Ok(())
}

fn stop<R: Runtime>(app: &AppHandle<R>) {
    let state = app.state::<AppState>();
    let mut st = lock(&state.mcp);
    if let Some(flag) = st.stop.take() {
        flag.store(true, Ordering::Relaxed);
    }
    st.port = None;
}

#[tauri::command]
pub fn mcp_status<R: Runtime>(app: AppHandle<R>) -> Status {
    status(&app)
}

#[tauri::command]
pub fn mcp_set_enabled<R: Runtime>(app: AppHandle<R>, enabled: bool) -> CmdResult<Status> {
    let mut s = load(&app);
    s.enabled = enabled;
    store(&app, &s)?;
    if enabled {
        start(&app, &s)?;
    } else {
        stop(&app);
    }
    Ok(status(&app))
}

/// A new token: clients set up with the old one stop working.
#[tauri::command]
pub fn mcp_new_token<R: Runtime>(app: AppHandle<R>) -> CmdResult<Status> {
    let mut s = load(&app);
    s.token = new_token();
    store(&app, &s)?;
    if s.enabled {
        start(&app, &s)?;
    }
    Ok(status(&app))
}

// ---- HTTP ----

fn respond(stream: &mut TcpStream, code: u16, reason: &str, body: Option<&str>) -> std::io::Result<()> {
    let body = body.unwrap_or("");
    let ctype = if body.is_empty() { "" } else { "Content-Type: application/json\r\n" };
    write!(stream, "HTTP/1.1 {code} {reason}\r\n{ctype}Content-Length: {}\r\nConnection: close\r\n\r\n{body}", body.len())?;
    stream.flush()
}

fn serve<R: Runtime>(app: &AppHandle<R>, token: &str, mut stream: TcpStream) -> std::io::Result<()> {
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(Duration::from_secs(30)))?;
    let mut buf = Vec::new();
    let mut chunk = [0u8; 8192];
    let header_end = loop {
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            return Ok(());
        }
        buf.extend_from_slice(&chunk[..n]);
        if let Some(i) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
            break i + 4;
        }
        if buf.len() > 64 << 10 {
            return respond(&mut stream, 431, "Request Header Fields Too Large", None);
        }
    };
    let head = String::from_utf8_lossy(&buf[..header_end]).into_owned();
    let mut lines = head.split("\r\n");
    let request_line = lines.next().unwrap_or_default();
    let mut parts = request_line.split_whitespace();
    let (method, path) = (parts.next().unwrap_or_default(), parts.next().unwrap_or_default());
    let header = |name: &str| {
        head.split("\r\n").skip(1).find_map(|l| l.split_once(':').filter(|(k, _)| k.trim().eq_ignore_ascii_case(name)).map(|(_, v)| v.trim().to_owned()))
    };
    // Only local pages may call it (DNS rebinding); clients send no Origin.
    if let Some(origin) = header("origin") {
        if !(origin.starts_with("http://127.0.0.1") || origin.starts_with("http://localhost")) {
            return respond(&mut stream, 403, "Forbidden", None);
        }
    }
    if header("authorization").as_deref() != Some(&format!("Bearer {token}")) {
        return respond(&mut stream, 401, "Unauthorized", Some(r#"{"error":"missing or wrong bearer token"}"#));
    }
    if path.split('?').next() != Some("/mcp") {
        return respond(&mut stream, 404, "Not Found", None);
    }
    if method != "POST" {
        // No server-sent event stream: every response comes with its request.
        return respond(&mut stream, 405, "Method Not Allowed", None);
    }
    let length: usize = header("content-length").and_then(|v| v.parse().ok()).unwrap_or(0);
    if length > MAX_REQUEST {
        return respond(&mut stream, 413, "Payload Too Large", None);
    }
    while buf.len() < header_end + length {
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&chunk[..n]);
    }
    let body = &buf[header_end..(header_end + length).min(buf.len())];
    let request: Value = match serde_json::from_slice(body) {
        Ok(v) => v,
        Err(e) => return respond(&mut stream, 400, "Bad Request", Some(&rpc_error(Value::Null, -32700, &format!("parse error: {e}")).to_string())),
    };
    let reply = match request {
        Value::Array(batch) => {
            let out: Vec<Value> = batch.into_iter().filter_map(|r| handle(app, r)).collect();
            (!out.is_empty()).then_some(Value::Array(out))
        }
        r => handle(app, r),
    };
    match reply {
        Some(v) => respond(&mut stream, 200, "OK", Some(&v.to_string())),
        None => respond(&mut stream, 202, "Accepted", None),
    }
}

// ---- JSON-RPC ----

fn rpc_error(id: Value, code: i64, message: &str) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": message } })
}

const INSTRUCTIONS: &str = "ArduTune is a desktop app for setting up, diagnosing and tuning ArduPilot multicopters. These tools read what the app reads: the connected vehicle (over USB), its parameters with ArduPilot's descriptions, live checks (setup guide, pre-arm, GPS, RC, serial ports, frame and hardware, AutoTune readiness) and flight logs with ArduTune's analyses (readiness, noise, filters, PIDs, crash diagnosis). \
Changing anything goes through propose_changes or propose_recommendation: the changes open in the app for the user to review, untick and confirm, with a backup and read-back; nothing is written until they do. Ask last_write_result afterwards. You cannot arm, run motors, reboot, calibrate or flash firmware: tell the user to do that in the app. \
Workflow: vehicle_status (connect_vehicle if needed), then the relevant check; for tuning, flight_history and analyse_log on the right flight. ArduTune's analyses can be wrong: when a result looks odd, check it against the raw messages with log_message_types, log_query and log_parameters. Make small changes, one area at a time, and say what to look for on the next flight.";

/// Handles one JSON-RPC message; `None` for notifications.
pub fn handle<R: Runtime>(app: &AppHandle<R>, request: Value) -> Option<Value> {
    let id = request.get("id").cloned();
    let method = request.get("method").and_then(Value::as_str).unwrap_or_default();
    let params = request.get("params").cloned().unwrap_or(Value::Null);
    let id = id?; // notifications (initialized, cancelled…) need no answer
    let result = match method {
        "initialize" => {
            let asked = params.get("protocolVersion").and_then(Value::as_str).unwrap_or(PROTOCOL_VERSIONS[0]);
            let version = PROTOCOL_VERSIONS.iter().find(|v| **v == asked).copied().unwrap_or(PROTOCOL_VERSIONS[0]);
            Ok(json!({
                "protocolVersion": version,
                "capabilities": { "tools": { "listChanged": false } },
                "serverInfo": { "name": "ardutune", "title": "ArduTune", "version": env!("CARGO_PKG_VERSION") },
                "instructions": INSTRUCTIONS,
            }))
        }
        "ping" => Ok(json!({})),
        "tools/list" => Ok(json!({ "tools": tools() })),
        "tools/call" => {
            let name = params.get("name").and_then(Value::as_str).unwrap_or_default().to_owned();
            let args = params.get("arguments").cloned().unwrap_or(json!({}));
            let outcome = call_tool(app, &name, &args);
            record(app, &name, outcome.is_ok());
            Ok(match outcome {
                Ok(v) => json!({ "content": [{ "type": "text", "text": serde_json::to_string_pretty(&v).unwrap_or_default() }], "isError": false }),
                Err(e) => json!({ "content": [{ "type": "text", "text": e }], "isError": true }),
            })
        }
        other => Err((-32601, format!("method not found: {other}"))),
    };
    Some(match result {
        Ok(r) => json!({ "jsonrpc": "2.0", "id": id, "result": r }),
        Err((code, msg)) => rpc_error(id, code, &msg),
    })
}

fn record<R: Runtime>(app: &AppHandle<R>, tool: &str, ok: bool) {
    let time = ardutune_core::utc::date_time(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs()));
    let entry = Activity { time, tool: tool.to_owned(), ok };
    {
        let state = app.state::<AppState>();
        let mut st = lock(&state.mcp);
        st.activity.push_back(entry.clone());
        while st.activity.len() > ACTIVITY {
            st.activity.pop_front();
        }
    }
    let _ = app.emit("mcp-activity", entry);
}

fn tool(name: &str, description: &str, properties: Value, required: &[&str]) -> Value {
    json!({
        "name": name,
        "description": description,
        "inputSchema": { "type": "object", "properties": properties, "required": required },
    })
}

fn tools() -> Vec<Value> {
    let none = json!({});
    vec![
        tool("vehicle_status", "The connected vehicle (type, board, flight controller ID, armed, parameter count), or null when none is connected.", none.clone(), &[]),
        tool(
            "connect_vehicle",
            "Connects to the vehicle and reads all its parameters (read-only). `address`: \"auto\" (the USB flight controller, default), serial:<port>:<baud>, udpin:0.0.0.0:14550 or tcpout:<host>:<port>.",
            json!({ "address": { "type": "string" } }),
            &[],
        ),
        tool(
            "get_parameters",
            "Parameters with value, default (when known) and ArduPilot's description. Filter by exact `names`, a `search` text (name or description), or `changed_only` (differs from default). At most `limit` (default 100).",
            json!({
                "names": { "type": "array", "items": { "type": "string" } },
                "search": { "type": "string" },
                "changed_only": { "type": "boolean" },
                "limit": { "type": "integer" }
            }),
            &[],
        ),
        tool("setup_guide", "The setup guide: each step (frame, ESC telemetry, radio, failsafes, battery, sensors, logging) with checks and proposed changes.", none.clone(), &[]),
        tool(
            "prearm_check",
            "Runs the vehicle's pre-arm checks, listens `listen_s` seconds (default 6) and explains what blocks arming, with fixes.",
            json!({ "listen_s": { "type": "number" } }),
            &[],
        ),
        tool("gps_status", "Each GPS live (fix, satellites, accuracy, position), the EKF's view, trust checks (spoofing, far from last position) and GPS settings.", none.clone(), &[]),
        tool("rc_status", "RC channels live: sticks, switches and their functions, flight modes per position, calibration and failsafe checks.", none.clone(), &[]),
        tool("serial_ports", "Serial ports: protocol, baud, live traffic and what's on each (GPS fix, receiver signal), with checks.", none.clone(), &[]),
        tool("frame_and_hardware", "Frame and layout, ESC protocol and firmware, bidirectional DShot, motor poles, spin armed/min, board orientation, battery monitor, with checks.", none.clone(), &[]),
        tool("motor_layout", "The frame's motors in test order, where each sits and which way it should spin.", none.clone(), &[]),
        tool(
            "autotune_status",
            "AutoTune readiness for the vehicle: steps, checks, AutoTune settings for `prop_in` (inches), QuikTune availability and what the open log's AutoTune saved.",
            json!({ "prop_in": { "type": "number" } }),
            &[],
        ),
        tool(
            "initial_tune",
            "Starting values for a build from its prop size (and optional `cells`, `chemistry`: li_po, li_po_hv, li_ion), compared with the vehicle.",
            json!({ "prop_in": { "type": "number" }, "cells": { "type": "integer" }, "chemistry": { "type": "string" } }),
            &["prop_in"],
        ),
        tool("list_vehicle_logs", "Logs on the vehicle's SD card: id, size and date (only when the vehicle's clock could be right).", none.clone(), &[]),
        tool(
            "download_vehicle_log",
            "Downloads log `id` from the vehicle to Documents/ArduTune/logs and opens it in the app for analysis. Big logs take minutes over USB.",
            json!({ "id": { "type": "integer" } }),
            &["id"],
        ),
        tool("flight_history", "Flights downloaded to this computer, grouped by drone and in order, with per-flight summaries (vibration, noise, hover, step response) and the parameter changes between flights.", none.clone(), &[]),
        tool("open_log", "Opens a DataFlash log file (`path`, e.g. from flight_history) in the app; returns its overview and readiness.", json!({ "path": { "type": "string" } }), &["path"]),
        tool(
            "analyse_log",
            "One analysis of the open log: `kind` = readiness, noise, filters (simulated filter settings with recommendations), pid (step response and suggestions) or diagnose (losses of control).",
            json!({ "kind": { "type": "string", "enum": ["readiness", "noise", "filters", "pid", "diagnose"] } }),
            &["kind"],
        ),
        tool(
            "log_message_types",
            "The open log's raw message types: rows, field names and time span of each (ATT, RATE, PIDR, GYR, IMU, VIBE, ESC, RCOU, BAT, CTUN, GPS, MSG, EV, MODE, ISBH/ISBD batches…), plus parse statistics. Use it to check ArduTune's analyses against the data.",
            none.clone(),
            &[],
        ),
        tool(
            "log_query",
            "Rows of one message type from the open log, as recorded (numbers in real units, time in seconds since boot). `fields` to pick columns (default all); `where` to keep rows whose fields equal values, e.g. {\"I\": 0} for the first IMU or {\"Instance\": 2} for an ESC; `from_s`/`to_s` for a time window; `max_rows` (default 200, at most 5000, evenly thinned beyond that); `stats: true` for count/min/max/mean/std per field instead of rows.",
            json!({
                "type": { "type": "string" },
                "fields": { "type": "array", "items": { "type": "string" } },
                "where": { "type": "object", "additionalProperties": { "type": "number" } },
                "from_s": { "type": "number" },
                "to_s": { "type": "number" },
                "max_rows": { "type": "integer" },
                "stats": { "type": "boolean" }
            }),
            &["type"],
        ),
        tool(
            "log_parameters",
            "The parameters recorded in the open log (as they were during that flight, not the vehicle's now), optionally only names containing `search`.",
            json!({ "search": { "type": "string" } }),
            &[],
        ),
        tool(
            "propose_changes",
            "Proposes parameter changes: they open in ArduTune for the user to review and confirm (with backup and read-back). Nothing is written until they do. `changes`: [{name, value, reason}]; `title` says what it's for.",
            json!({
                "title": { "type": "string" },
                "changes": { "type": "array", "items": { "type": "object", "properties": {
                    "name": { "type": "string" }, "value": { "type": "number" }, "reason": { "type": "string" } }, "required": ["name", "value"] } }
            }),
            &["title", "changes"],
        ),
        tool(
            "propose_recommendation",
            "Proposes one of ArduTune's own recommendations for review in the app, by `source`: filters, filters_quieter, pid, pid_autotune, readiness (from the open log), setup:<step id>, prearm:<area>, autotune:recommended:<prop inches>, serial:<n>:<setup>.",
            json!({ "source": { "type": "string" }, "title": { "type": "string" } }),
            &["source"],
        ),
        tool("last_write_result", "What the user wrote after the latest review: each parameter's verified value or error, and the backup made.", none, &[]),
    ]
}

fn block<T: Serialize>(f: impl std::future::Future<Output = CmdResult<T>>) -> Result<Value, String> {
    let v = tauri::async_runtime::block_on(f)?;
    serde_json::to_value(v).map_err(|e| e.to_string())
}

fn num(args: &Value, name: &str) -> Option<f64> {
    args.get(name).and_then(Value::as_f64)
}

fn call_tool<R: Runtime>(app: &AppHandle<R>, name: &str, args: &Value) -> Result<Value, String> {
    let app = app.clone();
    match name {
        "vehicle_status" => block(vehicle::vehicle_status(app)),
        "connect_vehicle" => {
            let address = args.get("address").and_then(Value::as_str).unwrap_or("auto").to_owned();
            let v = block(vehicle::connect(app.clone(), address))?;
            let _ = app.emit("vehicle-changed", &v);
            Ok(v)
        }
        "get_parameters" => parameters(&app, args),
        "setup_guide" => block(vehicle::setup_guide(app)),
        "prearm_check" => block(vehicle::arming_check(app, num(args, "listen_s"))),
        "gps_status" => block(vehicle::gps_status(app)),
        "rc_status" => block(vehicle::rc_status(app)),
        "serial_ports" => block(vehicle::serial_status(app)),
        "frame_and_hardware" => block(vehicle::vehicle_config(app)),
        "motor_layout" => block(vehicle::motor_layout(app)),
        "autotune_status" => block(vehicle::autotune_status(app, num(args, "prop_in"))),
        "initial_tune" => {
            let input = serde_json::from_value(json!({ "prop_in": num(args, "prop_in"), "cells": args.get("cells"), "chemistry": args.get("chemistry") }))
                .map_err(|e| e.to_string())?;
            block(vehicle::initial_tune(app, input))
        }
        "list_vehicle_logs" => block(vehicle::list_vehicle_logs(app)),
        "download_vehicle_log" => {
            let id = args.get("id").and_then(Value::as_u64).ok_or("give the log id")? as u16;
            let path = tauri::async_runtime::block_on(vehicle::download_vehicle_log(app.clone(), id))?;
            let overview = open(&app, path.clone())?;
            Ok(json!({ "path": path, "overview": overview }))
        }
        "flight_history" => block(history::flight_history(app)),
        "open_log" => {
            let path = args.get("path").and_then(Value::as_str).ok_or("give the log's path")?.to_owned();
            open(&app, path)
        }
        "analyse_log" => {
            let kind = args.get("kind").and_then(Value::as_str).ok_or("give the analysis kind")?;
            if kind == "readiness" {
                let state = app.state::<AppState>();
                let slot = lock(&state.log);
                let loaded = slot.as_ref().ok_or("Open a log first (open_log or download_vehicle_log).")?;
                return serde_json::to_value(&loaded.readiness).map_err(|e| e.to_string());
            }
            // The plot (SVG) is for the app's pages; the numbers are in the report.
            let v = block(analysis::analysis(app, kind.to_owned()))?;
            Ok(v.get("report").cloned().unwrap_or(v))
        }
        "log_message_types" | "log_query" | "log_parameters" => {
            let state = app.state::<AppState>();
            let slot = lock(&state.log);
            let loaded = slot.as_ref().ok_or("Open a log first (open_log or download_vehicle_log).")?;
            match name {
                "log_message_types" => Ok(rawlog::types(&loaded.log)),
                "log_query" => {
                    let q: rawlog::Query = serde_json::from_value(args.clone()).map_err(|e| format!("bad query: {e}"))?;
                    rawlog::query(&loaded.log, &q)
                }
                _ => Ok(rawlog::params(&loaded.log, args.get("search").and_then(Value::as_str))),
            }
        }
        "propose_changes" => {
            let title = args.get("title").and_then(Value::as_str).unwrap_or("changes from the AI assistant").to_owned();
            let changes = args.get("changes").and_then(Value::as_array).ok_or("give `changes`")?;
            let values: Vec<(String, f64)> = changes.iter().filter_map(|c| Some((c.get("name")?.as_str()?.to_owned(), c.get("value")?.as_f64()?))).collect();
            if values.is_empty() {
                return Err("No valid changes: each needs a name and a numeric value.".to_owned());
            }
            let reasons: Vec<(String, String)> =
                changes.iter().filter_map(|c| Some((c.get("name")?.as_str()?.to_owned(), c.get("reason")?.as_str()?.to_owned()))).collect();
            propose(&app, "edits".to_owned(), title, Some(values), reasons)
        }
        "propose_recommendation" => {
            let source = args.get("source").and_then(Value::as_str).ok_or("give the `source`")?.to_owned();
            let title = args.get("title").and_then(Value::as_str).map_or_else(|| source.clone(), str::to_owned);
            propose(&app, source, title, None, vec![])
        }
        "last_write_result" => {
            let state = app.state::<AppState>();
            let last = lock(&state.mcp).last_write.clone();
            Ok(last.unwrap_or(json!({ "result": "nothing written since the app started" })))
        }
        other => Err(format!("unknown tool {other}")),
    }
}

/// Opens a log in the app and tells the window, which shows it.
fn open<R: Runtime>(app: &AppHandle<R>, path: String) -> Result<Value, String> {
    let overview = tauri::async_runtime::block_on(analysis::open_log(app.clone(), path))?;
    let _ = app.emit("log-opened", &overview);
    Ok(overview)
}

/// Prepares the changes and shows the review in the app.
fn propose<R: Runtime>(
    app: &AppHandle<R>,
    source: String,
    title: String,
    values: Option<Vec<(String, f64)>>,
    reasons: Vec<(String, String)>,
) -> Result<Value, String> {
    let label = format!("writing {title} (proposed by the AI assistant)");
    let mut plan = block(vehicle::prepare_changes(app.clone(), source.clone(), None, Some(label), values, None))?;
    // The assistant's reasons, where the parameter docs gave none.
    if let Some(rows) = plan.get_mut("rows").and_then(Value::as_array_mut) {
        for row in rows {
            let name = row.get("name").and_then(Value::as_str).unwrap_or_default().to_owned();
            if let Some((_, why)) = reasons.iter().find(|(n, _)| *n == name) {
                let merged = match row.get("reason").and_then(Value::as_str) {
                    Some(warning) if !warning.is_empty() => format!("{why} ({warning})"),
                    _ => why.clone(),
                };
                row["reason"] = json!(merged);
            }
        }
    }
    let _ = app.emit("mcp-proposal", json!({ "title": title, "source": source, "plan": plan }));
    Ok(json!({
        "shown_for_review": true,
        "note": "The changes are open in ArduTune for the user to review and confirm; nothing is written until they do. Ask last_write_result afterwards.",
        "plan": plan,
    }))
}

fn parameters<R: Runtime>(app: &AppHandle<R>, args: &Value) -> Result<Value, String> {
    let state = app.state::<AppState>();
    let slot = lock(&state.vehicle);
    let c = slot.as_ref().ok_or("Not connected to a vehicle (connect_vehicle).")?;
    let names: Option<Vec<String>> = args.get("names").and_then(|n| serde_json::from_value(n.clone()).ok());
    let search = args.get("search").and_then(Value::as_str).map(str::to_lowercase);
    let changed_only = args.get("changed_only").and_then(Value::as_bool).unwrap_or(false);
    let limit = args.get("limit").and_then(Value::as_u64).unwrap_or(100) as usize;
    let mut out = Vec::new();
    let mut matched = 0usize;
    for (name, p) in c.params() {
        if names.as_ref().is_some_and(|n| !n.contains(name)) {
            continue;
        }
        let doc = paramdoc::doc(name);
        if let Some(q) = &search {
            let hit = name.to_lowercase().contains(q) || doc.is_some_and(|d| d.name.to_lowercase().contains(q) || d.desc.to_lowercase().contains(q));
            if !hit {
                continue;
            }
        }
        let default = c.default_of(name);
        if changed_only && default.is_none_or(|d| (d - p.value as f64).abs() < 1e-6) {
            continue;
        }
        matched += 1;
        if out.len() < limit {
            out.push(json!({
                "name": name,
                "value": p.value as f64,
                "default": default,
                "title": doc.map(|d| d.name.clone()),
                "description": doc.map(|d| d.desc.clone()),
                "units": doc.and_then(|d| d.units.clone()),
                "range": doc.and_then(|d| d.range),
                "values": doc.map(|d| d.values.clone()).filter(|v| !v.is_empty()),
                "reboot": doc.is_some_and(|d| d.reboot),
            }));
        }
    }
    if changed_only && c.defaults_unknown() {
        return Err("Defaults aren't loaded yet: open the Parameters page once (it reads them), or ask by names or search.".to_owned());
    }
    Ok(json!({ "matched": matched, "shown": out.len(), "parameters": out }))
}
