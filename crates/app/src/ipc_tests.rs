//! The commands end to end through Tauri's IPC layer (mock runtime), with a
//! simulated vehicle on the other end of the MAVLink link.

use std::path::Path;

use ardutune_core::synth::{sample_flight, Preset};
use ardutune_link::mock::{MockState, MockVehicle};
use serde_json::{json, Value};
use tauri::test::{get_ipc_response, mock_builder, INVOKE_KEY};
use tauri::webview::{InvokeRequest, WebviewWindowBuilder};

/// The app's own origin: app commands are only reachable from it.
#[cfg(any(windows, target_os = "android"))]
const APP_URL: &str = "http://tauri.localhost";
#[cfg(not(any(windows, target_os = "android")))]
const APP_URL: &str = "tauri://localhost";

fn invoke(webview: &tauri::WebviewWindow<tauri::test::MockRuntime>, cmd: &str, args: Value) -> Result<Value, String> {
    get_ipc_response(
        webview,
        InvokeRequest {
            cmd: cmd.into(),
            callback: tauri::ipc::CallbackFn(0),
            error: tauri::ipc::CallbackFn(1),
            url: APP_URL.parse().unwrap(),
            body: tauri::ipc::InvokeBody::Json(args),
            headers: Default::default(),
            invoke_key: INVOKE_KEY.to_string(),
        },
    )
    .map(|body| body.deserialize::<Value>().unwrap())
    .map_err(|e| e.as_str().map_or(e.to_string(), str::to_owned))
}

#[test]
fn analyse_connect_download_write_and_reboot() {
    let data = std::env::temp_dir().join(format!("ardutune-ipc-test-{}", std::process::id()));
    std::env::set_var("ARDUTUNE_DATA_DIR", &data);
    let mock = MockVehicle::start(MockState::quad(vec![sample_flight(Preset::DefaultLogging), sample_flight(Preset::Ready)])).unwrap();

    let app = crate::setup(mock_builder()).build(tauri::generate_context!()).unwrap();
    let webview = WebviewWindowBuilder::new(&app, "main", Default::default()).build().unwrap();
    let call = |cmd: &str, args: Value| invoke(&webview, cmd, args);

    // A log that is ready for tuning: every analysis runs.
    let overview = call("open_sample", json!({})).unwrap();
    assert_eq!(overview["readiness"]["filter_ready"], json!(true));
    for kind in ["diagnose", "noise", "filters", "pid"] {
        let v = call("analysis", json!({ "kind": kind })).unwrap_or_else(|e| panic!("{kind}: {e}"));
        assert!(v["report"].is_object(), "{kind}");
        assert_eq!(v["svg"].is_string(), kind != "diagnose", "{kind}");
    }
    let filters = call("analysis", json!({ "kind": "filters" })).unwrap();
    assert!(filters["report"]["current"]["description"].as_str().unwrap().contains("low-pass"));
    assert!(filters["report"]["current"].get("spectrum_db").is_none(), "plot data is left out");

    assert_eq!(call("prepare_changes", json!({ "source": "pid" })).unwrap_err(), "Not connected to a vehicle.");

    mock.state().params.insert("STAT_BOOTCNT".into(), (12.0, ardutune_link::ParamType::MAV_PARAM_TYPE_INT16));
    let info = call("connect", json!({ "address": mock.client_address() })).unwrap();
    assert_eq!(info["params"].as_u64().unwrap() as usize, mock.state().params.len());
    assert_eq!(info["armed"], json!(false));

    // Nothing blocks arming on the simulated quad, then a compass problem.
    let report = call("arming_check", json!({ "listen_s": 1.0 })).unwrap();
    assert_eq!(report["ready"], json!(true), "{report}");
    mock.state().prearm_failures = vec!["Check mag field (xy diff:262>100)".into()];
    let report = call("arming_check", json!({ "listen_s": 1.0 })).unwrap();
    assert_eq!(report["ready"], json!(false));
    assert_eq!(report["problems"][0]["area"], "Compass");
    mock.state().prearm_failures.clear();

    // The setup guide covers every step and proposes logging changes for the
    // simulated quad's default logging; they can be written like any other.
    let guide = call("setup_guide", json!({})).unwrap();
    let steps = guide["steps"].as_array().unwrap();
    assert_eq!(steps.len(), 7);
    let logging = steps.iter().find(|s| s["id"] == "logging").unwrap();
    assert!(!logging["changes"].as_array().unwrap().is_empty(), "{logging}");
    let plan = call("prepare_changes", json!({ "source": "setup:logging", "label": "writing logging for tuning" })).unwrap();
    assert!(!plan["rows"].as_array().unwrap().is_empty());

    // Initial tune from prop size: calculated, then planned like any write.
    let pack = call("vehicle_pack", json!({})).unwrap();
    assert!(pack["cells"].as_u64().unwrap() >= 1, "{pack}");
    let tune = json!({ "prop_in": 9.0, "cells": 4, "chemistry": "li_po" });
    let result = call("initial_tune", json!({ "tune": tune })).unwrap();
    let gyro = result["changes"].as_array().unwrap().iter().find(|c| c["name"] == "INS_GYRO_FILTER").map(|c| c["value"].clone());
    assert!(gyro.is_none() || gyro == Some(json!(46.0)), "{result}");
    let plan = call("prepare_changes", json!({ "source": "initial_tune", "label": "writing initial tune", "tune": tune })).unwrap();
    assert_eq!(plan["rows"].as_array().unwrap().len(), result["changes"].as_array().unwrap().len());
    assert!(call("initial_tune", json!({ "tune": { "prop_in": 0.0 } })).unwrap_err().contains("Prop size"));

    // The full parameter list: values, defaults, docs; edits are planned
    // with warnings for suspicious values, then written like anything else.
    let table = call("param_table", json!({})).unwrap();
    let rows = table["rows"].as_array().unwrap();
    assert_eq!(rows.len(), mock.state().params.len());
    assert_eq!(table["has_defaults"], json!(true));
    let mode = rows.iter().find(|r| r["name"] == "INS_HNTCH_MODE").unwrap();
    assert!(mode["doc"]["values"].as_array().unwrap().len() >= 4, "{mode}");
    let edits = json!([["ATC_RAT_RLL_P", 2.0], ["ATC_RAT_RLL_D", 0.004]]);
    let plan = call("prepare_changes", json!({ "source": "edits", "label": "writing parameter edits", "values": edits })).unwrap();
    let rows = plan["rows"].as_array().unwrap();
    let rll = rows.iter().find(|r| r["name"] == "ATC_RAT_RLL_P").unwrap();
    assert!(rll["reason"].as_str().unwrap().contains("outside the documented range"), "{plan}");
    let result = call("apply_changes", json!({ "names": ["ATC_RAT_RLL_D"] })).unwrap();
    assert_eq!(result["backup"], Value::Null, "no backup unless asked");
    assert!((mock.state().params["ATC_RAT_RLL_D"].0 - 0.004).abs() < 1e-6);

    // Firmware: what it runs, and firmware files checked against the board.
    let info = call("firmware_info", json!({})).unwrap();
    assert_eq!((info["version"].as_str(), info["board_id"].as_u64()), (Some("4.7.0-dev"), Some(1048)), "{info}");
    let good = data.join("good.apj");
    std::fs::create_dir_all(&data).unwrap();
    std::fs::write(&good, ardutune_link::firmware::make_apj(1048, &[1, 2, 3, 4, 5], "KakuteH7-bdshot")).unwrap();
    let prepared = call("firmware_prepare", json!({ "source": "file", "path": good })).unwrap();
    assert_eq!(prepared["matches_board"], json!(true));
    let wrong = data.join("wrong.apj");
    std::fs::write(&wrong, ardutune_link::firmware::make_apj(9, &[1, 2, 3, 4], "OtherBoard")).unwrap();
    let err = call("firmware_prepare", json!({ "source": "file", "path": wrong })).unwrap_err();
    assert!(err.contains("board ID 9"), "{err}");

    // Download the vehicle's log (default logging: not enough for tuning) and open it.
    let logs = call("list_vehicle_logs", json!({})).unwrap();
    let id = logs[0]["id"].clone();
    let path = call("download_vehicle_log", json!({ "id": id })).unwrap();
    let path = path.as_str().unwrap();
    assert!(Path::new(path).starts_with(data.join("logs")));
    let overview = call("open_log", json!({ "path": path })).unwrap();
    assert_eq!(overview["readiness"]["filter_ready"], json!(false));
    assert_eq!(
        call("prepare_changes", json!({ "source": "pid" })).unwrap_err(),
        "Run the PID analysis first.",
        "results of the previous log are gone"
    );

    // Write the logging recommendations: planned against live values, backed up, verified.
    let plan = call("prepare_changes", json!({ "source": "readiness" })).unwrap();
    let rows = plan["rows"].as_array().unwrap();
    assert!(rows.iter().any(|r| r["name"] == "INS_RAW_LOG_OPT"), "{plan}");
    let result = call("apply_changes", json!({ "backup": true })).unwrap();
    let results = result["results"].as_array().unwrap();
    assert_eq!(results.len(), rows.len());
    assert!(results.iter().all(|r| r["error"].is_null()), "{result}");
    assert!(Path::new(result["backup"].as_str().unwrap()).is_file());
    assert_eq!(mock.state().params["INS_RAW_LOG_OPT"].0, 9.0);
    assert_eq!(call("apply_changes", json!({})).unwrap_err(), "Nothing to write.");

    // Already applied: planning again finds nothing to change.
    let plan = call("prepare_changes", json!({ "source": "readiness" })).unwrap();
    assert!(plan["rows"].as_array().unwrap().is_empty(), "{plan}");

    // Delete the downloaded log, then erase what's left.
    let mock_logs = mock.state().logs.len();
    let newest = mock_logs as u16;
    let err = call("delete_vehicle_log", json!({ "id": newest })).unwrap_err();
    assert!(err.contains("newest log"), "{err}");
    let left = call("delete_vehicle_log", json!({ "id": newest, "rebootFirst": true })).unwrap();
    assert_eq!(left.as_array().unwrap().len(), mock_logs - 1);
    assert_eq!(mock.state().reboots, 1);
    assert_eq!(call("vehicle_status", json!({})).unwrap()["system_id"], json!(1), "still connected after the reboot");
    assert_eq!(call("erase_vehicle_logs", json!({})).unwrap(), json!([]));

    // Back up, change two values elsewhere, then restore just one of them.
    call("backup_params", json!({ "note": "Before maiden flight" })).unwrap();
    let backups = call("list_backups", json!({})).unwrap();
    let manual = backups.as_array().unwrap().iter().find(|b| b["reason"] == "Before maiden flight").unwrap().clone();
    assert!(backups.as_array().unwrap().iter().any(|b| b["reason"].as_str().unwrap().starts_with("Before writing")));
    {
        let mut st = mock.state();
        st.params.get_mut("ATC_RAT_RLL_SMAX").unwrap().0 = 30.0;
        st.params.get_mut("ATC_RAT_PIT_SMAX").unwrap().0 = 30.0;
        st.params.get_mut("STAT_BOOTCNT").unwrap().0 = 13.0;
    }
    call("backup_params", json!({})).unwrap(); // re-reads the vehicle
    let plan = call("prepare_changes", json!({ "source": "file", "path": manual["path"], "label": "restoring a backup" })).unwrap();
    let names: Vec<&str> = plan["rows"].as_array().unwrap().iter().map(|r| r["name"].as_str().unwrap()).collect();
    assert_eq!(names, ["ATC_RAT_PIT_SMAX", "ATC_RAT_RLL_SMAX"], "{plan}");
    assert_eq!(plan["skipped"], json!(["STAT_BOOTCNT"]));
    let result = call("apply_changes", json!({ "names": ["ATC_RAT_RLL_SMAX"], "backup": true })).unwrap();
    assert_eq!(result["results"].as_array().unwrap().len(), 1);
    assert_eq!(mock.state().params["ATC_RAT_RLL_SMAX"].0, 0.0);
    assert_eq!(mock.state().params["ATC_RAT_PIT_SMAX"].0, 30.0);
    let newest = &call("list_backups", json!({})).unwrap()[0];
    assert_eq!(newest["reason"], "Before restoring a backup");

    call("delete_backup", json!({ "path": manual["path"] })).unwrap();
    assert!(!Path::new(manual["path"].as_str().unwrap()).exists());
    assert!(call("delete_backup", json!({ "path": path })).is_err(), "only backups can be deleted");

    call("reboot", json!({})).unwrap();
    assert_eq!(mock.state().reboots, 2);
    assert_eq!(call("vehicle_status", json!({})).unwrap(), Value::Null);

    let _ = std::fs::remove_dir_all(&data);
}
