//! End-to-end tests of the MAVLink link against the simulated vehicle.

use std::time::Duration;

use ardutune_link::mock::{MockState, MockVehicle};
use ardutune_link::{plan, Error, Link};

const TIMEOUT: Duration = Duration::from_secs(3);

/// Deterministic pseudo-random bytes.
fn bytes(n: usize, seed: u32) -> Vec<u8> {
    let mut x = seed;
    (0..n)
        .map(|_| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (x >> 24) as u8
        })
        .collect()
}

fn connect(mock: &MockVehicle) -> Link {
    Link::connect(mock.client_address(), TIMEOUT).expect("connect to mock vehicle")
}

#[test]
fn connects_and_downloads_all_params_despite_drops() {
    let mut state = MockState::quad(Vec::new());
    state.drop_param_indices = [0, 3, 100, 250, 308].into();
    let total = state.params.len();
    let mock = MockVehicle::start(state).unwrap();
    let mut link = connect(&mock);

    let v = *link.vehicle();
    assert!(v.is_ardupilot());
    assert_eq!(v.type_name(), "quadcopter");
    assert!(!v.armed);

    let mut last = (0, 0);
    let params = link.fetch_params(&mut |got, of| last = (got, of)).unwrap();
    assert_eq!(params.len(), total);
    assert_eq!(last, (total, total));
    assert_eq!(params["LOG_BITMASK"].value, 176_126.0);
    assert!(params["LOG_BITMASK"].is_integer());
    assert!(!params["ATC_RAT_RLL_P"].is_integer());
}

#[test]
fn applies_and_verifies_changes() {
    let mock = MockVehicle::start(MockState::quad(Vec::new())).unwrap();
    let mut link = connect(&mock);
    let params = link.fetch_params(&mut |_, _| {}).unwrap();

    let wanted = vec![
        ("LOG_BITMASK".to_owned(), 180_223.0),
        ("INS_RAW_LOG_OPT".to_owned(), 9.0),
        ("ATC_RAT_RLL_P".to_owned(), 0.12),
        ("INS_GYRO_FILTER".to_owned(), 300.0), // vehicle clamps to 256
        ("MOT_THST_HOVER".to_owned(), 0.35),   // already set
        ("NOT_A_PARAM".to_owned(), 1.0),
    ];
    let p = plan(&params, &wanted);
    assert_eq!(p.unchanged, vec!["MOT_THST_HOVER"]);
    assert_eq!(p.unknown, vec!["NOT_A_PARAM"]);
    assert_eq!(p.changes.len(), 4);

    let mut seen = 0;
    let results = link.apply(&p.changes, &mut |_, _| seen += 1).unwrap();
    assert_eq!(seen, 4);
    assert_eq!(results[0].as_ref().unwrap(), &180_223.0);
    assert_eq!(results[1].as_ref().unwrap(), &9.0);
    assert!((results[2].as_ref().unwrap() - 0.12).abs() < 1e-6);
    match &results[3] {
        Err(Error::Rejected { name, got, .. }) => assert_eq!((name.as_str(), *got), ("INS_GYRO_FILTER", 256.0)),
        other => panic!("expected rejection, got {other:?}"),
    }

    let st = mock.state();
    assert_eq!(st.params["LOG_BITMASK"].0, 180_223.0);
    assert_eq!(st.params["INS_RAW_LOG_OPT"].0, 9.0);
}

#[test]
fn refuses_to_write_or_reboot_while_armed() {
    let mut state = MockState::quad(Vec::new());
    state.armed = true;
    let mock = MockVehicle::start(state).unwrap();
    let mut link = connect(&mock);
    assert!(link.vehicle().armed);
    let params = link.fetch_params(&mut |_, _| {}).unwrap();
    let p = plan(&params, &[("LOG_BITMASK".to_owned(), 180_223.0)]);

    assert!(matches!(link.apply(&p.changes, &mut |_, _| {}), Err(Error::Armed)));
    assert!(matches!(link.reboot(), Err(Error::Armed)));
    assert_eq!(mock.state().param_sets, 0);
    assert_eq!(mock.state().reboots, 0);

    mock.state().armed = false;
    link.reboot().unwrap();
    assert_eq!(mock.state().reboots, 1);
}

#[test]
fn lists_and_downloads_logs_with_lost_chunks() {
    // Second log size is not a multiple of the 90-byte chunk.
    let logs = vec![bytes(60_000, 1), bytes(12_345, 2)];
    let mut state = MockState::quad(logs.clone());
    state.drop_log_chunk_every = Some(37);
    let mock = MockVehicle::start(state).unwrap();
    let mut link = connect(&mock);

    let entries = link.list_logs().unwrap();
    assert_eq!(entries.iter().map(|e| (e.id, e.size)).collect::<Vec<_>>(), vec![(1, 60_000), (2, 12_345)]);

    for (entry, expected) in entries.iter().zip(&logs) {
        let mut last = 0;
        let data = link.download_log(entry, &mut |done, _| last = done).unwrap();
        assert_eq!(&data, expected, "log {}", entry.id);
        assert_eq!(last, expected.len());
    }
}

#[test]
fn empty_log_list() {
    let mock = MockVehicle::start(MockState::quad(Vec::new())).unwrap();
    let mut link = connect(&mock);
    assert!(link.list_logs().unwrap().is_empty());
}

#[test]
fn reports_missing_vehicle() {
    let port = std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let r = Link::connect(&format!("udpout:127.0.0.1:{port}"), Duration::from_millis(500));
    assert!(matches!(r, Err(Error::NoVehicle)), "{:?}", r.err());
}

#[test]
fn deletes_single_logs_by_file_and_erases_all() {
    let logs: Vec<Vec<u8>> = (0..4).map(|i| bytes(1000 + 300 * i, i as u32)).collect();
    let mut state = MockState::quad(logs.clone());
    state.first_log_num = 12;
    let mock = MockVehicle::start(state).unwrap();
    let mut link = connect(&mock);

    // The newest log may be open until the vehicle reboots.
    assert!(matches!(link.delete_log(4, false), Err(Error::NewestLogOpen(4))));

    // Log 2 is file 00000013.BIN; its entry stays, empty, like ArduPilot's.
    link.delete_log(2, false).unwrap();
    assert_eq!(mock.state().deleted, [1].into());
    let sizes: Vec<u32> = link.list_logs().unwrap().iter().map(|l| l.size).collect();
    assert_eq!(sizes, vec![1000, 0, 1600, 1900]);
    let entry = link.list_logs().unwrap()[2];
    assert_eq!(link.download_log(&entry, &mut |_, _| {}).unwrap(), logs[2]);

    // Deleting the oldest renumbers the rest from the next file.
    link.delete_log(1, false).unwrap();
    let listed = link.list_logs().unwrap();
    assert_eq!(listed.iter().map(|l| (l.id, l.size)).collect::<Vec<_>>(), vec![(1, 1600), (2, 1900)]);
    let mut link = link.reboot_and_reconnect(mock.client_address(), Duration::from_secs(10)).unwrap();
    assert_eq!(mock.state().reboots, 1);
    link.delete_log(2, true).unwrap();
    assert_eq!(mock.state().deleted, [0, 1, 3].into());
    assert!(matches!(link.delete_log(2, true), Err(Error::Command(_))));

    mock.state().armed = true;
    assert!(matches!(link.delete_log(1, true), Err(Error::Armed)));
    assert!(matches!(link.erase_all_logs(), Err(Error::Armed)));
    mock.state().armed = false;

    link.erase_all_logs().unwrap();
    assert!(link.list_logs().unwrap().is_empty());
    assert!(mock.state().logs.is_empty());
}

#[test]
fn reports_arming_status_and_joins_long_messages() {
    let mut state = MockState::quad(Vec::new());
    state.prearm_failures = vec!["Check mag field (xy diff:262>100)".into(), "AHRS: DCM Roll/Pitch inconsistent 20 deg. Wait or reboot".into()];
    state.gps = Some((mavlink::dialects::ardupilotmega::GpsFixType::GPS_FIX_TYPE_NO_FIX, 0, 99.99));
    state.mode = 5;
    state.rc[4] = 1900;
    let mock = MockVehicle::start(state).unwrap();
    let mut link = connect(&mock);

    let s = link.arming_status(Duration::from_millis(1200)).unwrap();
    assert_eq!(s.messages, ["PreArm: Check mag field (xy diff:262>100)", "PreArm: AHRS: DCM Roll/Pitch inconsistent 20 deg. Wait or reboot"]);
    assert_eq!(s.gps.len(), 1);
    assert_eq!((s.gps[0].fix_type, s.gps[0].satellites), (1, Some(0)));
    assert_eq!(s.custom_mode, Some(5));
    assert_eq!(s.rc.unwrap()[4], 1900);
    assert!(!s.armed);
}

#[test]
fn reads_parameter_defaults_over_ftp() {
    let mut state = MockState::quad(Vec::new());
    state.defaults.insert("ATC_RAT_RLL_SMAX".into(), 50.0);
    let total = state.params.len();
    let mock = MockVehicle::start(state).unwrap();
    let mut link = connect(&mock);
    let params = link.param_defaults().unwrap().expect("defaults offered");
    assert_eq!(params.len(), total);
    let smax = params.iter().find(|p| p.name == "ATC_RAT_RLL_SMAX").unwrap();
    assert_eq!((smax.value, smax.default), (0.0, Some(50.0)));
    let bitmask = params.iter().find(|p| p.name == "LOG_BITMASK").unwrap();
    assert_eq!(bitmask.value, 176_126.0);
}

#[test]
fn reads_firmware_info_and_reboots_to_bootloader() {
    let mock = MockVehicle::start(MockState::quad(Vec::new())).unwrap();
    let mut link = connect(&mock);
    let info = link.firmware_info().unwrap();
    assert_eq!(info.version.unwrap().to_string(), "4.7.0-dev");
    assert_eq!(info.board_id, Some(1048));
    assert_eq!(info.board.as_deref(), Some("KakuteH7-bdshot"));
    assert_eq!(info.git_hash.as_deref(), Some("d0b31066"));
    assert!(info.description.unwrap().starts_with("ArduCopter V4.7.0-dev"));
    assert_eq!(info.uid.as_deref(), Some("003A00243233510A33353533"));
    // The banner prints each 4-byte word of the ID most significant byte first.
    assert_eq!(info.serial.as_deref(), Some("24003A000A51333233353533"));
    assert_eq!(ardutune_core::history::serial_from_uid(info.uid.as_deref().unwrap()), info.serial);
    let started = std::time::Instant::now();
    let id = link.identity().unwrap();
    assert_eq!((id.uid, id.board), (info.uid.clone(), info.board.clone()));
    assert!(started.elapsed() < Duration::from_secs(1), "identity returns early");

    mock.state().armed = true;
    let link = match link.reboot_to_bootloader() {
        Err(Error::Armed) => connect(&mock),
        other => panic!("armed reboot not refused: {other:?}"),
    };
    mock.state().armed = false;
    link.reboot_to_bootloader().unwrap();
    assert_eq!(mock.state().bootloader_reboots, 1);
}

#[test]
fn runs_motor_tests_and_explains_refusals() {
    let mock = MockVehicle::start(MockState::quad(Vec::new())).unwrap();
    let mut link = connect(&mock);
    link.motor_test(2, 7.0, 1.0).unwrap();
    assert_eq!(mock.state().motor_tests, vec![(2, 7.0, 1.0)]);
    // The next one right away: the vehicle still says armed for the rest of
    // the first test, which isn't a reason to refuse.
    link.motor_test(1, 7.0, 0.5).unwrap();
    assert_eq!(mock.state().motor_tests.len(), 2);
    std::thread::sleep(Duration::from_millis(700));

    mock.state().motor_test_refusal = Some("RC not calibrated".to_owned());
    let err = link.motor_test(1, 7.0, 2.0).unwrap_err().to_string();
    assert!(err.contains("Motor Test: RC not calibrated"), "{err}");

    mock.state().motor_test_refusal = None;
    mock.state().armed = true;
    std::thread::sleep(Duration::from_millis(1200));
    assert!(matches!(link.motor_test(1, 7.0, 2.0), Err(Error::Armed)));
    assert_eq!(mock.state().motor_tests.len(), 2);
}

#[test]
fn calibrates_the_compass() {
    let mock = MockVehicle::start(MockState::quad(Vec::new())).unwrap();
    let mut link = connect(&mock);
    link.compass_cal_start().unwrap();
    let mut cal = ardutune_link::CompassCal::default();
    let mut seen_running = false;
    for _ in 0..20 {
        link.compass_cal_poll(&mut cal, Duration::from_millis(250)).unwrap();
        seen_running |= cal.compasses.first().is_some_and(|c| c.status == "running" && c.percent < 100);
        if cal.finished() {
            break;
        }
    }
    assert!(seen_running && cal.finished(), "{cal:?}");
    let c = &cal.compasses[0];
    assert!(c.ok && c.autosaved && c.percent == 100, "{c:?}");
    assert_eq!(c.offsets.map(|o| o[1]), Some(117.8));

    // Cancel a second run; a failure is reported as such.
    mock.state().mag_cal_failure = Some(mavlink::dialects::ardupilotmega::MagCalStatus::MAG_CAL_FAILED_RADIUS);
    link.compass_cal_start().unwrap();
    link.compass_cal_finish(false).unwrap();
    assert_eq!(mock.state().mag_cal_cancels, 1);
    mock.state().mag_cal_started = Some(std::time::Instant::now() - Duration::from_secs(5));
    let mut cal = ardutune_link::CompassCal::default();
    link.compass_cal_poll(&mut cal, Duration::from_millis(500)).unwrap();
    assert_eq!(cal.compasses[0].status, "bad_radius");
    assert!(cal.finished() && !cal.compasses[0].ok);
}
