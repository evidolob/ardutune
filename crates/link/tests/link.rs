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

    // Log 2 is file 00000013.BIN; its entry stays, empty, like ArduPilot's.
    link.delete_log(2).unwrap();
    assert_eq!(mock.state().deleted, [1].into());
    let sizes: Vec<u32> = link.list_logs().unwrap().iter().map(|l| l.size).collect();
    assert_eq!(sizes, vec![1000, 0, 1600, 1900]);
    let entry = link.list_logs().unwrap()[2];
    assert_eq!(link.download_log(&entry, &mut |_, _| {}).unwrap(), logs[2]);

    // Deleting the oldest renumbers the rest from the next file.
    link.delete_log(1).unwrap();
    let listed = link.list_logs().unwrap();
    assert_eq!(listed.iter().map(|l| (l.id, l.size)).collect::<Vec<_>>(), vec![(1, 1600), (2, 1900)]);
    link.delete_log(2).unwrap();
    assert_eq!(mock.state().deleted, [0, 1, 3].into());
    assert!(matches!(link.delete_log(2), Err(Error::Command(_))));

    mock.state().armed = true;
    assert!(matches!(link.delete_log(1), Err(Error::Armed)));
    assert!(matches!(link.erase_all_logs(), Err(Error::Armed)));
    mock.state().armed = false;

    link.erase_all_logs().unwrap();
    assert!(link.list_logs().unwrap().is_empty());
    assert!(mock.state().logs.is_empty());
}
