// Stand-in for the Tauri bridge, to preview the UI in a browser without
// building the app. Log views come from dev/fixtures (export them with the
// ignored `export_fixtures` test); the vehicle is simulated.
//   python3 -m http.server -d crates/app 8765  →  http://localhost:8765/dev/preview.html
(() => {
  const listeners = {};
  const emit = (event, payload) => (listeners[event] || []).forEach((cb) => cb({ payload }));
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const fixture = async (name) => {
    const res = await fetch(`fixtures/${name}.json`);
    if (!res.ok) throw `No fixture ${name}.json: run the export_fixtures test first.`;
    return res.json();
  };
  const KAKUTE = "003A00243233510A33353533";
  const MATEK = "0041002B3138510D37393833";
  const names = { [MATEK]: "7-inch long range" };
  const vehicle = {
    address: "serial:/dev/cu.usbmodem14301:115200",
    kind: "quadrotor",
    ardupilot: true,
    system_id: 1,
    armed: false,
    params: 1233,
    uid: KAKUTE,
    board: "KakuteH7-bdshot",
    name: null,
    label: "KakuteH7-bdshot …353533",
  };
  const label = (uid, board) => names[uid] ?? `${board} …${uid.slice(-6)}`;
  let connected = false;
  const tagged = (path, saved, uid, board, reason, params = 1233) => ({ path, file_name: path.slice(3), saved, system_id: 1, reason, params, uid, board, vehicle: label(uid, board), match: null });
  let backups = [
    tagged("/b/k2.param", "2026-11-12 09:10:00", KAKUTE, "KakuteH7-bdshot", "Before writing initial tune for 5\" props"),
    tagged("/b/m1.param", "2026-11-11 18:00:00", MATEK, "MatekH7A3", "Before maiden flight", 1241),
    tagged("/b/k1.param", "2026-11-11 12:15:51", KAKUTE, "KakuteH7-bdshot", "Before flashing ArduCopter 4.7.2"),
    { path: "/b/2.param", file_name: "backup-sys1-20261110-153000.param", saved: "2026-11-10 15:30:00", system_id: 1, reason: "Before writing recommended filter settings", params: 1233, uid: null, match: { level: "same", why: "Its accelerometer calibration is identical to the connected vehicle's." } },
    { path: "/b/1.param", file_name: "backup-sys1-20261109-090000.param", saved: "2026-11-09 09:00:00", system_id: 1, reason: "Before maiden flight", params: 1231, uid: null, match: null },
    { path: "/b/mp.param", file_name: "matekh7a3_params.param", saved: null, system_id: null, reason: null, params: 1233, uid: null, match: null },
  ];
  const relabel = () => {
    backups = backups.map((b) => (b.uid ? { ...b, vehicle: label(b.uid, b.board) } : b));
    vehicle.name = names[KAKUTE] ?? null;
    vehicle.label = label(KAKUTE, vehicle.board);
  };
  let logs = [
    { id: 1, size_mb: 12.4, date: null },
    { id: 2, size_mb: 89.6, date: "2026-05-17 14:02:11" },
    { id: 3, size_mb: 31.2, date: "2026-05-18 09:40:05" },
  ];
  const progress = async (task, total) => {
    for (let done = 0; done <= total; done += total / 10) {
      emit("progress", { task, done: Math.round(done), total });
      await sleep(80);
    }
  };

  const tuneChanges = [
    { name: "INS_GYRO_FILTER", current: 20, value: 75, reason: 'Gyro low-pass for 5" props: larger props are slower and noisier at lower frequencies' },
    { name: "ATC_RAT_RLL_FLTD", current: 20, value: 37.5, reason: "D-term filter at half the gyro filter" },
    { name: "ATC_ACC_R_MAX", current: 1100, value: 1686, reason: 'Acceleration limit for 5" props (1686 deg/s²); AutoTune refines it' },
    { name: "ACRO_Y_RATE", current: 202.5, value: 157.5, reason: "Acro yaw rate to match the yaw acceleration limit" },
  ];
  let calStart = 0;
  // BetaFlight X, as on the KakuteH7.
  const motorLayout = {
    frame: "Quad BetaFlight X",
    known: true,
    dshot: true,
    positions: [
      { letter: "A", motor: 2, name: "front right", angle: 45, spin: "ccw", output: 2 },
      { letter: "B", motor: 1, name: "back right", angle: 135, spin: "cw", output: 1 },
      { letter: "C", motor: 3, name: "back left", angle: -135, spin: "ccw", output: 3 },
      { letter: "D", motor: 4, name: "front left", angle: -45, spin: "cw", output: 4 },
    ],
  };
  // Only direction mistakes are simulated.
  const motorVerdict = (seen) => {
    const wrong = seen.map((o, i) => [o, i]).filter(([o, i]) => o.spun === i && o.spin !== motorLayout.positions[i].spin);
    if (!wrong.length)
      return { ok: true, needs_reboot: false, changes: [], items: [{ status: "pass", title: "Every motor is in the right place and spins the right way", detail: "Fit the propellers to match." }] };
    const mask = wrong.reduce((m, [, i]) => m | (1 << (motorLayout.positions[i].output - 1)), 0);
    return {
      ok: false,
      needs_reboot: true,
      items: wrong.map(([o, i]) => ({ status: "fail", title: `The ${motorLayout.positions[i].name} motor spins the wrong way`, detail: "Reverse it in software (SERVO_BLH_RVMASK), or swap any two of its three motor wires." })),
      changes: [{ name: "SERVO_BLH_RVMASK", current: 0, value: mask, reason: "Reverse the motors (takes effect after a reboot)" }],
    };
  };
  const commands = {
    pick_file: async () => "/Users/pilot/logs/flight.bin",
    open_log: () => fixture("overview"),
    open_sample: () => fixture("overview"),
    analysis: async ({ kind }) => {
      await sleep(300);
      const v = await fixture(kind);
      if (v.error) throw v.error;
      return v;
    },
    list_ports: async () => [
      { name: "/dev/cu.usbmodem14301", description: "MatekH7A3", address: "serial:/dev/cu.usbmodem14301:115200", likely_autopilot: true },
      { name: "/dev/cu.Bluetooth-Incoming-Port", description: "", address: "serial:/dev/cu.Bluetooth-Incoming-Port:115200", likely_autopilot: false },
    ],
    connect: async () => {
      await progress("Reading parameters", 1233);
      connected = true;
      return vehicle;
    },
    disconnect: async () => (connected = false),
    vehicle_status: async () => (connected ? vehicle : null),
    list_vehicle_logs: async () => logs,
    delete_vehicle_log: async ({ id }) => {
      await sleep(400);
      logs = logs.filter((l) => l.id !== id).map((l, i) => ({ ...l, id: i + 1 }));
      return logs;
    },
    erase_vehicle_logs: async () => {
      await sleep(800);
      return (logs = []);
    },
    download_vehicle_log: async ({ id }) => {
      await progress("Downloading log", 89_600_000);
      return `/Users/pilot/Documents/ArduTune/logs/log_${id}.bin`;
    },
    prepare_changes: async ({ source, path, values, observations }) => {
      if (source === "edits") return { rows: values.map(([name, value]) => ({ name, current: 0, value, reason: null })), unchanged: [], unknown: [], skipped: [], armed: false };
      if (source.startsWith("setup:")) {
        const step = (await fixture("setup")).steps.find((x) => x.id === source.slice(6));
        return { rows: step.changes.map((c) => ({ ...c, current: c.current ?? 0 })), unchanged: [], unknown: [], skipped: [], armed: false };
      }
      if (source === "motors") return { rows: motorVerdict(observations).changes, unchanged: [], unknown: [], skipped: [], armed: false };
      if (source === "initial_tune") return { rows: tuneChanges, unchanged: [], unknown: [], skipped: [], armed: false };
      if (source === "file")
        return {
          rows: [
            { name: "ATC_RAT_PIT_D", current: 0.0058, value: 0.0045, reason: null },
            { name: "INS_GYRO_FILTER", current: 60, value: 30, reason: null },
            { name: "INS_HNTCH_BW", current: 52, value: 25, reason: null },
          ],
          unchanged: Array(1220).fill("X"),
          unknown: [],
          skipped: ["STAT_BOOTCNT", "STAT_RUNTIME", "STAT_FLTTIME", "INS_GYR_ID"],
          warning: backups.find((b) => b.path === path && b.uid && b.uid !== KAKUTE)
            ? `This backup was made on ${label(MATEK, "MatekH7A3")}, not on the connected vehicle (${vehicle.label}). Restoring it copies that vehicle's calibration and setup: untick anything that must stay, such as calibration (INS_ACC*, COMPASS_OFS*) and outputs.`
            : null,
          armed: false,
        };
      const kind = source.startsWith("filters") ? "filters" : source.startsWith("pid") ? "pid" : null;
      const r = kind ? (await fixture(kind)).report : null;
      const list =
        source === "filters" ? r.changes : source === "filters_quieter" ? r.quieter_changes : source === "pid" ? r.suggestions : source === "pid_autotune" ? r.autotune : [];
      return {
        rows: list.map((c) => ({ name: c.name, current: c.current ?? 0, value: c.value, reason: c.reason ?? null })),
        unchanged: [],
        unknown: [],
        skipped: [],
        armed: false,
      };
    },
    apply_changes: async ({ backup }) => {
      await progress("Writing parameters", 4);
      return { backup: backup ? "/Users/pilot/Documents/ArduTune/backups/backup-sys1-20261110-153000.param" : null, results: [{ name: "INS_GYRO_FILTER", value: 60, error: null }] };
    },
    reboot: async () => (connected = false),
    arming_check: async () => {
      await sleep(1500);
      return {
        armed: false,
        ready: false,
        mode: "Stabilize",
        gps: [{ instance: 0, fix: "3D fix", satellites: 26, hdop: 0.55 }],
        problems: [
          { status: "fail", area: "Compass", title: "The compass reading doesn't match the Earth's field here", messages: ["Check mag field (xy diff:117>100)"], advice: "Move away from metal, cars, laptops and phones and check again. If it still fails, calibrate the compass outdoors with the battery mounted as it flies, then reboot. COMPASS_ORIENT is 0, so the GPS/compass arrow must point forward." },
          { status: "fail", area: "Attitude", title: "The two attitude estimates disagree", messages: ["AHRS: DCM Roll/Pitch inconsistent 19 deg. Wait or reboot"], advice: "Reboot with the vehicle level and still. A bad compass can also tilt the EKF's attitude, so fix any compass problem first." },
          { status: "fail", area: "Logging", title: "Logging to the SD card failed", messages: ["Logging failed"], advice: "Insert an SD card formatted FAT32 and reboot." },
        ],
        how_to_arm: ["Arm switch on channel 5: flip it from low to high. It is off now.", "Sticks: throttle down and yaw right for 2 s to arm, yaw left to disarm."],
        messages: ["PreArm: Check mag field (xy diff:117>100)", "PreArm: Logging failed"],
      };
    },
    firmware_info: async () => ({
      description: "ArduCopter V4.7.0-dev (d0b31066)",
      version: "4.7.0-dev",
      git_hash: "d0b31066",
      board: "KakuteH7-bdshot",
      board_id: 1048,
      banner: ["ArduCopter V4.7.0-dev (d0b31066)", "ChibiOS: 1fa4b8e0", "KakuteH7-bdshot 003A00243233510A33353533", "RCOut: DS600:1-4"],
    }),
    firmware_releases: async () => {
      await sleep(600);
      return [
        { channel: "stable", version: "4.7.2", newer: true, error: null },
        { channel: "beta", version: "4.7.3-beta1", newer: true, error: null },
        { channel: "latest", version: "4.8.0-dev", newer: true, error: null },
      ];
    },
    firmware_prepare: async ({ source }) => {
      await progress("Downloading firmware", 1288447);
      return { label: `${source} firmware for KakuteH7-bdshot`, board_id: 1048, board_name: "KakuteH7-bdshot", size: 1440580, git_identity: "fb773567", matches_board: true, vehicle_board_id: 1048 };
    },
    firmware_flash: async () => {
      await progress("Writing firmware (keep USB connected)", 1440580);
      return { backup: null, version: "4.7.2", reconnected: true, changed: [{ name: "ATC_ACC_R_MAX", before: null, after: 78000 }, { name: "ATC_ACCEL_R_MAX", before: 78000, after: null }], vehicle };
    },
    param_table: async () => {
      await sleep(400);
      return fixture("params");
    },
    compass_cal_start: async () => {
      calStart = Date.now();
    },
    compass_cal_poll: async () => {
      await sleep(400);
      const frac = Math.min(1, (Date.now() - calStart) / 8000);
      const bits = Math.floor(frac * 80);
      const coverage = Array.from({ length: 10 }, (_, i) => (bits >= (i + 1) * 8 ? 255 : bits > i * 8 ? (1 << (bits - i * 8)) - 1 : 0));
      const done = frac >= 1;
      return {
        compasses: [
          { id: 0, status: done ? "success" : "running", done, ok: done, percent: Math.round(frac * 100), attempt: 1, coverage, fitness: done ? 7.4 : null, offsets: done ? [-38.6, 117.8, -129.7] : null, autosaved: done, advice: null },
          { id: 1, status: done ? "bad_radius" : "running", done, ok: false, percent: Math.round(frac * 90), attempt: 1, coverage, fitness: done ? 31 : null, offsets: null, autosaved: false, advice: done ? "The measured field strength is wrong for your location: strong interference (battery leads, motors, steel nearby) or a faulty compass." : null },
        ],
        messages: done ? ["Mag(0) good orientation: 0 1.0"] : [],
      };
    },
    compass_cal_finish: async () => {},
    motor_layout: async () => motorLayout,
    motor_test: async () => sleep(200),
    motor_check: async ({ observations }) => motorVerdict(observations),
    vehicle_pack: async () => ({ cells: 8, chemistry: "li_ion", from_vehicle: true }),
    initial_tune: async ({ tune }) => {
      await sleep(200);
      return {
        prop_in: tune.prop_in,
        cells: tune.cells,
        chemistry: tune.chemistry,
        pack_from_vehicle: true,
        looks_tuned: true,
        changes: tuneChanges,
        after: [
          "Make a first short hover in AltHold and check the vibration and noise with ArduTune's analysis before going further.",
          "Once MOT_THST_HOVER has learned the hover throttle, set PSC_ACCZ_P to MOT_THST_HOVER and PSC_ACCZ_I to 2 × MOT_THST_HOVER.",
          "Set ATC_THR_MIX_MAN to 0.5, then tune the filters from a hover log and run AutoTune.",
        ],
      };
    },
    setup_guide: async () => {
      await sleep(800);
      return fixture("setup");
    },
    list_backups: async () => backups,
    backup_params: async ({ note }) => {
      await progress("Reading parameters", 1233);
      backups.unshift(tagged(`/b/${Date.now()}.param`, "2026-11-12 12:00:00", KAKUTE, "KakuteH7-bdshot", note || "Manual backup"));
      return vehicle;
    },
    flight_history: async () => {
      await progress("Reading flight logs", 3);
      let n = 0;
      const flight = (when, mins, vib, pre, post, ov, osc, changes, gps = true, autotune = false) => ({
        path: `/logs/${when}.bin`,
        file_name: `${when}.bin`,
        size_mb: 60,
        number: 0,
        when: gps ? when : null,
        summary: {
          boot_count: 170 - n++,
          after_boot_s: 38 + n * 7,
          flight_s: mins * 60,
          vibration: vib,
          noise_pre: pre,
          noise_post: post,
          autotune,
          axes: ["roll", "pitch", "yaw"].map((axis, i) => ({ axis, overshoot_pct: ov[i], rise_ms: 40, oscillation_hz: i === 1 ? osc : null })),
        },
        changes,
      });
      const numbered = (list) => list.map((f, i) => ({ ...f, number: list.length - i }));
      return [
        {
          id: KAKUTE,
          from_log: false,
          label: label(KAKUTE, "KakuteH7-bdshot"),
          named: !!names[KAKUTE],
          board: "KakuteH7-bdshot",
          connected,
          flights: numbered([
            flight("2026-11-14 10:20:05", 6.1, 7.9, 9.1, 1.4, [9, 12, 15], null, [["ATC_RAT_PIT_D", 0.0052, 0.0045], ["ATC_RAT_PIT_P", 0.11, 0.1]]),
            flight("2026-11-13 16:02:40", 5.4, 8.2, 9.3, 1.9, [14, 31, 22], 32, [["INS_HNTCH_MODE", 1, 3], ["INS_HNTCH_REF", 0, 1], ["INS_GYRO_FILTER", 20, 75]], true, true),
            flight("2026-11-13 11:47:12", 4.2, 8.0, 9.0, 3.6, [18, 45, 35], 32, [["INS_LOG_BAT_MASK", 0, 1], ["INS_RAW_LOG_OPT", 0, 9]]),
            flight("2026-11-12 18:30:00", 2.1, 12.5, null, null, [], null, [], false),
          ]),
        },
        {
          id: "0035003D4B564E0C20204232",
          from_log: true,
          label: names["0035003D4B564E0C20204232"] ?? "MatekH7A3 …204232",
          named: !!names["0035003D4B564E0C20204232"],
          board: "MatekH7A3",
          connected: false,
          flights: numbered([flight("2026-05-17 11:58:47", 6.4, 4.8, 8.7, 2.1, [17.7, 45.4, 35], 32, [], true, true)]),
        },
      ];
    },
    import_log: async ({ path }) => path,
    name_drone: async ({ uid, name }) => {
      names[uid] = name.trim();
      relabel();
      return connected ? { ...vehicle } : null;
    },
    assign_backup: async ({ path }) => {
      backups = backups.map((b) => (b.path === path ? { ...b, uid: KAKUTE, board: vehicle.board, vehicle: vehicle.label, match: null } : b));
    },
    delete_backup: async ({ path }) => {
      backups = backups.filter((b) => b.path !== path);
    },
  };

  window.__TAURI__ = {
    core: {
      invoke: async (cmd, args) => {
        if (!commands[cmd]) throw `mock: unknown command ${cmd}`;
        return commands[cmd](args || {});
      },
    },
    event: {
      listen: async (event, cb) => {
        (listeners[event] ||= []).push(cb);
        return () => {};
      },
    },
  };
})();
