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
    frame_type: 12,
    reversed: [18, "BetaFlight X reversed (props out)"],
    warning: null,
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
    prepare_changes: async ({ source, path, values, inputs }) => {
      const observations = inputs?.observations;
      if (source === "edits") return { rows: values.map(([name, value]) => ({ name, current: 0, value, reason: null })), unchanged: [], unknown: [], skipped: [], armed: false };
      if (source.startsWith("setup:")) {
        const step = (await fixture("setup")).steps.find((x) => x.id === source.slice(6));
        return { rows: step.changes.map((c) => ({ ...c, current: c.current ?? 0 })), unchanged: [], unknown: [], skipped: [], armed: false };
      }
      if (source === "motors") return { rows: motorVerdict(observations).changes, unchanged: [], unknown: [], skipped: [], armed: false };
      if (source === "prearm:GPS") return { rows: (await commands.arming_check()).problems[0].changes, unchanged: [], unknown: [], skipped: [], armed: false };
      if (source.startsWith("switch:")) {
        const [, ch, opt] = source.split(":");
        return { rows: [{ name: `RC${ch}_OPTION`, current: 0, value: Number(opt), reason: `Channel ${ch}` }], unchanged: [], unknown: [], skipped: [], armed: false };
      }
      if (source === "rc_calibration")
        return { rows: inputs.captured.flatMap(([ch, lo, hi, c]) => [{ name: `RC${ch}_MIN`, current: 988, value: lo, reason: `Channel ${ch} lowest` }, { name: `RC${ch}_MAX`, current: 2011, value: hi, reason: `Channel ${ch} highest` }]), unchanged: [], unknown: [], skipped: [], armed: false };
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
    rc_choices: async () => ({
      modes: [[0, "Stabilize"], [1, "Acro"], [2, "AltHold"], [3, "Auto"], [5, "Loiter"], [6, "RTL"], [9, "Land"], [16, "PosHold"]],
      options: [[4, "RTL"], [17, "AUTOTUNE Mode"], [31, "Motor Emergency Stop"], [65, "GPS Disable"], [9, "Camera Trigger"], [153, "ArmDisarm"], [90, "EKF Source Set"], [212, "Mount1 Roll"], [213, "Mount1 Pitch"], [214, "Mount1 Yaw"], [219, "Transmitter Tuning"]],
      tune: [[1, "Stab Roll/Pitch kP", "ATC_ANG_RLL_P"], [4, "Rate Roll/Pitch kP", "ATC_RAT_RLL_P"], [5, "Rate Roll/Pitch kI", "ATC_RAT_RLL_I"], [21, "Rate Roll/Pitch kD", "ATC_RAT_RLL_D"], [6, "Rate Yaw kP", "ATC_RAT_YAW_P"], [38, "Declination", null]],
      knob_options: [219, 212, 213, 214],
    }),
    rc_status: async () => {
      await sleep(250);
      const t = Date.now() / 1000;
      const wave = (f, a = 450) => Math.round(1500 + a * Math.sin(t * f));
      const sw3 = Math.floor(t / 2) % 3; // a 3-position switch split over channels 7 and 8
      const values = [wave(0.9), wave(0.7), Math.round(1250 + 250 * Math.sin(t * 0.5)), wave(1.1, 200), 999, 1550, sw3 === 0 ? 2011 : 988, sw3 === 2 ? 2011 : 988, 999, 999, wave(0.4, 480), 1500, 880, 880, 2011, 2011];
      const roles = [["stick", "Roll"], ["stick", "Pitch"], ["stick", "Throttle"], ["stick", "Yaw"], ["switch", "ArmDisarm", 153], ["mode", "Flight mode"], ["free", ""], ["free", ""]];
      const positions = { 153: ["disarm", "no change", "arm"] };
      const channels = Array.from({ length: 16 }, (_, i) => {
        const [kind, role, option] = roles[i] ?? ["free", ""];
        const value = values[i] ?? null;
        const position = kind === "switch" && value ? (value < 1200 ? "low" : value > 1800 ? "high" : "middle") : null;
        return { ch: i + 1, value, min: i < 8 ? 988 : null, max: i < 8 ? 2011 : null, trim: i < 8 ? (i === 2 ? 988 : 1500) : null, reversed: i === 1, kind, role, option: option ?? null, position, positions: positions[option] ?? null };
      });
      const modes = [[0, "Stabilize"], [0, "Stabilize"], [2, "AltHold"], [2, "AltHold"], [5, "Loiter"], [5, "Loiter"]].map(([mode, name], i) => ({
        position: i + 1,
        band: [[0, 1230], [1231, 1360], [1361, 1490], [1491, 1620], [1621, 1749], [1750, 2200]][i],
        mode,
        name,
        active: i === 3,
      }));
      return { view: { channels, mode_channel: 6, modes, notes: [], signal: true }, ranges: channels.map((c) => [c.ch, c.value ?? 0, c.value ?? 0]), rssi_pct: 87, tuning: { TUNE: 0, TUNE_MIN: 0, TUNE_MAX: 0, ATC_RAT_RLL_P: 0.135, ATC_RAT_RLL_I: 0.135, ATC_RAT_RLL_D: 0.0036, ATC_ANG_RLL_P: 4.5, ATC_RAT_YAW_P: 0.18 } };
    },
    vehicle_config: async () => {
      await sleep(200);
      const place = { 0: "front", 45: "front right", 90: "right", 135: "back right", 180: "back", "-135": "back left", "-90": "left", "-45": "front left" };
      const quad = (rows) => rows.map(([motor, angle, spin], i) => ({ letter: "ABCD"[i], motor, name: place[angle], angle, spin, output: null }));
      const rev = (rows) => rows.map(([m, a, s]) => [m, a, s === "cw" ? "ccw" : "cw"]);
      const X = [[1, 45, "ccw"], [4, 135, "cw"], [2, -135, "ccw"], [3, -45, "cw"]];
      const BF = [[2, 45, "ccw"], [1, 135, "cw"], [3, -135, "ccw"], [4, -45, "cw"]];
      const PLUS = [[3, 0, "cw"], [1, 90, "ccw"], [4, 180, "cw"], [2, -90, "ccw"]];
      const set = (name, label, value, kind, options, extra = {}) => ({ name, label, value, kind, options, unit: "", help: "", reboot: false, ...extra });
      return {
        groups: [
          { title: "Frame", settings: [
            set("FRAME_CLASS", "Frame", 1, "select", [[0, "Not set"], [1, "Quad"], [2, "Hexa"], [3, "Octa"], [4, "OctaQuad (X8)"], [5, "Y6"], [7, "Tri"]], { reboot: true }),
            set("FRAME_TYPE", "Layout", 18, "select", [[0, "Plus"], [1, "X (props in)"], [3, "H (X, props out)"], [12, "BetaFlight X (props in)"], [13, "DJI X"], [14, "Clockwise X"], [18, "BetaFlight X reversed (props out)"], [19, "Y4"]], { reboot: true, help: "Props in: the front motors spin towards each other at the top. Props out: away from each other. The motor test confirms it." }),
          ] },
          { title: "ESCs and motors", settings: [
            set("MOT_PWM_TYPE", "ESC protocol", 6, "select", [[0, "Normal"], [1, "OneShot"], [2, "OneShot125"], [4, "DShot150"], [5, "DShot300"], [6, "DShot600"], [7, "DShot1200"]], { reboot: true, help: "DShot600 for current BLHeli_32, AM32 and Bluejay ESCs; Normal PWM for older ones." }),
            set("SERVO_DSHOT_ESC", "ESC firmware", 1, "select", [[0, "None"], [1, "BLHeli32/Kiss/AM32"], [2, "BLHeli_S/BlueJay"], [3, "BLHeli32/AM32/Kiss+EDT"], [4, "BLHeli_S/BlueJay+EDT"]], { help: "Lets ArduPilot reverse motors in software and read extended telemetry (EDT)." }),
            set("SERVO_BLH_BDMASK", "Bidirectional DShot", 15, "select", [[0, "Off"], [15, "On, motor outputs 1, 2, 3, 4"]], { reboot: true, help: "The ESCs report RPM on the signal wire, so the noise filter follows the motor speed." }),
            set("SERVO_BLH_POLES", "Motor poles", 14, "number", [], { unit: "magnets", reboot: true, help: "Count the magnets in the bell: 14 on most 5-inch motors, 24-28 on large ones." }),
            set("MOT_SPIN_ARM", "Spin when armed", 0.1, "number", [[0, "Low"], [0.1, "Default"], [0.2, "High"]], { unit: "0-1", help: "Throttle the motors idle at once armed. Must be below Spin minimum." }),
            set("MOT_SPIN_MIN", "Spin minimum", 0.13, "number", [[0, "Low"], [0.15, "Default"], [0.25, "High"]], { unit: "0-1", help: "Lowest throttle in flight. About 0.03 above Spin when armed." }),
          ] },
          { title: "Board", settings: [set("AHRS_ORIENTATION", "Board orientation", 0, "select", [[0, "None"], [1, "Yaw45"], [2, "Yaw90"], [4, "Yaw180"], [6, "Yaw270"], [8, "Roll180"]], { help: "None when its arrow points forward and it's the right way up." })] },
          { title: "Battery", settings: [
            set("BATT_MONITOR", "Battery monitor", 4, "select", [[0, "Disabled"], [3, "Analog Voltage Only"], [4, "Analog Voltage and Current"], [9, "ESC"]], { reboot: true }),
            set("BATT_CAPACITY", "Capacity", 1500, "number", [], { unit: "mAh" }),
          ] },
        ],
        checks: [],
        previews: [[0, quad(PLUS)], [1, quad(X)], [3, quad(rev(X))], [6, quad(rev(PLUS))], [12, quad(BF)], [18, quad(rev(BF))]],
        motor_mask: 15,
      };
    },
    serial_status: async () => {
      await sleep(500);
      const port = (n, hw, protocol, protocol_name, baud, setup, live, rx, tx = 0) => ({ n, hw, usb: n === 0, protocol, protocol_name, baud, baud_name: baud ? { 57: "57600", 115: "115200", 230: "230400" }[baud] : null, options: [], rx_bps: rx, tx_bps: tx, live, setup });
      const ports = [
        port(0, "OTG1", 2, "MAVLink2", 115, null, "receiving 30 B/s, sending 900 B/s", 30, 900),
        port(1, "UART1", 2, "MAVLink2", 57, "radio", null, 0),
        port(2, "USART2", 42, "DisplayPort", 115, "osd", "receiving 12 B/s, sending 1.8 kB/s", 12, 1800),
        port(3, "USART3", 5, "GPS", 230, "gps", "GPS 1: 3D fix", 140, 12),
        port(4, "UART4", -1, "Nothing", 57, "off", "receiving 50 B/s, sending 0 B/s", 50),
        port(6, "UART7", 23, "RCIN", 115, "crsf", "receiver: signal", 260),
      ];
      return {
        ports,
        checks: [{ status: "info", title: "Data arrives on SERIAL4, which is off", detail: "Something sends 50 B/s to it. If you connected a device there, set the port up for it.", port: 4, setup: null }],
        setups: [
          { id: "gps", name: "GPS", detail: "GPS receiver (ArduPilot finds its baud rate itself)" },
          { id: "elrs", name: "ExpressLRS receiver", detail: "CRSF at 420 kbaud, link quality as RSSI, no flight-mode messages to the radio" },
          { id: "crsf", name: "TBS Crossfire / Tracer receiver", detail: "CRSF, link quality as RSSI" },
          { id: "osd", name: "Digital video OSD", detail: "DJI O3/O4, Walksnail or HDZero: MSP DisplayPort OSD" },
          { id: "esc", name: "ESC telemetry", detail: "BLHeli/AM32 telemetry wire: RPM, current, temperature" },
          { id: "radio", name: "Telemetry radio", detail: "SiK or RFD900 radio: MAVLink at 57600" },
          { id: "mavlink", name: "MAVLink 115200", detail: "WiFi bridge, ELRS backpack or companion computer" },
          { id: "off", name: "Nothing", detail: "Turn the port off" },
        ],
        traffic: true,
        protocols: [[-1, "None"], [1, "MAVLink1"], [2, "MAVLink2"], [5, "GPS"], [16, "ESC Telemetry"], [23, "RCIN"], [42, "DisplayPort"]],
        bauds: [[9, "9600"], [57, "57600"], [115, "115200"], [230, "230400"], [460, "460800"], [921, "921600"]],
        option_bits: [[0, "InvertRX"], [1, "InvertTX"], [2, "HalfDuplex"], [3, "SwapTXRX"]],
      };
    },
    gps_accept_position: async () => null,
    gps_status: async () => {
      await sleep(600);
      const spoof = window.mockSpoof === true;
      const t = Date.now() / 1000;
      const jitter = (a) => a * Math.sin(t * 0.7);
      const gnss = (window.mockStaged?.GPS1_GNSS_MODE ?? 0);
      const unit = {
        number: 1, kind: "AUTO", port: "SERIAL3", fix: "3D fix", fix_type: 3, satellites: spoof ? 21 : 17, hdop: spoof ? 0.6 : 0.78 + jitter(0.03),
        h_acc: spoof ? 0.9 : 1.4 + jitter(0.1), v_acc: spoof ? 1.5 : 2.6, speed: spoof ? 55.99 : 0.04, course: spoof ? 7 : 0,
        position: spoof ? [-12.03831, -77.05871, 150] : [50.450112 + jitter(0.000004), 30.523415, 186], constellations: gnss ? ["GPS", "Galileo"] : ["receiver default"], rate_hz: 5, first_fix_s: 41,
      };
      const checks = spoof
        ? [
            { status: "fail", title: "The GPS says the vehicle moves at 56 m/s while it stands still: spoofed or faulty", detail: "The GPS reports 56.0 m/s and puts it at -12.03831, -77.05871. If that isn't where the vehicle is, the receiver gets fake satellite signals. The EKF tilts its attitude to explain the false speed, in every flight mode: don't fly like this. To fly here, turn the GPS off (below), reboot and fly Stabilize or AltHold, or put the GPS on a switch.", changes: [{ name: "GPS1_TYPE", value: 0, reason: "Turn the GPS off" }, { name: "FS_THR_ENABLE", value: 3, reason: "Land when the radio link is lost" }], gps_switch: true, accept_position: false },
            { status: "fail", title: "The GPS puts the vehicle 12278 km from where it was last seen", detail: "On 2026-11-10 the GPS was trusted at 50.4501, 30.5234; now it says -12.0383, -77.0587. If you didn't travel there, the GPS is spoofed: turn it off or put it on a switch. If the vehicle really is here, say so and ArduTune remembers the new place.", changes: [], gps_switch: false, accept_position: true },
          ]
        : [];
      const bits = [[0, "GPS"], [1, "SBAS"], [2, "Galileo"], [3, "Beidou"], [4, "IMES"], [5, "QZSS"], [6, "GLONASS"], [7, "NAVIC"]];
      return {
        units: [unit],
        ekf: { using_gps: true, glitching: spoof, ratios: [["Velocity", spoof ? 1.4 : 0.12 + jitter(0.02)], ["Horizontal position", spoof ? 0.9 : 0.08], ["Height", 0.21], ["Compass", spoof ? 0.7 : 0.05]] },
        checks,
        settings: [
          { title: "GPS 1", settings: [
            { name: "GPS1_TYPE", label: "Receiver type", value: 1, kind: "select", options: [[0, "None"], [1, "AUTO"], [2, "uBlox"], [5, "NMEA"], [9, "DroneCAN"]], unit: "" },
            { name: "GPS1_GNSS_MODE", label: "Satellite systems", value: 0, kind: "bits", options: bits, unit: "" },
            { name: "GPS1_RATE_MS", label: "Update rate", value: 200, kind: "select", options: [[100, "10Hz"], [125, "8Hz"], [200, "5Hz"]], unit: "" },
            { name: "GPS1_POS_X", label: "Antenna offset forward", value: 0, kind: "number", options: [], unit: "m" },
            { name: "GPS1_POS_Y", label: "Antenna offset right", value: 0, kind: "number", options: [], unit: "m" },
            { name: "GPS1_POS_Z", label: "Antenna offset down", value: 0, kind: "number", options: [], unit: "m" },
          ] },
          { title: "GPS 2", settings: [{ name: "GPS2_TYPE", label: "Receiver type", value: 0, kind: "select", options: [[0, "None"], [1, "AUTO"], [2, "uBlox"]], unit: "" }] },
          { title: "General", settings: [
            { name: "GPS_AUTO_CONFIG", label: "ArduPilot configures the receiver", value: 1, kind: "select", options: [[0, "Disables automatic configuration"], [1, "Enable automatic configuration for Serial GPSes only"], [2, "Enable automatic configuration for DroneCAN as well"]], unit: "" },
            { name: "GPS_HDOP_GOOD", label: "HDOP to arm in position modes", value: 140, kind: "number", options: [], unit: "×100 (140 = 1.4)" },
          ] },
        ],
        messages: spoof ? ["EKF3 IMU0 GPS Glitch"] : [],
        trusted: !spoof,
      };
    },
    detect_switch: async () => {
      await sleep(2500);
      return 7;
    },
    arming_check: async () => {
      await sleep(1500);
      return {
        armed: false,
        ready: false,
        mode: "Stabilize",
        gps: [{ instance: 0, fix: "3D fix", satellites: 26, hdop: 0.55, speed: 55.99, position: [-12.03831, -77.05871, 985] }],
        problems: [
          {
            status: "fail",
            area: "GPS",
            title: "The GPS says the vehicle is moving at 56 m/s while it stands still: spoofed or faulty GPS",
            messages: ["GPS Glitch or Compass error"],
            advice:
              "The GPS reports 56.0 m/s towards 7° and puts it at -12.03831, -77.05871, 985 m altitude. If that isn't where the vehicle is, the receiver is getting fake satellite signals (GPS spoofing); a faulty receiver can do the same. The attitude estimate (EKF) tilts itself to explain the false speed, and ArduPilot stabilises with that estimate in every mode, Stabilize included: don't fly like this, and don't turn arming checks off. Compare with a phone's GPS, or try outdoors somewhere else. To fly here, turn the GPS off with the changes below, reboot, and fly Stabilize or AltHold only (position modes, return home and missions need GPS). Or put the GPS on a switch (RCn_OPTION = 65: high turns it off) to choose before each flight: set it before powering up, because a false position seen at start-up stays until the next reboot.",
            changes: [
              { name: "GPS1_TYPE", current: 1, value: 0, reason: "Turn the GPS off: the EKF stops using its false position and speed (after a reboot)" },
              { name: "FS_THR_ENABLE", current: 1, value: 3, reason: "Land when the radio link is lost (return home needs GPS)" },
              { name: "BATT_FS_LOW_ACT", current: 2, value: 1, reason: "Land at low battery (return home needs GPS)" },
            ],
          },
          { status: "warn", area: "Compass", title: "The compass reading doesn't match the Earth's field here", messages: ["Check mag field (xy diff:117>100)"], advice: "Most likely caused by the false GPS data (see above): don't recalibrate anything for this. Move away from metal, cars, laptops and phones and check again. If it still fails, calibrate the compass outdoors with the battery mounted as it flies, then reboot. COMPASS_ORIENT is 0, so the GPS/compass arrow must point forward." },
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
