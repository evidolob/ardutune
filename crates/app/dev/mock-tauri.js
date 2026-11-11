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
  const vehicle = { address: "serial:/dev/cu.usbmodem14301:115200", kind: "quadrotor", ardupilot: true, system_id: 1, armed: false, params: 1233 };
  let connected = false;
  let backups = [
    { path: "/b/2.param", file_name: "backup-sys1-20261110-153000.param", saved: "2026-11-10 15:30:00", system_id: 1, reason: "Before writing recommended filter settings", params: 1233 },
    { path: "/b/1.param", file_name: "backup-sys1-20261109-090000.param", saved: "2026-11-09 09:00:00", system_id: 1, reason: "Before maiden flight", params: 1231 },
    { path: "/b/mp.param", file_name: "matekh7a3_params.param", saved: null, system_id: null, reason: null, params: 1233 },
  ];
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
    prepare_changes: async ({ source }) => {
      if (source.startsWith("setup:")) {
        const step = (await fixture("setup")).steps.find((x) => x.id === source.slice(6));
        return { rows: step.changes.map((c) => ({ ...c, current: c.current ?? 0 })), unchanged: [], unknown: [], skipped: [], armed: false };
      }
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
    apply_changes: async () => {
      await progress("Writing parameters", 4);
      return { backup: "/Users/pilot/Documents/ArduTune/backups/backup-sys1-20261110-153000.param", results: [{ name: "INS_GYRO_FILTER", value: 60, error: null }] };
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
    setup_guide: async () => {
      await sleep(800);
      return fixture("setup");
    },
    list_backups: async () => backups,
    backup_params: async ({ note }) => {
      await progress("Reading parameters", 1233);
      backups.unshift({ path: `/b/${Date.now()}.param`, file_name: "backup.param", saved: "2026-11-11 12:00:00", system_id: 1, reason: note || "Manual backup", params: 1233 });
      return vehicle;
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
