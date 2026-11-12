// ArduTune desktop frontend. Plain ES module, no build step: the Rust side
// (crates/app/src) does the work; this file only renders and calls commands.

const tauri = window.__TAURI__;
const invoke = (cmd, args = {}) => tauri.core.invoke(cmd, args);

const LOG_VIEWS = [
  ["overview", "Readiness"],
  ["diagnose", "Crash diagnosis"],
  ["noise", "Noise"],
  ["filters", "Filters"],
  ["pid", "PIDs"],
];
const VEHICLE_VIEWS = [
  ["connection", "Connection"],
  ["setup", "Setup guide"],
  ["motors", "Motor test"],
  ["compass", "Compass calibration"],
  ["tune", "Initial tune"],
  ["prearm", "Pre-arm check"],
  ["allparams", "Parameters"],
  ["params", "Backups"],
  ["firmware", "Firmware"],
  ["vlogs", "Logs on vehicle"],
];
/** Rows shown at once in the parameter list; refine the search for more. */
const PARAM_ROWS = 300;

const state = {
  view: "welcome",
  log: null, // overview of the open log
  loadingLog: null, // file being read
  analyses: {}, // kind -> { loading } | { data } | { error }
  vehicle: null, // connected vehicle info
  ports: null,
  address: "auto",
  connecting: false,
  vehicleLogs: null,
  backups: null, // parameter backups on this computer
  arming: null, // latest pre-arm check report
  setup: null, // latest setup guide
  compass: { cal: null, running: false }, // compass calibration progress
  motors: { layout: null, propsOff: false, throttle: 7, seen: [], verdict: null, spinning: null }, // motor test
  tune: { prop: "", cells: null, chemistry: null, pack: null, result: null }, // initial tune inputs and result
  ptable: null, // full parameter list from the vehicle
  pfilter: { text: "", changed: false, standard: false, group: "" },
  popen: null, // parameter whose editor is open
  staged: {}, // name -> value waiting to be written
  fw: null, // firmware info from the vehicle
  releases: null, // what firmware.ardupilot.org offers for the board
  history: null, // flight history: logs grouped by vehicle
  historyBusy: false,
  historyOpen: new Set(), // flights whose parameter changes are shown
  vehicleBusy: null, // text while a vehicle task runs
  modal: null,
  toast: null,
};

// ---------- helpers ----------

const esc = (s) =>
  String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);

/** Parameter values as ArduPilot shows them: f32 noise trimmed. */
const fmt = (v) => (v === null || v === undefined ? "–" : String(Number(Number(v).toPrecision(6))));
const num = (v, digits = 0) => (v === null || v === undefined ? "–" : Number(v).toFixed(digits));
const ms = (v) => (v === null || v === undefined ? "–" : `${num(v)} ms`);
const chip = (kind, text = kind) => `<span class="chip ${esc(kind)}">${esc(text)}</span>`;
const list = (items) => (items?.length ? `<ul class="plain">${items.map((i) => `<li>${esc(i)}</li>`).join("")}</ul>` : "");
const stat = (label, value, sub = "") =>
  `<div class="stat"><div class="label">${esc(label)}</div><div class="value">${value}</div>${sub ? `<div class="sub">${sub}</div>` : ""}</div>`;
const busy = (text) => `<div class="busy"><div class="spinner"></div>${esc(text)}</div>`;

function set(patch) {
  Object.assign(state, patch);
  render();
}

function toast(text, kind = "error") {
  set({ toast: { text: String(text), kind } });
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => set({ toast: null }), kind === "error" ? 9000 : 5000);
}

// ---------- actions ----------

async function openLog(path) {
  set({ loadingLog: path.split(/[\\/]/).pop(), view: "overview" });
  try {
    const log = await invoke("open_log", { path });
    set({ log, analyses: {}, loadingLog: null });
  } catch (e) {
    set({ loadingLog: null, view: state.log ? "overview" : "welcome" });
    toast(e);
  }
}

async function analyse(kind) {
  if (state.analyses[kind]) return;
  state.analyses[kind] = { loading: true };
  render();
  try {
    state.analyses[kind] = { data: await invoke("analysis", { kind }) };
  } catch (e) {
    state.analyses[kind] = { error: String(e) };
  }
  render();
}

function go(view) {
  set({ view, modal: null });
  if (["diagnose", "noise", "filters", "pid"].includes(view)) analyse(view);
  if (view === "connection" && !state.ports) refreshPorts();
  if (view === "vlogs" && state.vehicle && !state.vehicleLogs) refreshVehicleLogs();
  if (view === "params") refreshBackups();
  if (view === "history") refreshHistory();
  if (view === "prearm" && state.vehicle && !state.arming && !state.vehicleBusy) armingCheck();
  if (view === "setup" && state.vehicle && !state.setup && !state.vehicleBusy) setupGuide();
  if (view === "motors" && state.vehicle && !state.motors.layout && !state.vehicleBusy) loadMotors();
  if (view === "tune" && state.vehicle && !state.tune.pack && !state.vehicleBusy) loadPack();
  if (view === "allparams" && state.vehicle && !state.ptable && !state.vehicleBusy) loadParams();
  if (view === "firmware" && state.vehicle && !state.fw && !state.vehicleBusy) loadFirmware();
}

async function refreshPorts() {
  try {
    set({ ports: await invoke("list_ports") });
  } catch (e) {
    toast(e);
  }
}

async function connect() {
  set({ connecting: true });
  try {
    const vehicle = await invoke("connect", { address: state.address });
    set({ vehicle, connecting: false, vehicleLogs: null, arming: null, setup: null, compass: { cal: null, running: false }, motors: { ...state.motors, layout: null, seen: [], verdict: null, propsOff: false }, tune: { ...state.tune, pack: null, result: null, cells: null, chemistry: null }, ptable: null, staged: {}, popen: null, fw: null, releases: null });
    toast(`Connected: ${vehicle.kind}, ${vehicle.params} parameters read.`, "info");
  } catch (e) {
    set({ connecting: false });
    toast(e);
  } finally {
    hideProgress();
  }
}

async function disconnect() {
  await invoke("disconnect");
  set({ vehicle: null, vehicleLogs: null });
}

async function refreshStatus() {
  try {
    set({ vehicle: await invoke("vehicle_status") });
  } catch (e) {
    set({ vehicle: await invoke("vehicle_status").catch(() => null) });
    toast(e);
  }
}

async function vehicleTask(text, fn) {
  set({ vehicleBusy: text });
  try {
    return await fn();
  } catch (e) {
    toast(e);
    if (String(e).includes("connection to the vehicle was lost")) set({ vehicle: null });
  } finally {
    set({ vehicleBusy: null });
    hideProgress();
  }
}

async function refreshVehicleLogs() {
  const logs = await vehicleTask("Reading the log list…", () => invoke("list_vehicle_logs"));
  if (logs) set({ vehicleLogs: logs });
}

async function downloadLog(id) {
  const path = await vehicleTask(`Downloading log ${id}…`, () => invoke("download_vehicle_log", { id: Number(id) }));
  if (path) {
    toast(`Saved to ${path}`, "info");
    await openLog(path);
  }
}

function askDelete(id) {
  const log = state.vehicleLogs?.find((l) => String(l.id) === String(id));
  const what = log ? `log ${log.id} (${log.date ?? "no GPS time"}, ${num(log.size_mb, 1)} MB)` : `log ${id}`;
  const newest = Math.max(...(state.vehicleLogs ?? []).map((l) => l.id)) === Number(id);
  set({ modal: { stage: "delete", title: `Delete ${what}?`, id: Number(id), newest } });
}

function askErase() {
  const n = state.vehicleLogs?.length ?? 0;
  set({ modal: { stage: "delete", title: `Erase all ${n} log${n === 1 ? "" : "s"} on the vehicle?`, id: null } });
}

async function confirmDelete() {
  const { id, newest } = state.modal;
  set({ modal: null });
  const text = id === null ? "Erasing all logs…" : newest ? `Rebooting the flight controller, then deleting log ${id}…` : `Deleting log ${id}…`;
  const logs = await vehicleTask(text, () =>
    id === null ? invoke("erase_vehicle_logs") : invoke("delete_vehicle_log", { id, rebootFirst: !!newest }),
  );
  if (logs) {
    set({ vehicleLogs: logs });
    toast(id === null ? "All logs erased." : `Log ${id} deleted. The remaining logs may be renumbered.`, "info");
  }
}

async function detectSwitch() {
  const m = state.modal;
  set({ modal: { ...m, stage: "switchListening", error: null } });
  try {
    const ch = await invoke("detect_switch", { listenS: 6, option: m.option });
    writeChanges(`switch:${ch}:${m.option}`, `GPS off switch on channel ${ch}`);
  } catch (e) {
    set({ modal: { ...m, stage: "switchLearn", error: String(e) } });
  }
}

async function armingCheck() {
  const listen = 8;
  const report = await vehicleTask(`Running pre-arm checks and listening for ${listen} s — try arming now (props off) to see why it refuses…`, () =>
    invoke("arming_check", { listenS: listen }),
  );
  if (report) set({ arming: report });
}

async function setupGuide() {
  const guide = await vehicleTask("Checking the setup…", () => invoke("setup_guide"));
  if (guide) set({ setup: guide });
}

async function startCompassCal() {
  try {
    await invoke("compass_cal_start");
  } catch (e) {
    return toast(e);
  }
  set({ compass: { cal: { compasses: [], messages: [] }, running: true } });
  while (state.compass.running && state.vehicle) {
    try {
      const cal = await invoke("compass_cal_poll");
      const finished = cal.compasses.length > 0 && cal.compasses.every((c) => c.done);
      set({ compass: { cal, running: state.compass.running && !finished } });
    } catch (e) {
      set({ compass: { ...state.compass, running: false } });
      toast(e);
    }
  }
}

async function finishCompassCal(accept) {
  try {
    await invoke("compass_cal_finish", { accept });
    set({ compass: accept ? { ...state.compass, accepted: true } : { cal: null, running: false } });
    if (accept) toast("Calibration saved. Reboot to use it.", "info");
  } catch (e) {
    toast(e);
  }
}

async function loadMotors() {
  try {
    const layout = await invoke("motor_layout");
    set({ motors: { ...state.motors, layout, seen: layout.positions.map(() => ({ spun: null, spin: null })), verdict: null } });
  } catch (e) {
    set({ motors: { ...state.motors, layout: { error: String(e) } } });
  }
}

const MOTOR_SECONDS = 2;

async function spinMotor(i) {
  const m = state.motors;
  if (!m.propsOff) return toast("Confirm the propellers are off first.");
  set({ motors: { ...m, spinning: i, verdict: null } });
  try {
    await invoke("motor_test", { seq: i + 1, throttle: m.throttle, seconds: MOTOR_SECONDS });
    await new Promise((r) => setTimeout(r, MOTOR_SECONDS * 1000));
  } catch (e) {
    toast(e);
  } finally {
    set({ motors: { ...state.motors, spinning: null } });
  }
}

function observations() {
  return state.motors.seen.map((o) => ({ spun: o.spun === "none" ? null : o.spun, spin: o.spin }));
}

async function checkMotors() {
  const verdict = await vehicleTask("Checking…", () => invoke("motor_check", { observations: observations() }));
  if (verdict) set({ motors: { ...state.motors, verdict } });
}

function setSeen(i, patch) {
  const seen = state.motors.seen.map((o, j) => (j === i ? { ...o, ...patch } : o));
  set({ motors: { ...state.motors, seen, verdict: null } });
}

async function loadPack() {
  try {
    const pack = await invoke("vehicle_pack");
    set({ tune: { ...state.tune, pack, cells: state.tune.cells ?? pack.cells, chemistry: state.tune.chemistry ?? pack.chemistry } });
  } catch (e) {
    toast(e);
  }
}

function tuneInput() {
  const t = state.tune;
  return { prop_in: Number(t.prop.replace(",", ".")), cells: t.cells, chemistry: t.chemistry };
}

async function calculateTune() {
  if (!state.tune.prop.trim()) return toast("Enter the prop size in inches.");
  const result = await vehicleTask("Calculating…", () => invoke("initial_tune", { tune: tuneInput() }));
  if (result) set({ tune: { ...state.tune, result } });
}

async function loadParams() {
  const ptable = await vehicleTask("Reading all parameters with their defaults…", () => invoke("param_table"));
  if (ptable) set({ ptable });
}

/** The value in the open editor, from its number box, option list or bit boxes. */
function editorValue() {
  const bits = [...document.querySelectorAll("[data-pbit]")];
  if (bits.length) return bits.reduce((v, b) => (b.checked ? v + 2 ** Number(b.dataset.pbit) : v), 0);
  const raw = document.getElementById("pval")?.value ?? "";
  return raw.trim() === "" ? NaN : Number(raw);
}

function stageParam(name, value) {
  const row = state.ptable?.rows.find((r) => r.name === name);
  if (!row || !Number.isFinite(value)) return toast("Enter a number.");
  if (row.integer && !Number.isInteger(value)) return toast(`${name} takes whole numbers only.`);
  const staged = { ...state.staged };
  if (value === row.value) delete staged[name];
  else staged[name] = value;
  set({ staged });
}

async function loadFirmware() {
  const fw = await vehicleTask("Reading the firmware version…", () => invoke("firmware_info"));
  if (fw) set({ fw });
}

async function checkReleases() {
  if (!state.fw?.board) return toast("The vehicle didn't report its board name.");
  set({ releases: "loading" });
  try {
    set({ releases: await invoke("firmware_releases", { board: state.fw.board, current: state.fw.version }) });
  } catch (e) {
    set({ releases: null });
    toast(e);
  }
}

async function prepareFirmware(source, path = null) {
  const release = Array.isArray(state.releases) ? state.releases.find((r) => r.channel === source) : null;
  set({ modal: { stage: "fw-loading", title: source === "file" ? "Checking the firmware file" : `Downloading ${source} firmware` } });
  try {
    const prep = await invoke("firmware_prepare", { source, path });
    set({ modal: { stage: "fw-confirm", title: `Flash ${prep.label}`, prep, release, backup: true } });
  } catch (e) {
    set({ modal: null });
    toast(e);
  } finally {
    hideProgress();
  }
}

async function pickFirmware() {
  const path = await invoke("pick_file", { kind: "apj" });
  if (path) prepareFirmware("file", path);
}

async function flashFirmware() {
  const m = state.modal;
  set({ modal: { ...m, stage: "fw-flashing" } });
  try {
    const result = await invoke("firmware_flash", { backup: m.backup });
    set({ modal: { ...m, stage: "fw-done", result }, vehicle: result.vehicle, fw: null, releases: null, ptable: null, setup: null, arming: null });
    if (result.reconnected) loadFirmware();
  } catch (e) {
    set({ modal: { ...m, stage: "fw-failed", error: String(e) }, vehicle: null });
  } finally {
    hideProgress();
  }
}

async function reboot() {
  const ok = await vehicleTask("Rebooting…", () => invoke("reboot").then(() => true));
  if (ok) {
    set({ vehicle: null, vehicleLogs: null, modal: null });
    toast("The flight controller is rebooting. Reconnect in a few seconds.", "info");
  }
}

/** Reviews and writes the staged parameter edits. */
async function writeEdits() {
  const values = Object.entries(state.staged);
  if (!values.length) return;
  set({ modal: { stage: "planning", title: "parameter edits", source: "edits" } });
  try {
    const plan = await invoke("prepare_changes", { source: "edits", label: "writing parameter edits", values });
    set({ modal: { stage: "confirm", title: "parameter edits", source: "edits", plan, selected: new Set(plan.rows.map((r) => r.name)) } });
  } catch (e) {
    set({ modal: null });
    toast(e);
  }
}

/** Shows what writing a set of recommendations would change, then writes on confirmation. */
async function writeChanges(source, title, path = null, extra = {}) {
  if (!state.vehicle) {
    set({ modal: { stage: "connect", title: `Write ${title}` } });
    return;
  }
  set({ modal: { stage: "planning", title, source } });
  try {
    const verb = source === "file" ? "restoring" : "writing";
    const plan = await invoke("prepare_changes", { source, path, label: `${verb} ${title}`, ...extra });
    set({ modal: { stage: "confirm", title, source, plan, selected: new Set(plan.rows.map((r) => r.name)) } });
  } catch (e) {
    set({ modal: null });
    toast(e);
  }
}

async function confirmWrite(backup) {
  const m = state.modal;
  set({ modal: { ...m, stage: "writing", backup } });
  try {
    const result = await invoke("apply_changes", { names: [...m.selected], backup });
    const staged = { ...state.staged };
    if (m.source === "edits") result.results.filter((x) => !x.error).forEach((x) => delete staged[x.name]);
    set({ modal: { ...m, stage: "done", result }, backups: null, setup: state.view === "setup" ? state.setup : null, staged });
    if (state.view === "allparams") loadParams();
    if (state.view === "setup") setupGuide();
    if (state.view === "tune") set({ tune: { ...state.tune, result: null } });
    if (state.view === "motors") {
      set({ motors: { ...state.motors, layout: null } });
      loadMotors();
    }
    if (state.view === "params") refreshBackups();
  } catch (e) {
    set({ modal: { ...m, stage: "confirm" } });
    toast(e);
  } finally {
    hideProgress();
  }
}

async function pickLog() {
  const path = await invoke("pick_file", { kind: "log" });
  if (path) openLog(path);
}

async function pickParamFile() {
  const path = await invoke("pick_file", { kind: "param" });
  if (path) writeChanges("file", path.split(/[\\/]/).pop(), path);
}

async function refreshHistory() {
  if (state.historyBusy) return;
  set({ historyBusy: true });
  try {
    set({ history: await invoke("flight_history") });
  } catch (e) {
    toast(e);
  } finally {
    set({ historyBusy: false });
    hideProgress();
  }
}

async function importLog() {
  const path = await invoke("pick_file", { kind: "log" });
  if (!path) return;
  try {
    await invoke("import_log", { path });
    refreshHistory();
  } catch (e) {
    toast(e);
  }
}

async function refreshBackups() {
  try {
    set({ backups: await invoke("list_backups") });
  } catch (e) {
    toast(e);
  }
}

async function backupNow() {
  const note = document.getElementById("note")?.value ?? "";
  const vehicle = await vehicleTask("Reading parameters for the backup…", () => invoke("backup_params", { note }));
  if (vehicle) {
    set({ vehicle });
    toast(`Backed up ${vehicle.params} parameters.`, "info");
    refreshBackups();
  }
}

async function nameDrone(uid, name) {
  try {
    const fromLog = state.modal?.fromLog ?? false;
    const board = state.modal?.board ?? null;
    const vehicle = await invoke("name_drone", { uid, name, board, fromLog });
    set({ modal: null, ...(vehicle && state.vehicle ? { vehicle } : {}) });
    if (state.view === "params") refreshBackups();
    if (state.view === "history") refreshHistory();
  } catch (e) {
    toast(e);
  }
}

async function assignBackup(path) {
  try {
    await invoke("assign_backup", { path });
    refreshBackups();
  } catch (e) {
    toast(e);
  }
}

function backupLabel(b) {
  return b.saved ? `the backup from ${b.saved} UTC` : b.file_name;
}

function restoreBackup(path) {
  const b = state.backups?.find((x) => x.path === path);
  writeChanges("file", b ? backupLabel(b) : path, path);
}

function askDeleteBackup(path) {
  const b = state.backups?.find((x) => x.path === path);
  set({ modal: { stage: "deleteBackup", title: `Delete ${b ? backupLabel(b) : "this backup"}?`, path } });
}

async function confirmDeleteBackup() {
  const { path } = state.modal;
  set({ modal: null });
  try {
    await invoke("delete_backup", { path });
    refreshBackups();
  } catch (e) {
    toast(e);
  }
}

async function openSample() {
  set({ loadingLog: "sample flight", view: "overview" });
  try {
    set({ log: await invoke("open_sample"), analyses: {}, loadingLog: null });
  } catch (e) {
    set({ loadingLog: null });
    toast(e);
  }
}

const actions = {
  go: (d) => go(d.view),
  pickLog,
  openSample,
  pickParamFile,
  backupNow,
  restoreBackup: (d) => restoreBackup(d.path),
  askDeleteBackup: (d) => askDeleteBackup(d.path),
  confirmDeleteBackup,
  assignBackup: (d) => assignBackup(d.path),
  askNameDrone: (d) =>
    set({ modal: { stage: "nameDrone", title: d.name ? `Rename ${d.name}` : "Name this vehicle", uid: d.uid, name: d.name ?? "", fromLog: d.fromLog === "true", board: d.board ?? null } }),
  refreshHistory,
  askGpsSwitch: () => set({ modal: { stage: "switchLearn", title: "Put the GPS on a switch", option: 65, error: null } }),
  detectSwitch,
  importLog,
  openHistoryLog: (d) => openLog(d.path),
  toggleChanges: (d) => {
    const open = new Set(state.historyOpen);
    open.has(d.path) ? open.delete(d.path) : open.add(d.path);
    set({ historyOpen: open });
  },
  saveDroneName: (d) => nameDrone(d.uid, document.getElementById(d.uid === state.modal?.uid ? "droneName" : "droneNameInline")?.value ?? ""),
  armingCheck,
  setupGuide,
  calculateTune,
  startCompassCal,
  cancelCompassCal: () => {
    state.compass.running = false;
    finishCompassCal(false);
  },
  acceptCompassCal: () => finishCompassCal(true),
  spinMotor: (d) => spinMotor(Number(d.index)),
  checkMotors,
  writeMotors: () => writeChanges("motors", "motor order and direction fixes", null, { observations: observations() }),
  writeTune: () => writeChanges("initial_tune", `initial tune for ${state.tune.result.prop_in}" props`, null, { tune: tuneInput() }),
  loadParams,
  loadFirmware,
  checkReleases,
  installRelease: (d) => prepareFirmware(d.channel),
  pickFirmware,
  flashFirmware,
  openParam: (d) => set({ popen: state.popen === d.name ? null : d.name }),
  stageParam: (d) => stageParam(d.name, editorValue()),
  defaultParam: (d) => stageParam(d.name, Number(d.value)),
  unstageParam: (d) => {
    const staged = { ...state.staged };
    delete staged[d.name];
    set({ staged });
  },
  discardStaged: () => set({ staged: {} }),
  writeStaged: () => writeEdits(),
  refreshPorts,
  connect,
  disconnect,
  refreshStatus,
  reboot,
  refreshVehicleLogs,
  downloadLog: (d) => downloadLog(d.id),
  askDelete: (d) => askDelete(d.id),
  askErase,
  confirmDelete,
  write: (d) => writeChanges(d.source, d.title),
  confirmWrite: () => confirmWrite(false),
  backupAndWrite: () => confirmWrite(true),
  closeModal: () => set({ modal: null }),
  closeToast: () => set({ toast: null }),
  retry: (d) => {
    delete state.analyses[d.kind];
    analyse(d.kind);
  },
};

document.addEventListener("click", (e) => {
  const el = e.target.closest("[data-action]");
  if (el && !el.disabled) actions[el.dataset.action]?.(el.dataset);
});

document.addEventListener("change", (e) => {
  const t = e.target;
  if (t.id === "fwbackup" && state.modal) state.modal.backup = t.checked;
  if (t.name === "port") set({ address: t.value });
  if (t.id === "propsOff") set({ motors: { ...state.motors, propsOff: t.checked } });
  if (t.id === "mthrottle") set({ motors: { ...state.motors, throttle: Number(t.value) } });
  if (t.dataset.mspun !== undefined) setSeen(Number(t.dataset.mspun), { spun: t.value === "" ? null : t.value === "none" ? "none" : Number(t.value) });
  if (t.dataset.mspin !== undefined) setSeen(Number(t.dataset.mspin), { spin: t.value || null });
  if (t.id === "tcells") set({ tune: { ...state.tune, cells: Number(t.value), result: null } });
  if (t.id === "tchem") set({ tune: { ...state.tune, chemistry: t.value, result: null } });
  const m = state.modal;
  if (m?.selected && t.dataset.name) {
    t.checked ? m.selected.add(t.dataset.name) : m.selected.delete(t.dataset.name);
    render();
  }
  if (m?.selected && t.id === "selectAll") {
    m.selected = new Set(t.checked ? m.plan.rows.map((r) => r.name) : []);
    render();
  }
});

document.addEventListener("input", (e) => {
  if (e.target.id === "address") state.address = e.target.value;
  if (e.target.id === "tprop") set({ tune: { ...state.tune, prop: e.target.value, result: null } });
  if (e.target.id === "psearch") set({ pfilter: { ...state.pfilter, text: e.target.value }, popen: null });
});

document.addEventListener("change", (e) => {
  const f = e.target.dataset.pfilter;
  if (f === "group") set({ pfilter: { ...state.pfilter, group: e.target.value } });
  else if (f) set({ pfilter: { ...state.pfilter, [f]: e.target.checked } });
});

document.addEventListener("keydown", (e) => {
  if (e.key === "Enter" && e.target.id === "tprop") calculateTune();
  if (e.key === "Enter" && e.target.id === "droneNameInline") nameDrone(state.vehicle?.uid, e.target.value);
  if (e.key === "Enter" && e.target.id === "droneName") nameDrone(state.modal?.uid, e.target.value);
  if (e.key === "Enter" && e.target.id === "pval" && state.popen) stageParam(state.popen, editorValue());
});

// ---------- progress (updated in place: events arrive ten times a second) ----------

let progressEl = null;

tauri.event.listen("progress", ({ payload: p }) => {
  if (!progressEl) {
    progressEl = document.createElement("div");
    progressEl.className = "progress";
    document.body.append(progressEl);
  }
  const pct = p.total ? (100 * p.done) / p.total : 0;
  const big = p.total > 100000;
  const amount = big ? `${(p.done / 1e6).toFixed(1)} / ${(p.total / 1e6).toFixed(1)} MB` : `${p.done} / ${p.total}`;
  progressEl.innerHTML = `<div>${esc(p.task)} — ${amount} (${pct.toFixed(0)}%)</div><div class="bar"><div style="width:${pct}%"></div></div>`;
});

function hideProgress() {
  progressEl?.remove();
  progressEl = null;
}

// ---------- views ----------

function header() {
  const v = state.vehicle;
  const pill = v
    ? `<span class="dot ${v.armed ? "armed" : "on"}"></span>${esc(v.label ?? `${v.kind} · sys ${v.system_id}`)} · ${v.armed ? "ARMED" : "disarmed"}`
    : `<span class="dot"></span>No vehicle`;
  return `
    <div class="brand">Ardu<span>Tune</span></div>
    <button class="btn" data-action="pickLog">Open log…</button>
    <div class="file">${state.log ? esc(state.log.path) : ""}</div>
    <button class="pill" data-action="go" data-view="connection">${pill}</button>`;
}

function nav() {
  const item = ([id, label], disabled) =>
    `<button data-action="go" data-view="${id}" class="${state.view === id ? "active" : ""}" ${disabled ? "disabled" : ""}>${esc(label)}${badge(id)}</button>`;
  return `
    <h3>Vehicle</h3>
    ${VEHICLE_VIEWS.map((v) => item(v, false)).join("")}
    <h3>Flight log</h3>
    ${item(["history", "Flight history"], false)}
    ${LOG_VIEWS.map((v) => item(v, !state.log)).join("")}`;
}

function badge(id) {
  if (id === "overview" && state.log) {
    const r = state.log.readiness;
    return r.filter_ready && r.pid_ready ? chip("pass", "ready") : chip("warn", "check");
  }
  if (id === "diagnose") {
    const d = state.analyses.diagnose?.data?.report;
    if (d?.incidents?.length) return chip("fail", d.incidents.length);
  }
  return "";
}

function welcome() {
  return `<div class="welcome">
    <h1>Diagnose and tune your multicopter</h1>
    <p class="lead">Open a DataFlash log from a test flight. ArduTune checks whether it has the data needed for tuning,
      explains crashes, analyses gyro noise and recommends filter and PID changes it can write straight to the vehicle.</p>
    <div class="card" data-action="pickLog"><h2>Open a flight log</h2><div class="muted">A <code>.bin</code> file from the SD card or a download.</div></div>
    <div class="card" data-action="go" data-view="connection"><h2>Connect to the vehicle</h2><div class="muted">Over USB or telemetry: download logs, back up and write parameters.</div></div>
    <div class="card" data-action="openSample"><h2>Try a sample flight</h2><div class="muted">A synthetic log with hover and stick snaps, to explore the app.</div></div>
  </div>`;
}

function overview() {
  if (state.loadingLog) return busy(`Reading ${state.loadingLog}…`);
  const l = state.log;
  const r = l.readiness;
  const areas = [
    ["general", "General"],
    ["vibration", "Vibration"],
    ["filters", "Filters"],
    ["pid", "PID"],
  ];
  const readyChip = (ok) => (ok ? chip("pass", "ready") : chip("fail", "not yet"));
  return `
    <h1>${esc(l.file_name)}</h1>
    <p class="lead">Log readiness: does this flight have what filter and PID tuning need?</p>
    <div class="stats">
      ${l.vehicle ? stat("Vehicle", esc(l.vehicle.label), l.vehicle.named ? esc(l.vehicle.board) : "") : ""}
      ${stat("Firmware", esc(r.firmware ?? "unknown").replace(/ \(.*\)$/, ""), esc(r.firmware?.match(/\((.*)\)/)?.[1] ?? ""))}
      ${stat("Flight", `${num(l.flight_s)} s`, `${l.flight_periods} period(s) · log ${num(r.log_duration_s)} s`)}
      ${stat("File", `${num(l.size_mb, 1)} MB`, `${l.messages.toLocaleString()} messages`)}
      ${stat("Filter tuning", readyChip(r.filter_ready))}
      ${stat("PID tuning", readyChip(r.pid_ready))}
    </div>
    ${l.truncated || l.corrupt_dropped ? `<div class="banner warn">The log is ${l.truncated ? "truncated" : "partly corrupt"}${l.corrupt_dropped ? ` (${l.corrupt_dropped} damaged records dropped)` : ""}. Results may be incomplete.</div>` : ""}
    ${recommendations(r)}
    <div class="card">
      ${areas
        .map(([area, title]) => {
          const checks = r.checks.filter((c) => c.area === area);
          if (!checks.length) return "";
          return `<h2 style="margin-top:6px">${title}</h2><ul class="checks">${checks
            .map((c) => `<li><div>${chip(c.status)}</div><div><div class="title">${esc(c.title)}</div>${c.detail ? `<div class="detail">${esc(c.detail)}</div>` : ""}</div></li>`)
            .join("")}</ul>`;
        })
        .join("")}
    </div>
    ${r.flight_plan.length ? `<div class="card"><h2>Next test flight</h2><ol>${r.flight_plan.map((s) => `<li>${esc(s)}</li>`).join("")}</ol></div>` : ""}`;
}

function recommendations(r) {
  if (!r.recommended_params.length) return "";
  return `<div class="card">
    <h2>Set these before the next flight</h2>
    <table><tr><th>Parameter</th><th class="num">In log</th><th class="num">Set to</th><th>Why</th></tr>
    ${r.recommended_params
      .map((p) => `<tr><td class="mono">${esc(p.name)}</td><td class="num">${p.current === null ? "–" : fmt(p.current)}</td><td class="num new">${fmt(p.value)}</td><td>${esc(p.reason)}</td></tr>`)
      .join("")}</table>
    <div class="actions"><button class="btn primary" data-action="write" data-source="readiness" data-title="logging for tuning">Write to vehicle…</button>
    <span class="hint">Checked again against the vehicle's current values first.</span></div>
  </div>`;
}

/** Shared loading / error wrapper for the analysis views. */
function analysisView(kind, render) {
  const a = state.analyses[kind];
  if (!a || a.loading) return busy("Analysing…");
  if (a.error)
    return `<div class="banner warn">${esc(a.error)}</div><button class="btn" data-action="retry" data-kind="${kind}">Try again</button>`;
  return render(a.data.report, a.data.svg);
}

function findingBlock(f, n) {
  return `<div class="finding ${esc(f.confidence)}">
    <div>${n ? `${n}. ` : ""}${chip(f.confidence === "high" ? "fail" : f.confidence === "medium" ? "warn" : "info", f.confidence)} <span class="title">${esc(f.title)}</span></div>
    ${list(f.evidence)}
    ${f.advice ? `<div class="advice"><b>What to do:</b> ${esc(f.advice)}</div>` : ""}
  </div>`;
}

function diagnoseView(d) {
  const incidents = d.incidents
    .map(
      (inc, i) => `<div class="card">
      <h2>Incident ${i + 1}${inc.mode ? ` · ${esc(inc.mode)}` : ""}</h2>
      <p>${esc(inc.summary)}</p>
      ${inc.findings.length ? `<h2 style="margin-top:18px">Likely causes, most likely first</h2>${inc.findings.map((f, j) => findingBlock(f, j + 1)).join("")}` : ""}
      <h2 style="margin-top:18px">Timeline</h2>
      <table>${inc.timeline.map((e) => `<tr><td class="num" style="width:90px">${num(e.time_s, 2)} s</td><td>${esc(e.text)}</td></tr>`).join("")}</table>
      ${inc.ruled_out.length ? `<h2 style="margin-top:18px">Ruled out</h2>${list(inc.ruled_out)}` : ""}
    </div>`,
    )
    .join("");
  return `
    <h1>Crash diagnosis</h1>
    <p class="lead">Losses of control and in-flight log ends, with their likely causes.</p>
    ${d.incidents.length ? incidents : `<div class="banner pass">No loss of control or crash found in this log.</div>`}
    ${d.warnings.length ? `<div class="card"><h2>Other problems in this flight</h2>${d.warnings.map((w) => findingBlock(w)).join("")}</div>` : ""}`;
}

function peakClass(c) {
  switch (c.kind) {
    case "motor_harmonic":
      return `Motor harmonic ${c.harmonic}${c.tracks_rpm === true ? " (tracks RPM)" : ""}`;
    case "fixed_frequency":
      return "Fixed frequency (frame resonance?)";
    case "control_band":
      return "Control-band motion";
    default:
      return "Unclassified";
  }
}

function noiseView(r, svg) {
  const yesNo = (v) => (v === true ? chip("pass", "yes") : v === false ? chip("warn", "no") : "–");
  const reduction = r.noise_rms_post ? 20 * Math.log10(r.noise_rms_pre / Math.max(r.noise_rms_post, 1e-9)) : null;
  return `
    <h1>Gyro noise</h1>
    <p class="lead">${r.source === "raw_imu" ? "Raw IMU" : "Batch sampler"} data at ${num(r.sample_rate_hz)} Hz, ${esc(r.selection)}.</p>
    <div class="stats">
      ${stat("Noise before filters", `${num(r.noise_rms_pre, 1)} °/s`, "above 30 Hz, RMS")}
      ${r.noise_rms_post !== null ? stat("After filters", `${num(r.noise_rms_post, 1)} °/s`, `${num(reduction)} dB less`) : ""}
      ${r.hover_motor_hz ? stat("Motor speed at hover", `${num(r.hover_motor_hz)} Hz`, r.hover_motor_hz_by_motor.map(([m, f]) => `M${m} ${num(f)}`).join(" · ")) : ""}
    </div>
    ${r.findings.length ? `<div class="card"><h2>Findings</h2>${list(r.findings)}</div>` : ""}
    ${svg ? `<div class="plot">${svg}</div>` : ""}
    <div class="card"><h2>Noise peaks</h2>
      ${
        r.peaks.length
          ? `<table><tr><th class="num">Freq</th><th class="num">Above floor</th><th>Axis</th><th>What</th><th class="num">Filters remove</th><th>Notched</th></tr>
        ${r.peaks
          .map(
            (p) => `<tr><td class="num">${num(p.freq_hz)} Hz</td><td class="num">${num(p.prominence_db)} dB</td><td>${esc(p.axis)}</td><td>${esc(peakClass(p.class))}</td>
            <td class="num">${p.filter_reduction_db === null ? "–" : `${num(p.filter_reduction_db)} dB`}</td><td>${yesNo(p.notch_covered)}</td></tr>`,
          )
          .join("")}</table>`
          : `<div class="muted">No peaks stand out.</div>`
      }
    </div>
    <div class="card"><h2>Harmonic notch</h2>
      ${
        r.notches.length
          ? r.notches
              .map(
                (n) =>
                  `<div><code>${esc(n.name)}</code> ${esc(n.mode)} · centre ≈ ${n.hover_center_hz ? `${num(n.hover_center_hz)} Hz` : "?"} at hover · bandwidth ${n.bandwidth_hz ? `${num(n.bandwidth_hz)} Hz` : "?"} · harmonics ${n.harmonics.join(", ")}</div>`,
              )
              .join("")
          : `<div class="muted">None enabled.</div>`
      }
    </div>
    ${r.notes.length ? `<div class="card"><h2>Notes</h2>${list(r.notes)}</div>` : ""}`;
}

function changesTable(changes, reasons = false) {
  return `<table class="changes"><tr><th>Parameter</th><th class="num">Current</th><th></th><th class="num">Proposed</th>${reasons ? "<th>Why</th>" : ""}</tr>
    ${changes
      .map(
        (c) =>
          `<tr><td class="mono">${esc(c.name)}</td><td class="num">${c.current === null ? "unset" : fmt(c.current)}</td><td class="arrow">→</td><td class="num new">${fmt(c.value)}</td>${reasons ? `<td>${esc(c.reason)}</td>` : ""}</tr>`,
      )
      .join("")}</table>`;
}

function filtersView(r, svg) {
  const row = (label, e, tag = "") =>
    e
      ? `<tr><td><b>${label}</b> ${tag}</td><td class="num">${num(e.noise_rms, 2)} °/s</td><td class="num">${num(e.lag_deg)}°</td><td class="num">${num(e.delay_ms, 1)} ms</td><td>${esc(e.description)}</td></tr>`
      : "";
  const n = r.frontier.length;
  const picks = [...new Set([0, 1, 2, 3, 4, 5].map((i) => Math.round((i * (n - 1)) / 5)))].filter((i) => n > 2 && i < n);
  return `
    <h1>Gyro filters</h1>
    <p class="lead">Your recorded noise run through ArduPilot's own filters: noise left above 45 Hz against the delay the filters add at ${num(r.lag_ref_hz)} Hz.</p>
    ${
      r.validation
        ? `<div class="banner ${r.validation.rms_error_db < 4 ? "pass" : "warn"}">Simulating your current filters reproduces the logged filtered gyro within ${num(r.validation.rms_error_db, 1)} dB (${num(r.validation.band_hz[0])}–${num(r.validation.band_hz[1])} Hz).</div>`
        : `<div class="banner info">This log has no post-filter gyro data to check the simulation against.</div>`
    }
    <div class="card">
      <table><tr><th></th><th class="num">Noise &gt;45 Hz</th><th class="num">Phase lag</th><th class="num">Delay</th><th>Settings</th></tr>
        ${row("Current", r.current)}
        ${row("Recommended", r.recommended, chip("pass", "same noise, less delay"))}
        ${row("Quieter", r.quieter, chip("info", "less noise, same delay"))}
      </table>
    </div>
    ${
      r.recommended
        ? `<div class="card"><h2>Recommended changes</h2>${changesTable(r.changes)}
        <div class="actions"><button class="btn primary" data-action="write" data-source="filters" data-title="recommended filter settings">Write to vehicle…</button></div></div>`
        : `<div class="banner pass">Your current filters are already a good balance: nothing gives the same noise with clearly less delay.</div>`
    }
    ${
      r.quieter_changes.length
        ? `<div class="card"><h2>Quieter option</h2>${changesTable(r.quieter_changes)}
        <div class="actions"><button class="btn" data-action="write" data-source="filters_quieter" data-title="quieter filter settings">Write to vehicle…</button></div></div>`
        : ""
    }
    ${svg ? `<div class="plot">${svg}</div>` : ""}
    ${
      picks.length
        ? `<div class="card"><h2>Other trade-offs</h2><p class="muted" style="margin-top:0">From least delay to least noise.</p>
        <table><tr><th class="num">Noise</th><th class="num">Lag</th><th class="num">Delay</th><th>Settings</th></tr>
        ${picks.map((i) => r.frontier[i]).map((e) => `<tr><td class="num">${num(e.noise_rms, 2)} °/s</td><td class="num">${num(e.lag_deg)}°</td><td class="num">${num(e.delay_ms, 1)} ms</td><td>${esc(e.description)}</td></tr>`).join("")}
        </table></div>`
        : ""
    }
    ${r.notes.length ? `<div class="card"><h2>Notes</h2>${list(r.notes)}</div>` : ""}
    <div class="banner info">Filters change how the attitude loop responds. After changing them, hover carefully first, watch for oscillation and check motor temperatures after a short flight.</div>`;
}

function axisCard(a) {
  const g = a.gains;
  const s = a.step;
  const fltd = g.fltd === null ? "–" : g.fltd === 0 ? "off" : `${fmt(g.fltd)} Hz`;
  const over = s ? (s.overshoot_pct > 25 ? "fail" : s.overshoot_pct > 15 ? "warn" : "pass") : null;
  return `<div class="card">
    <h2>${esc(a.axis[0].toUpperCase() + a.axis.slice(1))} ${s ? chip(over, `${num(s.overshoot_pct)}% overshoot`) : ""} ${a.includes_autotune ? chip("warn", "includes AutoTune") : ""}</h2>
    <div class="muted" style="margin-bottom:10px">P ${fmt(g.p)} · I ${fmt(g.i)} · D ${fmt(g.d)} · FF ${fmt(g.ff)} · D filter ${fltd} · SMAX ${fmt(g.smax)} · angle P ${fmt(g.angle_p)}</div>
    ${
      s
        ? `<dl class="kv">
        <dt>Rise time</dt><dd>${ms(s.rise_ms)}</dd>
        <dt>Delay</dt><dd>${ms(s.delay_ms)}</dd>
        <dt>Settles in</dt><dd>${ms(s.settling_ms)}</dd>
        <dt>Reaches</dt><dd>${num(s.steady_state * 100)}% of target</dd>
        <dt>D-term noise share</dt><dd>${num(a.terms.d_noise_share * 100)}%</dd>
        <dt>Windows averaged</dt><dd>${s.windows}</dd>
      </dl>`
        : `<div class="muted">Not enough stick inputs on this axis for a step response.</div>`
    }
    ${a.findings.length ? `<div style="margin-top:10px">${list(a.findings)}</div>` : ""}
  </div>`;
}

function pidView(r, svg) {
  return `
    <h1>Rate PIDs</h1>
    <p class="lead">Step response worked out from normal flight (rate target vs measured rate at ${num(r.sample_rate_hz)} Hz).</p>
    ${svg ? `<div class="plot">${svg}</div>` : ""}
    <div class="grid">${r.axes.map(axisCard).join("")}</div>
    ${
      r.suggestions.length
        ? `<div class="card"><h2>Suggested changes</h2>${changesTable(r.suggestions, true)}
        <div class="actions"><button class="btn primary" data-action="write" data-source="pid" data-title="suggested PID changes">Write to vehicle…</button>
        <span class="hint">Small steps: test-hover after each change.</span></div></div>`
        : ""
    }
    ${
      r.autotune.length
        ? `<div class="card"><h2>AutoTune results that weren't saved</h2>${changesTable(r.autotune, true)}
        <div class="actions"><button class="btn" data-action="write" data-source="pid_autotune" data-title="AutoTune gains">Write to vehicle…</button></div></div>`
        : ""
    }
    ${r.notes.length ? `<div class="card"><h2>Notes</h2>${list(r.notes)}</div>` : ""}
    <div class="banner info">Fix the filters before tuning PIDs, and change gains in small steps.</div>`;
}

const shortId = (uid) => uid.slice(-6);

const minutes = (s) => `${Math.floor(s / 60)}:${String(Math.round(s % 60)).padStart(2, "0")}`;

/** A tiny line of values, oldest first; gaps where a flight had no value. */
function sparkline(values) {
  const pts = values.map((v, i) => [i, v]).filter(([, v]) => v !== null && v !== undefined);
  if (pts.length < 2) return "";
  const [w, h] = [150, 34];
  const ys = pts.map(([, v]) => v);
  const [lo, hi] = [Math.min(...ys), Math.max(...ys)];
  const x = (i) => 3 + (i / Math.max(1, values.length - 1)) * (w - 6);
  const y = (v) => (hi === lo ? h / 2 : h - 4 - ((v - lo) / (hi - lo)) * (h - 8));
  const line = pts.map(([i, v]) => `${x(i).toFixed(1)},${y(v).toFixed(1)}`).join(" ");
  const [li, lv] = pts[pts.length - 1];
  return `<svg class="spark" viewBox="0 0 ${w} ${h}" width="100%" height="${h}" preserveAspectRatio="none"><polyline points="${line}" fill="none" stroke="var(--accent)" stroke-width="1.8" vector-effect="non-scaling-stroke"/><circle cx="${x(li)}" cy="${y(lv)}" r="2.5" fill="var(--accent)"/></svg>`;
}

/** Lower is better for every metric shown: colour a value against the previous flight's. */
function trend(now, before) {
  if (now === null || now === undefined || before === null || before === undefined || before === 0) return "";
  const change = (now - before) / Math.abs(before);
  return change < -0.1 ? "better" : change > 0.1 ? "worse" : "";
}

const worstOvershoot = (f) => {
  const v = f.summary.axes.map((a) => a.overshoot_pct).filter((x) => x !== null && x !== undefined);
  return v.length ? Math.max(...v) : null;
};

function historyView() {
  const head = `<h1>Flight history</h1>
    <p class="lead">Every log in Documents/ArduTune/logs, filed under the vehicle that flew it (each log records its flight controller's ID). Flights are numbered in the order flown, from the flight controller's power-up count, so the order is right even without GPS; a date is shown only when the log had a GPS fix. Lower is better for every number; green and red mark a change of more than 10% from the previous flight.</p>
    <div class="actions" style="margin-top:0">
      <button class="btn" data-action="refreshHistory" ${state.historyBusy ? "disabled" : ""}>Refresh</button>
      <button class="btn" data-action="importLog">Add a log file…</button>
      <span class="hint">Logs downloaded with another ground station or copied off the SD card can be added too.</span>
    </div>`;
  const groups = state.history;
  if (!groups) return head + busy("Reading flight logs…");
  if (!groups.length) return head + `<div class="card"><div class="muted">No logs yet. Download some from Logs on vehicle, or add log files.</div></div>`;
  return (
    head +
    (state.historyBusy ? busy("Reading new logs…") : "") +
    groups
      .map((g) => {
        const flights = g.flights;
        const oldest = [...flights].reverse();
        const metric = (label, unit, get, digits = 0) => {
          const vals = oldest.map(get);
          const latest = vals[vals.length - 1];
          return `<div class="metric"><div class="label">${label}</div><div class="value">${latest === null || latest === undefined ? "–" : `${num(latest, digits)}<span class="unit"> ${unit}</span>`}</div>${sparkline(vals)}</div>`;
        };
        const rows = flights
          .map((f, i) => {
            const prev = flights[i + 1];
            const s = f.summary;
            const cell = (v, pv, digits = 0) => `<span class="${trend(v, pv)}">${v === null || v === undefined ? "–" : num(v, digits)}</span>`;
            const axes = ["roll", "pitch", "yaw"]
              .map((ax) => {
                const a = s.axes.find((x) => x.axis === ax);
                const pa = prev?.summary.axes.find((x) => x.axis === ax);
                return cell(a?.overshoot_pct, pa?.overshoot_pct);
              })
              .join(" / ");
            const osc = s.axes.filter((a) => a.oscillation_hz).map((a) => `${a.axis} ${num(a.oscillation_hz)} Hz`);
            const open = state.historyOpen.has(f.path);
            const changes = f.changes.length
              ? `<button class="link" data-action="toggleChanges" data-path="${esc(f.path)}">${f.changes.length} change${f.changes.length === 1 ? "" : "s"}</button>`
              : prev
                ? `<span class="muted">none</span>`
                : `<span class="muted">first</span>`;
            return `<tr>
              <td style="min-width:190px"><button class="link" style="white-space:nowrap" data-action="openHistoryLog" data-path="${esc(f.path)}" title="Open ${esc(f.file_name)}">#${f.number} · ${f.when ? `${esc(f.when)} UTC` : "no GPS time"}</button>
                <div class="muted">${s.boot_count ? `boot ${s.boot_count}, ${minutes(s.after_boot_s)} after power-on · ` : ""}${minutes(s.flight_s)} flight${s.autotune ? " · AutoTune" : ""}</div></td>
              <td class="num">${cell(s.vibration, prev?.summary.vibration)}</td>
              <td class="num">${cell(s.noise_pre, prev?.summary.noise_pre, 1)} → ${cell(s.noise_post, prev?.summary.noise_post, 1)}</td>
              <td class="num">${axes}${osc.length ? `<div class="warn-text">oscillation: ${esc(osc.join(", "))}</div>` : ""}</td>
              <td style="white-space:nowrap">${changes}</td>
            </tr>
            ${
              open
                ? `<tr class="note"><td colspan="5"><table class="changes"><tr><th>Parameter</th><th class="num">Before</th><th></th><th class="num">This flight</th></tr>${f.changes
                    .map(([n, b, a]) => `<tr><td class="mono">${esc(n)}</td><td class="num">${fmt(b)}</td><td class="arrow">→</td><td class="num new">${fmt(a)}</td></tr>`)
                    .join("")}</table></td></tr>`
                : ""
            }`;
          })
          .join("");
        const name = g.id
          ? `<button class="link" data-action="askNameDrone" data-uid="${esc(g.id)}" data-from-log="${g.from_log}" data-board="${esc(g.board ?? "")}" data-name="${esc(g.named ? g.label : "")}">${g.named ? "Rename" : "Name"}</button>`
          : "";
        return `<div class="card">
          <h2>${esc(g.label)} ${g.connected ? chip("pass", "connected") : ""}</h2>
          <p class="muted">${g.board ? `${esc(g.board)} · ` : ""}${flights.length} flight${flights.length === 1 ? "" : "s"}${name ? ` · ${name}` : ""}${g.id ? "" : " · these logs don't record a flight controller ID"}</p>
          ${
            flights.length > 1
              ? `<div class="metrics">
                  ${metric("Vibration", "m/s²", (f) => f.summary.vibration)}
                  ${metric("Gyro noise after filters", "deg/s", (f) => f.summary.noise_post, 1)}
                  ${metric("Worst overshoot", "%", worstOvershoot)}
                </div>`
              : ""
          }
          <div class="scroll-x"><table>
            <tr><th>Flight</th><th class="num">Vibration<div class="muted">m/s²</div></th><th class="num">Gyro noise<div class="muted">before → after filters, deg/s</div></th><th class="num">Overshoot<div class="muted">roll / pitch / yaw, %</div></th><th>Tuning changes</th></tr>
            ${rows}
          </table></div>
        </div>`;
      })
      .join("")
  );
}

/** Asks for a name for a new vehicle; offers renaming a named one. */
function nameCard(v) {
  if (!v.uid)
    return `<div class="banner info">This flight controller doesn't report a chip ID, so its backups can't be told apart from other vehicles'. They are labelled by system ID (${v.system_id}).</div>`;
  if (v.name)
    return `<div class="actions" style="margin-top:-6px"><button class="link" data-action="askNameDrone" data-uid="${esc(v.uid)}" data-name="${esc(v.name)}">Rename this vehicle</button></div>`;
  return `<div class="card"><h2>Name this vehicle</h2>
      <p class="muted">ArduTune recognises each flight controller by its chip ID. A name makes this vehicle's backups easy to find when you have several.</p>
      <div class="actions" style="margin-top:0">
        <input type="text" id="droneNameInline" spellcheck="false" maxlength="40" placeholder="e.g. 5-inch freestyle, 7-inch long range">
        <button class="btn primary" data-action="saveDroneName" data-uid="${esc(v.uid)}">Save</button>
      </div></div>`;
}

function connectionView() {
  const v = state.vehicle;
  if (v)
    return `
      <h1>${esc(v.name ?? "Vehicle")}</h1>
      <p class="lead">Connected over <code>${esc(v.address)}</code>.</p>
      ${nameCard(v)}
      <div class="stats">
        ${stat("Type", esc(v.kind), v.ardupilot ? "ArduPilot" : "not ArduPilot")}
        ${v.uid ? stat("Flight controller", esc(v.board ?? "unknown board"), `ID …${esc(shortId(v.uid))}`) : ""}
        ${v.uid ? "" : stat("System ID", v.system_id)}
        ${stat("State", v.armed ? chip("fail", "armed") : chip("pass", "disarmed"))}
        ${stat("Parameters", v.params)}
      </div>
      ${v.armed ? `<div class="banner fail">The vehicle is armed. Parameters can only be written while disarmed.</div>` : ""}
      ${state.vehicleBusy ? busy(state.vehicleBusy) : ""}
      <div class="actions">
        <button class="btn" data-action="go" data-view="vlogs">Logs on vehicle</button>
        <button class="btn" data-action="refreshStatus">Refresh state</button>
        <button class="btn" data-action="reboot" ${v.armed ? "disabled" : ""}>Reboot</button>
        <button class="btn danger" data-action="disconnect">Disconnect</button>
      </div>`;

  const ports = state.ports;
  const portList = !ports
    ? busy("Looking for serial ports…")
    : `<label class="port"><input type="radio" name="port" value="auto" ${state.address === "auto" ? "checked" : ""}> <span><b>Automatic</b> <span class="muted">— the USB port that looks like a flight controller</span></span></label>
       ${ports
         .map(
           (p) =>
             `<label class="port"><input type="radio" name="port" value="${esc(p.address)}" ${state.address === p.address ? "checked" : ""}> <span><code>${esc(p.name)}</code> <span class="muted">${esc(p.description)}</span> ${p.likely_autopilot ? chip("pass", "flight controller") : ""}</span></label>`,
         )
         .join("")}`;
  return `
    <h1>Connect to the vehicle</h1>
    <p class="lead">Over USB, a telemetry radio or the network. ArduTune reads all parameters on connect and never writes without showing you the changes first.</p>
    <div class="card">
      <h2>Port</h2>
      ${portList}
      <div class="actions">
        <input type="text" id="address" value="${esc(state.address)}" spellcheck="false" placeholder="serial:/dev/ttyACM0:115200 · udpin:0.0.0.0:14550 · tcpout:host:5760">
        <button class="btn primary" data-action="connect" ${state.connecting ? "disabled" : ""}>${state.connecting ? "Connecting…" : "Connect"}</button>
        <button class="btn" data-action="refreshPorts">Rescan</button>
      </div>
      <div class="actions"><span class="hint">Address formats: <code>serial:&lt;port&gt;:&lt;baud&gt;</code>, <code>udpin:0.0.0.0:14550</code>, <code>tcpout:&lt;host&gt;:&lt;port&gt;</code>.</span></div>
    </div>`;
}

function notConnected(what) {
  return `<div class="banner info">Connect to the vehicle to ${what}.</div><button class="btn primary" data-action="go" data-view="connection">Connect…</button>`;
}

function vehicleLogsView() {
  const head = `<h1>Logs on the vehicle</h1><p class="lead">Download a log from the flight controller's SD card and analyse it. Logs are saved in Documents/ArduTune/logs.</p>`;
  if (!state.vehicle) return head + notConnected("download its logs");
  const logs = state.vehicleLogs;
  return `${head}
    ${state.vehicleBusy ? busy(state.vehicleBusy) : ""}
    <div class="card">
      ${
        !logs
          ? `<div class="muted">No list yet.</div>`
          : logs.length
            ? `<table><tr><th class="num">ID</th><th>Date (UTC)</th><th class="num">Size</th><th></th></tr>
          ${logs
            .slice()
            .reverse()
            .map(
              (l) =>
                `<tr><td class="num">${l.id}</td><td>${esc(l.date ?? "no GPS time")}</td><td class="num">${num(l.size_mb, 1)} MB</td>
                <td class="num"><button class="btn" data-action="downloadLog" data-id="${l.id}" ${state.vehicleBusy ? "disabled" : ""}>Download and open</button>
                <button class="btn danger" data-action="askDelete" data-id="${l.id}" ${state.vehicleBusy ? "disabled" : ""}>Delete</button></td></tr>`,
            )
            .join("")}</table>`
            : `<div class="muted">No logs on the flight controller.</div>`
      }
      <div class="actions"><button class="btn" data-action="refreshVehicleLogs" ${state.vehicleBusy ? "disabled" : ""}>Refresh list</button>
        ${logs?.length ? `<button class="btn danger" data-action="askErase" ${state.vehicleBusy ? "disabled" : ""}>Erase all logs…</button>` : ""}</div>
    </div>`;
}

function paramValueText(row, v) {
  const named = row.doc?.values?.find(([x]) => x === v);
  const units = row.doc?.units ? ` ${row.doc.units}` : "";
  return named && !row.doc?.bits?.length ? `${fmt(v)} <span class="muted">${esc(named[1])}</span>` : `${fmt(v)}${esc(units)}`;
}

function paramEditor(row) {
  const d = row.doc;
  const staged = state.staged[row.name];
  const v = staged ?? row.value;
  let input;
  if (d?.bits?.length) {
    input = `<div class="bits">${d.bits
      .map(([b, n]) => `<label><input type="checkbox" data-pbit="${b}" ${Math.floor(v / 2 ** b) % 2 ? "checked" : ""}> ${b}: ${esc(n)}</label>`)
      .join("")}</div>`;
  } else if (d?.values?.length) {
    const known = d.values.some(([x]) => x === v);
    input = `<select id="pval">${known ? "" : `<option value="${v}" selected>${fmt(v)} (not documented)</option>`}${d.values
      .map(([x, n]) => `<option value="${x}" ${x === v ? "selected" : ""}>${fmt(x)}: ${esc(n)}</option>`)
      .join("")}</select>`;
  } else {
    input = `<input type="text" id="pval" value="${fmt(v)}" spellcheck="false" inputmode="decimal">${d?.units ? ` <span class="muted">${esc(d.units)}</span>` : ""}`;
  }
  const facts = [
    d?.range ? `Range ${fmt(d.range[0])} to ${fmt(d.range[1])}` : null,
    d?.step ? `step ${fmt(d.step)}` : null,
    row.default !== null ? `default ${fmt(row.default)}` : null,
    row.integer ? "whole numbers" : null,
    d?.reboot ? "<b>reboot required</b>" : null,
    d?.readonly ? "<b>read-only</b>" : null,
  ].filter(Boolean);
  return `<tr class="editor"><td colspan="4">
    ${d ? `<div><b>${esc(d.name)}</b></div><p style="margin:4px 0 8px">${esc(d.desc)}</p>` : `<p class="muted">No description in the bundled metadata (the name may differ in this firmware version).</p>`}
    ${facts.length ? `<div class="muted" style="margin-bottom:8px">${facts.join(" · ")}</div>` : ""}
    ${input}
    <div class="actions">
      <button class="btn primary" data-action="stageParam" data-name="${esc(row.name)}" ${d?.readonly ? "disabled" : ""}>Stage change</button>
      ${row.default !== null && row.default !== row.value ? `<button class="btn" data-action="defaultParam" data-name="${esc(row.name)}" data-value="${row.default}">Stage default (${fmt(row.default)})</button>` : ""}
      ${staged !== undefined ? `<button class="btn" data-action="unstageParam" data-name="${esc(row.name)}">Undo</button>` : ""}
    </div>
  </td></tr>`;
}

function allParamsView() {
  const head = `<h1>Parameters</h1><p class="lead">Every parameter on the vehicle with its default and description. Stage changes, then review and write them together; every value is read back to verify it.</p>`;
  if (!state.vehicle) return head + notConnected("see and change its parameters");
  const t = state.ptable;
  if (!t) return head + (state.vehicleBusy ? busy(state.vehicleBusy) : `<button class="btn primary" data-action="loadParams">Read parameters</button>`);
  const f = state.pfilter;
  const words = f.text.trim().toLowerCase().split(/\s+/).filter(Boolean);
  const groups = [...new Set(t.rows.map((r) => r.name.split("_")[0]))].sort();
  // Best matches first: the name itself, name parts ("rc opt" → RC_OPTIONS),
  // then the display name, then the description.
  const score = (r) => {
    if (!words.length) return 1;
    const name = r.name.toLowerCase();
    const parts = name.split("_");
    const title = (r.doc?.name ?? "").toLowerCase();
    const desc = (r.doc?.desc ?? "").toLowerCase();
    if (words.join("_") === name || words.join("") === name.replace(/_/g, "")) return 100;
    let ordered = 0;
    let at = 0;
    for (const w of words) {
      const i = parts.findIndex((p, k) => k >= at && p.startsWith(w));
      if (i < 0) break;
      ordered += 1;
      at = i + 1;
    }
    if (ordered === words.length) return 50 + (parts[0].startsWith(words[0]) ? 10 : 0);
    if (words.every((w) => name.includes(w))) return 30;
    if (words.every((w) => title.includes(w) || name.includes(w))) return 20;
    if (words.every((w) => title.includes(w) || name.includes(w) || desc.includes(w))) return 10;
    return 0;
  };
  const matches = t.rows
    .filter((r) => {
      if (f.group && r.name.split("_")[0] !== f.group) return false;
      if (f.changed && (r.default === null || r.default === r.value)) return false;
      if (f.standard && r.doc?.advanced) return false;
      return true;
    })
    .map((r) => [score(r), r])
    .filter(([sc]) => sc > 0)
    .sort((a, b) => b[0] - a[0] || a[1].name.localeCompare(b[1].name, undefined, { numeric: true }))
    .map(([, r]) => r);
  const shown = matches.slice(0, PARAM_ROWS);
  const staged = Object.entries(state.staged);
  const rows = shown
    .map((r) => {
      const changed = r.default !== null && r.default !== r.value;
      const s = state.staged[r.name];
      return `<tr class="prow ${state.popen === r.name ? "open" : ""}" data-action="openParam" data-name="${esc(r.name)}">
        <td class="mono">${esc(r.name)}${changed ? ` <span class="dot on" title="changed from default"></span>` : ""}</td>
        <td class="num">${s !== undefined ? `<s class="muted">${fmt(r.value)}</s> <b class="new">${fmt(s)}</b>` : paramValueText(r, r.value)}</td>
        <td class="num muted">${r.default === null ? "–" : fmt(r.default)}</td>
        <td>${esc(r.doc?.name ?? "")}</td></tr>${state.popen === r.name ? paramEditor(r) : ""}`;
    })
    .join("");
  const stagedBar = staged.length
    ? `<div class="staged"><b>${staged.length} staged change${staged.length > 1 ? "s" : ""}</b>
        <span class="muted">${staged.map(([n, v]) => `${esc(n)} → ${fmt(v)}`).join(", ")}</span>
        <span style="flex:1"></span>
        <button class="btn" data-action="discardStaged">Discard</button>
        <button class="btn primary" data-action="writeStaged">Review and write…</button></div>`
    : "";
  return `${head}
    <div class="card ptools">
      <input type="text" id="psearch" value="${esc(f.text)}" placeholder="Search names and descriptions, e.g. notch, RC_OPTIONS, failsafe" spellcheck="false" autocomplete="off">
      <select data-pfilter="group"><option value="">All groups</option>${groups.map((g) => `<option ${g === f.group ? "selected" : ""}>${esc(g)}</option>`).join("")}</select>
      <label><input type="checkbox" data-pfilter="changed" ${f.changed ? "checked" : ""}> Changed from default</label>
      <label><input type="checkbox" data-pfilter="standard" ${f.standard ? "checked" : ""}> Hide advanced</label>
      <button class="btn" data-action="loadParams" ${state.vehicleBusy ? "disabled" : ""}>Reload</button>
      ${stagedBar}
    </div>
    ${state.vehicleBusy ? busy(state.vehicleBusy) : ""}
    ${t.has_defaults ? "" : `<div class="banner info">This firmware doesn't report default values, so "changed from default" isn't available.</div>`}
    <div class="card" style="padding:6px 10px">
      <table class="ptable"><tr><th>Name</th><th class="num">Value</th><th class="num">Default</th><th>What it is</th></tr>${rows}</table>
      <div class="muted" style="padding:8px">${matches.length > shown.length ? `Showing ${shown.length} of ${matches.length}: refine the search to see the rest.` : `${matches.length} of ${t.rows.length} parameters.`}
        Descriptions: ArduPilot parameter metadata from ${esc(t.docs_fetched)}.</div>
    </div>`;
}

function firmwareView() {
  const head = `<h1>Firmware</h1><p class="lead">What the flight controller runs, newer ArduPilot releases for its board, and installing firmware over USB.</p>`;
  if (!state.vehicle) return head + notConnected("see and update its firmware");
  const fw = state.fw;
  if (!fw) return head + (state.vehicleBusy ? busy(state.vehicleBusy) : `<button class="btn primary" data-action="loadFirmware">Read firmware version</button>`);
  const r = state.releases;
  const releases =
    r === "loading"
      ? busy("Checking firmware.ardupilot.org…")
      : Array.isArray(r)
        ? `<table><tr><th>Channel</th><th>Version</th><th></th><th></th></tr>${r
            .map((x) => {
              const what = { stable: "Stable: recommended", beta: "Beta: release candidate", latest: "Latest: development build" }[x.channel];
              const ver = x.error ? `<span class="muted">couldn't check: ${esc(x.error)}</span>` : x.version ? esc(x.version) : `<span class="muted">no build for this board</span>`;
              return `<tr><td>${esc(what)}</td><td>${ver}</td><td>${x.newer ? chip("pass", "newer") : x.version && x.version === fw.version ? chip("info", "installed") : ""}</td>
                <td class="num">${x.version ? `<button class="btn ${x.channel === "stable" && x.newer ? "primary" : ""}" data-action="installRelease" data-channel="${x.channel}" ${state.vehicleBusy ? "disabled" : ""}>Install…</button>` : ""}</td></tr>`;
            })
            .join("")}</table>`
        : `<p class="muted" style="margin:0">ArduTune works offline; checking for releases contacts firmware.ardupilot.org.</p>`;
  return `${head}
    <div class="stats">
      ${stat("Firmware", esc(fw.version ?? "unknown"), esc(fw.description ?? ""))}
      ${stat("Board", esc(fw.board ?? "unknown"), fw.board_id ? `board ID ${fw.board_id}` : "")}
      ${stat("Build", esc(fw.git_hash ?? "–"), fw.version?.includes("dev") ? "development build" : fw.version?.includes("beta") ? "beta" : "release")}
    </div>
    <div class="card"><h2>Releases for ${esc(fw.board ?? "this board")}</h2>${releases}
      <div class="actions">
        ${Array.isArray(r) ? "" : `<button class="btn primary" data-action="checkReleases" ${!fw.board ? "disabled" : ""}>Check for updates</button>`}
        <button class="btn" data-action="pickFirmware" ${state.vehicleBusy ? "disabled" : ""}>Install from file…</button>
        <span class="hint">A firmware file (.apj) must be built for this board; ArduTune checks the board ID before writing anything.</span>
      </div>
    </div>
    ${fw.banner?.length ? `<div class="card"><h2>Startup banner</h2>${list(fw.banner)}</div>` : ""}`;
}

function setupView() {
  const head = `<h1>Setup guide</h1><p class="lead">Your vehicle's setup checked step by step, from frame to logging, against its live parameters and status. Fix the steps in order; you review every change before it is written, and each value is verified.</p>`;
  if (!state.vehicle) return head + notConnected("check its setup");
  const g = state.setup;
  const again = `<div class="actions"><button class="btn" data-action="setupGuide" ${state.vehicleBusy ? "disabled" : ""}>${g ? "Check again" : "Check the setup"}</button></div>`;
  if (!g) return head + (state.vehicleBusy ? busy(state.vehicleBusy) : again);
  const label = { pass: "OK", warn: "check", fail: "fix", info: "not checked" };
  const counts = ["fail", "warn", "pass"].map((st) => [st, g.steps.filter((x) => x.status === st).length]);
  return `${head}
    <div class="stats">${counts.map(([st, n]) => stat(st === "fail" ? "To fix" : st === "warn" ? "To check" : "OK", n)).join("")}</div>
    ${state.vehicleBusy ? busy(state.vehicleBusy) : ""}
    ${g.steps
      .map(
        (st, i) => `<div class="card">
        <h2>${i + 1}. ${esc(st.title)} ${chip(st.status, label[st.status])}</h2>
        <p class="muted" style="margin-top:-4px">${esc(st.why)}</p>
        <ul class="checks">${st.items
          .map((it) => `<li><div>${chip(it.status)}</div><div><div class="title">${esc(it.title)}</div>${it.detail ? `<div class="detail">${esc(it.detail)}</div>` : ""}</div></li>`)
          .join("")}</ul>
        ${
          st.changes.length
            ? `<h2 style="margin-top:14px">Suggested changes</h2>${changesTable(st.changes, true)}
              <div class="actions"><button class="btn primary" data-action="write" data-source="setup:${esc(st.id)}" data-title="${esc(st.title.toLowerCase())} settings" ${state.vehicleBusy ? "disabled" : ""}>Write to vehicle…</button></div>`
            : ""
        }
      </div>`,
      )
      .join("")}
    ${again}`;
}

const CAL_STATUS = {
  waiting: ["info", "starting"],
  running: ["info", "turn the vehicle"],
  success: ["pass", "calibrated"],
  failed: ["fail", "failed"],
  bad_orientation: ["fail", "orientation wrong"],
  bad_radius: ["fail", "field strength wrong"],
  bad_offsets: ["fail", "offsets too large"],
  bad_scaling: ["fail", "scaling wrong"],
  bad_fit: ["fail", "poor fit"],
};

/** The 80 directions the calibration wants to see, filled as they are covered. */
function coverage(mask) {
  const cells = Array.from({ length: 80 }, (_, i) => (mask[i >> 3] >> (i & 7)) & 1);
  return `<div class="coverage" title="Directions covered">${cells.map((on) => `<span class="${on ? "on" : ""}"></span>`).join("")}</div>`;
}

function compassView() {
  const head = `<h1>Compass calibration</h1><p class="lead">The flight controller calibrates every compass while you turn the vehicle through all directions. A good calibration gives a steady heading; a bad one causes "toilet-bowling" and fly-aways in GPS modes.</p>`;
  if (!state.vehicle) return head + notConnected("calibrate its compass");
  const { cal, running, accepted } = state.compass;
  const how = `<div class="card"><h2>How</h2><ul class="plain">
      <li>Go outdoors or at least away from steel: desks, cars, rebar floors, speakers, phones. Take off any metal watch.</li>
      <li>USB power is enough. Fit everything that flies with the vehicle (battery, GPS mast, camera) in its flight position, so the calibration includes it.</li>
      <li>Hold the vehicle and turn it slowly so each side points down in turn (nose, tail, left, right, top, bottom), then roll it through the in-between angles until the bar is full.</li>
    </ul></div>`;
  const start = (label) => `<div class="actions"><button class="btn primary" data-action="startCompassCal" ${state.vehicleBusy || state.vehicle.armed ? "disabled" : ""}>${label}</button></div>`;
  if (!cal) return head + how + start("Start calibration");
  const all = cal.compasses;
  const finished = all.length > 0 && all.every((c) => c.done);
  const ok = finished && all.every((c) => c.ok);
  const saved = ok && (all.every((c) => c.autosaved) || accepted);
  const cards = all
    .map((c) => {
      const [kind, text] = CAL_STATUS[c.status] ?? ["info", c.status];
      const ofs = c.offsets ? c.offsets.map((v) => num(v)).join(", ") : null;
      return `<div class="card">
        <h2>Compass ${c.id + 1} ${chip(kind, text)}</h2>
        <div class="bar big"><div style="width:${c.percent}%"></div></div>
        <div class="muted" style="margin:6px 0">${c.percent}%${c.attempt > 1 ? ` · attempt ${c.attempt}` : ""}${c.fitness !== null ? ` · fitness ${num(c.fitness, 1)} (lower is better; over 16 is rejected)` : ""}${ofs ? ` · offsets ${ofs}` : ""}</div>
        ${c.done ? "" : coverage(c.coverage)}
        ${c.advice ? `<div class="advice"><b>What it means:</b> ${esc(c.advice)}</div>` : ""}
      </div>`;
    })
    .join("");
  const banner = !finished
    ? running
      ? `<div class="banner info">Keep turning the vehicle slowly through every direction.${all.length ? "" : " Waiting for the flight controller…"}</div>`
      : `<div class="banner warn">Calibration stopped.</div>`
    : saved
      ? `<div class="banner pass"><b>Calibration saved.</b> Reboot the flight controller to use it, then check the heading in a ground station matches the vehicle's nose.</div>`
      : ok
        ? `<div class="banner warn">Every compass calibrated, but the result wasn't saved automatically.</div>`
        : `<div class="banner fail">The calibration failed and nothing was saved. Read the advice below, then try again.</div>`;
  const buttons = running
    ? `<button class="btn" data-action="cancelCompassCal">Cancel</button>`
    : saved
      ? `<button class="btn primary" data-action="reboot" ${state.vehicleBusy ? "disabled" : ""}>Reboot now</button>`
      : ok
        ? `<button class="btn primary" data-action="acceptCompassCal">Save the calibration</button>`
        : `<button class="btn primary" data-action="startCompassCal" ${state.vehicleBusy ? "disabled" : ""}>Try again</button>`;
  return `${head}${banner}${cards}<div class="actions">${buttons}</div>
    ${cal.messages.length ? `<div class="card"><h2>What the vehicle said</h2>${list(cal.messages)}</div>` : ""}
    ${finished ? "" : how}`;
}

const SPIN = { cw: "clockwise ↻", ccw: "counter-clockwise ↺" };

/** Top view of the expected layout: nose up, each motor with its test letter, number and direction. */
function motorDiagram(l, spinning) {
  const c = 120;
  const at = (p) => {
    const a = (p.angle * Math.PI) / 180;
    return [c + 74 * Math.sin(a), c + 10 - 74 * Math.cos(a)];
  };
  const arms = l.positions.map((p) => at(p)).map(([x, y]) => `<line x1="${c}" y1="${c + 10}" x2="${x}" y2="${y}" class="arm"/>`);
  const motors = l.positions.map((p, i) => {
    const [x, y] = at(p);
    return `<circle cx="${x}" cy="${y}" r="29" class="prop ${spinning === i ? "spinning" : ""}"/>
        <text x="${x}" y="${y - 3}" class="letter">${esc(p.letter)}</text>
        <text x="${x}" y="${y + 14}" class="sub">M${p.motor} ${p.spin === "cw" ? "↻" : "↺"}</text>`;
  });
  return `<svg viewBox="0 0 240 250" class="quad" role="img" aria-label="Expected motor layout, seen from above">
    <text x="${c}" y="12" class="sub">FRONT</text>
    ${arms.join("")}
    <rect x="${c - 18}" y="${c - 8}" width="36" height="36" rx="7" class="body"/>
    <path d="M${c - 8} ${c + 4} L${c} ${c - 6} L${c + 8} ${c + 4} Z" class="nose"/>
    ${motors.join("")}
  </svg>`;
}

function motorsView() {
  const head = `<h1>Motor test</h1><p class="lead">Checks every motor is wired to the right output and spins the right way. ArduTune spins one motor at a time; you say which one turned and in which direction, and it works out the fix.</p>`;
  if (!state.vehicle) return head + notConnected("test its motors");
  const m = state.motors;
  const l = m.layout;
  if (!l) return head + busy("Reading the frame layout…");
  if (l.error) return head + `<div class="banner warn">${esc(l.error)}</div>`;
  const off = !m.propsOff || state.vehicleBusy || m.spinning !== null || state.vehicle.armed;
  const safety = `<div class="banner fail"><b>Remove all propellers.</b> The motors really spin. Connect the flight battery (ESCs need it) and keep clear of the motors.
      <label class="check"><input type="checkbox" id="propsOff" ${m.propsOff ? "checked" : ""}> The propellers are off</label></div>`;
  const throttle = `<label>Throttle <select id="mthrottle">${[5, 7, 10, 15].map((t) => `<option value="${t}" ${t === m.throttle ? "selected" : ""}>${t}%</option>`).join("")}</select></label>
    <span class="muted">for ${MOTOR_SECONDS} s. Raise it if a motor only twitches.</span>`;
  const options = (i) =>
    `<option value="" ${m.seen[i].spun === null ? "selected" : ""}>Which motor spun?</option>${l.positions
      .map((p, j) => `<option value="${j}" ${m.seen[i].spun === j ? "selected" : ""}>${esc(p.name)}</option>`)
      .join("")}<option value="none" ${m.seen[i].spun === "none" ? "selected" : ""}>Nothing spun</option>`;
  const dirs = (i) =>
    `<option value="" ${!m.seen[i].spin ? "selected" : ""}>Direction?</option>${Object.entries(SPIN)
      .map(([v, label]) => `<option value="${v}" ${m.seen[i].spin === v ? "selected" : ""}>${label}</option>`)
      .join("")}`;
  const rows = l.positions
    .map(
      (p, i) => `<div class="mrow">
        <button class="btn ${m.spinning === i ? "primary" : ""}" data-action="spinMotor" data-index="${i}" ${off ? "disabled" : ""}>${m.spinning === i ? "Spinning…" : `Spin ${esc(p.letter)}`}</button>
        <div class="mwhat">${l.known ? `<b>${esc(p.name)}</b>, ${SPIN[p.spin]}<div class="muted">motor ${p.motor}${p.output ? ` on output ${p.output}` : ", no output"}</div>` : `<span class="muted">test ${esc(p.letter)}</span>`}</div>
        ${l.known ? `<div class="mseen"><select data-mspun="${i}">${options(i)}</select><select data-mspin="${i}" ${m.seen[i].spun === "none" ? "disabled" : ""}>${dirs(i)}</select></div>` : ""}
      </div>`,
    )
    .join("");
  const complete = m.seen.every((o) => o.spun !== null && (o.spun === "none" || o.spin));
  const v = m.verdict;
  const verdict = v
    ? `<div class="card"><h2>Result ${v.ok ? chip("pass", "OK") : chip("fail", "fix")}</h2>
        <ul class="checks">${v.items
          .map((it) => `<li><div>${chip(it.status)}</div><div><div class="title">${esc(it.title)}</div>${it.detail ? `<div class="detail">${esc(it.detail)}</div>` : ""}</div></li>`)
          .join("")}</ul>
        ${
          v.changes.length
            ? `${changesTable(v.changes, true)}
              ${v.needs_reboot ? `<p class="muted">Motor direction changes take effect after a reboot (Connection → Reboot). Then test again.</p>` : `<p class="muted">Then test again.</p>`}
              <div class="actions"><button class="btn primary" data-action="writeMotors" ${state.vehicleBusy ? "disabled" : ""}>Write to vehicle…</button></div>`
            : ""
        }</div>`
    : "";
  return `${head}${safety}
    <div class="card">
      <h2>${esc(l.frame)}</h2>
      ${l.known ? "" : `<p class="muted">ArduTune knows the expected layout for quads only. Tests still go clockwise from the front: compare with the motor order diagram for your frame in the ArduPilot documentation.</p>`}
      <div class="motors">
        ${l.known ? motorDiagram(l, m.spinning) : ""}
        <div>
          <div class="form-inline">${throttle}</div>
          <div class="mrows">${rows}</div>
          ${l.known ? `<p class="muted">Watch the motor bell from above; a strip of tape on it makes the direction easy to see.</p>
            <div class="actions"><button class="btn primary" data-action="checkMotors" ${complete && !state.vehicleBusy ? "" : "disabled"}>Check the results</button></div>` : ""}
        </div>
      </div>
    </div>
    ${verdict}`;
}

const CHEMISTRIES = [
  ["li_po", "LiPo (4.2 V full)"],
  ["li_po_hv", "LiPo HV (4.35 V full)"],
  ["li_ion", "Li-ion (4.1 V full)"],
];
const CHEM_NAMES = { li_po: "LiPo", li_po_hv: "LiPo HV", li_ion: "Li-ion" };

function tuneView() {
  const head = `<h1>Initial tune</h1><p class="lead">Starting values for a new build from its prop size and battery: gyro and rate filters, acceleration limits, thrust curve and battery range. They are the values ArduPilot's initial tune parameters calculator gives, so the first hover and AutoTune start from somewhere sensible.</p>`;
  if (!state.vehicle) return head + notConnected("calculate its starting values");
  const t = state.tune;
  if (!t.pack) return head + busy("Reading the battery settings…");
  const r = t.result;
  const cellOptions = Array.from({ length: 14 }, (_, i) => i + 1)
    .map((n) => `<option value="${n}" ${n === t.cells ? "selected" : ""}>${n}S</option>`)
    .join("");
  const chemOptions = CHEMISTRIES.map(([v, label]) => `<option value="${v}" ${v === t.chemistry ? "selected" : ""}>${label}</option>`).join("");
  const form = `<div class="card">
    <h2>Your build</h2>
    <div class="form">
      <label>Prop size <span><input type="text" id="tprop" value="${esc(t.prop)}" placeholder="5" inputmode="decimal" spellcheck="false" style="width:6em"> inches</span></label>
      <label>Battery <span><select id="tcells">${cellOptions}</select> <select id="tchem">${chemOptions}</select></span></label>
    </div>
    <p class="muted">${t.pack.from_vehicle ? `The battery is read from MOT_BAT_VOLT_MAX/MIN (${t.pack.cells}S ${esc(CHEM_NAMES[t.pack.chemistry])}). Change it if that's not the pack you fly.` : "MOT_BAT_VOLT_MAX/MIN aren't set for a pack yet: choose the cell count and type of the battery you fly."}</p>
    <div class="actions"><button class="btn primary" data-action="calculateTune" ${state.vehicleBusy ? "disabled" : ""}>Calculate</button></div>
  </div>`;
  if (!r) return head + form + (state.vehicleBusy ? busy(state.vehicleBusy) : "");
  const tuned = r.looks_tuned
    ? `<div class="banner warn"><b>This vehicle looks tuned:</b> its roll or pitch rate gains differ from the defaults. These are starting values for a new build; writing them replaces filter and acceleration settings a tune may have chosen. Untick anything you want to keep.</div>`
    : "";
  return `${head}${form}
    <div class="card">
      <h2>Starting values for ${esc(fmt(r.prop_in))}" props, ${r.cells}S ${esc(CHEM_NAMES[r.chemistry])}</h2>
      ${tuned}
      ${
        r.changes.length
          ? `${changesTable(r.changes, true)}
            <div class="actions"><button class="btn primary" data-action="writeTune" ${state.vehicleBusy ? "disabled" : ""}>Write to vehicle…</button></div>`
          : `<div class="banner pass">The vehicle already has these values.</div>`
      }
    </div>
    <div class="card"><h2>After the first hover</h2>${list(r.after)}</div>`;
}

function prearmView() {
  const head = `<h1>Pre-arm check</h1><p class="lead">Why won't it arm? ArduTune asks the flight controller to run its pre-arm checks, listens to what it says and explains each problem. Start a check and try to arm while it listens to catch the reason it refuses.</p>`;
  if (!state.vehicle) return head + notConnected("check why it won't arm");
  const r = state.arming;
  const props = `<div class="banner warn"><b>Remove the propellers</b> before you try arming indoors or on the bench. If arming succeeds, the motors start spinning straight away.</div>`;
  const run = `${props}<div class="actions"><button class="btn primary" data-action="armingCheck" ${state.vehicleBusy ? "disabled" : ""}>${r ? "Check again" : "Run the check"}</button></div>`;
  if (!r) return head + (state.vehicleBusy ? busy(state.vehicleBusy) : run);
  const gps = r.gps.length
    ? r.gps
        .map(
          (g) =>
            `${esc(g.fix)}${g.satellites !== null ? `, ${g.satellites} sats` : ""}${g.hdop !== null && g.fix !== "no GPS" ? `, HDOP ${num(g.hdop, 2)}` : ""}${g.speed > 0.5 ? `, moving ${num(g.speed, 1)} m/s` : ""}`,
        )
        .join("; ")
    : "no data";
  const banner = r.armed
    ? `<div class="banner info">The vehicle is armed.</div>`
    : r.ready
      ? `<div class="banner pass">No pre-arm problems reported: it should arm. If it still refuses, run the check again and try arming while it listens.</div>`
      : `<div class="banner fail">It won't arm until the problems marked FAIL are fixed. Fix them in the order shown.</div>`;
  const kind = (st) => (st === "fail" ? "fail" : st === "warn" ? "warn" : "info");
  return `${head}
    <div class="stats">
      ${stat("Flight mode", esc(r.mode ?? "unknown"))}
      ${stat("GPS", esc(gps))}
      ${stat("Ready to arm", r.armed ? chip("info", "armed") : r.ready ? chip("pass", "yes") : chip("fail", "no"))}
    </div>
    ${banner}
    ${state.vehicleBusy ? busy(state.vehicleBusy) : ""}
    ${r.problems
      .map(
        (p) => `<div class="card">
        <h2>${chip(kind(p.status), p.status)} <span class="muted">${esc(p.area)}:</span> ${esc(p.title)}</h2>
        ${p.messages.length ? `<div class="muted" style="margin-bottom:6px">Vehicle says: ${p.messages.map((m) => `<code>${esc(m)}</code>`).join(" ")}</div>` : ""}
        ${p.advice ? `<div class="advice"><b>What to do:</b> ${esc(p.advice)}</div>` : ""}
        ${
          p.changes?.length
            ? `<h2 style="margin-top:14px">Suggested changes</h2>${changesTable(p.changes, true)}
              <div class="actions">
                <button class="btn primary" data-action="write" data-source="prearm:${esc(p.area)}" data-title="${p.area === "GPS" ? "GPS off and landing failsafes" : `${esc(p.area.toLowerCase())} fixes`}" ${state.vehicleBusy ? "disabled" : ""}>Write to vehicle…</button>
                ${p.area === "GPS" ? `<button class="btn" data-action="askGpsSwitch" ${state.vehicleBusy ? "disabled" : ""}>Put GPS on a switch instead…</button>` : ""}
              </div>`
            : ""
        }
      </div>`,
      )
      .join("")}
    <div class="card"><h2>How to arm this vehicle</h2>${list(r.how_to_arm)}</div>
    ${r.messages.length ? `<div class="card"><h2>Everything the vehicle said</h2>${list(r.messages)}</div>` : ""}
    ${run}`;
}

/** Backups grouped by vehicle: the connected one first, then the others by their latest backup, untagged files last. */
function backupGroups(backups) {
  const groups = new Map();
  const here = state.vehicle?.uid;
  if (here && backups.some((b) => b.uid === here)) groups.set(here, []);
  for (const b of backups.filter((b) => b.uid)) {
    if (!groups.has(b.uid)) groups.set(b.uid, []);
    groups.get(b.uid).push(b);
  }
  const untagged = backups.filter((b) => !b.uid);
  return [...groups.entries()].map(([uid, list]) => ({ uid, list })).concat(untagged.length ? [{ uid: null, list: untagged }] : []);
}

function backupTable(list, untagged) {
  const v = state.vehicle;
  const busyNow = !!state.vehicleBusy;
  return `<table><tr><th>Saved (UTC)</th><th>Note</th><th class="num">Values</th><th></th></tr>
    ${list
      .map((b) => {
        const match = untagged && b.match ? `${chip(b.match.level === "same" ? "pass" : "info", b.match.level === "same" ? "this vehicle" : "probably this vehicle")} <span class="muted">${esc(b.match.why)}</span>` : "";
        return `<tr>
          <td style="white-space:nowrap">${b.saved ? esc(b.saved) : `<code>${esc(b.file_name)}</code>`}${untagged && b.system_id !== null && b.system_id !== undefined ? `<div class="muted">system ${b.system_id}</div>` : ""}</td>
          <td>${esc(b.reason ?? "")}</td>
          <td class="num">${b.params}</td>
          <td class="num" style="white-space:nowrap">
            ${match ? `<button class="btn" data-action="assignBackup" data-path="${esc(b.path)}" title="Mark it as a backup of ${esc(v?.label ?? "")}">Assign</button>` : ""}
            <button class="btn" data-action="restoreBackup" data-path="${esc(b.path)}" ${!v || busyNow ? "disabled" : ""}>Restore…</button>
            <button class="btn danger" data-action="askDeleteBackup" data-path="${esc(b.path)}">Delete</button></td></tr>
          ${match ? `<tr class="note"><td colspan="4">${match}</td></tr>` : ""}`;
      })
      .join("")}</table>`;
}

function paramsView() {
  const v = state.vehicle;
  const busyNow = !!state.vehicleBusy;
  const backups = state.backups;
  const groups = backups?.length
    ? backupGroups(backups)
        .map(({ uid, list }) => {
          if (!uid)
            return `<div class="card"><h2>Not tagged with a vehicle</h2>
              <p class="muted">Made before ArduTune recorded which vehicle a backup is from, or by another program.${v?.uid ? " Ones that look like the connected vehicle are marked; Assign files them under it." : " Connect a vehicle to see which of these look like it."}</p>
              ${backupTable(list, true)}</div>`;
          const b = list[0];
          const here = uid === v?.uid;
          const name = here ? v.label : b.vehicle;
          return `<div class="card"><h2>${esc(name)} ${here ? chip("pass", "connected") : ""}</h2>
            <p class="muted">${esc(b.board ?? "unknown board")} · ID …${esc(shortId(uid))} · ${list.length} backup${list.length === 1 ? "" : "s"}
              · <button class="link" data-action="askNameDrone" data-uid="${esc(uid)}" data-name="${esc(here ? (v.name ?? "") : b.vehicle)}">${here && !v.name ? "Name" : "Rename"}</button></p>
            ${here || !v ? "" : `<p class="muted">Not the connected vehicle: restoring one of these copies that vehicle's setup.</p>`}
            ${backupTable(list, false)}</div>`;
        })
        .join("")
    : backups
      ? `<div class="card"><h2>Backups</h2><div class="muted">No backups yet.</div></div>`
      : busy("Reading backups…");
  return `
    <h1>Backups</h1>
    <p class="lead">Back up all of a vehicle's parameters before you experiment, and put any of them back later. Each backup records which vehicle it came from, by its flight controller's chip ID. When writing changes you can also choose "Back up and write". Backups are kept in Documents/ArduTune/backups.</p>
    ${
      v
        ? `<div class="card"><h2>Back up ${esc(v.label ?? "now")}</h2>
          <div class="actions" style="margin-top:0">
            <input type="text" id="note" spellcheck="false" placeholder="Note (optional), e.g. before changing filters">
            <button class="btn primary" data-action="backupNow" ${busyNow ? "disabled" : ""}>Back up</button>
          </div>
          <div class="actions"><span class="hint">Reads every parameter from the vehicle again, so the backup matches it exactly.</span></div>
        </div>`
        : `<div class="banner info">Connect to a vehicle to back up or restore parameters. <button class="link" data-action="go" data-view="connection">Connect…</button></div>`
    }
    ${state.vehicleBusy ? busy(state.vehicleBusy) : ""}
    ${groups}
    <div class="actions"><button class="btn" data-action="pickParamFile" ${!v || busyNow ? "disabled" : ""}>Restore from another file…</button>
      <span class="hint">Restoring shows every difference first, and you choose which values to write.</span></div>`;
}

/** When a backup before writing is worth it, and how to make one. */
function backupAdvice(m, n) {
  const big = n >= 10 || m.source === "file";
  return big
    ? "<b>A backup is recommended</b> for a change this size: Back up and write saves every parameter first, so you can undo it from Backups."
    : "Back up and write saves every parameter first, so you can undo it from Backups. Worth doing before bigger changes; a backup from Backups works too.";
}

function modal() {
  const m = state.modal;
  if (!m) return "";
  let body;
  switch (m.stage) {
    case "connect":
      body = `<p>Connect to the vehicle first: the changes are checked against its current parameters before anything is written.</p>
        <div class="actions"><button class="btn primary" data-action="go" data-view="connection">Connect…</button><button class="btn" data-action="closeModal">Cancel</button></div>`;
      break;
    case "delete":
      body = `<p>This permanently removes ${m.id === null ? "every log" : "the log"} from the flight controller's SD card. It can't be undone; download anything you want to keep first.</p>
        ${m.id === null ? "" : `<p class="muted">Single logs are deleted as files over MAVLink FTP. The vehicle numbers logs from the oldest one, so the others may get new numbers.</p>`}
        ${m.newest ? `<div class="banner warn">This is the newest log. The flight controller keeps it open after disarming, and deleting an open file can corrupt the SD card, so ArduTune reboots the flight controller first, reconnects and then deletes it.</div>` : ""}
        ${state.vehicle?.armed ? `<div class="banner fail">The vehicle is armed. Disarm it first.</div>` : ""}
        <div class="actions"><button class="btn destroy" data-action="confirmDelete" ${state.vehicle?.armed ? "disabled" : ""}>${m.id === null ? "Erase all" : m.newest ? "Reboot and delete" : "Delete"}</button>
        <button class="btn" data-action="closeModal">Cancel</button></div>`;
      break;
    case "fw-loading":
      body = busy("Checking the firmware…");
      break;
    case "fw-confirm": {
      const p = m.prep;
      const cur = state.fw?.version;
      const target = m.release?.version;
      const direction = target && cur ? (m.release.newer ? "upgrade" : target === cur ? "reinstall" : "downgrade") : null;
      body = `
        <table>
          <tr><td>Firmware</td><td><b>${esc(target ?? p.label)}</b>${p.git_identity ? ` <span class="muted">(${esc(p.git_identity)})</span>` : ""}</td></tr>
          <tr><td>Installed now</td><td>${esc(cur ?? "unknown")}</td></tr>
          <tr><td>Board</td><td>${p.matches_board ? chip("pass", "matches") : chip("warn", "not checked")} ${esc(p.board_name ?? "")} · board ID ${p.board_id}</td></tr>
          <tr><td>Size</td><td>${num(p.size / 1e6, 2)} MB</td></tr>
        </table>
        ${direction === "downgrade" ? `<div class="banner warn">This is older than what's installed. Downgrading can reset or rename parameters; check them afterwards.</div>` : ""}
        ${direction === "upgrade" && cur && target && cur.split(".").slice(0, 2).join(".") !== target.split(".").slice(0, 2).join(".") ? `<div class="banner info">A new major version: read the release notes, as some parameters may be renamed or have new defaults.</div>` : ""}
        <div class="banner warn"><b>Before you start:</b> props off, keep the USB cable connected, don't power off until it says done (about a minute). The flight controller restarts twice.</div>
        <label class="port"><input type="checkbox" id="fwbackup" ${m.backup ? "checked" : ""}> <span><b>Back up parameters first</b> <span class="muted">(recommended: a new firmware can change parameters)</span></span></label>
        <div class="actions">
          <button class="btn destroy" data-action="flashFirmware" ${state.vehicle?.armed ? "disabled" : ""}>Flash firmware</button>
          <button class="btn" data-action="closeModal">Cancel</button>
        </div>`;
      break;
    }
    case "fw-flashing":
      body = busy("Flashing: keep USB connected and don't power off…");
      break;
    case "fw-done": {
      const r = m.result;
      body = `
        <div class="banner pass">Firmware written and verified${r.version ? `: now running <b>${esc(r.version)}</b>` : ""}.</div>
        ${r.backup ? `<p>Previous parameters saved to <code>${esc(r.backup)}</code>.</p>` : ""}
        ${
          !r.reconnected
            ? `<div class="banner warn">ArduTune couldn't reconnect. Unplug and replug USB, then connect again and check the parameters.</div>`
            : r.changed.length
              ? `<p><b>${r.changed.length} parameter${r.changed.length > 1 ? "s" : ""} changed</b> with the new firmware:</p>
                 <div class="scroll"><table class="changes"><tr><th>Parameter</th><th class="num">Before</th><th></th><th class="num">After</th></tr>${r.changed
                   .map((c) => `<tr><td class="mono">${esc(c.name)}</td><td class="num">${c.before === null ? "new" : fmt(c.before)}</td><td class="arrow">→</td><td class="num">${c.after === null ? "removed" : fmt(c.after)}</td></tr>`)
                   .join("")}</table></div>
                 <p class="muted">Restore any of them from Backups if needed.</p>`
              : `<p>All parameters are unchanged.</p>`
        }
        <div class="actions"><button class="btn primary" data-action="closeModal">Done</button></div>`;
      break;
    }
    case "fw-failed":
      body = `<div class="banner fail">${esc(m.error)}</div>
        <p>If the flight controller doesn't start, it is waiting in its bootloader: keep USB connected and flash again. If nothing responds at all, hold the board's BOOT button while plugging in USB and flash it with DFU (STM32CubeProgrammer, or ArduPilot's firmware with bootloader).</p>
        <div class="actions"><button class="btn" data-action="go" data-view="connection">Connection</button><button class="btn" data-action="closeModal">Close</button></div>`;
      break;
    case "deleteBackup":
      body = `<p>The backup file is deleted from this computer. This can't be undone.</p>
        <div class="actions"><button class="btn destroy" data-action="confirmDeleteBackup">Delete</button>
        <button class="btn" data-action="closeModal">Cancel</button></div>`;
      break;
    case "planning":
      body = busy("Comparing with the vehicle's parameters…");
      break;
    case "switchLearn":
    case "switchListening": {
      const listening = m.stage === "switchListening";
      body = `<p>A switch on your transmitter will turn the GPS off (switch high) or on (low), so you can choose before each flight. ArduPilot's switch function for this is "GPS Disable" (RCn_OPTION = 65).</p>
        <ul class="plain">
          <li>Turn the transmitter on, and connect the battery if the receiver only gets power from it.</li>
          <li>Pick a free two-position switch: not the arm switch or the flight mode switch.</li>
          <li>Click Start, then flip that switch from one end to the other a few times within 6 seconds.</li>
        </ul>
        <p class="muted">With GPS spoofed, put the switch on GPS off before powering up: a false position seen at start-up stays until the next reboot. With the GPS off, fly Stabilize or AltHold only.</p>
        ${m.error ? `<div class="banner warn">${esc(m.error)}</div>` : ""}
        ${listening ? busy("Flip the switch now…") : ""}
        <div class="actions">
          <button class="btn primary" data-action="detectSwitch" ${listening ? "disabled" : ""}>${m.error ? "Try again" : "Start"}</button>
          <button class="btn" data-action="closeModal" ${listening ? "disabled" : ""}>Cancel</button>
        </div>`;
      break;
    }
    case "nameDrone":
      body = `<p class="muted">Backups made on this flight controller are listed under this name.</p>
        <div class="actions" style="margin-top:0">
          <input type="text" id="droneName" value="${esc(m.name)}" spellcheck="false" maxlength="40" autofocus>
          <button class="btn primary" data-action="saveDroneName" data-uid="${esc(m.uid)}">Save</button>
          <button class="btn" data-action="closeModal">Cancel</button>
        </div>`;
      break;
    case "confirm": {
      const p = m.plan;
      const why = p.rows.some((r) => r.reason);
      const n = m.selected.size;
      const all = n === p.rows.length;
      body = `
        ${p.armed ? `<div class="banner fail">The vehicle is armed. Disarm it to write parameters.</div>` : ""}
        ${p.warning ? `<div class="banner warn"><b>Another vehicle's backup.</b> ${esc(p.warning)}</div>` : ""}
        ${
          p.rows.length
            ? `<div class="scroll"><table><tr><th><input type="checkbox" id="selectAll" ${all ? "checked" : ""} title="Select all"></th><th>Parameter</th><th class="num">On vehicle</th><th></th><th class="num">Proposed</th>${why ? "<th>Why</th>" : ""}</tr>
            ${p.rows
              .map(
                (r) =>
                  `<tr><td><input type="checkbox" data-name="${esc(r.name)}" ${m.selected.has(r.name) ? "checked" : ""}></td><td class="mono">${esc(r.name)}</td><td class="num">${fmt(r.current)}</td><td class="arrow">→</td><td class="num new">${fmt(r.value)}</td>${why ? `<td>${esc(r.reason ?? "")}</td>` : ""}</tr>`,
              )
              .join("")}</table></div>`
            : `<div class="banner pass">Nothing to change: the vehicle already has these values.</div>`
        }
        ${p.unchanged.length ? `<p class="muted">${p.unchanged.length > 12 ? `${p.unchanged.length} values already match.` : `Already set: ${p.unchanged.map(esc).join(", ")}`}</p>` : ""}
        ${p.skipped?.length ? `<p class="muted">Left out ${p.skipped.length} value${p.skipped.length > 1 ? "s" : ""} the firmware manages itself (statistics, versions, sensor IDs).</p>` : ""}
        ${p.unknown.length ? `<div class="banner warn">The vehicle has no parameter ${p.unknown.map(esc).join(", ")}; skipped.</div>` : ""}
        <div class="actions">
          ${
            p.rows.length
              ? `<button class="btn primary" data-action="confirmWrite" ${p.armed || !n ? "disabled" : ""}>Write ${n} parameter${n === 1 ? "" : "s"}</button>
                 <button class="btn" data-action="backupAndWrite" ${p.armed || !n ? "disabled" : ""}>Back up and write</button>`
              : ""
          }
          <button class="btn" data-action="closeModal">${p.rows.length ? "Cancel" : "Close"}</button>
        </div>
        ${p.rows.length ? `<p class="muted" style="margin-bottom:0">${backupAdvice(m, n)}</p>` : ""}`;
      break;
    }
    case "writing":
      body = busy(m.backup ? "Backing up, writing and verifying…" : "Writing and verifying…");
      break;
    case "done": {
      const r = m.result;
      const ok = r.results.filter((x) => !x.error).length;
      body = `
        ${r.backup ? `<p>Previous parameters saved to <code>${esc(r.backup)}</code>.</p>` : ""}
        <table>${r.results
          .map((x) => `<tr><td>${x.error ? chip("fail", "failed") : chip("pass", "ok")}</td><td class="mono">${esc(x.name)}</td><td>${x.error ? esc(x.error) : fmt(x.value)}</td></tr>`)
          .join("")}</table>
        <p><b>${ok} of ${r.results.length}</b> written and verified. Reboot the flight controller so every change takes effect.</p>
        <div class="actions">
          ${ok ? `<button class="btn primary" data-action="reboot">Reboot now</button>` : ""}
          <button class="btn" data-action="closeModal">Later</button>
        </div>`;
      break;
    }
  }
  const writing = ["planning", "confirm", "writing", "done"].includes(m.stage) && m.source !== undefined;
  const title = writing ? `${m.source === "file" ? "Restore" : "Write"} ${m.title}` : m.title;
  return `<div class="modal-back"><div class="modal"><h2>${esc(title)}</h2>${body}</div></div>`;
}

function content() {
  if (state.view !== "overview" && LOG_VIEWS.some(([id]) => id === state.view) && !state.log) return welcome();
  switch (state.view) {
    case "overview":
      return state.log || state.loadingLog ? overview() : welcome();
    case "diagnose":
      return analysisView("diagnose", diagnoseView);
    case "noise":
      return analysisView("noise", noiseView);
    case "filters":
      return analysisView("filters", filtersView);
    case "pid":
      return analysisView("pid", pidView);
    case "connection":
      return connectionView();
    case "vlogs":
      return vehicleLogsView();
    case "params":
      return paramsView();
    case "prearm":
      return prearmView();
    case "setup":
      return setupView();
    case "tune":
      return tuneView();
    case "history":
      return historyView();
    case "motors":
      return motorsView();
    case "compass":
      return compassView();
    case "firmware":
      return firmwareView();
    case "allparams":
      return allParamsView();
    default:
      return welcome();
  }
}

const root = document.getElementById("app");
root.innerHTML = `<header></header><nav></nav><main></main><div id="overlay"></div>`;
const regions = Object.fromEntries(["header", "nav", "main", "#overlay"].map((sel) => [sel, root.querySelector(sel)]));
const shown = {};

/** Replaces a region only when its HTML changed: plots are large, and this keeps scroll position. */
function put(sel, html) {
  if (shown[sel] === html) return;
  const sameView = sel === "main" && shown.view === state.view;
  const focused = document.activeElement;
  const keep = focused?.id && regions[sel].contains(focused) ? { id: focused.id, start: focused.selectionStart, end: focused.selectionEnd } : null;
  const scroll = regions[sel].scrollTop;
  shown[sel] = html;
  regions[sel].innerHTML = html;
  if (sel === "main") {
    regions.main.scrollTop = sameView ? scroll : 0;
    shown.view = state.view;
  }
  if (keep) {
    const el = document.getElementById(keep.id);
    el?.focus();
    if (el && keep.start !== null && el.setSelectionRange) el.setSelectionRange(keep.start, keep.end);
  }
}

function render() {
  const t = state.toast;
  put("header", header());
  put("nav", nav());
  put("main", content());
  put(
    "#overlay",
    modal() + (t ? `<div class="toast" data-action="closeToast" style="${t.kind === "info" ? "background:var(--info-bg);color:var(--info)" : ""}">${esc(t.text)}</div>` : ""),
  );
}

render();
