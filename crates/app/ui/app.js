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
  ptable: null, // full parameter list from the vehicle
  pfilter: { text: "", changed: false, standard: false, group: "" },
  popen: null, // parameter whose editor is open
  staged: {}, // name -> value waiting to be written
  fw: null, // firmware info from the vehicle
  releases: null, // what firmware.ardupilot.org offers for the board
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
  if (view === "prearm" && state.vehicle && !state.arming && !state.vehicleBusy) armingCheck();
  if (view === "setup" && state.vehicle && !state.setup && !state.vehicleBusy) setupGuide();
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
    set({ vehicle, connecting: false, vehicleLogs: null, arming: null, setup: null, ptable: null, staged: {}, popen: null, fw: null, releases: null });
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
async function writeChanges(source, title, path = null) {
  if (!state.vehicle) {
    set({ modal: { stage: "connect", title: `Write ${title}` } });
    return;
  }
  set({ modal: { stage: "planning", title, source } });
  try {
    const verb = source === "file" ? "restoring" : "writing";
    const plan = await invoke("prepare_changes", { source, path, label: `${verb} ${title}` });
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
  armingCheck,
  setupGuide,
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
  if (e.target.id === "psearch") set({ pfilter: { ...state.pfilter, text: e.target.value }, popen: null });
});

document.addEventListener("change", (e) => {
  const f = e.target.dataset.pfilter;
  if (f === "group") set({ pfilter: { ...state.pfilter, group: e.target.value } });
  else if (f) set({ pfilter: { ...state.pfilter, [f]: e.target.checked } });
});

document.addEventListener("keydown", (e) => {
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
    ? `<span class="dot ${v.armed ? "armed" : "on"}"></span>${esc(v.kind)} · sys ${v.system_id} · ${v.armed ? "ARMED" : "disarmed"}`
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

function connectionView() {
  const v = state.vehicle;
  if (v)
    return `
      <h1>Vehicle</h1>
      <p class="lead">Connected over <code>${esc(v.address)}</code>.</p>
      <div class="stats">
        ${stat("Type", esc(v.kind), v.ardupilot ? "ArduPilot" : "not ArduPilot")}
        ${stat("System ID", v.system_id)}
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

function prearmView() {
  const head = `<h1>Pre-arm check</h1><p class="lead">Why won't it arm? ArduTune asks the flight controller to run its pre-arm checks, listens to what it says and explains each problem. Start a check and try to arm while it listens to catch the reason it refuses.</p>`;
  if (!state.vehicle) return head + notConnected("check why it won't arm");
  const r = state.arming;
  const props = `<div class="banner warn"><b>Remove the propellers</b> before you try arming indoors or on the bench. If arming succeeds, the motors start spinning straight away.</div>`;
  const run = `${props}<div class="actions"><button class="btn primary" data-action="armingCheck" ${state.vehicleBusy ? "disabled" : ""}>${r ? "Check again" : "Run the check"}</button></div>`;
  if (!r) return head + (state.vehicleBusy ? busy(state.vehicleBusy) : run);
  const gps = r.gps.length
    ? r.gps.map((g) => `${esc(g.fix)}${g.satellites !== null ? `, ${g.satellites} sats` : ""}${g.hdop !== null && g.fix !== "no GPS" ? `, HDOP ${num(g.hdop, 2)}` : ""}`).join("; ")
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
      </div>`,
      )
      .join("")}
    <div class="card"><h2>How to arm this vehicle</h2>${list(r.how_to_arm)}</div>
    ${r.messages.length ? `<div class="card"><h2>Everything the vehicle said</h2>${list(r.messages)}</div>` : ""}
    ${run}`;
}

function paramsView() {
  const v = state.vehicle;
  const busyNow = !!state.vehicleBusy;
  const backups = state.backups;
  const rows = backups?.length
    ? `<table><tr><th>Saved (UTC)</th><th>Note</th><th class="num">Values</th><th></th></tr>
      ${backups
        .map(
          (b) => `<tr>
            <td style="white-space:nowrap">${b.saved ? esc(b.saved) : `<code>${esc(b.file_name)}</code>`}${b.system_id !== null && b.system_id !== undefined ? `<div class="muted">system ${b.system_id}</div>` : ""}</td>
            <td>${esc(b.reason ?? "")}</td>
            <td class="num">${b.params}</td>
            <td class="num" style="white-space:nowrap"><button class="btn" data-action="restoreBackup" data-path="${esc(b.path)}" ${!v || busyNow ? "disabled" : ""}>Restore…</button>
              <button class="btn danger" data-action="askDeleteBackup" data-path="${esc(b.path)}">Delete</button></td></tr>`,
        )
        .join("")}</table>`
    : backups
      ? `<div class="muted">No backups yet.</div>`
      : busy("Reading backups…");
  return `
    <h1>Parameters</h1>
    <p class="lead">Back up all of the vehicle's parameters before you experiment, and put any of them back later. When writing changes you can also choose "Back up and write". Backups are kept in Documents/ArduTune/backups.</p>
    ${
      v
        ? `<div class="card"><h2>Back up now</h2>
          <div class="actions" style="margin-top:0">
            <input type="text" id="note" spellcheck="false" placeholder="Note (optional), e.g. before changing filters">
            <button class="btn primary" data-action="backupNow" ${busyNow ? "disabled" : ""}>Back up</button>
          </div>
          <div class="actions"><span class="hint">Reads every parameter from the vehicle again, so the backup matches it exactly.</span></div>
        </div>`
        : `<div class="banner info">Connect to the vehicle to back up or restore parameters. <button class="link" data-action="go" data-view="connection">Connect…</button></div>`
    }
    ${state.vehicleBusy ? busy(state.vehicleBusy) : ""}
    <div class="card"><h2>Backups</h2>${rows}
      <div class="actions"><button class="btn" data-action="pickParamFile" ${!v || busyNow ? "disabled" : ""}>Restore from another file…</button>
      <span class="hint">Restoring shows every difference first, and you choose which values to write.</span></div>
    </div>`;
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
    case "confirm": {
      const p = m.plan;
      const why = p.rows.some((r) => r.reason);
      const n = m.selected.size;
      const all = n === p.rows.length;
      body = `
        ${p.armed ? `<div class="banner fail">The vehicle is armed. Disarm it to write parameters.</div>` : ""}
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
