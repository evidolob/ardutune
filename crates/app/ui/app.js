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
  ["vlogs", "Logs on vehicle"],
  ["restore", "Write a .param file"],
];

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
    set({ vehicle, connecting: false, vehicleLogs: null });
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
  set({ modal: { stage: "delete", title: `Delete ${what}?`, id: Number(id) } });
}

function askErase() {
  const n = state.vehicleLogs?.length ?? 0;
  set({ modal: { stage: "delete", title: `Erase all ${n} log${n === 1 ? "" : "s"} on the vehicle?`, id: null } });
}

async function confirmDelete() {
  const { id } = state.modal;
  set({ modal: null });
  const logs = await vehicleTask(id === null ? "Erasing all logs…" : `Deleting log ${id}…`, () =>
    id === null ? invoke("erase_vehicle_logs") : invoke("delete_vehicle_log", { id }),
  );
  if (logs) {
    set({ vehicleLogs: logs });
    toast(id === null ? "All logs erased." : `Log ${id} deleted. The remaining logs may be renumbered.`, "info");
  }
}

async function reboot() {
  const ok = await vehicleTask("Rebooting…", () => invoke("reboot").then(() => true));
  if (ok) {
    set({ vehicle: null, vehicleLogs: null, modal: null });
    toast("The flight controller is rebooting. Reconnect in a few seconds.", "info");
  }
}

/** Shows what writing a set of recommendations would change, then writes on confirmation. */
async function writeChanges(source, title, path = null) {
  if (!state.vehicle) {
    set({ modal: { stage: "connect", title } });
    return;
  }
  set({ modal: { stage: "planning", title } });
  try {
    const plan = await invoke("prepare_changes", { source, path });
    set({ modal: { stage: "confirm", title, plan } });
  } catch (e) {
    set({ modal: null });
    toast(e);
  }
}

async function confirmWrite() {
  const m = state.modal;
  set({ modal: { ...m, stage: "writing" } });
  try {
    const result = await invoke("apply_changes");
    set({ modal: { ...m, stage: "done", result } });
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
  if (path) writeChanges("file", `Write ${path.split(/[\\/]/).pop()}`, path);
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
  confirmWrite,
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
  if (e.target.name === "port") set({ address: e.target.value });
});

document.addEventListener("input", (e) => {
  if (e.target.id === "address") state.address = e.target.value;
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
    <div class="actions"><button class="btn primary" data-action="write" data-source="readiness" data-title="Logging for tuning">Write to vehicle…</button>
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
        <div class="actions"><button class="btn primary" data-action="write" data-source="filters" data-title="Recommended filter settings">Write to vehicle…</button></div></div>`
        : `<div class="banner pass">Your current filters are already a good balance: nothing gives the same noise with clearly less delay.</div>`
    }
    ${
      r.quieter_changes.length
        ? `<div class="card"><h2>Quieter option</h2>${changesTable(r.quieter_changes)}
        <div class="actions"><button class="btn" data-action="write" data-source="filters_quieter" data-title="Quieter filter settings">Write to vehicle…</button></div></div>`
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
        <div class="actions"><button class="btn primary" data-action="write" data-source="pid" data-title="Suggested PID changes">Write to vehicle…</button>
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

function restoreView() {
  const head = `<h1>Write a .param file</h1><p class="lead">Restore a backup or load a parameter file from another tool. You'll see every change before anything is written, and the current parameters are backed up first (Documents/ArduTune/backups).</p>`;
  if (!state.vehicle) return head + notConnected("write parameters");
  return `${head}<button class="btn primary" data-action="pickParamFile">Choose a .param file…</button>`;
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
        ${state.vehicle?.armed ? `<div class="banner fail">The vehicle is armed. Disarm it first.</div>` : ""}
        <div class="actions"><button class="btn destroy" data-action="confirmDelete" ${state.vehicle?.armed ? "disabled" : ""}>${m.id === null ? "Erase all" : "Delete"}</button>
        <button class="btn" data-action="closeModal">Cancel</button></div>`;
      break;
    case "planning":
      body = busy("Comparing with the vehicle's parameters…");
      break;
    case "confirm": {
      const p = m.plan;
      const why = p.rows.some((r) => r.reason);
      body = `
        ${p.armed ? `<div class="banner fail">The vehicle is armed. Disarm it to write parameters.</div>` : ""}
        ${
          p.rows.length
            ? `<table><tr><th>Parameter</th><th class="num">On vehicle</th><th></th><th class="num">Proposed</th>${why ? "<th>Why</th>" : ""}</tr>
            ${p.rows.map((r) => `<tr><td class="mono">${esc(r.name)}</td><td class="num">${fmt(r.current)}</td><td class="arrow">→</td><td class="num new">${fmt(r.value)}</td>${why ? `<td>${esc(r.reason ?? "")}</td>` : ""}</tr>`).join("")}</table>`
            : `<div class="banner pass">Nothing to change: the vehicle already has these values.</div>`
        }
        ${p.unchanged.length ? `<p class="muted">Already set: ${p.unchanged.map(esc).join(", ")}</p>` : ""}
        ${p.unknown.length ? `<div class="banner warn">The vehicle has no parameter ${p.unknown.map(esc).join(", ")}; skipped.</div>` : ""}
        <div class="actions">
          ${p.rows.length ? `<button class="btn primary" data-action="confirmWrite" ${p.armed ? "disabled" : ""}>Back up and write ${p.rows.length} parameter${p.rows.length > 1 ? "s" : ""}</button>` : ""}
          <button class="btn" data-action="closeModal">${p.rows.length ? "Cancel" : "Close"}</button>
        </div>`;
      break;
    }
    case "writing":
      body = busy("Backing up, writing and verifying…");
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
  return `<div class="modal-back"><div class="modal"><h2>${esc(m.title)}</h2>${body}</div></div>`;
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
    case "restore":
      return restoreView();
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
  shown[sel] = html;
  regions[sel].innerHTML = html;
  if (sel === "main") regions.main.scrollTop = 0;
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
