// The web app's stand-in for Tauri: the UI (crates/app/ui) calls
// window.__TAURI__.core.invoke as in the desktop app. Commands run in a Web
// Worker on the WebAssembly build of ArduTune: log analysis, and the vehicle
// over Web Serial (the page reads the USB port into a shared ring buffer the
// worker waits on). What needs the computer's files says it needs the
// desktop app.
(() => {
  window.ARDUTUNE_WEB = true;

  const worker = new Worker("worker.js", { type: "module" });
  const pending = new Map();
  const listeners = new Map();
  let seq = 0;
  worker.onmessage = ({ data }) => {
    if (data.type === "write") return serialWrite(data.bytes);
    if (data.type === "event") return (listeners.get(data.name) ?? []).forEach((f) => f({ payload: data.payload }));
    const p = pending.get(data.id);
    if (!p) return;
    pending.delete(data.id);
    data.ok ? p.resolve(data.value) : p.reject(data.error);
  };
  worker.onerror = (e) => {
    for (const p of pending.values()) p.reject(`The analysis stopped: ${e.message || "out of memory?"}`);
    pending.clear();
  };
  const call = (cmd, args = {}, bytes = null) =>
    new Promise((resolve, reject) => {
      const id = ++seq;
      pending.set(id, { resolve, reject });
      worker.postMessage({ id, cmd, args, bytes }, bytes ? [bytes] : []);
    });

  // ---- Web Serial ----
  const RING = 1 << 20;
  let serial = null; // { port, reader, writer, ctrl }

  const serialWrite = (bytes) => serial?.writer.write(bytes).catch(() => {});

  async function closeSerial() {
    const s = serial;
    serial = null;
    if (!s) return;
    Atomics.store(s.ctrl, 1, 1);
    Atomics.notify(s.ctrl, 0);
    try {
      await s.reader.cancel();
    } catch {}
    try {
      s.reader.releaseLock();
      s.writer.releaseLock();
      await s.port.close();
    } catch {}
  }

  async function connectSerial() {
    if (!("serial" in navigator)) throw "This browser can't open USB serial ports: use Chrome, Edge or Opera on a computer, or Firefox 151 or later.";
    if (!crossOriginIsolated) throw "Reload the page once (the app needs to restart its background worker), then connect again.";
    // Asked first, while the click still counts as the user's gesture.
    const port = await navigator.serial.requestPort();
    await closeSerial();
    try {
      await port.open({ baudRate: 115200, bufferSize: 65536 });
    } catch (e) {
      throw `Couldn't open the port: ${e.message}. Close other programs using it (Mission Planner, QGroundControl, the ArduTune desktop app).`;
    }
    const ctrl = new Int32Array(new SharedArrayBuffer(8));
    const data = new Uint8Array(new SharedArrayBuffer(RING));
    const reader = port.readable.getReader();
    const writer = port.writable.getWriter();
    serial = { port, reader, writer, ctrl };
    (async () => {
      let written = 0;
      try {
        for (;;) {
          const { value, done } = await reader.read();
          if (done) break;
          for (let i = 0; i < value.length; i++) data[(written + i) % RING] = value[i];
          written = (written + value.length) >>> 0;
          Atomics.store(ctrl, 0, written | 0);
          Atomics.notify(ctrl, 0);
        }
      } catch {}
      Atomics.store(ctrl, 1, 1);
      Atomics.notify(ctrl, 0);
    })();
    try {
      return JSON.parse(await call("vehicle_connect", { ctrl, data }));
    } catch (e) {
      await closeSerial();
      throw e;
    }
  }

  // Commands the web version runs on the vehicle (the rest need the desktop app).
  const VEHICLE = new Set([
    "list_ports", "vehicle_status", "param_table", "prepare_changes", "apply_changes", "reboot", "vehicle_config", "autotune_status",
    "vehicle_pack", "initial_tune", "motor_layout", "motor_test", "motor_check", "rc_status", "rc_choices", "detect_switch", "gps_status",
    "serial_status", "arming_check", "setup_guide",
  ]);

  // Files the page was given (picked or dropped), by the "path" the UI uses.
  const files = new Map();
  let fileSeq = 0;
  const keep = (file) => {
    const path = `web-file:${++fileSeq}/${file.name}`;
    files.set(path, file);
    return path;
  };

  const pickFile = (kind) =>
    new Promise((resolve, reject) => {
      if (kind !== "log") return reject("Opening that kind of file needs the desktop app for now.");
      const input = document.createElement("input");
      input.type = "file";
      input.accept = ".bin,.BIN";
      input.onchange = () => resolve(input.files[0] ? keep(input.files[0]) : null);
      input.oncancel = () => resolve(null);
      input.click();
    });

  async function invoke(cmd, args = {}) {
    switch (cmd) {
      case "pick_file":
        return pickFile(args.kind);
      case "open_log": {
        const file = files.get(args.path);
        if (!file) throw "That file is no longer available: open it again.";
        return JSON.parse(await call("open_log", { name: file.name }, await file.arrayBuffer()));
      }
      case "open_sample":
        return JSON.parse(await call("open_sample"));
      case "analysis":
        return JSON.parse(await call("analysis", { kind: args.kind }));
      case "close_log":
        return call("close_log");
      case "connect":
        return connectSerial();
      case "disconnect":
        await call("vehicle", { cmd: "disconnect", args: {} }).catch(() => {});
        return closeSerial();
      case "reboot": {
        const r = await call("vehicle", { cmd: "reboot", args: {} }).catch((e) => e);
        await closeSerial();
        if (typeof r === "string" && !/connection|closed|USB/i.test(r)) throw r;
        return null;
      }
      default:
        if (VEHICLE.has(cmd)) {
          const out = JSON.parse(await call("vehicle", { cmd, args }));
          if (cmd === "vehicle_status" && out === null) await closeSerial();
          return out;
        }
        throw "This needs the ArduTune desktop app for now.";
    }
  }

  const listen = async (name, f) => {
    listeners.set(name, [...(listeners.get(name) ?? []), f]);
    return () => listeners.set(name, (listeners.get(name) ?? []).filter((g) => g !== f));
  };
  window.__TAURI__ = { core: { invoke }, event: { listen } };
  navigator.serial?.addEventListener("disconnect", (e) => {
    if (serial && e.target === serial.port) closeSerial();
  });

  // Drop a .bin anywhere on the page to open it.
  addEventListener("dragover", (e) => e.preventDefault());
  addEventListener("drop", (e) => {
    e.preventDefault();
    const file = [...(e.dataTransfer?.files ?? [])].find((f) => /\.bin$/i.test(f.name));
    if (file) window.ardutuneOpenLog?.(keep(file));
  });

  // Offline use, and cross-origin isolation (needed to share memory with
  // the worker): the service worker caches the app and adds the headers.
  // The first visit isn't controlled by it yet, so reload once it is.
  if ("serviceWorker" in navigator && location.protocol !== "file:") {
    navigator.serviceWorker.register("sw.js").then(() => {
      if (!crossOriginIsolated && !sessionStorage.getItem("ardutune-isolate-reload")) {
        navigator.serviceWorker.ready.then(() => {
          sessionStorage.setItem("ardutune-isolate-reload", "1");
          location.reload();
        });
      }
    }).catch(() => {});
    if (crossOriginIsolated) sessionStorage.removeItem("ardutune-isolate-reload");
  }
})();
