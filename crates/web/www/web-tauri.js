// The web app's stand-in for Tauri: the UI (crates/app/ui) calls
// window.__TAURI__.core.invoke as in the desktop app. Log analysis runs in
// a Web Worker on the WebAssembly build of ArduTune's core; everything that
// needs the vehicle or the file system says it needs the desktop app.
(() => {
  window.ARDUTUNE_WEB = true;

  const worker = new Worker("worker.js", { type: "module" });
  const pending = new Map();
  let seq = 0;
  worker.onmessage = ({ data }) => {
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
      default:
        throw "This needs the ArduTune desktop app for now: the web version analyses flight logs.";
    }
  }

  window.__TAURI__ = { core: { invoke }, event: { listen: async () => () => {} } };

  // Drop a .bin anywhere on the page to open it.
  addEventListener("dragover", (e) => e.preventDefault());
  addEventListener("drop", (e) => {
    e.preventDefault();
    const file = [...(e.dataTransfer?.files ?? [])].find((f) => /\.bin$/i.test(f.name));
    if (file) window.ardutuneOpenLog?.(keep(file));
  });

  // Offline use: cache the app once loaded.
  if ("serviceWorker" in navigator && location.protocol !== "file:") {
    addEventListener("load", () => navigator.serviceWorker.register("sw.js").catch(() => {}));
  }
})();
