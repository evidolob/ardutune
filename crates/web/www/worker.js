// Runs ArduTune (WebAssembly) off the page's thread: log analysis, so big
// logs don't freeze the page, and the vehicle link, which waits for the
// serial bytes the page shares (Atomics.wait is allowed in workers).
import init, * as ardutune from "./pkg/ardutune_web.js";

const ready = init();

self.onmessage = async ({ data: { id, cmd, args, bytes } }) => {
  try {
    await ready;
    let value;
    switch (cmd) {
      case "open_log":
        value = ardutune.open_log(args.name, new Uint8Array(bytes));
        break;
      case "open_sample":
        value = ardutune.open_sample();
        break;
      case "analysis":
        value = ardutune.analysis(args.kind);
        break;
      case "close_log":
        ardutune.close_log();
        value = null;
        break;
      case "vehicle_connect":
        value = ardutune.vehicle_connect(
          args.ctrl,
          args.data,
          (bytes) => self.postMessage({ type: "write", bytes }),
          (done, total) => self.postMessage({ type: "event", name: "progress", payload: { task: "Reading parameters", done, total } }),
        );
        break;
      case "vehicle_download_log":
        value = ardutune.vehicle_download_log(args.id, (done, total) =>
          self.postMessage({ type: "event", name: "progress", payload: { task: "Downloading log", done, total } }),
        );
        break;
      case "vehicle":
        value = ardutune.vehicle_call(args.cmd, JSON.stringify(args.args ?? {}));
        break;
      default:
        throw new Error(`unknown command ${cmd}`);
    }
    self.postMessage({ id, ok: true, value });
  } catch (e) {
    self.postMessage({ id, ok: false, error: String(e?.message ?? e) });
  }
};
