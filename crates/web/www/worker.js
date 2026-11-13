// Runs ArduTune's log analysis (WebAssembly) off the page's thread, so big
// logs don't freeze the page.
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
      default:
        throw new Error(`unknown command ${cmd}`);
    }
    self.postMessage({ id, ok: true, value });
  } catch (e) {
    self.postMessage({ id, ok: false, error: String(e?.message ?? e) });
  }
};
