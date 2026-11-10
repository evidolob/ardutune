# ArduTune

Desktop, offline tool that diagnoses ArduPilot multicopters from test-flight
DataFlash logs and helps tune gyro filters and rate PIDs. See
[docs/PLAN.md](docs/PLAN.md) for the research and roadmap.

## Status

- Log parser and log readiness check (does this log have what filter and PID
  tuning need?).
- Incident diagnosis: finds losses of control and crashes in a log, shows a
  timeline, ranks likely causes with evidence and lists what was ruled out.
- Gyro noise analysis: spectrum before and after the filters, spectrogram,
  noise peaks classified against motor speed, harmonic notch coverage, and
  plain-language findings; optional SVG plot.
- Gyro filter simulator: replays the recorded noise through ArduPilot's notch
  and low-pass filters, checks itself against the logged filtered data, and
  recommends settings with the same noise and less delay (or less noise for
  the same delay), which can be written straight to the vehicle.
- Rate-loop PID analysis: step response per axis from flight data (Wiener
  deconvolution), rate-loop oscillations, PID term health (D-term noise,
  slew limiter, I offset, saturation), cautious bounded gain suggestions, and
  gains from a finished AutoTune that weren't saved; optional SVG plot and
  direct apply.
- Vehicle connection over MAVLink (USB, UDP, TCP): download and delete logs
  (single logs over MAVLink FTP, or erase all), back up parameters, and write
  recommended parameters directly, with verification.

- Desktop app (Tauri 2): open a log or download one from the vehicle, then
  readiness, crash diagnosis, noise, filters and PIDs with plots; every
  recommendation can be written to the vehicle after a review of the changes,
  with an automatic backup, verification and an optional reboot. `.param`
  files (such as backups) can be written back the same way.
- Command-line tool with the same analyses and vehicle commands.

## Layout

- `crates/core`: analysis library: DataFlash parser (`dataflash`), log model
  (`log`), readiness checks (`readiness`), incident diagnosis (`diagnose`),
  gyro noise analysis (`noise`), filter simulator (`filters`), PID analysis
  (`pid`), SVG plots (`plot`), parameter files (`params`), synthetic logs
  (`synth`).
- `crates/link`: MAVLink link to the vehicle: parameters, reboot, log
  download and deletion (`Link`, `ftp`), USB port discovery (`ports`),
  simulated vehicle (`mock`).
- `crates/cli`: `ardutune` command-line front end.
- `crates/app`: desktop app. Rust commands in `src/`, frontend in `ui/`
  (plain HTML, CSS and JavaScript: no Node or build step).

## Desktop app

```bash
cargo run --release -p ardutune-app
```

Downloaded logs go to `Documents/ArduTune/logs`, parameter backups to
`Documents/ArduTune/backups`. Building an installer (`.app`/`.dmg`, `.msi`,
`.deb`) needs the Tauri CLI: `cargo install tauri-cli`, then
`cargo tauri build` in `crates/app`.

## Command line

The connection defaults to `auto` (the single USB flight controller plugged
in). Override with `--conn` or `ARDUTUNE_CONN`, e.g. `--conn udpin:0.0.0.0:14550`
for a telemetry link or `--conn tcpout:127.0.0.1:5760` for SITL.

```bash
ardutune ports                          # serial ports; flight controllers marked
ardutune logs list                      # logs on the flight controller
ardutune logs download --check          # download the latest log and check it
ardutune diagnose flight.bin            # why did it crash / lose control?
ardutune noise flight.bin --svg n.svg   # gyro noise, peaks, notch coverage, plot
ardutune filters flight.bin --svg f.svg # simulate filter settings, recommend better ones
ardutune filters flight.bin --apply     # write the recommended filters to the vehicle
ardutune pid flight.bin --svg p.svg     # step response, oscillation, PID health, AutoTune results
ardutune pid flight.bin --apply --pick autotune   # write unsaved AutoTune gains
ardutune check flight.bin --apply       # write recommended params to the vehicle
ardutune params -o backup.param         # save all parameters
ardutune apply backup.param             # write a .param file (e.g. restore a backup)
ardutune check flight.bin --param-out next.param   # offline: write a file instead
```

Writing parameters always shows the changes and asks first (`--yes` skips
the question), refuses while the vehicle is armed, saves a backup of all
current parameters, reads each value back to verify it, and offers to reboot
(`--reboot` reboots without asking).

To try it without hardware, run a simulated vehicle in one terminal and point
the other commands at it:

```bash
ardutune mock-vehicle --listen udpin:127.0.0.1:14550
ardutune --conn udpout:127.0.0.1:14550 logs list
```

Run from source with `cargo run --release -- <command>`.

## Development

```bash
cargo test
```

The app's IPC test drives every command through Tauri's mock runtime against
the simulated vehicle. To preview the UI in a browser without building the
app, export what the app shows for a log, then serve `crates/app` and open
`dev/preview.html` (vehicle actions are simulated there):

```bash
ARDUTUNE_FIXTURE_LOG=$PWD/flight.bin cargo test --release -p ardutune-app -- --ignored export_fixtures
python3 -m http.server -d crates/app 8765   # http://localhost:8765/dev/preview.html
```
