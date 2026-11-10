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
- Vehicle connection over MAVLink (USB, UDP, TCP): download logs, back up
  parameters, and write recommended parameters directly, with verification.

The desktop UI (Tauri) comes later; for now there is a command-line tool.

## Layout

- `crates/core`: analysis library: DataFlash parser (`dataflash`), log model
  (`log`), readiness checks (`readiness`), incident diagnosis (`diagnose`),
  gyro noise analysis (`noise`), parameter files (`params`), synthetic logs
  (`synth`).
- `crates/link`: MAVLink link to the vehicle: parameters, reboot, log
  download (`Link`), USB port discovery (`ports`), simulated vehicle (`mock`).
- `crates/cli`: `ardutune` command-line front end.

## Usage

The connection defaults to `auto` (the single USB flight controller plugged
in). Override with `--conn` or `ARDUTUNE_CONN`, e.g. `--conn udpin:0.0.0.0:14550`
for a telemetry link or `--conn tcpout:127.0.0.1:5760` for SITL.

```bash
ardutune ports                          # serial ports; flight controllers marked
ardutune logs list                      # logs on the flight controller
ardutune logs download --check          # download the latest log and check it
ardutune diagnose flight.bin            # why did it crash / lose control?
ardutune noise flight.bin --svg n.svg   # gyro noise, peaks, notch coverage, plot
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
