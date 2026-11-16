# ArduTune

Sets up, diagnoses and tunes ArduPilot multicopters, offline: from the
vehicle over USB and from test-flight DataFlash logs (gyro filters, rate
PIDs, AutoTune). See [docs/PLAN.md](docs/PLAN.md) for the research and
roadmap.

**Try it in the browser: [evidolob.github.io/ardutune](https://evidolob.github.io/ardutune/)**,
nothing to install. Open a flight log, or connect the flight controller
over USB from Chrome, Edge or Opera. It works offline once loaded and can be
installed as an app. For everything (backups, firmware, compass
calibration, telemetry radios), use the
[desktop app](https://github.com/evidolob/ardutune/releases) for Windows,
macOS and Linux.

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
- Parameters: every parameter of the connected vehicle with its current
  value, default (read from the vehicle in under a second over MAVLink FTP)
  and ArduPilot's description, units, range, named values and bitmask bits;
  search, filter by group or "changed from default", stage edits (or
  defaults) and write them through the review/backup/verify flow, with
  warnings for out-of-range or undocumented values. Descriptions are bundled
  for offline use; refresh them with `tools/update-param-docs.py`.
  `ardutune params --changed` lists what differs from the defaults.
- Firmware: version, board and board ID from the vehicle; newer Copter
  releases for the board from firmware.ardupilot.org (stable, beta, latest;
  the only feature that goes online, and only when asked); flashing a
  release or an `.apj` file through the ArduPilot bootloader over USB, with
  the board ID checked before anything is erased, CRC verification, an
  optional parameter backup, and a before/after parameter comparison.
  `ardutune firmware [--check] [--install stable | --flash FILE]`.
- Setup guide (start of the configuration guide): the live vehicle checked
  step by step (frame and motors, ESC telemetry and RPM, radio, failsafes,
  battery monitor, sensors and calibration, logging for tuning), each with
  plain findings and safe fixes written through the review/backup/verify
  flow. `ardutune setup` on the command line.
- Motor test: spins each motor in turn (props off, disarmed), you say which
  motor turned and which way, and ArduTune works out the fix: output
  remapping (`SERVOn_FUNCTION`) and DShot direction reversal
  (`SERVO_BLH_RVMASK`) for ArduPilot's quad layouts, shown on a top-view
  diagram. `ardutune motors` asks the same questions on the command line.
- Compass calibration: runs the flight controller's onboard calibration of
  every compass with live progress and direction coverage, explains
  failures (field strength, orientation, fit, large offsets) and saves the
  result. `ardutune compass`.
- Serial ports: what each SERIAL port does, with its UART name and the data
  arriving on it live (ArduPilot 4.1+), the GPS fix and receiver signal on
  their ports. Set a port up for a GPS, ExpressLRS/Crossfire receiver,
  digital video OSD, ESC telemetry or telemetry radio in one step, or choose
  protocol, baud and wiring options yourself. It checks for a GPS with no
  port, a receiver or OSD on two ports, a device that sends nothing, and data
  arriving on a port that's off.
- RC channels: the sticks as two animated gimbals (mode 1 or 2) to check
  directions, with reversing; switches and knobs with their position now and
  what the function does low, in the middle and high; channels that haven't
  moved stay hidden until you flip a switch or turn a knob. Put any of
  ArduPilot's switch functions on a switch by flipping it; two functions on
  one 3-position switch (with the transmitter mix to set up and a live check
  of all three positions); a knob for in-flight tuning of a gain between two
  values, or for a camera mount axis; flight mode per switch position; radio
  calibration with the failsafe value kept below the lowest throttle.
- GPS: each GPS live (fix, satellites, accuracy, position, speed,
  satellite systems, update rate, time to first fix) next to the EKF's view
  of it. It checks for spoofing, a position far from where this drone was last
  seen, a GPS that isn't detected, a slow fix, too few satellite systems,
  HDOP above what arming needs, and EKF glitches. It offers GPS off or a GPS
  switch where needed. Edit the receiver type, satellite systems, rate,
  antenna offset and HDOP limit.
- Frame and hardware: frame and layout (with a diagram of the chosen quad
  layout), ESC protocol and firmware, bidirectional DShot, motor poles, spin
  armed/minimum, board orientation and battery monitor on one page, with
  advice for each. It checks for an unset frame, Y4 on a quad, motors without
  an output, DShot settings that don't fit together, and spin armed not below
  spin minimum.
- Initial tune: starting filter, acceleration-limit, thrust-curve and battery
  values for a new build from its prop size and pack (the formulas of
  ArduPilot's initial tune parameters calculator, with ArduCopter 4.7 names),
  plus what to set after the first hover. `ardutune tune --prop 5 [--apply]`.
- AutoTune: prepares ArduPilot's AutoTune.
  - The steps before it, from starting values and the noise filter to
    QuikTune.
  - Checks: AltHold and AutoTune on switches, the notch, voltage
    compensation, thrust curve, spin minimum, and 10 Hz filters on large
    props.
  - Settings for the prop size, and QuikTune setup.
  - A field checklist for starting, testing, saving or discarding the tune.
  - What the flight's log saved, with optional changes after the tune.
- Pre-arm check: asks the vehicle to run its pre-arm checks, collects what it
  says (also while you try to arm), and explains each problem in fix order
  with advice: compass, attitude, radio, logging, GPS (live fix, satellites,
  HDOP, serial setup), battery, safety switch; plus how this vehicle arms
  (switch or sticks) and the current stick and switch positions. It also
  catches a spoofed or faulty GPS (moving while the vehicle stands
  disarmed), which tilts the EKF and fails the compass check, says not to
  recalibrate for it, and offers the fix: GPS off (`GPS1_TYPE` = 0) with
  failsafes that land instead of returning home, or a GPS off switch
  (`RCn_OPTION` = 65, "GPS Disable"): flip the switch and ArduTune finds the
  channel, refusing sticks, the arm switch and the flight mode channel.
- `ardutune level [--watch SECONDS]`: why a vehicle standing level reports a
  tilt: EKF and DCM attitude, raw accelerometer and compass, GPS fix,
  speed and position, EKF innovation ratios and status texts.
- Vehicle connection over MAVLink (USB, UDP, TCP): download and delete logs
  (single logs over MAVLink FTP, or erase all), back up parameters, and write
  recommended parameters directly, with verification.

- Desktop app (Tauri 2): open a log or download one from the vehicle, then
  readiness, crash diagnosis, noise, filters and PIDs with plots; every
  recommendation can be written to the vehicle after a review of the changes
  (pick which ones), with an optional backup, verification and an optional
  reboot. Backups screen: back up on demand with a note, and restore any
  backup (or another `.param` file) value by value; values the firmware owns
  (statistics, sensor IDs) are left out of restores.
- Several vehicles: each flight controller is recognised by its chip ID
  (AUTOPILOT_VERSION `uid2`) and can be given a name. Backups record the
  vehicle they came from and are grouped by vehicle, restoring another
  vehicle's backup warns first, and older untagged backups that match the
  connected vehicle (identical accelerometer calibration, or nearly all
  values) can be assigned to it.
- Flight history: every log in the logs folder filed under the vehicle that
  flew it (the flight controller ID ArduPilot writes into each log's
  banner, so logs from other ground stations count too), with vibration,
  gyro noise before and after the filters, step overshoot per axis and
  oscillations per flight, trends over flights, and the tuning parameters
  changed since the previous flight. Flights are ordered by the flight
  controller's power-up count (`STAT_BOOTCNT`) and time since power-up, not
  by clock: its clock is only right after a GPS fix, so dates come from GPS
  time in the log when it had a fix. Downloaded logs are named after the log
  number, power-up and vehicle; summaries are cached so each log is read
  once.
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

## Install

Or skip installing and use the [web app](https://evidolob.github.io/ardutune/).
For the desktop app, download the installer for your system from
[Releases](https://github.com/evidolob/ardutune/releases).

- **Windows:** `ArduTune_<version>_x64-setup.exe` (installs for your user),
  or the `.msi`. The installers aren't signed yet: if SmartScreen says
  "Windows protected your PC", click **More info**, then **Run anyway**.
- **macOS** (Apple Silicon and Intel): `ArduTune_<version>_universal.dmg`.
  Open it and drag ArduTune to Applications. It isn't signed by Apple yet,
  so macOS blocks the first start: open **System Settings > Privacy &
  Security** and click **Open Anyway**, or run
  `xattr -dr com.apple.quarantine /Applications/ArduTune.app`.
- **Linux:** either
  - the AppImage: `chmod +x ArduTune_<version>_amd64.AppImage`, then run
    it;
  - or the Flatpak: `flatpak install --user ArduTune.flatpak` (it needs the
    GNOME runtime from Flathub), then start ArduTune from the menu.

  To open the flight controller's USB port without root, add yourself to
  the `dialout` group (`uucp` on Arch), then log out and back in:
  `sudo usermod -aG dialout $USER`.
- **Command-line tool:** `ardutune-cli-<system>` archives on the same page.

Windows 10 and later and macOS need no driver for ArduPilot flight
controllers.

## Web app

[evidolob.github.io/ardutune](https://evidolob.github.io/ardutune/) runs the
log analysis in the browser: readiness, noise, filters, PIDs and crash
diagnosis. Open a `.bin` or drop it on the page; nothing is uploaded, the
analysis runs on your computer (WebAssembly). It works offline once loaded,
and Chrome or Edge can install it as an app.

It also connects to the vehicle over USB (Web Serial: Chrome, Edge or Opera
on a computer, or Firefox 151+). That covers the setup guide, frame and
hardware, motor test, serial ports, RC channels, GPS, AutoTune, the initial
tune, pre-arm checks and the parameter list, all writing through the usual
review, plus downloading logs from the vehicle (saved to your downloads and
opened for analysis). Backups, firmware, compass calibration and network
telemetry need the desktop app for now.

To build it (needs `rustup target add wasm32-unknown-unknown` and
`cargo install wasm-bindgen-cli` at the version `crates/web/Cargo.toml`
pins):

```bash
tools/build-web.sh
```

```bash
tools/serve-web.py
```

(It sends the cross-origin isolation headers the serial link needs; on
GitHub Pages the service worker adds them.)

## Desktop app

```bash
cargo run --release -p ardutune-app
```

Downloaded logs go to `Documents/ArduTune/logs`, parameter backups to
`Documents/ArduTune/backups`, vehicle names to `Documents/ArduTune/drones.json`.

To build the installers yourself, use the Tauri CLI in `crates/app`
(`cargo install tauri-cli`, then `cargo tauri build`). For the Flatpak, see
`packaging/flatpak/`.

## Releases

Push a version tag matching `version` in `Cargo.toml` (e.g. `v0.1.0`). The
Release workflow then builds:
- the Windows installers;
- the universal macOS `.dmg`;
- the Linux AppImage and Flatpak;
- the command-line tool for each system.

It attaches them all to a draft GitHub release, which you publish by hand.
Running the workflow from the Actions tab builds the same files as workflow
artifacts, without a release.

## AI assistant (MCP)

The desktop app can serve ArduTune to an AI assistant over MCP (Model
Context Protocol). Claude, or any MCP client, can then help set up, tune
and troubleshoot the vehicle with what ArduTune knows: the vehicle and its
parameters with ArduPilot's descriptions, the checks, flight history and the
log analyses. It can also read the raw log messages (any type, filtered by
time or instance, thinned or summarised) to check ArduTune's analyses
against the data. It can only **propose** changes. They open in the app's usual
review, where you confirm them, with backup and read-back. It can't arm,
run motors, reboot, calibrate or flash firmware.

Turn it on under **AI assistant** in the app (it's off by default and only
listens on 127.0.0.1, with a token). Then, for Claude Code:

```bash
claude mcp add --transport http ardutune http://127.0.0.1:47653/mcp --header "Authorization: Bearer <token>"
```

The page shows the command with your token, a configuration for other MCP
clients, and the assistant's recent activity.

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

## License

ArduTune is free software under the GNU General Public License, version 3
or (at your option) any later version: see [LICENSE](LICENSE). It bundles
ArduPilot's parameter metadata (GPLv3); see [THIRD-PARTY.md](THIRD-PARTY.md).
