# ArduTune — research & plan

Desktop, offline app that diagnoses ArduPilot multicopters (quads first) from a
test-flight DataFlash log and recommends **filter** and **PID** settings.

## Goals

- Input: one or more `.bin` DataFlash logs from test flights.
- Output: ranked findings with evidence plots + parameter changes written
  straight to the vehicle over MAVLink (backup, confirm, verify, reboot), with
  a `.param` file export as the offline fallback.
- Logs downloaded from the flight controller over MAVLink, or opened from disk.
- Desktop first (Windows, macOS, Linux), fully offline.
- Focus: gyro noise / filter tuning and rate-loop PID tuning. Other diagnostics
  (vibration, hover thrust, EKF, compass, power) only as gates or later additions.

## Existing tools (and the gap)

- ArduPilot WebTools — Filter Review (noise FFT, filter what-if), PID Review
  (step response estimate), MAGFit, Hardware Report. Browser-based, client-side.
- UAV Log Viewer, MAVExplorer, Mission Planner — plotting, no diagnosis.
- Methodic Configurator — guided parameter sequence + tuning guide.

Gap: tools are fragmented and expert-level; they show plots, not a guided
"what's wrong → what to change → predicted result" workflow with flight history.

## Stack

- **Core:** Rust — streaming DataFlash parser (FMT self-describing format,
  header `0xA3 0x95`, FMT id `0x80`), columnar storage, DSP (FFT/Welch, filters,
  deconvolution, model fitting).
- **Shell/UI:** Tauri 2. Frontend is plain HTML/CSS/JS with no build step (no
  Node toolchain needed); plots are SVG rendered by the core crate. Fully
  bundled, no network. Interactive plots (zoom, cursors) can come later.
- **Research:** prototype algorithms in Python/Jupyter (`pymavlink`, numpy,
  scipy), then port to Rust and cross-check outputs.
- Licensing note: ArduPilot and WebTools are GPLv3, pymavlink is LGPL — porting
  their code affects our license choice.

## Workflow (wizard, enforced order)

1. **Log readiness** — check logged params/messages; if insufficient, generate a
   `.param` file + test-flight script for the next flight.
2. **Vibration gate** — `VIBE` (<30 m/s² ok, >60 bad), clipping must be 0.
3. **Filters.**
4. **Rate PIDs** — roll/pitch, then yaw.
5. Angle P, then position/altitude controllers (later).

### Test-flight script

30–60 s steady hover, stick snaps per axis, altitude steps, optional System ID
(chirp) flight per axis.

## Filter tuning

Data: pre-filter gyro at sufficient rate — `INS_LOG_BAT_MASK`, `INS_LOG_BAT_OPT`,
`INS_RAW_LOG_OPT` (4.5+); ESC RPM telemetry (`ESC`) strongly preferred. Warn about
aliasing (logging rate ≥ 2× noise frequency). Exact bit values resolved per
firmware version.

1. Segment hover; Welch PSD per axis; spectrograms vs time / throttle / RPM.
2. Classify peaks: tracks RPM/throttle → motor/prop → harmonic notch;
   fixed → frame resonance → static notch or mechanical fix;
   broadband → props/mounting → hardware first.
3. Recommend `INS_HNTCH_*` (MODE: ESC telem > RPM > in-flight FFT > throttle;
   FREQ, REF, BW, ATT, HMNCS, FM_RAT, OPTS — double / per-motor / loop-rate),
   `INS_HNTC2_*` for a second source, `INS_GYRO_FILTER`.
4. **Offline filter simulator (key feature):** bit-exact re-implementation of
   ArduPilot notch biquads, LPF2p gyro filter, `ATC_RAT_*_FLTD/FLTT/FLTE`. Apply
   to pre-filter data; show resulting PSD plus the trade-off:
   noise reduction (dB) vs added phase lag (deg) near loop crossover.
5. Verify on next flight's post-filter log: predicted vs actual.

## PID tuning

Data: `PIDR/PIDP/PIDY` (Tar, Act, P, I, D, FF, Dmod, SRate), `RATE`, `ATT`;
System ID mode logs (`SID_*` params) when available.

- **Step response from normal flight:** Wiener deconvolution of target→actual
  over sliding windows, keep windows with enough excitation; rise time,
  overshoot, settling, latency.
- **Frequency response from System ID chirps:** Bode plot, gain/phase margins,
  crossover frequency.
- **Diagnostics:**

  | Symptom | Likely cause | Suggested change |
  |---|---|---|
  | Low/mid-freq oscillation, overshoot >~15% | P/D too high, low phase margin | lower P/D |
  | High-freq D-term energy | D too high / filtering weak | lower D or revisit filters |
  | `Dmod` often < 1 | slew limiter active: gain/noise too high | lower gain, review `SMAX` |
  | Sluggish, little overshoot | P/FF low | raise P/FF |
  | Persistent I offset | CoG / motor / frame issue | hardware, not tuning |

- **Predict before flying:** fit a low-order plant model (gain + poles + delay)
  from SysID or step data; simulate closed loop with proposed gains + filters;
  show predicted step response and margins, labelled with confidence.
- Bounded changes (~10–25% per iteration). Check prerequisites before suggesting
  AutoTune/QuickTune; evaluate their resulting gains.
- Parameter-name mapping per firmware version (4.x changes).

## Validation

- SITL with injected faults (`SIM_*` vibration / gyro noise / motor) for labelled
  test logs.
- Real logs from own flights and public forum posts.
- Cross-check against WebTools Filter Review / PID Review on the same logs.

## MVP roadmap

1. Parser + log readiness check + next-flight `.param` / flight script. ✅
1b. MAVLink link: log download, parameter backup, direct parameter writes with
    verification, reboot; simulated vehicle for tests. ✅
1c. Incident diagnosis (`diagnose`): find losses of control and in-flight log
    ends, build a timeline, rank causes with evidence (ESC telemetry dropout,
    RPM collapse, ArduPilot thrust-loss messages, output saturation, rate
    oscillation, battery sag, brownout, vibration, EKF) and list what was
    ruled out; flight-wide warnings (ESC telemetry error rate, EKF
    innovations). ✅ First validated on a real AutoTune crash: two motors'
    ESCs dropped out 0.11 s before the flip.
2. Noise analyser (PSD, spectrogram, peak classification). ✅ Welch PSD of
   batch-sampler or raw gyro data in steady flight, pre vs post filter;
   spectrogram with motor speed; peaks matched to per-motor speed ranges
   (harmonic k), RPM-tracking test, control-band and fixed-frequency classes;
   notch coverage and filter attenuation per peak; per-motor hover speed
   spread; SVG plot. Next: confirm the raw-IMU post-filter instance numbering
   on a real log.
3. Filter simulator + notch recommendations + noise-vs-lag view. ✅ Bit-exact
   ArduPilot notch (A/Q from FREQ/BW/ATT, Q shared across harmonics) and
   LowPassFilter2p biquads; notch centres per mode (fixed, throttle, RPM
   sensor, ESC average or per motor, FM_RAT floor); applied per batch to the
   recorded pre-filter spectrum; validated against logged post-filter data
   (rounding floor of the batch sampler accounted for: 2.3 dB RMS on the real
   log); search over low-pass cutoff, notch harmonics, bandwidth (≥ FREQ/2)
   and per-motor mode; Pareto frontier of noise vs phase lag; "same noise,
   less delay" and "less noise, same delay" picks; direct apply; SVG plot.
   Not yet: double/triple notch geometry, D-term filters, time-varying
   notch during manoeuvres.
4. Step-response analysis + PID diagnostics + `.param` export. ✅ RATE target
   vs actual resampled to a uniform grid (impossible values dropped);
   2 s windows, Tukey taper, Wiener deconvolution with roll-off above 30 Hz,
   averaged step response with outlier rejection (recovers 26.1% overshoot
   on a known 25.4% system); rise/delay/overshoot/settling/steady state;
   rate-error oscillation confirmed in the measured rate; D-term noise share,
   slew-limiter activity, I offset, output saturation; AutoTune periods
   excluded or flagged (no gain suggestions from them); bounded ±15% gain
   suggestions and SMAX safety net; unsaved AutoTune results parsed from
   messages; direct apply; SVG plot. Next: a clean AltHold stick-snap flight
   to validate suggestions on real data.
UI. Desktop app (Tauri 2) over the steps above. ✅ Open a log or download one
    from the vehicle; readiness, crash diagnosis, noise, filters and PIDs with
    plots; vehicle connection with port discovery; every recommendation (and
    any `.param` file) written through one review → backup → write and verify
    → reboot flow, with changes re-planned against live values. Commands
    tested end to end through Tauri's IPC against the simulated vehicle.
    Next: installers, interactive plots.
5. System ID analysis + plant fit + predicted closed-loop response.
6. Flight-to-flight comparison / history. Done: Flight history screen, logs
   grouped by vehicle (flight controller ID from the log banner), per-flight
   vibration, noise, overshoot and oscillation with trends, and tuning
   parameter changes between flights. Not yet: predicted vs actual after a
   filter change, a CLI view.
7. Later: other diagnostics. Incident diagnosis to grow with more causes
   (GPS glitch, failsafes, mechanical failure signatures, AutoTune gain
   sanity) as real logs show them.

## Separate track: configuration guide

A guided setup sequence in the spirit of ArduPilot Methodic Configurator, but
more robust: done before tuning, as its own step in the app.

- Ordered steps from first boot to ready-to-tune: frame and motor order/
  direction, ESC protocol and bidirectional DShot, RC and modes, failsafes,
  battery monitor, sensor calibration, logging for tuning, initial filter and
  PID defaults scaled to prop size (as the Initial Tune Parameters
  calculator does).
- Every step reads the live vehicle, validates the result (not just writes
  values) and explains why; uses the same backup / confirm / verify / reboot
  path as parameter writes.
- Checks that catch real problems: motor order and spin direction tests,
  ESC telemetry health per motor, compass interference vs throttle,
  vibration from a short hover, failsafe actions actually set.
- Resumable, with a record of what was configured and when; diffs against the
  previous state so changes are always visible and reversible.

Started: Setup guide screen and `ardutune setup` check frame and motors, ESC
telemetry/RPM, radio (incl. CRSF RSSI), failsafes, battery monitor, sensor
calibration and logging for tuning from live parameters and status, with
safe fixes. Pre-arm check explains why a vehicle won't arm. Initial tune
gives prop-size starting values (`ardutune tune`). Motor test checks order
and direction and fixes them in software (`ardutune motors`). Compass
calibration runs the onboard calibration with progress and advice
(`ardutune compass`). Next: a record of what was configured; ESC telemetry
health per motor and compass interference vs throttle from a log.

## Separate track: vehicle tools

### Full parameter list

Every parameter of the connected vehicle, searchable and editable, as a
complete replacement for a ground station's parameter screen.

- Table of all parameters: name, current value, default, units, range or
  allowed values (enum names, bitmask bits as checkboxes), and the
  description; filter by name, description text, group (`INS_`, `ATC_`…)
  and "changed from default".
- Descriptions, units, ranges and enums from ArduPilot's parameter metadata
  (`apm.pdef.json` per vehicle and version), bundled with the app so it works
  offline; the matching version picked from the vehicle's firmware, with the
  nearest bundled one as fallback and a warning.
- Defaults read from the vehicle where firmware supports it (MAVLink FTP
  `@PARAM/param.pck` with defaults), otherwise from the metadata.
- Edits are validated against range and type, staged, then written through
  the same review → backup → write and verify → reboot flow; parameters
  marked "reboot required" say so.
- Compare with a backup or `.param` file, and with defaults.

Done: Parameters screen and `ardutune params --changed`. Values and defaults
from `@PARAM/param.pck` (0.4 s for 1273 parameters on a KakuteH7), Copter
metadata bundled (latest master, 162 kB compressed). Not yet: metadata per
firmware version (renamed parameters show no description), comparing with
a backup inside the list.

### Firmware version and upgrade

- Show what the vehicle runs: vehicle type, version and git hash, board
  and its ID (AUTOPILOT_VERSION, the boot banner), and whether the build is
  stable, beta or dev.
- Tell whether a newer release exists for this board: an optional online
  check against firmware.ardupilot.org (the app stays usable offline), or a
  firmware file the user downloaded.
- Upload firmware: back up parameters first, reboot into the bootloader
  (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN), flash an `.apj` with the ArduPilot
  bootloader protocol over USB, verify the CRC, reconnect and compare
  parameters with the backup (major upgrades can rename or reset some).
- Safety: refuse an `.apj` whose board ID doesn't match the board, refuse
  while armed, warn about release notes between versions, and explain DFU
  recovery if an upload is interrupted. Variant builds matter (e.g. the
  KakuteH7 "-bdshot" firmware for bidirectional DShot).

Done: Firmware screen and `ardutune firmware`. Bootloader protocol (sync,
identify, erase, program, CRC verify, reboot) tested against a simulated
bootloader, including the real KakuteH7 4.7.2 image; refuses wrong board,
oversize images, external-flash images and armed vehicles before erasing.
Flashed on real hardware: KakuteH7-bdshot from 4.6.3 to 4.7.2, parameters
converted by the firmware and compared. Not yet: external flash, DFU, heli/other
vehicle types.

### Serial ports

A page that shows what each of the flight controller's serial ports does
and lets it be changed safely, instead of editing `SERIALn_*` by hand.

- One row per port: SERIALn, the board's pad or connector name where known
  (UART mapping per board), protocol (`SERIALn_PROTOCOL` by name: GPS, RC
  input, MAVLink, ESC telemetry, MSP/DisplayPort, CRSF/ELRS…), baud rate and
  options (`SERIALn_OPTIONS`: inverted, half duplex, swap TX/RX), and what
  is live on it (GPS detected, receiver signal, telemetry).
- Common setups as one action: GPS, CRSF/ELRS receiver (with RC_OPTIONS and
  RSSI_TYPE), ESC telemetry, DJI/HDZero OSD, a telemetry radio; each sets
  protocol, baud and options together.
- Checks: two ports with the same single-use protocol, a protocol on a port
  that can't do it (e.g. the USB port), RC input set on a port and on the
  RC pad, settings that need a reboot.
- Same review / backup / verify / reboot path as other writes.

Done: Serial ports page (Configuration).
- Each port shows its protocol, baud and options, plus its UART name and live
  traffic. Both come from ArduPilot's `@SYS/uarts.txt` over MAVFTP. It also
  shows the GPS fix and receiver signal on their ports.
- One-step setups: GPS, ExpressLRS or Crossfire receiver (with RSSI_TYPE and
  RC_OPTIONS), digital video OSD (OSD_TYPE), ESC telemetry, telemetry radio,
  MAVLink 115200, and off. A custom protocol, baud and wiring are possible
  too.
- Checks: a GPS that's on with no port for it, RC input or OSD on two ports,
  nothing arriving from the GPS or receiver, and data arriving on a port
  that's off.
- The GPS page's "not detected" problem links here.

Not yet: pad and connector names per board (only the UART name), SBUS
setups, a CLI view.

### RC channels and switches

A page with every RC channel live and what it does, and a way to put
functions on switches.

- Live bars for channels 1-16 with their range (`RCn_MIN/MAX/TRIM`), what
  each does: sticks (`RCMAP_*`), flight mode channel and the current mode
  per position (`FLTMODE_CH`, `FLTMODE1-6`), and switch functions
  (`RCn_OPTION` by name: arm/disarm, GPS disable, RTL, AutoTune, motor
  emergency stop, camera…) with the switch position now.
- Assign a function: pick it from a searchable list, flip the switch you
  want, ArduTune finds the channel (as the GPS off switch already does),
  refuses sticks, the mode channel and channels in use, and explains
  two- vs three-position behaviour. Flight modes per switch position too.
- Radio calibration (min, max, trim per channel) and reversing, with the
  failsafe value checked against the lowest throttle.

Done: RC channels screen (live bars, roles, switch positions, flight modes
per position, assign by flipping or picking a channel, remove, reverse,
radio calibration). Then: sticks as animated gimbals, switches and knobs
apart, what each function does per switch position, two functions on one
3-position switch (transmitter mix recipe plus a live check), knobs for
transmitter tuning (`TUNE`, `TUNE_MIN/MAX`) and mount axes, channels that
haven't moved hidden. Not yet: a CLI view.

Revisit: the page works but is still too complicated. Simplify it: fewer
buttons and dialogs (one "set up a control" flow that starts from flipping or
turning the control and offers switch, 3-position or knob uses from what it
saw), less text, and the advanced parts (reversing, calibration, transmitter
tuning) out of the main view.

### Basic vehicle configuration

Frame and the few settings every build needs, on one page; it could join
the Initial tune page ("New build"), which already asks for prop size and
battery.

- Frame class and type (`FRAME_CLASS`, `FRAME_TYPE`) with a diagram of the
  motor order and directions for each choice, props in vs props out (the
  reversed layouts), then a motor test to confirm.
- ESC protocol (`MOT_PWM_TYPE`), bidirectional DShot (`SERVO_BLH_BDMASK`),
  motor poles, spin arm/min, board orientation (`AHRS_ORIENTATION`).
- Battery: monitor type, cell count and chemistry, capacity; then the
  initial tune values and failsafes from the same inputs.
- Shows which changes need a reboot, and links to the setup guide steps it
  satisfies.

Done: Frame and hardware page (Initial setup).
- Frame and layout, with a diagram of the chosen quad layout.
- ESC protocol and firmware, bidirectional DShot on the motor outputs,
  motor poles, spin armed/minimum.
- Board orientation, battery monitor and capacity. Battery voltages stay on
  Initial tune.
- Advice under each setting and a reboot mark on staged changes that need
  one; everything is written at once.
- Checks: frame not set, Y4 on a quad, motors without an output,
  bidirectional DShot without DShot or not on every motor, ESC firmware
  unset, spin armed not below spin minimum, no battery monitor or capacity.

While building it, frame type numbers were found wrong in the motor test:
ArduPilot has 12 BetaFlight X, 13 DJI X, 14 clockwise X, 18 BetaFlight X
reversed and 19 Y4. Now fixed, and a quad on Y4 is flagged.

### GPS configuration and status

A page for the GPS: what it sees now, whether it can be trusted, and its
setup.

- Live status per GPS: fix type, satellites, HDOP/accuracy, position,
  altitude, speed and course, and how long since power-up the fix took;
  the EKF's view next to it (using GPS or not, innovation ratios, "GPS
  Glitch" messages), so a bad GPS shows before arming.
- Trust: the spoofing check from the pre-arm page (moving while still), a
  position far from the last known one, jumps in position or altitude,
  and jamming/spoofing state where the receiver reports it (u-blox M10).
  With the fixes already offered there: GPS off, or a GPS off switch.
- Setup: `GPS1_TYPE` (and a second GPS), the serial port and baud it is on,
  constellations (`GPS_GNSS_MODE`: GPS, Galileo, GLONASS, BeiDou), update
  rate, `GPS_HDOP_GOOD`, antenna offset (`GPS1_POS_*`), GPS for yaw where
  supported, and the compass in the GPS module (orientation, external).
- Checks: no GPS detected on the configured port, wrong baud, no fix after
  a few minutes outdoors, constellations that leave too few satellites.

Done: GPS page (Configuration). Each GPS live (fix, satellites, accuracy,
HDOP, position, speed, systems, rate, time to first fix) and the EKF's test
ratios. Checks:
- spoofing, with GPS off or a GPS switch offered;
- a position more than 50 km from where this drone's GPS was last trusted
  (remembered per drone in drones.json, with "the vehicle really is here");
- GPS not detected (no GPS serial port, or wiring);
- no fix after five minutes;
- one satellite system with few satellites;
- HDOP above GPS_HDOP_GOOD, poor accuracy;
- EKF glitch or rejection;
- GPS off, with a way to turn it back on.

Settings per GPS and general ones, for both 4.6+ and older parameter names.

Not yet:
- jamming/spoofing state from the receiver (u-blox M10);
- the compass in the GPS module;
- GPS for yaw;
- a CLI view.

### AutoTune

Done: AutoTune page (Tuning), following ArduPilot's AutoTune and QuikTune
documentation.
- The path: starting values (compared with Initial tune for the prop
  size), harmonic notch, QuikTune (optional), AutoTune, then checking the
  log.
- Checks before flying:
  - AltHold on the mode switch, and AutoTune on a switch (or as a mode),
    with flip-to-assign;
  - the notch is on;
  - battery voltage compensation and thrust curve are set;
  - spin minimum isn't too low;
  - 10 Hz rate filters for props of 13 inches and more.
- Settings by size:
  - AUTOTUNE_AGGR 0.1, or 0.075 for 13 inches and up (ArduTune's own
    suggestion; the docs give 0.05-0.1);
  - all axes, with one axis per flight suggested for large copters.
- QuikTune: built in (QUIK_* parameters), the script, scripting off, or not
  supported, with setup and a switch.
- A field checklist using the vehicle's own channels: start, pause, test
  and save, discard, plus an optional test-gains switch (option 180).
- After the flight: which axes the open log saved, and the docs' optional
  changes (softer gains if too stiff, ATC_THR_MIX_MAX 0.9 on large props).

Not yet:
- uploading the QuikTune script over MAVFTP;
- reading AUTOTUNE_AXES/AGGR results per axis from ATUN messages;
- yaw D tuning (4.4+ bit).

## Revisit: log analysis UI, comparing logs

The log analysis pages (Readiness, Noise, Filters, PIDs) work on one open
log at a time. Revisit all of them so that two or more logs can be looked
at together. A comparison tells the app much more about the drone and how it
responds to changes than any single flight.

- Open several logs at once, usually flights of the same drone from Flight
  history (they're already grouped by drone and ordered by boot count). Mark
  one as the reference and compare the others with it.
- The parameter changes between the compared flights head the comparison
  (Flight history already works them out), so each difference in response
  sits next to what was changed.
- Side by side or overlaid:
  - noise spectra before and after a filter change, with the notch
    positions;
  - step responses before and after a PID or AutoTune change;
  - vibration levels;
  - hover throttle;
  - motor output balance.
- What changed, in words: "The filter change cut noise at 180 Hz by 12 dB
  but added 3 ms of delay", or "AutoTune raised roll D; overshoot went from
  18% to 6%".
- Learning from several flights:
  - how noise moves with throttle and battery voltage;
  - whether a recommendation actually helped;
  - which flight had the best response, as the one to go back to.
- A clearer way to switch, close and add logs. The sidebar's single
  open-log line becomes a small list of the compared logs.
- Keep it fast with several large logs: analyse in the background and cache
  per log (Flight history's summary cache is a start).

## MCP server and a Claude extension

Put an MCP (Model Context Protocol) server in the desktop app, and package a
Claude extension that uses it. Claude, or any other AI assistant that speaks
MCP, can then help set up, tune and troubleshoot the drone through what
ArduTune already knows: the parameters and their docs, the checks, and the
log analyses.

- **Where it runs:** inside the desktop app, because the app owns the serial
  link and only one program can hold the port.
  - Either the app serves MCP over HTTP on localhost, with a per-session
    token;
  - or `ardutune mcp` (in the CLI) is a stdio server that forwards to the
    running app.
  - Without the app running, the CLI could still serve the offline parts:
    logs and parameter docs.
- **Tools (read):**
  - vehicle status and identity, parameters with their docs and defaults;
  - the setup guide, pre-arm explanation, GPS, serial ports, RC and frame
    and hardware views;
  - flight history and per-drone backups;
  - the vehicle's logs: list, download, analyse;
  - the analyses of a log: readiness, noise, filters, PIDs, crash
    diagnosis.
- **Tools (change):** the AI only proposes changes.
  - A proposal opens the app's usual review: the user sees each change and
    its reason, then confirms in the app, with backup and read-back as
    always.
  - Nothing is written, armed, spun or flashed from the AI side alone.
  - Motor test, compass calibration, reboot and firmware stay in-app
    actions the user starts.
- **Resources and prompts:**
  - ArduPilot parameter docs, the open log's summary, and the drone's
    history as resources;
  - prompts for common jobs: "Why won't it arm?", "Tune the filters from
    this log", "Prepare for AutoTune", "What changed between these
    flights?".
- **The extension:**
  - a Claude Desktop extension (MCP bundle) and a Claude Code plugin that
    install the connection to the app, with a short skill describing the
    tuning workflow and the safety rules;
  - plain MCP config instructions for other clients.
- **Safety and privacy:**
  - off by default, turned on in the app;
  - the app shows when an AI client is connected and what it asked for;
  - data stays on the computer except what the user's AI client sends to
    its own model;
  - a read-only mode.
- **Before building:** make the app's commands callable from one place, so
  the UI and MCP share them. The Tauri commands are already that layer.

## Before publishing on GitHub: license

Done: GPL-3.0-or-later, like ArduPilot, Mission Planner and the Methodic
Configurator. It fits the bundled ArduPilot parameter metadata (GPLv3) as it
is. The project has LICENSE (GPLv3 text), `license` in `Cargo.toml`, a README
section, and THIRD-PARTY.md for the parameter metadata, the initial tune
formulas and the Rust dependencies (mostly MIT/Apache-2.0; serialport is
MPL-2.0; all are GPLv3-compatible).

## Installers for Windows, macOS and Linux

Installers for each desktop OS, built by Tauri's bundler. The bundle is
already on (`"targets": "all"` in `crates/app/tauri.conf.json`); there is no
CI or release workflow yet.

- Windows: an NSIS `.exe` installer and an `.msi`.
- macOS: a `.dmg` with a universal build (Apple Silicon and Intel).
- Linux: an `.AppImage` (Tauri's bundler) and a Flatpak only, no `.deb` or
  `.rpm`. Tauri doesn't build Flatpaks itself, so the Flatpak needs a
  `flatpak-builder` manifest that packages the release binary. It needs
  `--device=all` in finish-args to reach serial ports, and could go on
  Flathub later.
- GitHub Actions: a matrix build on windows-latest, macos-latest and
  ubuntu-latest (`tauri-apps/tauri-action`), triggered by a version tag. It
  attaches the installers to a GitHub release, with the CLI binary for each OS
  next to them.
- Signing:
  - macOS needs a Developer ID certificate and notarization, or Gatekeeper
    blocks the app. That needs an Apple Developer account.
  - Without a code-signing certificate, Windows SmartScreen warns on first
    run. Start unsigned and document the "Run anyway" step.
- Icons: generate the full set (`.icns`, `.ico`, sizes) from `icons/icon.png`
  with `tauri icon`.
- Serial access notes in the README: on Linux, add the user to the `dialout`
  (or `uucp`) group; Windows 10 and later and macOS need no driver for
  ArduPilot boards.
- CI (done): `.github/workflows/ci.yml` runs on every push to main and every
  pull request.
  - Formatting and clippy on Linux.
  - The tests on Linux, macOS and Windows, plus a syntax check of the UI
    scripts.
- Release workflow (`.github/workflows/release.yml`):
  - A version tag `vX.Y.Z` (checked against Cargo.toml) builds:
    - the NSIS `.exe` and the `.msi`;
    - the universal `.dmg`;
    - the AppImage (on ubuntu-22.04 for older glibc);
    - the Flatpak, built in the GNOME SDK container from
      `packaging/flatpak/`;
    - the CLI for each system.
  - It drafts a GitHub release with all of them. Running it by hand builds
    the same files as artifacts only.
  - Unsigned for now. Next: Apple notarization and Windows signing once
    there are certificates, then Flathub (needs vendored crate sources
    instead of network access during the build).
- Later: in-app update checks against GitHub releases (Tauri updater, which
  needs signed update files).

## Last step: static web app

After the features above, ship ArduTune as a static web app as well (no
backend), hosted on GitHub Pages, working fully offline and installable from
the browser (a PWA, like Betaflight's app). It's for distribution: nothing to
download or sign.

Feasible with limits (checked November 2026):
- Web Serial (USB) works in Chrome, Edge and Opera on the desktop, and in
  Firefox 151 and later; Android Chrome is rolling it out (148). No Safari or
  iOS.
- A web page can't open UDP or TCP sockets, so there are no Wi-Fi telemetry
  or SITL links without a WebSocket bridge.
- firmware.ardupilot.org sends no CORS headers, so the page can't fetch the
  firmware manifest or images.

The web version drops what the browser can't do rather than working around
it: no firmware update and no network links. It is USB only and points to
the desktop app for those. The desktop app and the CLI stay.

Steps:
1. Log analysis on the web: `core` compiled to WebAssembly (it only needs
   serde, rustfft and flate2), logs opened by file picker or drag and drop,
   analysis in a Web Worker, a service worker for offline use. The UI is
   unchanged: it already talks to the backend through named commands (as
   `dev/mock-tauri.js` shows), so a web build answers them from WebAssembly
   instead of Tauri.
   Done:
   - `crates/web` compiles the log analysis to WebAssembly. The views are
     shared with the desktop app in `core::views`.
   - `crates/web/www` holds the web stand-in for Tauri (`web-tauri.js`), a
     Web Worker, a service worker for offline use and a manifest for
     installing.
   - In the UI's web mode, the vehicle pages point to the desktop app.
   - `tools/build-web.sh` builds it, and the Web app workflow publishes it
     to GitHub Pages.
2. The vehicle over Web Serial: the `link` crate's blocking serial loop
   becomes async over a byte transport, with MAVLink parsing kept. This
   covers parameters, setup, RC, motors, compass, pre-arm, logs from the
   vehicle and MAVFTP. The app-side commands move too (vehicle, backups,
   history, drones).
3. Storage in the browser: backups, drone names and flight history in
   IndexedDB or OPFS, or a folder the user picks once (File System Access, in
   Chrome) such as `Documents/ArduTune`.
4. Hide the unsupported features in the web build: firmware update and
   network connections.

## Risks

- Bad advice from poor logs → readiness check + confidence on every finding.
- Firmware differences → per-version param/message mapping.
- Liability → recommendations only, never auto-apply, safety disclaimer.

## References

- https://ardupilot.org/copter/docs/common-webtools.html
- https://ardupilot.org/copter/docs/common-magfit.html
- https://discuss.ardupilot.org/t/ardupilot-pid-review-tool/109765
- https://docs.rs/ardupilot-binlog
- https://github.com/ArduPilot/MethodicConfigurator
- https://ardupilot.github.io/MethodicConfigurator/TUNING_GUIDE_ArduCopter.html
