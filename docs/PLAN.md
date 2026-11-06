# ArduTune — research & plan

Desktop, offline app that diagnoses ArduPilot multicopters (quads first) from a
test-flight DataFlash log and recommends **filter** and **PID** settings.

## Goals

- Input: one or more `.bin` DataFlash logs from test flights.
- Output: ranked findings with evidence plots + a `.param` diff loadable in
  Mission Planner / QGroundControl / MAVProxy.
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
- **Shell/UI:** Tauri 2 + TypeScript (React or Svelte), WebGL/uPlot plotting with
  decimation for multi-million-point series. Fully bundled, no network.
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

1. Parser + log readiness check + next-flight `.param` / flight script.
2. Noise analyser (PSD, spectrogram, peak classification).
3. Filter simulator + notch recommendations + noise-vs-lag view.
4. Step-response analysis + PID diagnostics + `.param` export.
5. System ID analysis + plant fit + predicted closed-loop response.
6. Flight-to-flight comparison / history.
7. Later: MAVLink log download & confirmed param upload, other diagnostics.

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
