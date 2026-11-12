# Third-party material

## ArduPilot parameter metadata

`crates/core/data/copter-params.json.gz` is ArduPilot's ArduCopter parameter
metadata (names, descriptions, ranges, units and value names), fetched from
<https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.json>. It is
generated from the ArduPilot source code, which is licensed under the GNU
General Public License version 3: <https://github.com/ArduPilot/ardupilot>.
ArduTune compiles it into the binary to explain parameters offline.

## Initial tune formulas

The starting values in `crates/core/src/tune.rs` follow the formulas of
Mission Planner's initial tune parameters calculator (GPLv3,
<https://github.com/ArduPilot/MissionPlanner>). They are reimplemented, not
copied code.

## Rust dependencies

Most of ArduTune's Rust dependencies (Tauri, mavlink, rustfft, serde, flate2,
ureq, clap and others) are licensed under MIT and/or Apache-2.0. A few use
other GPLv3-compatible licenses: MPL-2.0 (serialport, and cssparser and
selectors through Tauri), BSD-3-Clause, ISC, Zlib, Unicode-3.0 and
CDLA-Permissive-2.0. Each crate carries its own license; `cargo metadata`
lists them.
