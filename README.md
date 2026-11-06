# ArduTune

Desktop, offline tool that diagnoses ArduPilot multicopters from test-flight
DataFlash logs and helps tune gyro filters and rate PIDs. See
[docs/PLAN.md](docs/PLAN.md) for the research and roadmap.

## Status

Roadmap step 1 is done: log parser, log readiness check, and next-flight logging parameters.
The desktop UI (Tauri) comes later; for now there is a command-line tool.

## Layout

- `crates/core`: library: DataFlash parser (`dataflash`), log model (`log`),
  readiness checks (`readiness`), synthetic log writer (`synth`).
- `crates/cli`: `ardutune` command-line front end.

## Usage

```bash
cargo run --release -- check flight.bin                       # readiness report
cargo run --release -- check flight.bin --param-out next.param  # + params for next flight
cargo run --release -- check flight.bin --json                # machine-readable report
cargo run --release -- info flight.bin --fields               # message types and fields
cargo run --release -- sample demo.bin --preset default       # synthetic log to try it out
```

## Development

```bash
cargo test
```
