# Bombolab — AGENTS.md

## Quick start

```bash
cargo run                          # launch GUI app
cargo run --release                # release build
cargo test --workspace             # all tests
cargo test -p bombolab-core        # single crate
cargo clippy --workspace -- -D warnings
cargo fmt --all -- --check
mdbook build                       # build docs (docs/src/ → docs/book/)
```

CI runs: `cargo check --workspace` → `cargo test --workspace` → `cargo clippy --workspace -- -D warnings` → `cargo fmt --all -- --check`.

## Workspace layout

| Path | Type | Description |
|------|------|-------------|
| `src/main.rs` | binary (root) | GUI entrypoint – wires `bombolab-gui` into eframe |
| `crates/bombolab-core/` | lib + 2 bins | Domain model, kinematics, math. **CLI binaries**: `dh-solve`, `quaternion-solve` |
| `crates/bombolab-gui/` | lib | egui/eframe rendering layer |
| `arduino/` | PlatformIO (C++) | Arduino Nano firmware (servo control via `Servo` lib) |
| `esp32/` | PlatformIO (C++) | ESP32 firmware (servo control via `ESP32Servo` lib) |
| `docs/` | mdBook | Documentation source (`docs/src/`) |

The workspace has **3 Rust crates**: root (app), `bombolab-core`, `bombolab-gui`. All use edition 2024.

## Key commands

- **Run a single binary**: `cargo run --bin dh-solve -p bombolab-core`
- **Run a single test**: `cargo test -p bombolab-core forward::tests::test_forward_kinematics_two_segments` (inline tests, no test harness quirks)
- **Build firmware**: open `arduino/` or `esp32/` dirs, use `pio run` (requires PlatformIO, not Rust)
- **Docs**: `mdbook build` (requires `mdbook` CLI), output in `docs/book/`

## Architecture notes

- `bombolab-core` re-exports all public types from `robot`, `math`, `kinematics` modules at crate root (`crates/bombolab-core/src/lib.rs`).
- `bombolab-gui` depends on `bombolab-core`; root app depends on both.
- DH parameters use **nalgebra 0.35** `Iso3` (isometry) for transformation.
- Joint angles are in **radians** throughout (constants `DEG_TO_RAD` / `RAD_TO_DEG` in `math` module).

## Out-of-date docs

The README shows a flat `src/` directory tree that no longer matches the actual workspace structure. Trust `src/` + `crates/` layout above the README diagram.
