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
| `src/main.rs` | binary (root) | App stub – prints a pointer to the web viewer (GUI lives in `web/`) |
| `crates/bombolab-core/` | lib + 6 bins | Domain model, kinematics, math. **CLI binaries**: `dh-solve`, `quaternion-solve`, `dynamics-report`, `test-case-report`, `serial-test`, `ws-bridge` |
| `crates/bombolab-wasm/` | lib (wasm) | wasm-bindgen bridge: FK, IK, robot factory |
| `arduino/Arduino Nano/` | PlatformIO (C++) | Arduino Nano firmware (servo control via `Servo` lib) |
| — | — | — |
| `book/src/` | mdBook | Documentation source (built to `docs/` via `book.toml`) |

The project has **3 Rust packages**: root (app stub), `bombolab-core`, `bombolab-wasm`. All use edition 2024. Note: the packages are **not** members of a Cargo `[workspace]` — build/test them with `--manifest-path crates/<crate>/Cargo.toml` or from inside the crate directory; `--workspace` from the root only covers the root package.

## Key commands

- **Run a single binary**: `cargo run --bin dh-solve --manifest-path crates/bombolab-core/Cargo.toml`
- **Run a single test**: `cargo test --manifest-path crates/bombolab-core/Cargo.toml forward::tests::test_forward_kinematics_two_segments` (inline tests, no test harness quirks)
- **Build firmware**: open `arduino/` dir, use `pio run` (requires PlatformIO, not Rust)
- **Docs**: `mdbook build` (requires `mdbook` CLI), output in `docs/`

## Architecture notes

- `bombolab-core` re-exports all public types from `robot`, `math`, `kinematics` modules at crate root (`crates/bombolab-core/src/lib.rs`).
- `bombolab-wasm` depends on `bombolab-core`; the root app depends on `bombolab-core` (the GUI is the `web/` React app).
- DH parameters use **nalgebra 0.35** `Iso3` (isometry) for transformation.
- Joint angles are in **radians** throughout (constants `DEG_TO_RAD` / `RAD_TO_DEG` in `math` module).

## Out-of-date docs

The README shows a flat `src/` directory tree that no longer matches the actual workspace structure. Trust `src/` + `crates/` layout above the README diagram.
