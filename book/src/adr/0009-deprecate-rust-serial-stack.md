# ADR-0009: Deprecate Rust Serial Stack

- **Status**: Accepted
- **Date**: 2026-08-12

## Context

The host-side serial path lived in Rust: the `communication` module in
`bombolab-core` (`ArduinoNano`, `ServoMapper`, `ServoCommand`,
interpolation), the `serial-test` CLI debug tool, the `ws-bridge`
WebSocket→serial server, the `serial`/`ws-bridge` feature gates, and the
`serialport`/`ctrlc`/`futures-util`/`serde`/`serde_json`/`tokio`/
`tokio-tungstenite` dependencies. The browser talks to the firmware
directly via WebSerial (protocol v2 manifest, ADR-0008): it performs the
handshake, streams `SAMPLE` lines with ACK pacing, and executes plans
without any Rust intermediary. The Rust serial stack was dead code —
nothing referenced it — and the wasm build never compiled it
(`default-features = false`).

## Decision

Remove the entire legacy Rust serial stack from the repository:

- Delete the `communication` module (11 files) and its re-exports.
- Delete the `serial-test` and `ws-bridge` binaries and their tests.
- Delete the `serial` / `ws-bridge` features and the now-unused
  dependencies (`serialport`, `ctrlc`, `futures-util`, `serde`,
  `serde_json`, `tokio`, `tokio-tungstenite`).
- Remove the `core-ws-bridge` CI job and the `cli_serial_test`
  integration test.
- Drop the `communication` and `serial-test` book pages; record this
  decision as ADR-0009.

## Consequences

- Fewer dependencies: `bombolab-core` now depends only on `nalgebra`.
- No dead hardware path: there is exactly one way to talk to the
  firmware, the browser WebSerial implementation.
- CLI debug tooling moved to the browser (the WebSerial code paths in
  `web/src/serial.ts` and `web/src/motion/manifestProtocol.ts` are the
  live debug/verification surface).
- The wire protocol decisions (ADR-0002, ADR-0008) remain in force,
  now implemented in TypeScript instead of Rust.
- Rollback: the deleted code is recoverable from git history; no
  firmware or wasm changes were required.
