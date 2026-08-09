# ADR-0008: Manifest Protocol v2 (delta timing, streamed)

- **Status**: Accepted
- **Date**: 2026-08-08

## Context

The host-paced frame protocol (ADR-0002) suffers scheduling jitter: frames are
paced by the host (rAF, `setInterval`, React effects), so command timing is at
the mercy of the browser. The Thalos execution manifest (waypoints + delta
timing, firmware executes autonomously) fixes this, but a full manifest cannot
fit the Arduino Nano's 2 KB RAM: real drawing trajectories need 1.6–9.6 KB
(square ~100 samples, gcode 200–600 at 50 ms dt), while only ~1.7 KB are free.

## Decision

Add a v2 text protocol that keeps the delta-timing waypoint model but streams
it through a ring-buffer executor with ACK-paced upload:

- `HELLO 2` → `HELLO 2 OK CHUNK_MAX 24` (additive handshake; legacy hosts never
  send it and keep working unchanged in IDLE).
- `MANIFEST <count> <duration_us>` / `SAMPLE <a1..a6 us> <dt_us>` /
  `END_UPLOAD` / `EXECUTE` / `STOP` / `STATUS`.
- `ACK <free>` flow control: a slot is freed by consumption; the host never
  sends more samples than the last announced free count (window ≤ CHUNK_MAX).
  `EXECUTE` is legal before `END_UPLOAD` because large manifests cannot fit in
  RAM; `END_UPLOAD` validates count and Σdt (tolerance `max(1000 µs, 1%)`).
- Executor: ring buffer of 2×24 samples (768 B), paced by `micros()`
  (wrap-safe delta); dt of a sample is the delta to the NEXT distinct sample
  (the plan's dedupe frames are absorbed into dt, so Σdt equals the plan
  duration). First sample `dt=0` applies at start.
- Watchdog: fed by execution progress (consumed samples, ACKs, STOP), not by
  incoming frames; park at 5 s also resets the v2 state to IDLE so a revived
  host recovers with legacy frames or a fresh HELLO.
- Web playback uploads chunks with ACK pacing and streams the rest during
  execution; calibration and interactive mode stay on legacy frames.

## Consequences

- Command jitter moves to the MCU (`micros()`, 4 µs tick). Honest limitation:
  actuation stays quantized by the servo ~20 ms refresh — the manifest fixes
  command timing, not servo latency.
- RAM: 1316/2048 B (64.3%) with CHUNK_MAX=24 (732 B free); Flash 25.4%.
- Errors are explicit tokens (`BAD_LINE`, `OUT_OF_RANGE`, `BAD_DT`,
  `BAD_STATE`, `COUNT_MISMATCH`, `DURATION_MISMATCH`, `NOT_READY`).
- Host death mid-run: buffered samples drain (~1–2 s), then park at 5 s.
- Rollback: legacy frames are untouched; playback can flip back to the
  interpolator path.
