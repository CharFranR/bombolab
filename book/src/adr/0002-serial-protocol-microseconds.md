# ADR-0002: Serial Wire Protocol in Microseconds

- **Status**: Accepted
- **Date**: 2026-08-08

## Context

The serial protocol between the host (Rust core / web app) and the Arduino Nano
started in joint degrees. Servo hardware is natively addressed in pulse widths
(microseconds), so every degree value had to be converted at the firmware
boundary, losing resolution and adding a unit convention to keep consistent on
both sides.

## Decision

The wire now carries **servo pulse widths in microseconds (500–2400 µs)**, one
channel per joint plus gripper (6 channels total). The firmware auto-detects
units per frame: values ≤ 175 are interpreted as degrees (backward compatible),
values ≥ 500 as microseconds. A heartbeat keeps the link failure detection
fast and deterministic.

## Consequences

- Full servo resolution on the wire (1 µs steps instead of 1° steps).
- The firmware conversion layer disappeared; it stores and forwards native
  pulse widths.
- Unit ambiguity is resolved by range detection, documented in
  `book/src/core-concepts/communication.md` and covered by the
  `cli_serial_test` integration test.
- Host pacing remains the responsibility of the host (interpolation delay and
  motion player `update(dt)`); the firmware is a deterministic executor.
