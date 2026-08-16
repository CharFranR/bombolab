# ADR-0007: Plan vs. Execution Trace

- **Status**: Accepted
- **Date**: 2026-08-08

## Context

The web app commands the arm through a single serial write path (the
`ServoInterpolator` send callback in `App.tsx`): every commanded frame —
interpolator steps and the one-second keep-alive heartbeat — leaves through
that callback. When a drawing run looks wrong, the only evidence is the drawn
output or a live look at the viewer. There is no record of what was actually
commanded, when, or how the commanded stream differed from the intended plan.

The platform has no encoders: the firmware reports nothing about the arm's
physical pose. Any metric derived here is a statement about the COMMANDED wire
stream versus the PLANNED stream — never about physical accuracy, repeatability,
or servo response. This vocabulary constraint is load-bearing: labeling a
wire-vs-plan delta as "accuracy" would claim a measurement the hardware cannot
make.

## Decision

Record every wire frame host-side by wrapping the single interpolator send
callback, build an analytic plan timeline from the same command list, compare
the two, and export the trace as CSV. Four pure TypeScript modules in
`web/src/motion/` (`trace.ts`, `planTimeline.ts`, `comparator.ts`, `csv.ts`),
thin React glue in `App.tsx`, and one export button beside the playback
controls. Slice 2 adds an import panel with per-joint SVG charts.

### Vocabulary

- **Commanded**: what the wire actually carried — a frame written to the serial
  port (measured, with timestamps).
- **Planned**: the analytic expectation derived from the command list and the
  robot's IK — a model, not a measurement.
- **Deviation**: |commanded − planned| in wire units (µs). It measures how the
  commanded stream differs from the plan. It is NOT accuracy, precision, or
  error of the physical arm.
- **Send-time error**: commanded timestamp − expected plan timestamp. It
  measures when the wire frame left the host, not when the servo moved.

"Accuracy" and "precision" phrasing is banned in UI labels and docs for this
feature: there are no encoders, so nothing here measures the physical arm.

### Trace model (dedupe heartbeat)

Each recorded frame becomes `{ ts_ms, q_us[6], count }`:

- `ts_ms` — first send time relative to the run start (`performance.now()`,
  µs resolution in ms).
- `q_us` — the six wire values as integer µs (J1..J5 + gripper).
- `count` — consecutive identical frames coalesced into one sample (the
  one-second keep-alive heartbeat re-sends the same frame, so a held pose
  accumulates count instead of inflating the timeline).

The model deliberately keeps only the FIRST timestamp plus count: gap
statistics consume first-ts and count only, which keeps the CSV round-trip
exact. Coalescing is strictly consecutive — an identical frame after a
different frame is a new sample.

Capacity is capped at 100 000 samples (≈10 h at 20 frames/s). Overflow stops
recording and flags the run as `truncated`.

### Plan timeline

`planTimeline(cmds, { ik, robot, startQ, gripperPct, dt = 0.04, startTcp })`
samples the analytic trajectory every `dt` seconds:

- `move` → linear segment at `speed` (duration = distance / speed);
- `wait` → hold the last commanded pose;
- `penUp` / `penDown` → instant, no time, no pose change.

The IK function is injected so tests use deterministic fakes; production passes
the same solver wrapper the app uses. Playback semantics are mirrored exactly:
a target that moved less than 0.5 mm skips the solve, a failed solve holds the
previous q, and q is converted to wire µs via `qToServoUs` + `gripperToServoUs`
so plan and trace share units. Consecutive identical samples coalesce with a
count, exactly like the trace, so a wait becomes one sample with `count ≈
duration/dt`.

### Comparator (ordinal alignment)

Both sequences are aligned ordinally (index to index) on their common t0 —
plan time is in seconds, trace time in ms; the comparator converts plan times
to ms. Metrics:

- **send-time error** per paired sample: `trace.ts_ms − plan.t*1000`, reported
  as mean, max |error|, and sample σ (n−1).
- **inter-frame gaps** over the full trace: min/mean/max/σ, count, and how many
  exceed 1 s. Pairs adjacent to a `count > 1` sample are excluded — that is the
  heartbeat-dedupe exclusion, so a held pose never inflates gap stats.
  Throttled gaps (browser background tabs clamp the rAF delta to MAX_DT 0.1)
  stay in the stats raw; nothing is normalized away.
- **commanded-vs-planned deviation** per joint (mean, max) and global
  (mean, max), in µs of wire value.
- **duration**: plan (last plan sample time plus coalesced hold ticks at the
  plan dt) vs real (last observed frame timestamp).

Paired samples only cover `min(plan, trace)` indices; gap and duration metrics
always use observed timestamps only — never an assumed tick rate. The trace's
interpolator sub-frames (5° steps) and heartbeat frames mean the trace is
typically longer than the plan; ordinal pairing is a deliberate, documented
approximation, not a time-warped alignment.

### CSV format

```
ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count
0,544,1472,1379,1523,1162,1678,1
50.1234567890123,1472,1472,1379,1523,1162,1678,2
```

- One row per sample, header fixed as above.
- `ts_ms` is the first send time relative to the run start; joint values and
  count are integers.
- Import parses symmetrically and reproduces samples, counts, and the derived
  fields (`framesWritten = Σ count`, `dedupe = Σ (count−1)`). Malformed rows
  (wrong arity, non-numeric, non-positive count, wrong header) throw without
  any partial state. `t0` and `truncated` are not part of the CSV: `t0` is an
  absolute wall-clock anchor meaningless across sessions, and truncation is
  a runtime flag, so an imported trace reports `truncated = false`.
- Export is a pure function of the trace: exporting twice yields byte-identical
  files and never mutates state.

### Lifecycle

- A playback run resets and starts the recorder after `motionPlayerPlay`.
- Pause, stop, and the completed state finalize the trace (finalized = frozen,
  exportable).
- Pause → resume keeps the trace finalized: the resumed segment is untraced —
  honest, and ordinal alignment is preserved because the trace keeps only
  distinct commanded frames.
- Replay (from stopped/completed) starts a fresh trace.
- Discard paths (clear block, failed CIPRA draw, exit drawing mode) drop the
  trace and disable export.

## Consequences

- The app gains a record of every commanded frame with host timestamps, an
  analytic plan to compare against, and an honest vocabulary for what the
  comparison means.
- No protocol, firmware, or core changes: the hook wraps the existing send
  callback, so the wire path is untouched (rollback = unhook the callback and
  delete the modules).
- Deviation is measured in wire µs, which conflates plan-model error with
  quantization and clamping; the ADR documents that the plan is a model, not
  ground truth.
- The trace has no encoders, so nothing can be said about the physical arm —
  any future "did the servo actually move" question requires different
  hardware, not more host-side statistics.
- Duration comparisons are lower bounds: the trace knows when frames left the
  host, not when the arm finished moving.
