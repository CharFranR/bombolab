# ADR-0006: Deterministic Workspace Analysis Cloud

- **Status**: Accepted
- **Date**: 2026-08-08

## Context

The web app historically rendered a fake "workspace" cloud: `generateWorkspace`
sampled every joint uniformly with `Math.random()` and ran JS-side forward
kinematics — 2000 points with no relationship to the drawing constraint, no
repeatability, no per-point conditioning metrics, and no statistics.

The actual workspace of the FABRI Creator is a constrained variety for drawing:

```
M = { q1, q2, q3 uniform in joint limits; q4 = 0; q5 = −(q2+q3) }
```

(ADR-0001) with the J5 pitch limit rejecting q5 outside [−115°, 55°]. The
meaningful analysis is not "where can the TCP go" but "how well-conditioned is
the drawing-plane motion there": every sample should carry the reduced-Jacobian
singularity metrics (ADR-0005) so the cloud becomes a conditioning map, not a
random scatter.

A second constraint comes from the platform: the sampler runs in Rust core and
is called from the web through wasm-bindgen. Transferring 10k–50k per-sample
objects across the boundary is GC churn; a flat numeric buffer is the natural
contract. And because the RNG lives in Rust, the web can no longer rely on
`Math.random()` — seed handling and determinism must be explicit contracts.

## Decision

Replace the JS random cloud with a stateful `WorkspaceSampler` in
`bombolab-core` (kinematics/workspace.rs) driven by a hand-rolled
xoshiro256** RNG (math/rng.rs, zero deps, wasm-safe). The web runs "Run
Analysis": it creates a sampler, pulls 1k-sample chunks with a yield between
chunks, aggregates the cloud, and renders it colored by conditioning.

### Sampling modes

- **DrawingPlane (default)**: q1..q3 uniform within joint limits, q4 = 0,
  q5 = −(q2+q3). Samples whose q5 falls outside the J5 limits [−115°, 55°]
  are rejected and counted in `n_rejected`. `N` counts attempted samples;
  `n_valid = N − n_rejected`.
- **Full5Dof**: all five joints uniform within limits, no rejection. Keeps
  the legacy "all joints free" cloud semantics available for comparison.

### Determinism and seed

The core sampler ALWAYS takes an explicit u64 seed (`WorkspaceSampler::new(
seed, mode)`) and is bit-identical for the same seed and mode — randomness is
a caller concern. The web wrapper supplies the seed from `Date.now()` when the
user runs an analysis (D3), so consecutive runs differ unless the caller pins
a seed. The CLI `workspace-report` derives its default seed from the system
clock.

### Flat transfer format (stride-5)

Batch results cross the wasm boundary as a single `Float64Array` with a fixed
per-sample layout, no per-sample objects:

```
[x, y, z, sigma_min, kappa]   (mm, mm, mm, mm, dimensionless)
```

`sample_batch(id, k)` returns one record per ACCEPTED sample (rejected
attempts never appear in the buffer; they only increment `n_rejected`). The
web wrapper parses the stride: positions with the DH→three.js swap
(x, y, z) → (x, z, y) and per-point colors derived from `sigma_min`.

### Display bands (50/25) vs gate thresholds (25/20/5/100)

The singularity gate (ADR-0005) classifies waypoints with
`warn: sigma_min < 25 mm OR kappa > 20` and `block: sigma_min < 5 mm OR
kappa > 100`. Those thresholds are calibrated for trajectory gating: the
reachable workspace has a conditioning floor of sigma_min ≈ 19.8 mm, so the
gate's warn band is 1.25× above the worst in-band measurement and the demo
paths are verified silent.

The cloud is a display artifact, not a decision gate, and the two purposes
need different bands:

| Display band | sigma_min | Visual |
|---|---|---|
| high | ≥ 50 mm | red |
| medium | 25–50 mm | yellow |
| low | < 25 mm | blue |

The 50/25 bands are intentionally coarser than the gate's 25/20/5/100:
applying the gate bands to a cloud would paint almost every point "warn"
(floor ≈ 19.8 mm < 25) and the drawing-plane range never reaches sigma_min
< 10, so the "block" color would never appear. The display bands spread the
probe range [19.8, 82] mm across three discernible buckets instead. The gate
thresholds are NEVER used for coloring: the two band systems are separate by
contract, and this ADR is the single source of truth for both.

### Per-sample metrics

Each accepted sample computes TCP xyz via forward kinematics, the reduced
Jacobian Jr = [J1, J2 − J5, J3 − J5] (ADR-0001), and `waypoint_metrics` +
`classify` (ADR-0005): sigma_min, kappa, and level. Level is used by the CLI
report; the web colors from sigma_min only.

### Statistics (single pass)

`sampler_stats()` returns, accumulated in one O(n) pass over accepted
samples:

- `bounds_min` / `bounds_max`: per-axis min/max, absent when n_valid = 0
- `centroid`: mean TCP position, absent when n_valid = 0
- `reach`: maximum TCP distance from the base origin, absent when n_valid = 0
- `n_valid`, `n_rejected`

### Wasm registry

Samplers live in a `Mutex<Vec<Option<WorkspaceSampler>>>` id registry
(motion-player pattern). `sampler_new(seed, mode)` returns an id,
`sample_batch(id, k)` / `sampler_stats(id)` operate on it,
`sampler_drop(id)` frees the slot. Unknown or dropped ids return a JS error
("sampler not found"), never a panic; unknown modes error at creation.
Sampling is JS-driven chunked streaming: the web requests 1k samples per
chunk and awaits a microtask between chunks so the UI stays responsive and
progress is observable after every chunk.

### CLI

`workspace-report` (bombolab-core bin) samples with
`--mode drawing-plane|full-5dof --seed N --n N` and prints the stats plus the
worst-10 samples by sigma_min. Invalid mode exits non-zero with usage on
stderr. The robot model is always the internal FABRI Creator with its base
transform — consumers cannot inject a different robot.

## Consequences

- The web cloud is now a real, repeatable conditioning map: same seed, same
  cloud, bit-identical arrays; `Date.now()` seeds keep interactive runs
  varied.
- DrawingPlane clouds visibly show the constrained variety (q5 rejection
  removes roughly a quarter of attempts in the far corners), while Full5Dof
  preserves the old free-joint look for comparison.
- 10k–50k points transfer as flat buffers with zero per-sample allocation;
  the 1k chunking with yield keeps the main thread free between chunks and
  supports cancellation (a cancel flag checked between chunks).
- Colors are display-only: changing them never affects the gate decision,
  and the gate thresholds remain the only arbiter of warn/block behavior.
- `generateWorkspace` is deleted; the app's old "mostrar/ocultar workspace"
  toggle is replaced by the Run Analysis panel (N selector 1k/5k/10k/50k,
  mode selector, progress, cancel, stats, N > 0 validation).
- The RNG is a 40-line zero-dependency module; the sampler adds no new
  dependencies to core or wasm.
