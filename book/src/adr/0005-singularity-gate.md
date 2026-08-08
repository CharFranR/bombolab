# ADR-0005: SVD Singularity Gate per Waypoint

- **Status**: Accepted
- **Date**: 2026-08-08

## Context

ADR-0001 derives the constrained drawing IK on the variety

```
M = { q4 = 0, q5 = −(q2+q3) }
```

using the chain-rule reduced Jacobian

```
Jᵣ = [ J₁ , J₂ − J₅ , J₃ − J₅ ].
```

The web pre-flight `validateDrawingCommands` proves every sampled waypoint
sits inside the reachable drawing band, but reachability says nothing about
the conditioning of `Jᵣ` at that waypoint. A trajectory whose waypoints are
all reachable can still force the arm through a pose where the drawing-plane
motion loses a degree of freedom — the Jacobian degenerates and the IK solve
becomes ill-conditioned or diverges.

A numerical probe of the FABRI Creator (4 mm sampling, warm-started solves,
finite-difference verification of `Jᵣ` with d = 1e-6) establishes the
singularity landscape before any gate thresholds are chosen:

| Probe | Result |
|---|---|
| Square 5x5 cm @ (200,0) z80, 40 pts | sigma_min in [81.96, 82.05] mm, kappa <= 2.77 — silent |
| Lines demo, 483 pts | sigma_min >= 76.47 mm, kappa <= 3.10 — silent |
| Arc r180, 141 pts | sigma_min >= 83.15 mm, kappa <= 2.18 — silent |
| Grid z in [60..120], r <= 320 mm | min sigma_min 19.76 mm at z120 outer corner; max kappa 16.72 |
| Radial theta=0 z80 | IK fails for r < 160 (J5 pitch limit q5 = −q23 -> ±115°); at r = 160, q23 ≈ 107°, sigma_min = 78 mm |
| Artificial double-fold q23 -> 180° (metrics only) | sigma_min: 63 mm at 115°, 45 at 135°, 31 at 150°, 15 at 165°, 0.0 at 180° (det -> 3.4e-11) |

Two facts follow:

1. **The singularity locus det Jᵣ = 0 is the double-fold q23 -> 180°** (TCP
   collapses onto the J1 axis). It is unreachable: the J5 pitch hard stop at
   q23 = ±115° halts the arm at r ≈ 139 mm with sigma_min ≈ 63 mm. That stop
   is a joint limit, NOT a Jacobian degeneracy.
2. **Inside joint limits the arm has a conditioning floor**: sigma_min >=
   ~19.8 mm and kappa <= ~17 across the reachable workspace. The historical
   "77 mm corner" is not singular either — corners measure sigma_min ≈ 82 mm;
   the 77 mm value was the pre-ADR-0001 free-wrist z-deviation.

The only realistic block event for this arm is therefore warm-started IK
non-convergence (pitch-limit crossing or a q jump between samples). The
sigma_min/kappa block thresholds remain as a safety net for other tools and
robots.

## Decision

Gate every drawing trajectory between reachability validation and playback
creation. The core owns the analysis: it samples the path, solves warm-started
IK, computes `Jᵣ`, and classifies every sampled waypoint with the SVD of
`Jᵣ` (nalgebra, wasm-safe). The web layer renders the result and never
re-derives it.

- **Thresholds** (calibrated, parameterized, defaults in `SingularityThresholds`):
  - warn: sigma_min < 25 mm OR kappa > 20
  - block: sigma_min < 5 mm OR kappa > 100 OR warm-started IK non-convergence
  - The warn values are 1.25x above the worst in-band measurements (19.8 mm /
    16.7) and the shipped demo paths are verified silent.
- **Sampling**: 4 mm step, capped at 5000 solved points (step grows past the
  cap), warm-started from the previous waypoint's converged q — never from
  ZERO_Q — with the first point at `kinematic_home()`. Worst-N = 10 waypoints
  sorted by sigma_min. Deterministic by construction: no RNG anywhere in the
  gate.
- **Classification policy**: one block waypoint blocks the whole trajectory;
  otherwise one warn waypoint warns; otherwise the gate is silent.
- **Web behavior**: block sets `drawingBlock` (reason + offending waypoint +
  canRefit for gcode) and playback never starts; warn appends to
  `gcodeWarnings` and asks a draw-anyway confirm, after which playback
  proceeds with the ORIGINAL command list — the gate never mutates the
  trajectory (MP-1). Non-convergence rows report the last converged q and zero
  metrics, mirroring the binding's `converged: false` convention.

### Payload contract (`analyze_path_singularity`)

```
{
  "sampled": number,
  "worst": [
    {
      "index": number,
      "target": [x, y, z],
      "q": [q1, q2, q3, q4, q5],
      "sigma_min": number,
      "kappa": number,
      "yoshikawa": number,
      "level": "ok" | "warn" | "block",
      "reason": "metrics" | "ik_non_convergence"
    }
  ]
}
```

`worst` holds at most 10 waypoints. `level` is the waypoint classification;
`reason` distinguishes a threshold violation from a failed warm-started IK
solve. Thresholds travel from the web wrapper, never hardcoded in core.

## Consequences

- The demo paths (square, lines, arc) stay silent; a real gcode job crossing
  the pitch limit is blocked with a concrete reason instead of failing
  mid-drawing.
- Because the J5 pitch stop is a joint limit, not a degeneracy, block events
  are reported as IK non-convergence — the reason field makes this
  diagnosable in the UI.
- The gate is a pre-flight guarantee: no forbidden movement is ever commanded;
  draw-anyway requires explicit user confirmation and leaves the trajectory
  unchanged.
- Thresholds are parameters; a future tool or robot re-calibrates them in the
  web wrapper and this ADR, not in core code.
