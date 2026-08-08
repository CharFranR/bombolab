# ADR-0001: Constrained Drawing IK with Reduced Jacobian

- **Status**: Accepted
- **Date**: 2026-08-08

## Context

The drawing mode solves inverse kinematics for a 5-DOF arm carrying a marker
mounted perpendicular to the last joint axis. The original two-step solver
(`solve_drawing_ik_v2`) first solved position with a free wrist (DLS, 5-DOF,
1 mm tolerance) and then replaced q4/q5 with the analytic wrist orientation.
This left the TCP deviating **60–77 mm in z** at the demo corner: the free-wrist
solve converged to a pose whose wrist was ~60° away from the drawing posture.

A second analysis showed that for a target reached with the drawing constraint,
the orientation residual `R35 = R03ᵀ · R_target` is analytically
`[c,s,0; 0,1,0; −s,c,0]`, which forces **q4 = 0 and q5 = −(q2+q3) exactly**
(mod 2π). The search could therefore be restricted to a 3-DOF manifold.

## Decision

Solve directly on the constrained variety

```
M = { q4 = 0, q5 = −(q2+q3) }
```

using the chain-rule reduced Jacobian

```
Jᵣ = [ J₁ , J₂ − J₅ , J₃ − J₅ ]
```

verified by finite differences (error ~2e-8). Reachability is checked before
iterating with the test `|R35[0,2]| < tol`, which is the exact reachability
condition on the constrained variety.

## Consequences

- TCP deviation dropped to **0.09–0.57 mm** in the demo.
- The arm has **no yaw axis**: not all orientations in SO(3) are reachable;
  unreachable orientations are rejected deterministically.
- The wrist is never free during drawing: every iteration is a physically valid
  drawing posture.
- Null-space optimization does **not apply** to drawing mode: the wrist DOF are
  fixed by the constraint, not redundant.
- The drawing workspace is bounded by the J5 pitch limits (`q5 = −q23` must be
  within its range).
- The reduced Jacobian is also the basis for future per-waypoint analysis
  (singularity detection), since it is the exact linearization of the
  constrained motion.
