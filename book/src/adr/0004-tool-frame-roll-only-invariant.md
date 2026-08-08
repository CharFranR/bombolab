# ADR-0004: Tool-Frame Roll-Only Invariant

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

That derivation assumes the tool is mounted as a pure translation along the
last joint X-axis: the tool pose has **identity rotation** relative to the
flange. The offset only shifts the TCP along X, so the wrist orientation
residual `R35 = R03ᵀ · R_target` keeps the analytic form `[c,s,0; 0,1,0;
−s,c,0]` that forces `q4 = 0` and `q5 = −(q2+q3)` exactly.

## Decision

Only **roll about the tool X-axis** preserves the drawing variety `M` and the
reduced Jacobian `Jᵣ` of ADR-0001. A roll rotates the marker about the tool
axis itself: it changes no constraint equation and no Jacobian column, so the
reduced derivation stays exact.

Any tool rotation about **Y or Z invalidates the derivation**: the tool pose
is no longer a pure X translation, the wrist residual no longer has the
analytic form, `q4 = 0` and `q5 = −(q2+q3)` cease to hold, and the columns
`J₂ − J₅`, `J₃ − J₅` are no longer the exact linearization of the constrained
motion.

Consequently every `ToolFrame` preset (marker, pen, gripper) uses identity
rotation with a translation-only offset along X. The same invariant is stated
in the `ToolFrame` rustdoc.

## Consequences

- Tool presets are constrained to X translations; tools with Y/Z-rotated poses
  are outside the drawing-IK contract.
- The reduced Jacobian stays valid for singularity analysis of drawing motions.
- A future tool with a non-roll rotation requires re-deriving the constraint
  (new ADR), not a parameter tweak.
