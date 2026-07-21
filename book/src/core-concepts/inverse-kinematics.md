# Inverse Kinematics

**The IK solver finds joint angles that place the end effector at a desired position and orientation.** You specify a 3D target + rotation, and the solver returns joint angles.

## Architecture

The IK is split into two independent solvers composed into a pipeline:

```
TargetPose (position + rotation)
      │
      ▼
 PositionSolver (DLS 3×n, posición)
      │
    q₁,q₂,q₃
      │
      ▼
 FK parcial → R₀₃
      │
      ▼
 OrientationSolver (analítico, 2-DOF wrist)
   R₃₅ = R₀₃ᵀ · R_target
   q₄ = atan2(-R₃₅[2,2], -R₃₅[1,2])
   q₅ = atan2(-R₃₅[0,1],  R₃₅[0,0])
      │
    q₄,q₅
      │
      ▼
 Solución: [q₁, q₂, q₃, q₄, q₅]
```

## Position Solver (IkSolver)

The position solver uses a **damped pseudoinverse (Levenberg–Marquardt)** approach:

```
Δq = Jᵀ(J·Jᵀ + λ²·I)⁻¹ · Δx
```

Where:
- `Δq` = joint correction
- `J` = linear Jacobian (3×n)
- `λ` = damping factor (regularizes near singularities)
- `Δx` = position error (3×1)

## IkSolver

The solver takes a `Robot`, base transform, tool transform, target position, and initial guess:

```rust
use bombolab_core::{
    Robot, Iso3, IkSolver, fabri_creator, base_transform, tool_transform,
};

let robot = fabri_creator();
let base = base_transform();
let tool = tool_transform();

// Target: position (200, 0, 280) in mm
let target = [200.0, 0.0, 280.0];
let q_init = vec![0.0; 5];

let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
//   max_iterations: 200
//   position tolerance: 1mm
//   damping: 0.05
//   max step size: 0.5 rad

let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
```

## Result

```rust
match result {
    Ok(q) => {
        println!("Converged! Joint angles: {:?}", q);
    }
    Err(IkError::MaxIterationsReached { error }) => {
        println!("Did not converge, best error: {:.3}mm", error);
    }
    Err(IkError::DegenerateChain) => {
        println!("Robot has no joints");
    }
}
```

The solver returns `Ok(q)` when the position error is below `tolerance` (default 1mm). If it reaches `max_iterations` without converging, it returns the best attempt in the error variant.

## Options

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_iterations` | 50 | Maximum iterations before giving up |
| `tolerance` | 1.0 | Position convergence threshold (mm) |
| `damping` | 0.1 | Initial damping factor λ for pseudoinverse regularization |
| `step_size` | 0.5 | Maximum joint angle change per iteration (rad, ~28°) |

### How Damping Works

The damping term `λ²·I` ensures the pseudoinverse is well-behaved near singularities. Higher damping = smaller, safer steps. Lower damping = faster convergence but risk of instability near singular configurations.

### Step Size Clamping

The solver limits each iteration's total joint change to `step_size` radians. If the raw DLS step exceeds this, the entire step vector is scaled down. This prevents wildly large joint movements when far from the target.

## Error Handling

| Error | Cause |
|-------|-------|
| `IkError::DegenerateChain` | Robot has 0 DOF |
| `IkError::MaxIterationsReached { error }` | Reached max iterations without converging; `error` is the final position error in mm |

Even when `MaxIterationsReached` is returned, the `q` values in the error represent the best attempt — useful for re-seeding with a different initial guess.

## Jacobian: How Twist Joints Work

The solver builds a 3×n linear Jacobian. Each column is the linear velocity contribution of joint `i`:

```
J[:, i] = axis_i × (p_ee − p_i)
```

The **axis** depends on joint type:
- **Revolute/Prismatic**: `Z_{i-1}` (Z axis of the previous frame)
- **Twist**: `X_{i-1}` (X axis of the previous frame)

This correctly handles wrist roll (J4 in FABRI Creator) where rotation is about the forearm's X axis.

## Example: FABRI Creator at Home Pose

```rust
let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
let robot = fabri_creator();
let base = base_transform();
let tool = tool_transform();

// At home (q=0), the tool tip is at approximately (236, 0, 314)
// Asking for that position should give q≈0
let target = [236.0, 0.0, 314.0];
let q_init = vec![0.0; 5];

let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
assert!(result.is_ok());
let q = result.unwrap();
for (i, &val) in q.iter().enumerate() {
    assert!(val.abs() < 0.05, "J{} should be near 0", i + 1);
}
```

## Orientation Solver

The `OrientationSolver` handles the wrist analytically. Given R₀₃ (from the position solution) and R_target (desired tool orientation):

```
R₃₅ = R₀₃ᵀ · R_target

q₄ = atan2(-R₃₅[2,2], -R₃₅[1,2])
q₅ = atan2(-R₃₅[0,1],  R₃₅[0,0])
```

The FABRI Creator has a 2-DOF wrist (roll on X via Twist, pitch on Z via Revolute). There is **no yaw** — the condition `|R₃₅[0,2]| < ε` tests reachability. If violated, the orientation is physically impossible.

```rust
use bombolab_core::{OrientationSolver, OrientationError};

let orient_solver = OrientationSolver::new(1e-6);
match orient_solver.solve(&r03, &r_target, &robot) {
    Ok([q4, q5]) => { /* wrist solution */ }
    Err(OrientationError::UnreachableOrientation { .. }) => { /* not reachable */ }
}
```

## Full IK Pipeline

`solve_full_ik` composes position + orientation:

```rust
use bombolab_core::{solve_full_ik, IkSolver, OrientationSolver};

let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
let orient_solver = OrientationSolver::new(1e-6);
let target = [200.0, 0.0, 80.0];
let target_rot = /* any Rot3 */;

let result = solve_full_ik(
    &pos_solver, &orient_solver,
    &target, &target_rot, &[0.0; 5],
    &robot, &base, &tool,
);
```

## Drawing Mode

The `PoseGenerator` layer generates target poses for specific tasks without modifying the IK.

### Drawing Pose (Modo 1 — marker along X₅)

Constant orientation, works for centered positions (q₁ ≈ 0):

```rust
use bombolab_core::PoseGenerator;

let pose = PoseGenerator::drawing_pose([200.0, 0.0, 80.0]);
// pose.rotation has X₅ = [0, 0, -1] (marker down)
```

### Adaptive Drawing Pose (Modo 2 — marker along Y₅)

Orientation adapts to q₁: R_target(q₁) makes Y₅ = [0, 0, -1]. Works for **any** arm position where |q₂+q₃| < 80°:

```rust
let pose = PoseGenerator::drawing_pose_v2([200.0, 80.0, 80.0], q1);
```

### Convenience Functions

`solve_drawing_ik` (modo 1) and `solve_drawing_ik_v2` (modo 2) run the full pipeline without manually composing:

```rust
use bombolab_core::{solve_drawing_ik_v2};

let result = solve_drawing_ik_v2(
    &pos_solver, &orient_solver,
    &target, &[0.0; 5], &robot, &base, &tool,
);
// Returns [q1, q2, q3, q4, q5] with the marker vertical
```

### When It Fails

- **Unreachable orientation:** `IkError::UnreachableOrientation` — the target orientation cannot be achieved within joint limits. Falls back to position-only IK.
- **Unreachable position:** same as position solver — `MaxIterationsReached`.

## References

- [Jacobian](./jacobian.md) — the geometric Jacobian used inside the solver
- [Forward Kinematics](./forward-kinematics.md) — the FK round-trip check
- [Robot Model](./robot-model.md) — how segments and joints compose a robot
- [bombolab-core API](../api/core.md) — full function signatures
