# Inverse Kinematics

**The IK solver finds joint values that place the end effector at a desired pose.** You specify a target (position + optional orientation), and the solver iteratively adjusts joint angles until the robot reaches it.

## The Core Idea

Forward kinematics answers: *given joint values, where is the end effector?* Inverse kinematics answers the opposite: *given a target pose, what joint values get me there?*

For serial chains with more than a few joints, there's no closed-form solution. Bombolab uses a **damped pseudoinverse (Levenberg–Marquardt)** approach:

```
Δq = Jᵀ(J·Jᵀ + λ²·I)⁻¹ · Δx
```

Where:
- `Δq` = joint correction
- `J` = geometric Jacobian (6×n)
- `λ` = damping factor (regularizes near singularities)
- `Δx` = pose error (6×1: position + orientation)

## How It Works

### Step 1: Set up the target

```rust
use bombolab_core::{inverse_kinematics, IkOptions, solve, JointKind::Revolute};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

let table = vec![
    DHParameter::new(0.0, 1.0, 0.0, 0.0),
    DHParameter::new(0.0, 1.0, 0.0, 0.0),
];
let kinds = vec![Revolute, Revolute];
let initial_guess = vec![0.0, 0.0];

// Target: position (1.5, 0.5, 0.0), no orientation constraint
let target = Isometry3::from_parts(
    Translation3::new(1.5, 0.5, 0.0),
    UnitQuaternion::identity(),
);
```

### Step 2: Configure and solve

```rust
let options = IkOptions::default();
let result = inverse_kinematics(&table, &kinds, &initial_guess, &target, &options)?;

println!("Converged: {} in {} iterations", result.converged, result.iterations);
println!("Joint angles: {:?}", result.joint_angles);
println!("Final pose error: {}", result.final_error);
```

### Step 3: Verify

```rust
// Round-trip check: FK(IK) ≈ target
let fk_solution = solve_segments(&table, &result.joint_angles);
let end_pos = fk_solution.translation();
println!("End effector at: ({:.3}, {:.3}, {:.3})", end_pos.x, end_pos.y, end_pos.z);
```

## Options

| Option | Default | Description |
|--------|---------|-------------|
| `tolerance_pos` | 1.0 | Position convergence threshold (mm) |
| `tolerance_angle` | 0.1 | Orientation convergence threshold (rad, ~6°) |
| `max_iterations` | 200 | Maximum iterations before giving up |
| `damping` | 0.1 | Initial damping factor `λ` for pseudoinverse regularization |
| `min_damping` | 1e-6 | Floor for damping — prevents rank-1 updates from going to zero |
| `damping_update` | `GainRatio` | Adaptation strategy: `GainRatio` (trust-region) or `Fixed` |
| `joint_limits` | `None` | Joint limits as `Vec<(f64, f64)>` — clamps after each iteration |

### Damping Strategies

**`GainRatio` (default):** Trust-region approach — adapts `λ` based on how well the linearized model predicted the actual error reduction:
- Large reduction → `λ` decreases (bigger steps)
- Poor reduction → `λ` increases (smaller, safer steps)

**`Fixed`:** Constant damping. Simpler, but less robust near singularities.

## Result

```rust
pub struct IkResult {
    pub converged: bool,           // Did we hit all tolerances?
    pub joint_angles: Vec<f64>,    // Final joint values
    pub iterations: u32,           // Iterations used
    pub final_error: f64,          // Final pose error norm
    pub position_error: f64,       // Final position error
    pub orientation_error: f64,    // Final orientation error
    pub damping_used: f64,         // Final damping value
}
```

## Error Handling

`inverse_kinematics` returns `Result<IkResult, IkError>`:

| Error | Cause |
|-------|-------|
| `MismatchedLengths` | Joint count doesn't match DH table length |
| `EmptyChain` | No joints provided |
| `InvalidOptions` | Invalid damping or tolerance values |
| `DidNotConverge` | Reached `max_iterations` without converging |

Even when `converged` is `false`, `joint_angles` contains the best attempt — useful for re-seeding with a different initial guess.

## Task DOF: Free Z-Roll

The solver includes a **Task DOF** model for the orientation error. By default, rotation about the end-effector's Z axis is treated as a **free DOF** — the solver does not penalize Z-roll errors.

This matches how most serial robots work: the last joint typically controls Z rotation, and imposing a fixed Z orientation over-constrains the problem. You can disable this via `IkOptions` (not yet exposed — open an issue if you need it).

## Example: FABRI Creator at Home Pose

```rust
let home = vec![0.0, 0.0, 0.0, 0.0, 0.0];
let target = fk(&fabri_creator_robot, &home); // FK computes where home is
let result = ik(&table, &kinds, &home, &target, &IkOptions::default())?;
assert!(result.converged);
```

## When It Fails

- **Far from a reachable target:** the damped pseudoinverse converges locally. If the target is unreachable or the initial guess is far off, try a better `initial_guess` or use a multi-start strategy.
- **Near singularities:** the solver handles this with damping, but extremely high joint speeds may still appear. Increase `damping` if the solver becomes unstable.
- **Orientation-only targets are not supported yet** — `TaskDOF` still requires a position component.

## References

- [Jacobian](./jacobian.md) — the geometric Jacobian used inside the solver
- [Forward Kinematics](./forward-kinematics.md) — the FK round-trip check
- [Robot Model](./robot-model.md) — how segments and joints compose a robot
- [bombolab-core API](../api/core.md) — full function signatures
