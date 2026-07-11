# Jacobian

**The geometric Jacobian relates joint velocities to end-effector velocities.** Given joint speeds `q̇`, it computes the resulting linear and angular velocity of the end effector:

```text
[v; ω] = J · q̇
```

Where `v` is linear velocity (3×1) and `ω` is angular velocity (3×1).

## How It's Computed

Each column of J depends on the joint type:

| Joint | J_i |
|-------|-----|
| Revolute | `[z_i × (p_n − p_i); z_i]` |
| Prismatic | `[z_i; 0]` |

- `z_i` = z-axis of frame i (third column of its rotation matrix), in base coordinates
- `p_i` = origin of frame i, in base coordinates
- `p_n` = end-effector position, in base coordinates

The top 3 rows are the linear contribution (`J_v`), the bottom 3 the angular (`J_ω`).

## Usage

```rust
use bombolab_core::{DHParameter, JointKind, geometric_jacobian, solve};

let table = vec![
    DHParameter::new(0.0, 1.0, 0.0, 0.0),
    DHParameter::new(0.0, 1.0, 0.0, 0.0),
];
let sol = solve(&table);
let kinds = [JointKind::Revolute, JointKind::Revolute];
let j = geometric_jacobian(&sol.intermediates, &kinds, &sol.final_transform)
    .expect("valid chain");

assert_eq!(j.nrows(), 6);
assert_eq!(j.ncols(), 2);
```

Takes `intermediates` from `DHSolution`, a matching slice of `JointKind`, and the end-effector transform. Returns `Result` — check `JacobianError` for invalid inputs.

## What It's Used For

- **Inverse kinematics** — `Δq = J⁺ · Δx` where `J⁺` is the pseudo-inverse and `Δx` is the pose error
- **Singularity analysis** — when `det(J·Jᵀ) ≈ 0`, the robot is in a singular configuration
- **Velocity planning** — map desired end-effector speeds to joint speeds
