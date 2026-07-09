# ik-solve

**Interactive CLI tool for inverse kinematics on the FABRI Creator 5-DOF robot.** Given a target position (and optional orientation), finds joint angles that place the end effector at the target.

## Usage

```bash
# From the bombolab-core crate directory:
cargo run --bin ik-solve <x_mm> <y_mm> <z_mm> [roll_deg] [pitch_deg] [yaw_deg]
```

## Examples

### Position only (orientation follows seed)

```bash
cargo run --bin ik-solve 200 0 150
```

Uses the seed orientation (home pose FK). The solver only needs to match the position.

### Position + orientation

```bash
cargo run --bin ik-solve 180 50 120 0 -45 0
```

Solves for both position and orientation. Roll, pitch, yaw are intrinsic ZYX Euler angles in degrees.

## Output

The tool shows:

- **FK at home pose** — where the robot is before solving
- **Target** — desired position and orientation
- **Seed** — initial joint guess (home pose q=[0,0,0,0,0])
- **Result** — convergence status, iterations, errors
- **Joint angles** — both kinematic q and servo angles (rad/deg)
- **Round-trip FK(IK)** — where the IK solution actually places the end effector
- **Joint limits** — whether each joint is within its allowable range

## Tips

- If the solver doesn't converge, the target may be unreachable or too far from the seed. Try a different seed (not yet supported via CLI).
- The `Inverse Kinematics` core concept page explains the algorithm and limitations.
- Source: `crates/bombolab-core/src/bin/ik-solve.rs`

## References

- [Inverse Kinematics](../core-concepts/inverse-kinematics.md) — algorithm and API details
- [Forward Kinematics](../core-concepts/forward-kinematics.md) — the FK round-trip
- [FABRI Creator](../core-concepts/fabri-creator.md) — robot model
