# DH Parameters

**Denavit-Hartenberg parameters describe the geometry between consecutive joints in a robot arm.** Each link has exactly four parameters that define how one frame is positioned relative to the next.

Bombolab uses the **Craig convention** (also called the modified DH convention).

## The Four Parameters

| Parameter | Symbol | Meaning | Unit |
|-----------|--------|---------|------|
| **theta** | θ | Rotation around Z axis | radians |
| **d** | d | Translation along Z axis | length (m, mm) |
| **a** | a | Translation along X axis (link length) | length (m, mm) |
| **alpha** | α | Rotation around X axis (twist) | radians |

## What Each Parameter Does

### theta (θ) -- Rotation Around Z

The angle the current frame rotates around its Z axis. For revolute joints, this is the joint variable.

```
theta = 0       →  link points in +X direction
theta = π/2     →  link points in +Y direction
theta = π       →  link points in -X direction
```

### d -- Translation Along Z

How far the current frame is offset from the previous one along Z. For prismatic joints, this is the joint variable.

- `d = 5` → the base is 5 units tall
- `d = 0` → no vertical offset

### a -- Link Length (Translation Along X)

The distance from the current Z axis to the next Z axis, measured along X. This is the physical length of the link.

When theta changes, the link length projects into the XY plane:

```
X component = a * cos(theta)
Y component = a * sin(theta)
```

### alpha (α) -- Twist Between Z Axes

**This is the key to 3D robots.** Alpha rotates the frame around X to reorient Z for the next joint.

| alpha | Effect |
|-------|--------|
| 0 | Z axes are parallel -- arm stays in one plane |
| π/2 | Next Z is perpendicular -- arm changes plane |
| -π/2 | Next Z is perpendicular (opposite direction) |
| π | Z axes are anti-parallel |

**Without alpha, all joints rotate in the same 2D plane.** With alpha, you can model real 3D robots like articulated arms with shoulder elbows.

## The A Matrix

Each set of DH parameters produces a 4x4 transformation matrix:

```
A_i = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)
```

Expanded:

```
A_i = ┌ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) ┐
      │ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) │
      │   0        sin(α)          cos(α)           d      │
      └   0          0                0              1      ┘
```

Bombolab computes this with `compute_a_matrix()`:

```rust
use bombolab_core::{DHParameter, compute_a_matrix};

let p = DHParameter::new(0.0, 1.0, 0.5, 0.0); // alpha, a, d, theta
let a_matrix = compute_a_matrix(p);
// a_matrix is a nalgebra::Matrix4<f64>
```

## DHParameter vs DHParams

Bombolab has two DH parameter types for different contexts:

| Type | Module | Purpose |
|------|--------|---------|
| `DHParameter` | `math::dh` | Standalone numeric parameters for the solver |
| `DHParams` | `robot::link` | Fixed geometry stored in a robot segment |

They have the same four fields but live in different parts of the system. `DHParameter` is used by the math solver; `DHParams` is part of the robot model.

## Symbolic Support

The `dh-solve` CLI supports symbolic values. Instead of numbers, you can enter variable names:

```
α: 0
a: L1
d: 0
θ: theta1
```

This produces symbolic A matrices showing the trigonometric expressions rather than numeric values. Useful for deriving closed-form solutions.

The symbolic type is `DHParameterSymbolic` with `DHValue::Num(f64) | DHValue::Sym(String)`.

## Example: 2-Link Planar Arm

A simple arm with two revolute joints, both rotating in the XY plane:

| Link | α | a | d | θ |
|------|---|---|---|---|
| 1 | 0 | 1.0 | 0 | θ₁ |
| 2 | 0 | 1.0 | 0 | θ₂ |

With θ₁ = 0, θ₂ = 0: end-effector at (2.0, 0.0, 0.0)
With θ₁ = π/2, θ₂ = 0: end-effector at (0.0, 2.0, 0.0)

## Example: 3D Arm with Plane Change

A shoulder-elbow robot that moves in 3D:

| Link | α | a | d | θ |
|------|---|---|---|---|
| 1 (base) | 0 | 0 | 5.0 | θ₁ |
| 2 (shoulder) | π/2 | 3.0 | 0 | θ₂ |
| 3 (elbow) | 0 | 2.0 | 0 | θ₃ |

The α = π/2 at the shoulder twists the plane so the arm can reach up and out.

## References

- [Forward Kinematics](./forward-kinematics.md) -- how A matrices compose into the full solution
- [Robot Model](./robot-model.md) -- how DH parameters fit into the Segment/Robot structs
- [dh-solve CLI](../cli/dh-solve.md) -- interactive DH table solver
