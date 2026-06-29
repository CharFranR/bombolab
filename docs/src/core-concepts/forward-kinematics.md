# Forward Kinematics

**Forward kinematics computes the position and orientation of every link in a robot given its joint values.** You provide the joint angles (or displacements), and Bombolab tells you where each frame is in 3D space.

## The Core Idea

A robot is a chain of frames. Each frame is positioned relative to the previous one by a transformation matrix derived from DH parameters. Forward kinematics multiplies these matrices together:

```
T₀₁ = A₁
T₀₂ = A₁ · A₂
T₀₃ = A₁ · A₂ · A₃
...
T₀ₙ = A₁ · A₂ · ... · Aₙ
```

Each `T₀ᵢ` is the pose (position + rotation) of frame `i` expressed in the base frame.

## How Bombolab Computes It

### Step 1: Build Each A Matrix

For each segment, `compute_a_matrix()` produces a 4x4 transformation from the DH parameters:

```rust
use bombolab_core::{DHParameter, compute_a_matrix};

let p = DHParameter::new(alpha, a, d, theta);
let a_i = compute_a_matrix(p);
```

### Step 2: Compose the Chain

`forward_kinematics()` multiplies them sequentially:

```rust
use bombolab_core::{Iso3, Robot, forward_kinematics};

let (frames, end_effector) = forward_kinematics(base, &robot);
```

The function:

1. Starts with the `base` transformation (usually `Iso3::identity()`)
2. For each segment, computes `current = current * matrix_from_segment(segment)`
3. Stores each cumulative result in `frames`
4. Returns the final frame as `end_effector`

### Step 3: Read the Results

```rust
// Position of each frame
for (i, frame) in frames.iter().enumerate() {
    let pos = frame.translation.vector;
    println!("Frame {}: ({:.3}, {:.3}, {:.3})", i + 1, pos.x, pos.y, pos.z);
}

// End-effector pose
let pos = end_effector.translation.vector;
let rot = end_effector.rotation;
```

## matrix_from_segment

Each segment produces an `Iso3<f64>` (a combined rotation + translation from nalgebra):

```rust
pub fn matrix_from_segment(segment: &Segment) -> Iso3<f64> {
    let (theta, d, a, alpha) = segment.dh_params();

    let rot_z = Rot3::from_axis_angle(&Vec3::z_axis(), theta);
    let rot_x = Rot3::from_axis_angle(&Vec3::x_axis(), alpha);
    let rotation = UnitQuaternion::from_rotation_matrix(&(rot_z * rot_x));

    let translation = Translation3::new(a * theta.cos(), a * theta.sin(), d);

    Iso3::from_parts(translation, rotation)
}
```

### Why UnitQuaternion?

`Iso3` stores rotation internally as a `UnitQuaternion`, not a `Rot3`. The conversion via `UnitQuaternion::from_rotation_matrix()` is necessary because `Iso3::from_parts()` expects a quaternion.

### The Translation

The translation components are:

- **X**: `a * cos(theta)` -- link length projected after rotation
- **Y**: `a * sin(theta)` -- link length projected after rotation
- **Z**: `d` -- direct translation along Z

This is not "movement in X" -- it's the result of rotating a link of length `a` by angle `theta`.

## The Base Transformation

The `base` parameter positions the robot in the world:

```rust
use nalgebra::{Iso3, Translation3, UnitQuaternion};

// Robot at world origin
let base = Iso3::identity();

// Robot displaced 10 units up
let base = Iso3::from_parts(
    Translation3::new(0.0, 0.0, 10.0),
    UnitQuaternion::identity(),
);
```

The same robot definition works in any world position -- you don't need to modify the robot to move it.

## Multiplication Order

The chain multiplies left-to-right:

```
current = current * T_i
```

Each `T_i` transforms **from frame i-1 to frame i**, expressed in frame i-1. This is post-multiplication, which is correct for serial chains where each movement is relative to the previous link.

If you used `T_i * current`, each movement would be in the world's global frame -- wrong for an articulated robot.

## The DHSolution Struct

For the numeric solver (`solve()`), the result is a `DHSolution`:

```rust
pub struct DHSolution {
    pub table: Vec<DHParameter>,         // input parameters
    pub a_matrices: Vec<Matrix4<f64>>,   // individual A_i matrices
    pub intermediates: Vec<Matrix4<f64>>, // cumulative products T₀ᵢ
    pub final_transform: Matrix4<f64>,    // T₀ₙ (end-effector)
}
```

Access methods:

```rust
let solution = solve(&table);

let rotation = solution.rotation();    // Matrix3<f64> -- 3x3 rotation
let translation = solution.translation(); // Vec3<f64> -- position
```

## Example: Planar 2R Robot

Two revolute joints, both in the XY plane (alpha = 0):

```rust
use bombolab_core::{DHParameter, solve};

let table = vec![
    DHParameter::new(0.0, 1.0, 0.0, 0.5),  // link 1: a=1, theta=0.5 rad
    DHParameter::new(0.0, 1.0, 0.0, 0.3),  // link 2: a=1, theta=0.3 rad
];

let solution = solve(&table);
let pos = solution.translation();
println!("End-effector: ({:.3}, {:.3}, {:.3})", pos.x, pos.y, pos.z);
```

## References

- [DH Parameters](./dh-parameters.md) -- what goes into each A matrix
- [Robot Model](./robot-model.md) -- how segments and joints work
- [bombolab-core API](../api/core.md) -- full function signatures
