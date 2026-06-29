# bombolab-core API Reference

Core library for Bombolab. Provides DH parameter math, robot modeling, and forward kinematics computation.

**Crate**: `bombolab-core`
**Dependency**: `nalgebra = "0.35.0"`

## Re-exports

The top-level `bombolab_core` module re-exports the most commonly used items:

```rust
// Math
pub use math::{DHParameter, DHSolution, DEG_TO_RAD, PI, RAD_TO_DEG, compute_a_matrix, solve};

// Kinematics
pub use kinematics::{forward_kinematics, matrix_from_segment};

// Robot model
pub use robot::{DHParams, Error, Joint, JointType, Result, Robot, Segment};

// nalgebra types
pub use nalgebra::Isometry3;
```

---

## Math Module (`math`)

### `DHParameter`

Numeric DH parameters for a single link.

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DHParameter {
    pub alpha: f64,
    pub a: f64,
    pub d: f64,
    pub theta: f64,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(alpha: f64, a: f64, d: f64, theta: f64) -> Self` | Create parameters (Craig convention order) |

### `compute_a_matrix`

Compute the 4x4 transformation matrix from DH parameters.

```rust
pub fn compute_a_matrix(p: DHParameter) -> Matrix4<f64>
```

Returns `A_i = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)`.

**Example**:

```rust
use bombolab_core::{DHParameter, compute_a_matrix};

let p = DHParameter::new(0.0, 1.0, 0.5, 0.0);
let m = compute_a_matrix(p);
// m is a 4x4 identity-like matrix with translation (1.0, 0.0, 0.5)
```

### `solve`

Solve a complete DH table and return all intermediate results.

```rust
pub fn solve(table: &[DHParameter]) -> DHSolution
```

**Parameters**: slice of `DHParameter` (one per link)
**Returns**: `DHSolution` with the full solution

### `DHSolution`

Complete result of solving a DH table.

```rust
pub struct DHSolution {
    pub table: Vec<DHParameter>,
    pub a_matrices: Vec<Matrix4<f64>>,
    pub intermediates: Vec<Matrix4<f64>>,
    pub final_transform: Matrix4<f64>,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `rotation` | `(&self) -> Matrix3<f64>` | 3x3 rotation matrix from the final transform |
| `translation` | `(&self) -> Vector3<f64>` | Position vector from the final transform |

Implements `Display` for formatted output (DH table, A matrices, frames, final pose).

### `DHValue`

Numeric or symbolic value for DH parameters.

```rust
#[derive(Debug, Clone)]
pub enum DHValue {
    Num(f64),
    Sym(String),
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `is_numeric` | `(&self) -> bool` | True if `Num` variant |
| `as_num` | `(&self) -> Option<f64>` | Extract numeric value |
| `as_str` | `(&self) -> &str` | Extract symbolic string |

### `DHParameterSymbolic`

DH parameters that can contain symbolic variables.

```rust
#[derive(Debug, Clone)]
pub struct DHParameterSymbolic {
    pub alpha: DHValue,
    pub a: DHValue,
    pub d: DHValue,
    pub theta: DHValue,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(alpha: DHValue, a: DHValue, d: DHValue, theta: DHValue) -> Self` | Create symbolic parameters |
| `is_numeric` | `(&self) -> bool` | True if all values are numeric |
| `to_numeric` | `(&self) -> Option<DHParameter>` | Convert to numeric (if possible) |

### `format_symbolic_matrix`

Format a symbolic DH matrix as a string.

```rust
pub fn format_symbolic_matrix(p: &DHParameterSymbolic, angle_unit: &str) -> String
```

**Parameters**:
- `p` -- symbolic DH parameters
- `angle_unit` -- `"grados"` or `"radianes"` (controls display format)

### Constants

```rust
pub const PI: f64 = std::f64::consts::PI;
pub const DEG_TO_RAD: f64 = PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / PI;
pub const FRAC_PI_2: f64 = std::f64::consts::FRAC_PI_2;
pub const FRAC_PI_4: f64 = std::f64::consts::FRAC_PI_4;
pub const EPS: f64 = f64::EPSILON;
pub const TAU: f64 = std::f64::consts::TAU;
```

---

## Quaternion Module (`math::quaternion`)

### `Quaternion`

```rust
#[derive(Debug, Clone, PartialEq)]
pub struct Quaternion {
    pub a: f64,  // real
    pub b: f64,  // i
    pub c: f64,  // j
    pub d: f64,  // k
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(a, b, c, d: f64) -> Self` | Create quaternion |
| `identity` | `() -> Self` | `(1, 0, 0, 0)` |
| `zero` | `() -> Self` | `(0, 0, 0, 0)` |
| `norm` | `(&self) -> f64` | Magnitude |
| `norm_sq` | `(&self) -> f64` | Squared magnitude |
| `normalize` | `(&self) -> Self` | Unit quaternion |
| `conjugate` | `(&self) -> Self` | `(a, -b, -c, -d)` |
| `inverse` | `(&self) -> Self` | Conjugate / norm² |

### Operations

```rust
pub fn solve_add(quaternions: &[Quaternion]) -> Quaternion
pub fn solve_subtract(quaternions: &[Quaternion]) -> Quaternion
pub fn solve_multiply(quaternions: &[Quaternion]) -> Quaternion
pub fn solve_divide(quaternions: &[Quaternion]) -> Quaternion
```

All operations are sequential: `solve_op(&[q1, q2, q3])` applies left to right.
`solve_add` starts from zero; the others start from identity.

---

## Robot Module (`robot`)

### `JointType`

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,
    Prismatic,
}
```

Implements `Display`: `"R"` for Revolute, `"P"` for Prismatic.

### `Joint`

```rust
pub struct Joint {
    pub joint_type: JointType,
    pub value: f64,
    pub value_max: f64,
    pub value_min: f64,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(joint_type, value, value_max, value_min: f64) -> Self` | Create joint |
| `range` | `(&self) -> Vec<f64>` | `[value_min, value_max]` |
| `is_within_limits` | `(&self) -> bool` | Check if value is in range |
| `clamp` | `(&mut self)` | Force value within limits |
| `set_value` | `(&mut self, value: f64) -> Result<()>` | Set with bounds check |

### `DHParams`

```rust
pub struct DHParams {
    pub theta: f64,
    pub d: f64,
    pub a: f64,
    pub alpha: f64,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(theta, d, a, alpha: f64) -> Self` | Create DH parameters |

### `Segment`

```rust
pub struct Segment {
    pub joint: Joint,
    pub dh: DHParams,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(joint: Joint, dh: DHParams) -> Self` | Create segment |
| `dh_params` | `(&self) -> (f64, f64, f64, f64)` | Resolve `(theta, d, a, alpha)` based on joint type |

### `Robot`

```rust
pub struct Robot {
    pub segments: Vec<Segment>,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(segments: Vec<Segment>) -> Self` | Create robot |
| `dof` | `(&self) -> usize` | Degrees of freedom (segment count) |
| `segment` | `(&self, index: usize) -> Result<&Segment>` | Get segment by index |
| `segment_mut` | `(&mut self, index: usize) -> Result<&mut Segment>` | Get mutable segment |
| `set_joint_values` | `(&mut self, joints: Vec<Joint>) -> Result<()>` | Update all joints (validates limits) |
| `reset_to_zero` | `(&mut self)` | Set all joint values to 0.0 |
| `is_empty` | `(&self) -> bool` | True if no segments |
| `add_segment` | `(&mut self, segment: Segment)` | Append segment |
| `remove_segment` | `(&mut self, index: usize) -> Result<Segment>` | Remove and return segment |

### `Error`

```rust
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    JointCountMismatch { expected: usize, got: usize },
    IndexOutOfBounds { index: usize, len: usize },
    JointValueOutOfLimits { value: f64, min: f64, max: f64 },
}
```

Implements `Display` and `std::error::Error`.

---

## Kinematics Module (`kinematics`)

### `forward_kinematics`

Compute forward kinematics for a robot chain.

```rust
pub fn forward_kinematics(
    base: Isometry3<f64>,
    robot: &Robot,
) -> (Vec<Isometry3<f64>>, Isometry3<f64>)
```

**Parameters**:
- `base` -- world-to-base transformation (use `Isometry3::identity()` for origin)
- `robot` -- the robot to solve

**Returns**: `(frames, end_effector)`
- `frames` -- one `Isometry3` per segment (cumulative pose at each joint)
- `end_effector` -- the final pose (last element of `frames`)

**Example**:

```rust
use bombolab_core::{DHParams, Joint, JointType, Robot, Segment, Isometry3, forward_kinematics};

let robot = Robot::new(vec![
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, 3.14, -3.14),
        DHParams::new(0.0, 0.0, 1.0, 0.0),
    ),
]);

let (frames, effector) = forward_kinematics(Isometry3::identity(), &robot);
assert_eq!(frames.len(), 1);
```

### `matrix_from_segment`

Compute the transformation for a single segment.

```rust
pub fn matrix_from_segment(segment: &Segment) -> Isometry3<f64>
```

Returns the isometry representing the segment's DH transformation: `RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)`.

---

## HMatrix Module (`math::hmatrix`)

### `Movement`

```rust
pub struct Movement {
    pub translation: Vector3<f64>,
    pub angles: f64,
    pub axis: Vector3<f64>,
    pub isometry: bool,
}
```

### Functions

```rust
pub fn rotation_and_translation(
    axis: Vector3<f64>, angle: f64, translation: Vector3<f64>
) -> Isometry3<f64>

pub fn translation_and_rotation(
    axis: Vector3<f64>, angle: f64, translation: Vector3<f64>
) -> Isometry3<f64>

pub fn make_movement(
    initial: Isometry3<f64>, movements: &[Movement]
) -> (Vec<Isometry3<f64>>, Isometry3<f64>)
```

`make_movement` composes a sequence of movements from an initial pose, returning the trajectory and final pose.
