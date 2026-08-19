# bombolab-core API Reference

Core library for Bombolab. Provides DH parameter math, robot modeling, and forward kinematics computation.

**Crate**: `bombolab-core`
**Dependency**: `nalgebra = "0.35.0"`

## Re-exports

The top-level `bombolab_core` module re-exports the most commonly used items:

```rust
// Math
pub use math::{DHParameter, DHSolution, DEG_TO_RAD, PI, RAD_TO_DEG, compute_a_matrix, solve};
pub use math::{Iso3, Rot3, Mat4, Vec3, Quat};
pub use math::{geometric_jacobian, JacobianError, Movement};

// Kinematics
pub use kinematics::forward_kinematics;
pub use kinematics::{IkSolver, IkError};

// Robot model
pub use robot::{DHParams, Error, Joint, JointType, Result, Robot, Segment};
pub use robot::fabri_creator::{base_transform, fabri_creator, tool_transform};
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
| `translation` | `(&self) -> Vec3<f64>` | Position vector from the final transform |

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

### `JointKind`

Joint type for Jacobian column computation.

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
}
```

### `JacobianError`

```rust
#[derive(Debug, Clone, PartialEq)]
pub enum JacobianError {
    EmptyChain,
    JointKindMismatch { intermediates: usize, kinds: usize },
}
```

### `geometric_jacobian`

Compute the 6×n geometric Jacobian for a serial chain.

```rust
pub fn geometric_jacobian(
    intermediates: &[Matrix4<f64>],
    joint_kinds: &[JointKind],
    end_effector: &Matrix4<f64>,
) -> Result<MatDyn, JacobianError>
```

**Parameters**:
- `intermediates` — cumulative transforms from `DHSolution::intermediates`
- `joint_kinds` — joint type per column (must match `intermediates.len()`)
- `end_effector` — end-effector transform (usually `intermediates.last()`)

**Returns**: `6 × n` matrix where column `i` = `[z_i × (p_ee − p_i); z_i]` for revolute, `[z_i; 0]` for prismatic.

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
    Twist,
}
```

Implements `Display`: `"R"` for Revolute, `"P"` for Prismatic, `"T"` for Twist.

| Variant | Joint variable | DH formula | Used for |
|---------|---------------|------------|----------|
| `Revolute` | `q` → `theta` | `RotZ(θ)·TransZ(d)·TransX(a)·RotX(α)` | Standard rotary joints |
| `Prismatic` | `q` → `d` | `RotZ(θ)·TransZ(d)·TransX(a)·RotX(α)` | Linear/sliding joints |
| `Twist` | `q` → `alpha` | `RotX(α+q)·TransX(a)` | Wrist roll (rotation about forearm axis) |

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
    pub home_pose: Vec<f64>,         // servo angles at kinematic zero (radians)
    pub servo_offsets: Vec<f64>,     // servo_angle = q + offset (radians)
    pub servo_directions: Vec<f64>,  // +1 horario, -1 anti horario
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(segments: Vec<Segment>) -> Self` | Create robot with zero offsets |
| `with_offsets` | `(segments, home_pose, offsets) -> Self` | Create with explicit servo mapping |
| `with_directions` | `(segments, home_pose, offsets, directions) -> Self` | Create with offsets + directions |
| `dof` | `(&self) -> usize` | Degrees of freedom (segment count) |
| `segment` | `(&self, index: usize) -> Result<&Segment>` | Get segment by index |
| `segment_mut` | `(&mut self, index: usize) -> Result<&mut Segment>` | Get mutable segment |
| `set_joint_values` | `(&mut self, joints: Vec<Joint>) -> Result<()>` | Update all joints (validates limits) |
| `reset_to_zero` | `(&mut self)` | Set all joint values to 0.0 |
| `is_empty` | `(&self) -> bool` | True if no segments |
| `add_segment` | `(&mut self, segment: Segment)` | Append segment |
| `remove_segment` | `(&mut self, index: usize) -> Result<Segment>` | Remove and return segment |
| `q_to_servo` | `(&self, q: &[f64]) -> Vec<f64>` | Kinematic q → servo angles |
| `servo_to_q` | `(&self, servo: &[f64]) -> Vec<f64>` | Servo angles → kinematic q |
| `kinematic_home` | `(&self) -> Vec<f64>` | Home in kinematic space (should be zeros) |

The servo conversion formulas:

```
horario (dir = +1):    servo = q + offset   → q = servo - offset
anti horario (dir = -1): servo = offset - q → q = offset - servo
```

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

### FABRI Creator

Ready-made robot configuration for the 5-DOF educational arm:

```rust
pub fn fabri_creator() -> Robot
pub fn base_transform() -> Iso3  // 57mm vertical offset from ground
pub fn tool_transform() -> Iso3  // 117mm along X (marker tip)
```

---

## Kinematics Module (`kinematics`)

### `forward_kinematics`

Compute forward kinematics for a robot chain.

```rust
pub fn forward_kinematics(
    base: Iso3<f64>,
    robot: &Robot,
) -> (Vec<Iso3<f64>>, Iso3<f64>)
```

**Parameters**:
- `base` -- world-to-base transformation (use `Iso3::identity()` for origin)
- `robot` -- the robot to solve

**Returns**: `(frames, end_effector)`
- `frames` -- one `Iso3` per segment (cumulative pose at each joint)
- `end_effector` -- the final pose (last element of `frames`)

**Example**:

```rust
use bombolab_core::{DHParams, Joint, JointType, Robot, Segment, Iso3, forward_kinematics};

let robot = Robot::new(vec![
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, 3.14, -3.14),
        DHParams::new(0.0, 0.0, 1.0, 0.0),
    ),
]);

let (frames, effector) = forward_kinematics(Iso3::identity(), &robot);
assert_eq!(frames.len(), 1);
```

### `matrix_from_segment`

Compute the transformation for a single segment.

```rust
pub fn matrix_from_segment(segment: &Segment) -> Iso3<f64>
```

Returns the isometry representing the segment's DH transformation: `RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)`.

### `IkSolver`

Position-only IK solver using damped pseudoinverse (Levenberg–Marquardt). Operates on a complete `Robot` with base and tool transforms.

```rust
pub struct IkSolver {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub damping: f64,
    pub step_size: f64,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(max_iterations, tolerance, damping, step_size) -> Self` | Create solver |
| `solve_position` | `(&self, target, q_init, robot, base, tool) -> Result<Vec<f64>, IkError>` | Solve IK |

**Parameters for `solve_position`**:
- `target: &[f64; 3]` — desired [x, y, z] position in world coordinates (mm)
- `q_init: &[f64]` — seed joint angles (radians)
- `robot: &Robot` — the robot to solve
- `base: &Iso3` — base transform from ground to joint 1
- `tool: &Iso3` — tool transform from last joint to end effector

**Returns**: `Ok(q)` on convergence, `Err(IkError)` otherwise.

### `IkError`

```rust
pub enum IkError {
    DegenerateChain,
    MaxIterationsReached { error: f64 },
}
```

### `geometric_jacobian`

Compute the 6×n geometric Jacobian for a serial chain.

```rust
pub fn geometric_jacobian(
    intermediates: &[Iso3],
    joint_types: &[JointType],
    end_effector: &Iso3,
) -> Result<MatDyn, JacobianError>
```

The Jacobian uses the correct rotation axis per joint type:
- **Revolute/Prismatic**: `Z_{i-1}` (Z axis of the previous frame)
- **Twist**: `X_{i-1}` (X axis of the previous frame)

### `JacobianError`

```rust
pub enum JacobianError {
    EmptyChain,
    JointKindMismatch { intermediates: usize, kinds: usize },
}
```

---

## HMatrix Module (`math::hmatrix`)

### `Movement`

```rust
pub struct Movement {
    pub translation: Vec3<f64>,
    pub angles: f64,
    pub axis: Vec3<f64>,
    pub isometry: bool,
}
```

### Functions

```rust
pub fn rotation_and_translation(
    axis: Vec3<f64>, angle: f64, translation: Vec3<f64>
) -> Iso3<f64>

pub fn translation_and_rotation(
    axis: Vec3<f64>, angle: f64, translation: Vec3<f64>
) -> Iso3<f64>

pub fn make_movement(
    initial: Iso3<f64>, movements: &[Movement]
) -> (Vec<Iso3<f64>>, Iso3<f64>)
```

`make_movement` composes a sequence of movements from an initial pose, returning the trajectory and final pose.
