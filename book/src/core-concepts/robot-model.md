# Robot Model

**A robot in Bombolab is a serial chain of segments, each combining a joint (the actuator) with fixed geometry (DH parameters).** This page explains the data model and how to build robots programmatically.

## The Chain: Robot → Segment → Joint + DHParams

```
Robot
 └── segments: Vec<Segment>
      ├── joint: Joint      (the motor: type, value, limits)
      └── dh: DHParams      (fixed geometry: theta, d, a, alpha)
```

### Joint

```rust
pub struct Joint {
    pub joint_type: JointType,  // Revolute or Prismatic
    pub value: f64,             // current joint value (radians or meters)
    pub value_max: f64,         // upper limit
    pub value_min: f64,         // lower limit
}
```

**JointType** determines what `value` means:

| JointType | `value` represents | Movement |
|-----------|-------------------|----------|
| `Revolute` | angle (radians) | rotation around Z |
| `Prismatic` | displacement (meters) | translation along Z |

Key methods:

```rust
let joint = Joint::new(JointType::Revolute, 0.5, PI, -PI);

joint.is_within_limits()  // true if value_min <= value <= value_max
joint.clamp()             // force value within limits
joint.set_value(1.0)?     // set with bounds check (returns Err if out of limits)
joint.range()             // returns [value_min, value_max]
```

### DHParams

```rust
pub struct DHParams {
    pub theta: f64,  // rotation around Z (radians)
    pub d: f64,      // translation along Z
    pub a: f64,      // translation along X (link length)
    pub alpha: f64,  // twist around X (radians)
}
```

These are **fixed** for a given robot configuration. They don't change at runtime -- only the joint value changes.

### Segment

```rust
pub struct Segment {
    pub joint: Joint,
    pub dh: DHParams,
}
```

The `dh_params()` method resolves which DH parameter is the joint variable:

```rust
let (theta, d, a, alpha) = segment.dh_params();

// Revolute:  theta = joint.value,  d = dh.d
// Prismatic: theta = dh.theta,     d = joint.value
```

This is the key insight: **the same struct handles both joint types** by swapping which parameter comes from the joint.

### Robot

```rust
pub struct Robot {
    pub segments: Vec<Segment>,
    pub home_pose: Vec<f64>,     // servo angles at kinematic zero (radians)
    pub servo_offsets: Vec<f64>, // servo_angle = q + offset (radians)
}
```

Methods:

| Method | Description |
|--------|-------------|
| `Robot::new(segments)` | Create a robot with zero offsets (backward compatible) |
| `Robot::with_offsets(segments, home_pose, servo_offsets)` | Create with explicit servo mapping |
| `robot.dof()` | Number of degrees of freedom (segment count) |
| `robot.segment(i)` | Get segment by index (returns `Result`) |
| `robot.segment_mut(i)` | Get mutable segment by index |
| `robot.set_joint_values(joints)` | Update all joint values at once (validates limits) |
| `robot.reset_to_zero()` | Set all joint values to 0.0 |
| `robot.add_segment(segment)` | Append a segment |
| `robot.remove_segment(i)` | Remove and return a segment by index |
| `robot.q_to_servo(q)` | Convert kinematic coordinates to servo angles |
| `robot.servo_to_q(servo)` | Convert servo angles to kinematic coordinates |
| `robot.kinematic_home()` | Home pose in kinematic space (should be all zeros) |
| `robot.is_empty()` | True if no segments |

## Kinematic Coordinates vs Servo Angles

FK/IK/Jacobians operate on **kinematic coordinates** (`q`), where `q=0` is the home pose. Physical servo angles are different because servos have an offset at their neutral position.

```
servo_angle = q + offset
```

For example, if J1's servo is at 90° when the robot is at its home pose:

```
q = 0°        →  servo = 0° + 90° = 90°   (home position)
q = 30°       →  servo = 30° + 90° = 120°
q = -45°      →  servo = -45° + 90° = 45°
```

The `home_pose` field stores the servo angles at `q=[0,0,...,0]`. The `servo_offsets` field stores the constant offset per joint. For a well-configured robot, `home_pose == servo_offsets` (since at `q=0`, `servo = 0 + offset = offset`).

```rust
let robot = fabri_creator();

// At kinematic zero, servo angles equal home pose
let q_zero = vec![0.0; 5];
let servo = robot.q_to_servo(&q_zero);
assert_eq!(servo, robot.home_pose);

// Round-trip conversion
let q_test = vec![0.1, -0.2, 0.3, -0.1, 0.15];
let servo_test = robot.q_to_servo(&q_test);
let q_back = robot.servo_to_q(&servo_test);
assert_eq!(q_test, q_back);
```

This separation matters because:
- **Kinematics math** works in `q` space (centered at zero)
- **Hardware communication** works in servo space (absolute angles)
- **Joint limits** are defined in physical servo space (e.g., 10°–170°), then converted to kinematic space for validation

## Building a Robot

### Revolute Arm (Most Common)

```rust
use bombolab_core::{DHParams, Joint, JointType, Robot, Segment};
use std::f64::consts::{PI, FRAC_PI_2};

let robot = Robot::new(vec![
    // Base: rotates, height 5
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, PI, -PI),
        DHParams::new(0.0, 5.0, 0.0, 0.0),
    ),
    // Shoulder: rotates, link length 3, twists plane
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, PI, -PI),
        DHParams::new(0.0, 0.0, 3.0, FRAC_PI_2),
    ),
    // Elbow: rotates, link length 2
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, PI, -PI),
        DHParams::new(0.0, 0.0, 2.0, 0.0),
    ),
]);

assert_eq!(robot.dof(), 3);
```

### Prismatic Joint

```rust
let segment = Segment::new(
    Joint::new(JointType::Prismatic, 0.5, 2.0, 0.0), // extended 0.5m
    DHParams::new(0.0, 0.0, 1.0, 0.0),
);

// dh_params() returns: (theta=0.0, d=0.5, a=1.0, alpha=0.0)
//                       ^^^joint.value goes to d for prismatic
```

### Mixed Joint Types

```rust
let robot = Robot::new(vec![
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, PI, -PI),
        DHParams::new(0.0, 0.0, 1.0, 0.0),
    ),
    Segment::new(
        Joint::new(JointType::Prismatic, 0.0, 2.0, 0.0),
        DHParams::new(0.0, 0.0, 0.5, FRAC_PI_2),
    ),
]);
```

## Error Handling

The robot model uses a custom `Error` enum:

```rust
pub enum Error {
    JointCountMismatch { expected: usize, got: usize },
    IndexOutOfBounds { index: usize, len: usize },
    JointValueOutOfLimits { value: f64, min: f64, max: f64 },
}
```

All fallible operations return `Result<T>`:

```rust
use bombolab_core::Error;

match robot.set_joint_values(joints) {
    Ok(()) => println!("Joints updated"),
    Err(Error::JointCountMismatch { expected, got }) => {
        eprintln!("Expected {} joints, got {}", expected, got);
    }
    Err(Error::JointValueOutOfLimits { value, min, max }) => {
        eprintln!("Value {} out of limits [{}, {}]", value, min, max);
    }
    Err(e) => eprintln!("Error: {}", e),
}
```

## GUI State Model

The GUI layer adds its own types for UI state:

```rust
// UI representation of a segment
pub struct SegmentUi {
    pub joint_type: JointType,
    pub theta: f64,
    pub d: f64,
    pub a: f64,
    pub alpha: f64,
}

// Full robot definition for the UI
pub struct RobotDef {
    pub name: String,
    pub segments: Vec<SegmentUi>,
}
```

`RobotDef::to_robot()` converts to a domain `Robot` for kinematics computation.

## References

- [DH Parameters](./dh-parameters.md) -- what the DHParams fields mean
- [Forward Kinematics](./forward-kinematics.md) -- how the model is solved
- [bombolab-core API](../api/core.md) -- full type reference
