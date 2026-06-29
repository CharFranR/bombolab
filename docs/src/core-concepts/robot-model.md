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
}
```

Methods:

| Method | Description |
|--------|-------------|
| `Robot::new(segments)` | Create a robot from a segment list |
| `robot.dof()` | Number of degrees of freedom (segment count) |
| `robot.segment(i)` | Get segment by index (returns `Result`) |
| `robot.segment_mut(i)` | Get mutable segment by index |
| `robot.set_joint_values(joints)` | Update all joint values at once (validates limits) |
| `robot.reset_to_zero()` | Set all joint values to 0.0 |
| `robot.add_segment(segment)` | Append a segment |
| `robot.remove_segment(i)` | Remove and return a segment by index |
| `robot.is_empty()` | True if no segments |

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
