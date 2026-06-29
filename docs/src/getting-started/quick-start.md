# Quick Start

Build a robot, solve its kinematics, and see where the end-effector lands.

## 1. Define a Robot

A robot is a chain of `Segment`s. Each segment combines a `Joint` (the motor) with `DHParams` (the fixed geometry).

```rust
use bombolab_core::{
    DHParams, Joint, JointType, Robot, Segment,
    Isometry3, forward_kinematics,
};

// 2-link planar arm: both joints revolute, links of length 1.0
let segments = vec![
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, std::f64::consts::PI, -std::f64::consts::PI),
        DHParams::new(0.0, 0.0, 1.0, 0.0),
    ),
    Segment::new(
        Joint::new(JointType::Revolute, 0.0, std::f64::consts::PI, -std::f64::consts::PI),
        DHParams::new(0.0, 0.0, 1.0, 0.0),
    ),
];

let robot = Robot::new(segments);
```

## 2. Solve Forward Kinematics

```rust
let base = Isometry3::identity(); // robot at world origin
let (frames, end_effector) = forward_kinematics(base, &robot);
```

- `frames` -- one `Isometry3` per segment, showing the cumulative pose at each joint
- `end_effector` -- the final pose after all joints

## 3. Read the Result

```rust
let pos = end_effector.translation.vector;
println!("End-effector: ({:.3}, {:.3}, {:.3})", pos.x, pos.y, pos.z);
// With both angles at 0, the end-effector is at (2.0, 0.0, 0.0)

let rot = end_effector.rotation;
println!("Rotation: {:?}", rot);
```

## 4. Move a Joint

Change the joint value and re-solve:

```rust
use std::f64::consts::FRAC_PI_2;

// Rotate first joint to 90 degrees
robot.segments[0].joint.value = FRAC_PI_2;

let (_, end_effector) = forward_kinematics(Isometry3::identity(), &robot);
let pos = end_effector.translation.vector;
println!("After rotation: ({:.3}, {:.3}, {:.3})", pos.x, pos.y, pos.z);
// Now the arm points upward
```

## 5. Use the CLI

For quick DH table solving without writing code:

```bash
cargo run --bin dh-solve
```

The interactive solver lets you:
1. Choose degrees or radians
2. Enter the number of links
3. Input alpha, a, d, theta for each link
4. See the full solution: A matrices, intermediate frames, and final pose

## Next Steps

- [DH Parameters](../core-concepts/dh-parameters.md) -- understand what each parameter means
- [Robot Model](../core-concepts/robot-model.md) -- learn about joints, segments, and the robot struct
- [dh-solve CLI](../cli/dh-solve.md) -- detailed CLI reference
