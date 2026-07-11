# Bombolab

**Forward Kinematics Visualizer for Robotic Arms**

Bombolab computes where a robot arm is in 3D space given its joint angles. You describe a robot as a chain of joints and links using Denavit-Hartenberg (DH) parameters, Bombolab solves the math, and a desktop GUI shows the result.

## What It Does

- Models robots as serial chains of revolute or prismatic joints
- Computes forward kinematics using DH parameters (Craig convention)
- Provides an interactive desktop GUI for visualization
- Includes CLI tools for DH table solving and quaternion operations

## Who It's For

- Robotics students learning kinematics
- Engineers prototyping robot configurations
- Anyone who needs to visualize how DH parameters map to 3D positions

## How It Works

1. **Define** your robot as a chain of segments, each with a joint type and DH parameters
2. **Solve** forward kinematics to get the pose of every frame and the end-effector
3. **Visualize** the result in a GUI or inspect the matrices in the CLI

## Quick Example

```rust
use bombolab_core::{
    DHParams, Joint, JointType, Robot, Segment,
    Iso3, forward_kinematics,
};

// A simple 2-link planar arm
let segments = vec![
    Segment::new(
        Joint::new(JointType::Revolute, 0.5, 3.14, -3.14),
        DHParams::new(0.0, 0.0, 1.0, 0.0),
    ),
    Segment::new(
        Joint::new(JointType::Revolute, 0.3, 3.14, -3.14),
        DHParams::new(0.0, 0.0, 1.0, 0.0),
    ),
];

let robot = Robot::new(segments);
let (frames, end_effector) = forward_kinematics(Iso3::identity(), &robot);

let pos = end_effector.translation.vector;
println!("End-effector at: ({:.3}, {:.3}, {:.3})", pos.x, pos.y, pos.z);
```

## Project Structure

Bombolab is a Rust workspace with two crates:

| Crate | Purpose |
|-------|---------|
| `bombolab-core` | Core library: math, robot model, kinematics solvers |
| `bombolab-gui` | Desktop GUI built with egui/eframe |

## Next Steps

- [Installation](./getting-started/installation.md) -- set up the project
- [DH Parameters](./core-concepts/dh-parameters.md) -- understand the theory
- [Quick Start](./getting-started/quick-start.md) -- build your first robot
