# FABRI Creator

**The first concrete robot configuration in Bombolab — a 5-DOF educational robotic arm.**

The FABRI Creator is defined in `robot/fabri_creator.rs` and serves as the reference implementation for the robot model. It demonstrates how to configure a real robot with DH parameters, joint limits, servo offsets, and transforms.

## Hardware Overview

The FABRI Creator is a 5-DOF robotic arm built with:

- **Microcontroller:** Arduino Nano (ATmega328P)
- **Servos:** 5× SG90 micro servos (joints) + 1× SG90 (gripper)
- **Servo driver:** PCA9685 I2C servo driver (or direct PWM from Arduino)
- **Communication:** USB serial at 115200 baud
- **Firmware:** Custom Arduino sketch accepting 6 comma-separated angles per command

The robot is designed for educational use — teaching kinematics, DH parameters, and forward kinematics with real hardware.

## Quick Start

```rust
use bombolab_core::{fabri_creator, base_transform, tool_transform, forward_kinematics};

let robot = fabri_creator();
let base = base_transform();

// Forward kinematics at home pose (all q=0)
let (frames, end_effector) = forward_kinematics(base, &robot);

// Apply tool transform to get marker tip position
let tool = tool_transform() * end_effector;
let pos = tool.translation.vector;
println!("Marker at: ({:.1}, {:.1}, {:.1})", pos.x, pos.y, pos.z);
```

## DH Parameters

Standard DH convention, units: millimeters. These values are defined in `crates/bombolab-core/src/robot/fabri_creator.rs` and verified against the physical robot through the web simulator.

| i | α | a | d | θ | Type |
|---|-----|-----|-----|-----|------|
| 1 | -90° | 15 | 85 | q₁ | Revolute |
| 2 | 0° | 120 | 0 | q₂ − 90° | Revolute |
| 3 | -90° | 90 | 0 | q₃ + 90° | Revolute |
| 4 | 90° | 35 | 15 | — | **Twist** (α = q₄ + 90°) |
| 5 | 0° | 0 | 0 | q₅ | Revolute |

**Why these values?**

- **α₁ = -90°**: J1 rotates around Z (yaw). The -90° twist reorients the next Z axis from vertical to horizontal so J2 can pitch the arm up and down.
- **a₁ = 15 mm**: Horizontal offset from J1 rotation axis to J2 axis, measured from the center of the base servo horn.
- **d₁ = 85 mm**: Vertical offset from J1 to J2 along Z₁ (7 cm base plate + 15 mm hardware).
- **a₂ = 120 mm**: The "upper arm" link from shoulder to elbow. This is the main reaching segment. In home pose this link points upward (Z direction after the accumulated rotation).
- **a₃ = 90 mm**: The "forearm" link from elbow to wrist. Combined with a₄ this gives the robot its horizontal reach.
- **a₄ = 35 mm**: Short link from wrist roll to wrist pitch. The physical distance between the two wrist servo axes.
- **d₄ = 15 mm**: Lateral offset of the wrist assembly from the forearm axis.

**Joint types:**

| Joint | Type | Motion | Implementation |
|-------|------|--------|----------------|
| J1 | Revolute | Base yaw (Z₀) | Standard DH: `RotZ(q₁) · TransZ(85) · TransX(15) · RotX(-90°)` |
| J2 | Revolute | Shoulder pitch (Z₁) | Standard DH: `RotZ(q₂−90°) · TransZ(0) · TransX(120) · RotX(0)` |
| J3 | Revolute | Elbow pitch (Z₂) | Standard DH: `RotZ(q₃+90°) · TransZ(0) · TransX(90) · RotX(-90°)` |
| J4 | **Twist** | Wrist roll (X₃) | **`Iso3::from_parts((35,15,0), Rot_X(q₄+90°))`** |
| J5 | Revolute | Wrist pitch (Z₄) | Standard DH: `RotZ(q₅) · TransZ(0) · TransX(0) · RotX(0)` |

J4 uses `Twist` instead of `Revolute` because the wrist roll rotates about the forearm axis (X in DH convention). The Twist type rotates around the X axis of the previous frame, and the translation is applied as `(a, d, 0)` in the local frame without further rotation — this is the behavior of `Iso3::from_parts(translation, rotation)` in nalgebra. The joint variable q₄ adds to α₄ rather than θ₄.

## Physical Dimensions

```
         marcador
            ↑ 75mm (tool transform)
            |
         J5 ──── J4 ──── J3
                          |
                          | 90mm (a₃)
                          |
                         J2
                        /
                       / 120mm (a₂, vertical in home)
                      /
                     /
                    J1 ← 15mm offset (a₁)
                    |
               57mm base (base transform)
```

The robot stands 57mm above its mounting surface. The base servo (J1) rotates the entire arm. The shoulder (J2) and elbow (J3) provide the main reach. The wrist has two degrees of freedom: roll (J4) and pitch (J5).

| Parameter | Value | Source |
|-----------|-------|--------|
| Base height | 57 mm | Distance from mounting surface to J1 rotation axis |
| J1→J2 horizontal | 15 mm | Measured from base servo center to shoulder servo horn |
| J1→J2 vertical | 85 mm | Base plate (70 mm) + hardware offset (15 mm) |
| J2→J3 (upper arm) | 120 mm | Length of upper arm link (vertical in home pose) |
| J3→J4 (forearm) | 90 mm | Length of forearm link (horizontal in home pose) |
| J4→J5 (wrist) | 35 mm horizontal, 15 mm lateral | Distance between wrist servo axes |
| Tool (J5→marker) | 75 mm along X | Marker holder length; perpendicular to end effector |

## Joint Limits

| Space | Range | Notes |
|-------|-------|-------|
| Physical servo | 10°–170° | Mechanical safety limits (servos bind outside this range) |
| Kinematic (q) | `[10°−offset, 170°−offset]` | Per-joint, depends on servo offset |

The 10°–170° range is conservative — SG90 servos can technically do 0°–180°, but the mechanical linkage binds near the extremes. Staying within 10°–170° prevents gear damage.

```rust
use bombolab_core::fabri_creator;
use std::f64::consts::PI;

let robot = fabri_creator();

// Joint limits are in kinematic space
for seg in &robot.segments {
    println!("q ∈ [{:.2}°, {:.2}°]",
        seg.joint.value_min.to_degrees(),
        seg.joint.value_max.to_degrees());
}
```

## Home Pose

The home pose defines the servo angles when the robot is at its kinematic zero (`q=[0,0,0,0,0]`). These are the "resting" positions — where the robot sits when all kinematic angles are zero.

| Joint | Home (servo) | Notes |
|-------|-------------|-------|
| J1 | 90° | Base centered (servo midpoint) |
| J2 | 90° | Shoulder at kinematic zero |
| J3 | 81° | Elbow adjusted for physical alignment |
| J4 | 95° | Wrist roll centered |
| J5 | 60° | Wrist pitch with asymmetric limits |

```rust
let robot = fabri_creator();

// At kinematic zero, servos are at home pose
let servo = robot.q_to_servo(&vec![0.0; 5]);
assert_eq!(servo, robot.home_pose);

// The kinematic home is all zeros
let khome = robot.kinematic_home();
assert!(khome.iter().all(|q| q.abs() < 1e-10));
```

## Transforms

### Base Transform

Vertical offset from ground to joint 1:

```rust
use bombolab_core::base_transform;

let bt = base_transform();
// Translation: (0, 0, 57mm)
// Rotation: identity
```

### Tool Transform

Translation from J5 frame to marker tip. The marker is **perpendicular** to the end effector (extends along X, not Z):

```rust
use bombolab_core::tool_transform;

let tt = tool_transform();
// Translation: (75mm, 0, 0)
// Rotation: identity
```

**Why X and not Z?** The marker is mounted perpendicular to the last joint's rotation axis. If J5 rotates around Z, the marker extends along X (or Y) — not along Z. This is a common point of confusion: the tool transform direction depends on the physical mounting, not the joint axis.

**Usage:** Apply AFTER forward kinematics:

```rust
let (_, effector) = forward_kinematics(base, &robot);
let marker = tool_transform() * effector;
```

## Serial Protocol

The FABRI Creator communicates with an Arduino Nano over serial:

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Format | 6 comma-separated integers: `j1,j2,j3,j4,j5,gripper\n` |
| Response | `OK\n` or `ERR\n` |
| Angle range | 10–170 degrees |

The 6th value controls the gripper (not part of the 5-DOF kinematic model). The firmware parses the CSV, validates ranges, and drives the servos via PWM.

See [Serial Communication](./communication.md) for implementation details.

## Testing

The FABRI Creator includes comprehensive tests:

```bash
cargo test -p bombolab-core fabri_creator
```

Test coverage:
- DH table parameter verification
- Joint limits boundary values
- Home pose within physical limits
- Base/tool transform values
- q-to-servo round-trip conversion
- FK with base transform integration

## Adding a New Robot

To add a different robot configuration, create a new module under `robot/`:

```rust
// robot/my_robot.rs
use crate::robot::{DHParams, Joint, JointType, Robot, Segment};

pub fn my_robot() -> Robot {
    let dh_table = vec![
        // (alpha, a, d, theta_initial)
        // Measure YOUR robot's link lengths and twist angles
    ];

    let segments: Vec<Segment> = dh_table
        .into_iter()
        .map(|(alpha, a, d, _)| {
            let joint = Joint::new(JointType::Revolute, 0.0, q_max, q_min);
            let dh = DHParams::new(0.0, d, a, alpha);
            Segment::new(joint, dh)
        })
        .collect();

    Robot::new(segments) // or Robot::with_offsets() if you have servo offsets
}
```

**Steps:**

1. Measure your robot's link lengths (a, d) and twist angles (α) with calipers
2. Determine joint types (revolute vs prismatic)
3. Set joint limits based on mechanical constraints
4. If using servo offsets, define `home_pose` and `servo_offsets`
5. Write tests verifying your DH table matches the measurements
6. Add base/tool transforms if your mounting differs

The [Robot Model](./robot-model.md) page explains the data model in detail.
