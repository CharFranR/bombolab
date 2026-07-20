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

Craig convention, units: millimeters. These values come from **physical measurements** of the actual FABRI Creator robot — link lengths, joint offsets, and twist angles measured with calipers and protractors.

| i | α | a | d | θ | Type | Source |
|---|-----|-----|-----|-----|------|--------|
| 1 | -90° | 15 | 95 | θ₁ | Revolute | 15mm horizontal offset from J1 axis to J2 axis; 95mm from J1 to J2 along Z |
| 2 | 0° | 0 | 162 | θ₂ | Revolute | 162mm vertical link (J2→J3); no horizontal offset (pure vertical) |
| 3 | -90° | 111 | 0 | θ₃ | Revolute | 111mm horizontal link (J3→J4); twist -90° reorients rotation plane |
| 4 | 90° | 35 | 0 | — | **Twist** | 35mm horizontal link (J4→J5); joint value adds to α, rotates about X |
| 5 | 0° | 0 | 0 | θ₅ | Revolute | Zero-length final joint (wrist pitch); tool transform handles the rest |

**Why these values?**

- **α₁ = -90°**: J1 rotates around Z (yaw), but J2 rotates around a horizontal axis. The -90° twist reorients from vertical to horizontal.
- **a₁ = 15mm**: The physical offset between J1's rotation axis and J2's rotation axis. Measured from the center of the base servo horn to the shoulder servo horn.
- **d₂ = 162mm**: The longest link — the "upper arm" connecting shoulder to elbow. This is the main vertical segment.
- **a₃ = 111mm**: The "forearm" link from elbow to wrist. Combined with a₄, gives the robot its reach.
- **a₄ = 35mm**: Short link from wrist roll to wrist pitch. The 35mm offset is the physical distance between the two wrist servo axes.

**Joint types:**

| Joint | Type | Motion | DH Formula |
|-------|------|--------|------------|
| J1 | Revolute | Base yaw | `RotZ(θ₁+q) · TransZ(d₁) · TransX(a₁) · RotX(α₁)` |
| J2 | Revolute | Shoulder pitch | `RotZ(θ₂+q) · TransZ(d₂) · TransX(a₂) · RotX(α₂)` |
| J3 | Revolute | Elbow pitch | `RotZ(θ₃+q) · TransZ(d₃) · TransX(a₃) · RotX(α₃)` |
| J4 | **Twist** | Wrist roll (X-axis) | **`RotX(α₄+q) · TransX(a₄)`** |
| J5 | Revolute | Wrist pitch | `RotZ(θ₅+q) · TransZ(d₅) · TransX(a₅) · RotX(α₅)` |

J4 uses `Twist` instead of `Revolute` because the wrist roll rotates about the forearm axis (X in DH convention). The Twist type swaps the rotation axis from Z to X, and the joint value adds directly to `alpha` instead of `theta`.

## Physical Dimensions

```
        marcador
           ↑ 75mm (tool transform)
           |
        J5 ──── J4 ──── J3
                         |
                         | 111mm (a₃)
                         |
                        J2
                       /
                      / 162mm (d₂)
                     /
                    /
                   J1 ← 15mm offset (a₁)
                   |
              57mm base (base transform)
```

The robot stands 57mm above its mounting surface. The base servo (J1) rotates the entire arm. The shoulder (J2) and elbow (J3) provide the main reach. The wrist has two degrees of freedom: roll (J4) and pitch (J5).

| Parameter | Value | Measurement Source |
|-----------|-------|-------------------|
| Base height | 57mm | Distance from mounting surface to J1 rotation axis |
| J1→J2 | 15mm horizontal, 95mm vertical | Caliper measurement of base-to-shoulder offset |
| J2→J3 | 162mm vertical | Length of upper arm link |
| J3→J4 | 111mm horizontal | Length of forearm link |
| J4→J5 | 35mm horizontal | Distance between wrist servo axes |
| Tool (J5→marker) | 75mm along X | Marker holder length; perpendicular to end effector |

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
| J2 | 115° | Shoulder slightly raised (not vertical) |
| J3 | 110° | Elbow bent (arm folded back slightly) |
| J4 | 170° | Wrist rolled (corrected from Arduino default of 175°) |
| J5 | 90° | Wrist pitch centered (servo midpoint) |

**Why J4 = 170° instead of 175°?** The Arduino firmware originally defaulted J4 to 175°, but this exceeds `JOINT_MAX` (170°). The value was corrected to 170° to stay within mechanical safety limits. If your physical robot has J4 at 175°, the servo may be slightly misaligned — check the mechanical stop.

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
