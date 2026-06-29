# Forward Kinematics with DH Parameters

## Overview

This document explains how forward kinematics works in Bombolab:
how you describe a robot, how each transformation matrix is built, and how they
are composed to obtain the position of each link and the end-effector.

---

## 1. How You Describe a Robot

A robot is a chain of **Segments**. Each segment has two parts:

```
Segment {
    joint: Joint,    // the motor: type, rotation, limits
    dh: DHParams,    // fixed geometry between this joint and the next
}
```

### Joint

```rust
pub struct Joint {
    pub joint_type: JointType,  // Revolute | Prismatic
    pub value: f64,             // CURRENT joint value (radians)
    pub value_max: f64,
    pub value_min: f64,
}
```

- **Revolute**: the joint **rotates** around its Z axis. `value` is an angle (theta).
- **Prismatic**: the joint **slides** along its Z axis. `value` is a distance (d).

Movement is always along Z. If you need movement in another direction, you
reorient Z with `alpha` (explained below).

### DHParams

```rust
pub struct DHParams {
    pub theta: f64,  // angle around Z (radians)
    pub d: f64,      // translation along Z
    pub a: f64,      // translation along X -- "link length"
    pub alpha: f64,  // twist around X -- angle between Zs (radians)
}
```

These are the 4 Denavit-Hartenberg parameters. They define the **fixed geometry**
of the link. The only value that changes dynamically is the joint value:

| JointType  | theta comes from | d comes from |
|------------|-----------------|--------------|
| Revolute   | joint.value     | dh.d (fixed) |
| Prismatic  | dh.theta (fixed)| joint.value  |

This is resolved automatically by `segment.dh_params()`.

---

## 2. Parameter by Parameter

### theta -- rotation around Z (radians)

This is the joint angle if Revolute. **Always around the local frame's Z axis.**

theta = 0 -- the link points in pure X direction.
theta = 90 degrees -- the link points in pure Y direction.

### d -- translation along Z

Separates the current frame from the previous one along the Z axis. It represents:

- Base height (d = 5 -- the base is 5 units tall)
- Vertical offset between joints
- Prismatic displacement if the joint is Prismatic

**d does not generate movement in X or Y.** It is purely Z.

### a -- translation along X ("link length")

The distance from the current Z axis to the next Z axis, measured along X.
This is the link length. When theta changes, this length is projected into X and Y:

```
a*cos(theta)  -- X component
a*sin(theta)  -- Y component
```

With theta = 0, `a` goes entirely in X. With theta = 90 degrees, `a` goes entirely in Y.

### alpha -- twist between Zs (radians)

**The key to DH.** It is the rotation angle around X that twists the current Z
to align with the next joint's Z.

- alpha = 0 -- Zs are parallel (the arm stays in the same plane)
- alpha = 90 degrees (pi/2) -- the next Z is perpendicular to the current one (an elbow that changes plane)

**Without alpha, all joints of your robot rotate in the same 2D plane.**
With alpha, you can model real 3D robots.

---

## 3. The Trick with Isometry3 in `matrix_from_segment`

The function that builds each segment's matrix:

```rust
pub fn matrix_from_segment(segment: &Segment) -> Isometry3<f64> {
    let (theta, d, a, alpha) = segment.dh_params();

    // Final rotation: RotZ(theta) * RotX(alpha)
    let rot_z = Rotation3::from_axis_angle(&Vector3::z_axis(), theta);
    let rot_x = Rotation3::from_axis_angle(&Vector3::x_axis(), alpha);
    let rotation = UnitQuaternion::from_rotation_matrix(&(rot_z * rot_x));

    // Translation: a*cos(theta) in X, a*sin(theta) in Y, d in Z
    let translation = Translation3::new(a * theta.cos(), a * theta.sin(), d);

    Isometry3::from_parts(translation, rotation)
}
```

### Why UnitQuaternion?

`Isometry3<f64>` stores rotation internally as `UnitQuaternion<f64>`,
not as `Rotation3<f64>`. That's why we need to convert with
`UnitQuaternion::from_rotation_matrix()`.

You can also write `rot.into()` if you have a `Rotation3` -- the conversion
is automatic:

```rust
let r: Rotation3<f64> = rot_z * rot_x;
Isometry3::from_parts(translation, r.into())
```

### Why rot_z * rot_x?

The canonical DH formula order is:

```
T = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
```

1. RotZ(theta) -- rotates the frame
2. TransZ(d) -- moves up/down along Z
3. TransX(a) -- advances along X (link length)
4. RotX(alpha) -- twists Z for the next frame

RotZ and RotX compose as `rot_z * rot_x` because in the DH formula,
RotX(alpha) is post-multiplied: first RotZ(theta) is applied, then RotX(alpha)
to the output frame.

**If alpha = 0**, RotX(alpha) is identity and all rotation reduces to RotZ(theta).

### The translation is not a free vector

Note that `a*cos(theta)` and `a*sin(theta)` appear in X and Y -- this is the
**link length projected after rotation**. It's not "movement in X", it's the
result of rotating a link of length `a`.

d goes directly to Z because RotZ(theta) doesn't affect Z. RotX(alpha) could
affect it, but in the DH formula the translation is applied BEFORE RotX(alpha).

---

## 4. Complete Forward Kinematics

```rust
pub fn forward_kinematics(
    base: Isometry3<f64>,   // where the world is relative to the robot
    robot: &Robot,
) -> (Vec<Isometry3<f64>>, Isometry3<f64>) {
    let mut frames = Vec::new();
    let mut current = base;

    for segment in &robot.segments {
        current = current * matrix_from_segment(segment);
        frames.push(current);
    }

    (frames, current)
}
```

- **frames**: `Vec<Isometry3<f64>>` -- the pose of EACH link. `frames[i]` is
  the position of frame i+1 after applying joints 0..=i.
- **current** (end-effector): the pose of the last frame, after all joints.

### What about the base?

`base` is an external `Isometry3<f64>` -- it's not tied to the robot. You can pass
`Isometry3::identity()` if the robot is at the origin, or any transformation
if it's displaced/rotated in the world. The same robot can be used in different
positions without modifying it.

---

## 5. How to Describe Your Robot to the System

### 2D Robot (planar arm, SCARA)

```
Segment 1 (base):    Revolute | joint + d (height)
Segment 2 (arm):     Revolute | a (length)
Segment 3 (forearm): Revolute | a (length)
alpha = 0 always      -> arm moves in XY plane

Example:
  S1: d=5, a=0    -> base of height 5
  S2: d=0, a=3    -> forearm of length 3
  S3: d=0, a=3    -> arm of length 3
```

### 3D Robot (with plane change)

```
Segment 1 (base):    Revolute | d + alpha=0
Segment 2 (shoulder): Revolute | a + alpha=pi/2 (twists the plane)
Segment 3 (elbow):   Revolute | a + alpha=0    (follows the new plane)
```

alpha = pi/2 at the shoulder makes the arm move vertically instead of horizontally.

### Quick Reference

| Parameter | Represents | Unit |
|-----------|-----------|------|
| theta | joint rotation around Z | radians |
| d | translation along Z | mm / m |
| a | link length (along X) | mm / m |
| alpha | twist between Zs | radians |

### What Each Parameter Is NOT

- theta is **not** the link length.
- d is **not** horizontal movement.
- a is **not** the segment height.
- alpha is **not** the joint angle.

---

## 6. Multiplication Order: Local vs Global

Forward kinematics composes like this:

```
current = current * T_i
```

Each `T_i` is the transformation **from frame i-1 to frame i**, expressed in
frame i-1. Post-multiplying means each step is relative to the previous frame --
which is exactly how a kinematic chain works.

If you change to `current = T_i * current`, you would be applying each movement
in the world's global frame. That's useful for moving a free rigid body in space,
**not** for an articulated robot.
