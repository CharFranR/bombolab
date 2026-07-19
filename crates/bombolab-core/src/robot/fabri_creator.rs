use std::f64::consts::FRAC_PI_2;

use crate::math::Iso3;

use super::joint::{Joint, JointType};
use super::link::DHParams;
use super::segment::{Robot, Segment};

/// Create the FABRI Creator robot configuration.
///
/// Robot de 5 GDL con servos SG90, Arduino Nano y PCA9685.
/// Basado en la tabla de `docs/fabri-creator/table-definition.md`.
pub fn fabri_creator() -> Robot {
    let q_max = 80.0_f64.to_radians();
    let q_min = (-80.0_f64).to_radians();

    let segments = vec![
        // Joint 1 — Base (Yaw)
        // θ=0,  d=95,  a=15,  α=-π/2
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_max, q_min),
            DHParams::new(0.0, 95.0, 15.0, -FRAC_PI_2),
        ),
        // Joint 2 — Shoulder (eleva el brazo)
        // θ=-π/2,  d=0,  a=162,  α=0
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_max, q_min),
            DHParams::new(-FRAC_PI_2, 0.0, 162.0, 0.0),
        ),
        // Joint 3 — Elbow (extiende el antebrazo)
        // θ=+π/2,  d=0,  a=111,  α=-π/2
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_max, q_min),
            DHParams::new(FRAC_PI_2, 0.0, 111.0, -FRAC_PI_2),
        ),
        // Joint 4 — Wrist Roll
        // θ=0,  d=0,  a=35,  α=+π/2
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_max, q_min),
            DHParams::new(0.0, 0.0, 35.0, FRAC_PI_2),
        ),
        // Joint 5 — Wrist Pitch
        // θ=0,  d=0,  a=0,  α=0
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_max, q_min),
            DHParams::new(0.0, 0.0, 0.0, 0.0),
        ),
    ];

    // Home pose: servo = 90° en todos los joints en q = [0; 5]
    let home_pose = vec![90.0_f64.to_radians(); 5];

    // Offset = home_pose, así q_to_servo(&[0; 5]) = home_pose
    let servo_offsets = home_pose.clone();

    // Direcciones de giro por tabla:
    // J1: Anti Horario (-1), J2: Anti Horario (-1), J3: Horario (+1),
    // J4: Anti Horario (-1), J5: Anti Horario (-1)
    let servo_directions = vec![-1.0, -1.0, 1.0, -1.0, -1.0];

    Robot::with_directions(segments, home_pose, servo_offsets, servo_directions)
}

/// Base transform: vertical offset from ground to joint 1.
///
/// Translation: (0, 0, 57 mm), rotation: identity.
pub fn base_transform() -> Iso3 {
    Iso3::translation(0.0, 0.0, 57.0)
}

/// Tool transform: from J5 frame to marker tip.
///
/// The marker is mounted perpendicular to the last joint's rotation axis.
/// Translation: (75 mm, 0, 0), rotation: identity.
pub fn tool_transform() -> Iso3 {
    Iso3::translation(75.0, 0.0, 0.0)
}
