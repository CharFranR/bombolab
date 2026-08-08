use std::f64::consts::FRAC_PI_2;

use crate::math::Iso3;

use super::joint::{Joint, JointType};
use super::link::DHParams;
use super::segment::{Robot, Segment};
use super::tool_frame::ToolFrame;

pub fn fabri_creator() -> Robot {
    let q_j1_j2 = 85.0_f64.to_radians();
    let q_j3_max = 85.0_f64.to_radians();
    let q_j3_min = (-76.0_f64).to_radians();
    let q_j4_max = 85.0_f64.to_radians();
    let q_j4_min = (-80.0_f64).to_radians();

    let segments = vec![
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_j1_j2, -q_j1_j2),
            DHParams::new(0.0, 85.0, 15.0, -FRAC_PI_2),
        ),
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_j1_j2, -q_j1_j2),
            DHParams::new(-FRAC_PI_2, 0.0, 120.0, 0.0),
        ),
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_j3_max, q_j3_min),
            DHParams::new(FRAC_PI_2, 0.0, 90.0, -FRAC_PI_2),
        ),
        Segment::new(
            Joint::new(JointType::Twist, 0.0, q_j4_max, q_j4_min),
            DHParams::new(0.0, 15.0, 35.0, FRAC_PI_2),
        ),
        Segment::new(
            Joint::new(
                JointType::Revolute,
                0.0,
                55.0_f64.to_radians(),
                (-115.0_f64).to_radians(),
            ),
            DHParams::new(0.0, 0.0, 0.0, 0.0),
        ),
    ];

    let home_pose = vec![
        90.0_f64.to_radians(),
        90.0_f64.to_radians(),
        81.0_f64.to_radians(),
        95.0_f64.to_radians(),
        60.0_f64.to_radians(),
    ];

    let servo_offsets = home_pose.clone();

    let servo_directions = vec![-1.0, -1.0, 1.0, -1.0, -1.0];

    Robot::with_directions(segments, home_pose, servo_offsets, servo_directions)
}

pub fn base_transform() -> Iso3 {
    Iso3::translation(0.0, 0.0, 57.0)
}

/// Legacy marker tool pose: 75 mm along X with identity rotation.
///
/// Deprecated in favor of [`ToolFrame::marker_perpendicular`].
#[deprecated(since = "0.2.0", note = "use ToolFrame::marker_perpendicular().pose()")]
pub fn tool_transform() -> Iso3 {
    *ToolFrame::marker_perpendicular().pose()
}
