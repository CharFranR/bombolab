//! Unit tests for `forward_tests`.

use super::*;
use crate::robot::{DHParams, Joint, JointType, Robot, Segment};

fn make_segment(joint_type: JointType, value: f64, dh: DHParams) -> Segment {
    let joint = Joint::new(
        joint_type,
        value,
        std::f64::consts::PI,
        -std::f64::consts::PI,
    );
    Segment::new(joint, dh)
}

#[test]
fn test_matrix_from_segment_identity() {
    let dh = DHParams::new(0.0, 0.0, 0.0, 0.0);
    let seg = make_segment(JointType::Revolute, 0.0, dh);
    let m = matrix_from_segment(&seg);

    let t = m.translation.vector;
    assert!((t.x - 0.0).abs() < 1e-10);
    assert!((t.y - 0.0).abs() < 1e-10);
    assert!((t.z - 0.0).abs() < 1e-10);
}

#[test]
fn test_matrix_from_segment_translation() {
    let dh = DHParams::new(0.0, 5.0, 3.0, 0.0);
    let seg = make_segment(JointType::Revolute, 0.0, dh);
    let m = matrix_from_segment(&seg);

    let t = m.translation.vector;
    assert!((t.x - 3.0).abs() < 1e-10);
    assert!((t.y - 0.0).abs() < 1e-10);
    assert!((t.z - 5.0).abs() < 1e-10);
}

#[test]
fn test_matrix_from_segment_rotation() {
    let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
    let seg = make_segment(JointType::Revolute, std::f64::consts::FRAC_PI_2, dh);
    let m = matrix_from_segment(&seg);

    let t = m.translation.vector;
    assert!((t.x - 0.0).abs() < 1e-10);
    assert!((t.y - 1.0).abs() < 1e-10);
    assert!((t.z - 0.0).abs() < 1e-10);
}

#[test]
fn test_forward_kinematics_single_segment() {
    let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
    let segments = vec![make_segment(JointType::Revolute, 0.0, dh)];
    let robot = Robot::new(segments);

    let base = Iso3::identity();
    let (frames, effector) = forward_kinematics(base, &robot);

    assert_eq!(frames.len(), 1);

    let t = effector.translation.vector;
    assert!((t.x - 1.0).abs() < 1e-10);
    assert!((t.y - 0.0).abs() < 1e-10);
    assert!((t.z - 0.0).abs() < 1e-10);
}

#[test]
fn test_forward_kinematics_two_segments() {
    let dh1 = DHParams::new(0.0, 0.0, 1.0, 0.0);
    let dh2 = DHParams::new(0.0, 0.0, 1.0, 0.0);
    let segments = vec![
        make_segment(JointType::Revolute, 0.0, dh1),
        make_segment(JointType::Revolute, 0.0, dh2),
    ];
    let robot = Robot::new(segments);

    let base = Iso3::identity();
    let (frames, effector) = forward_kinematics(base, &robot);

    assert_eq!(frames.len(), 2);

    let t = effector.translation.vector;
    assert!((t.x - 2.0).abs() < 1e-10);
    assert!((t.y - 0.0).abs() < 1e-10);
    assert!((t.z - 0.0).abs() < 1e-10);
}

#[test]
fn test_forward_kinematics_with_joint_angle() {
    let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
    let segments = vec![make_segment(
        JointType::Revolute,
        std::f64::consts::FRAC_PI_2,
        dh,
    )];
    let robot = Robot::new(segments);

    let base = Iso3::identity();
    let (_frames, effector) = forward_kinematics(base, &robot);

    let t = effector.translation.vector;
    assert!((t.x - 0.0).abs() < 1e-10);
    assert!((t.y - 1.0).abs() < 1e-10);
    assert!((t.z - 0.0).abs() < 1e-10);
}
