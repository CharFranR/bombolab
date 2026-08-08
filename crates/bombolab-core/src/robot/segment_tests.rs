//! Unit tests for `segment_tests`.

use super::*;

fn make_test_segment(joint_type: JointType, value: f64) -> Segment {
    let joint = Joint::new(joint_type, value, 1.0, -1.0);
    let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
    Segment::new(joint, dh)
}

#[test]
fn test_segment_new() {
    let seg = make_test_segment(JointType::Revolute, 0.5);
    assert_eq!(seg.joint.joint_type, JointType::Revolute);
    assert_eq!(seg.joint.value, 0.5);
    assert_eq!(seg.dh.a, 1.0);
}

#[test]
fn test_segment_dh_params_revolute() {
    let seg = make_test_segment(JointType::Revolute, 0.5);
    let (theta, d, a, alpha) = seg.dh_params();
    assert_eq!(theta, 0.5);
    assert_eq!(d, 0.0);
    assert_eq!(a, 1.0);
    assert_eq!(alpha, 0.0);
}

#[test]
fn test_segment_dh_params_prismatic() {
    let seg = make_test_segment(JointType::Prismatic, 0.5);
    let (theta, d, a, alpha) = seg.dh_params();
    assert_eq!(theta, 0.0);
    assert_eq!(d, 0.5);
    assert_eq!(a, 1.0);
    assert_eq!(alpha, 0.0);
}

#[test]
fn test_robot_new() {
    let segments = vec![make_test_segment(JointType::Revolute, 0.0)];
    let robot = Robot::new(segments);
    assert_eq!(robot.dof(), 1);
    assert!(!robot.is_empty());
}

#[test]
fn test_robot_empty() {
    let robot = Robot::new(vec![]);
    assert_eq!(robot.dof(), 0);
    assert!(robot.is_empty());
}

#[test]
fn test_robot_segment() {
    let segments = vec![
        make_test_segment(JointType::Revolute, 0.0),
        make_test_segment(JointType::Revolute, 0.5),
    ];
    let robot = Robot::new(segments);

    assert!(robot.segment(0).is_ok());
    assert!(robot.segment(1).is_ok());
    assert!(robot.segment(2).is_err());
}

#[test]
fn test_robot_add_remove_segment() {
    let mut robot = Robot::new(vec![]);
    assert!(robot.is_empty());

    robot.add_segment(make_test_segment(JointType::Revolute, 0.0));
    assert_eq!(robot.dof(), 1);

    let removed = robot.remove_segment(0);
    assert!(removed.is_ok());
    assert!(robot.is_empty());
}

#[test]
fn test_robot_remove_segment_out_of_bounds() {
    let mut robot = Robot::new(vec![]);
    assert!(robot.remove_segment(0).is_err());
}

#[test]
fn test_robot_reset_to_zero() {
    let segments = vec![make_test_segment(JointType::Revolute, 0.5)];
    let mut robot = Robot::new(segments);
    robot.reset_to_zero();
    assert_eq!(robot.segments[0].joint.value, 0.0);
}
