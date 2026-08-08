//! Unit tests for `joint_tests`.

use super::*;

#[test]
fn test_joint_new() {
    let joint = Joint::new(JointType::Revolute, 0.5, 1.0, -1.0);
    assert_eq!(joint.joint_type, JointType::Revolute);
    assert_eq!(joint.value, 0.5);
    assert_eq!(joint.value_max, 1.0);
    assert_eq!(joint.value_min, -1.0);
}

#[test]
fn test_joint_is_within_limits() {
    let mut joint = Joint::new(JointType::Revolute, 0.5, 1.0, -1.0);
    assert!(joint.is_within_limits());

    joint.value = 1.5;
    assert!(!joint.is_within_limits());

    joint.value = -1.5;
    assert!(!joint.is_within_limits());
}

#[test]
fn test_joint_clamp() {
    let mut joint = Joint::new(JointType::Revolute, 2.0, 1.0, -1.0);
    joint.clamp();
    assert_eq!(joint.value, 1.0);

    joint.value = -2.0;
    joint.clamp();
    assert_eq!(joint.value, -1.0);
}

#[test]
fn test_joint_set_value() {
    let mut joint = Joint::new(JointType::Revolute, 0.0, 1.0, -1.0);
    assert!(joint.set_value(0.5).is_ok());
    assert_eq!(joint.value, 0.5);

    assert!(joint.set_value(1.5).is_err());
    assert!(joint.set_value(-1.5).is_err());
}

#[test]
fn test_joint_type_display() {
    assert_eq!(JointType::Revolute.to_string(), "R");
    assert_eq!(JointType::Prismatic.to_string(), "P");
    assert_eq!(JointType::Twist.to_string(), "T");
}
