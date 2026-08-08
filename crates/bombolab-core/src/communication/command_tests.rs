//! Unit tests for `command_tests`.

use super::*;

#[test]
fn test_valid_construction_mid_range() {
    let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90).unwrap();
    assert_eq!(cmd.joints[0], 90.0);
    assert_eq!(cmd.joints[1], 115.0);
    assert_eq!(cmd.joints[2], 110.0);
    assert_eq!(cmd.joints[3], 170.0);
    assert_eq!(cmd.joints[4], 90.0);
    assert_eq!(cmd.gripper, 90);
}

#[test]
fn test_valid_construction_at_boundaries() {
    let cmd = ServoCommand::new([5.0, 175.0, 5.0, 175.0, 90.0], 5).unwrap();
    assert_eq!(cmd.joints[0], 5.0);
    assert_eq!(cmd.joints[1], 175.0);
    assert_eq!(cmd.gripper, 5);
}

#[test]
fn test_joint_below_min_rejected() {
    let result = ServoCommand::new([4.9, 115.0, 110.0, 170.0, 90.0], 90);
    assert!(result.is_err());
}

#[test]
fn test_joint_above_max_rejected() {
    let result = ServoCommand::new([90.0, 115.0, 110.0, 175.1, 90.0], 90);
    assert!(result.is_err());
}

#[test]
fn test_joint_nan_rejected() {
    let result = ServoCommand::new([f64::NAN, 115.0, 110.0, 170.0, 90.0], 90);
    assert!(result.is_err());
}

#[test]
fn test_joint_positive_inf_rejected() {
    let result = ServoCommand::new([f64::INFINITY, 115.0, 110.0, 170.0, 90.0], 90);
    assert!(result.is_err());
}

#[test]
fn test_joint_negative_inf_rejected() {
    let result = ServoCommand::new([f64::NEG_INFINITY, 115.0, 110.0, 170.0, 90.0], 90);
    assert!(result.is_err());
}

#[test]
fn test_gripper_at_max_accepted() {
    let cmd = ServoCommand::new([90.0; 5], 175).unwrap();
    assert_eq!(cmd.gripper, 175);
}

#[test]
fn test_gripper_above_max_rejected() {
    let result = ServoCommand::new([90.0; 5], 176);
    assert!(result.is_err());
}

#[test]
fn test_gripper_below_min_rejected() {
    let result = ServoCommand::new([90.0; 5], 4);
    assert!(result.is_err());
}

#[test]
fn test_to_wire_matches_protocol() {
    let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90).unwrap();
    assert_eq!(cmd.to_wire(), "90,115,110,170,90,90\n");
}

#[test]
fn test_to_wire_rounds_floats() {
    let cmd = ServoCommand::new([90.4, 115.6, 110.0, 170.0, 89.5], 45).unwrap();
    assert_eq!(cmd.to_wire(), "90,116,110,170,90,45\n");
}

#[test]
fn test_to_raw_array_format() {
    let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90).unwrap();
    let raw = cmd.to_raw_array();
    assert_eq!(raw, [90, 115, 110, 170, 90, 90]);
}

#[test]
fn test_from_raw_array_roundtrip() {
    let raw = [90, 115, 110, 170, 90, 90];
    let cmd = ServoCommand::from_raw_array(&raw);
    assert_eq!(cmd.joints[0], 90.0);
    assert_eq!(cmd.joints[1], 115.0);
    assert_eq!(cmd.joints[2], 110.0);
    assert_eq!(cmd.joints[3], 170.0);
    assert_eq!(cmd.joints[4], 90.0);
    assert_eq!(cmd.gripper, 90);
}

#[test]
fn test_raw_array_roundtrip_identity() {
    let cmd = ServoCommand::new([45.0, 67.0, 89.0, 120.0, 33.0], 127).unwrap();
    let raw = cmd.to_raw_array();
    let back = ServoCommand::from_raw_array(&raw);
    assert_eq!(cmd, back);
}

#[test]
fn test_gripper_min_accepted() {
    let cmd = ServoCommand::new([90.0; 5], 10).unwrap();
    assert_eq!(cmd.gripper, 10);
}

#[test]
fn test_gripper_255_rejected() {
    let result = ServoCommand::new([90.0; 5], 255);
    assert!(result.is_err());
}
