//! Unit tests for `ws_bridge_tests`.

use super::*;

fn assert_deg(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "esperado {expected}°, obtenido {actual}°"
    );
}

#[test]
fn test_build_servo_command_home_pose_in_degrees() {
    // En home (q = [0; 5]) los offsets son [90, 90, 81, 95, 60] grados,
    // todos dentro de [10, 170] → el comando debe ser válido.
    let robot = fabri_creator();
    let cmd = build_servo_command(&robot, &[0.0; 5], 90).unwrap();
    assert_deg(cmd.joints[0], 90.0);
    assert_deg(cmd.joints[1], 90.0);
    assert_deg(cmd.joints[2], 81.0);
    assert_deg(cmd.joints[3], 95.0);
    assert_deg(cmd.joints[4], 60.0);
    assert_eq!(cmd.gripper, 90);
}

#[test]
fn test_build_servo_command_rejects_out_of_range_degrees() {
    // q que mapea a un servo fuera de [10, 170] grados → error.
    let robot = fabri_creator();
    let result = build_servo_command(&robot, &[2.5; 5], 90);
    assert!(result.is_err());
}

#[test]
fn test_build_servo_command_rejects_wrong_length() {
    let robot = fabri_creator();
    assert!(build_servo_command(&robot, &[], 90).is_err());
    assert!(build_servo_command(&robot, &[0.0; 4], 90).is_err());
    assert!(build_servo_command(&robot, &[0.0; 6], 90).is_err());
}

#[test]
fn test_build_servo_command_rejects_non_finite() {
    let robot = fabri_creator();
    assert!(build_servo_command(&robot, &[f64::NAN, 0.0, 0.0, 0.0, 0.0], 90).is_err());
    assert!(build_servo_command(&robot, &[f64::INFINITY, 0.0, 0.0, 0.0, 0.0], 90).is_err());
    assert!(build_servo_command(&robot, &[f64::NEG_INFINITY, 0.0, 0.0, 0.0, 0.0], 90).is_err());
}

#[test]
fn test_origin_allowed_empty_list_accepts_all() {
    let allowlist: Vec<String> = vec![];
    assert!(origin_allowed(Some("http://evil.example"), &allowlist));
    assert!(origin_allowed(None, &allowlist));
}

#[test]
fn test_origin_allowed_allowlist() {
    let allowlist = vec!["http://localhost:5173".to_string()];
    assert!(origin_allowed(None, &allowlist));
    assert!(origin_allowed(Some("http://localhost:5173"), &allowlist));
    assert!(!origin_allowed(Some("http://evil.example"), &allowlist));
    assert!(!origin_allowed(Some("http://localhost:5174"), &allowlist));
}
