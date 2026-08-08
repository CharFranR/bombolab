//! Unit tests for `interpolation_tests`.

use super::*;

#[test]
fn interpolate_joint_exact_multiple() {
    let steps = interpolate_joint(90, 100, 5);
    assert_eq!(steps, vec![95, 100]);
}

#[test]
fn interpolate_joint_non_exact_final_adjusts() {
    let steps = interpolate_joint(90, 102, 5);
    assert_eq!(steps, vec![95, 100, 102]);
}

#[test]
fn interpolate_joint_no_movement() {
    let steps = interpolate_joint(90, 90, 5);
    assert!(steps.is_empty());
}

#[test]
fn interpolate_joint_step_exceeds_distance() {
    let steps = interpolate_joint(90, 92, 5);
    assert_eq!(steps, vec![92]);
}

#[test]
fn interpolate_joint_descending() {
    let steps = interpolate_joint(100, 90, 5);
    assert_eq!(steps, vec![95, 90]);
}

#[test]
fn interpolate_joint_descending_non_exact() {
    let steps = interpolate_joint(100, 88, 5);
    assert_eq!(steps, vec![95, 90, 88]);
}

#[test]
fn interpolate_all_same_length_joints() {
    let current = [90, 90, 90, 90, 90, 90];
    let target = [95, 100, 85, 115, 90, 90];
    let config = InterpolationConfig {
        step_size: 5,
        delay_ms: 0,
    };
    let steps = interpolate_all(&current, &target, &config);

    assert_eq!(steps.len(), 5);
    assert_eq!(steps[0], [95, 95, 85, 95, 90, 90]);
    assert_eq!(steps[1], [95, 100, 85, 100, 90, 90]);
    assert_eq!(steps[2], [95, 100, 85, 105, 90, 90]);
    assert_eq!(steps[3], [95, 100, 85, 110, 90, 90]);
    assert_eq!(steps[4], [95, 100, 85, 115, 90, 90]);
}

#[test]
fn interpolate_all_shorter_joint_pads() {
    let current = [90, 90, 90, 90, 90, 90];

    let target = [100, 110, 90, 90, 90, 90];
    let config = InterpolationConfig {
        step_size: 5,
        delay_ms: 0,
    };
    let steps = interpolate_all(&current, &target, &config);

    assert_eq!(steps.len(), 4);

    assert_eq!(steps[0], [95, 95, 90, 90, 90, 90]);
    assert_eq!(steps[1], [100, 100, 90, 90, 90, 90]);
    assert_eq!(steps[2], [100, 105, 90, 90, 90, 90]);
    assert_eq!(steps[3], [100, 110, 90, 90, 90, 90]);
}

#[test]
fn interpolate_all_no_movement() {
    let current = [90; 6];
    let target = [90; 6];
    let config = InterpolationConfig::default();
    let steps = interpolate_all(&current, &target, &config);
    assert!(steps.is_empty());
}

#[test]
fn interpolate_all_single_step() {
    let current = [90; 6];
    let target = [92; 6];
    let config = InterpolationConfig {
        step_size: 5,
        delay_ms: 0,
    };
    let steps = interpolate_all(&current, &target, &config);
    assert_eq!(steps.len(), 1);
    assert_eq!(steps[0], [92; 6]);
}

#[test]
#[should_panic(expected = "step_size must be > 0")]
fn interpolate_joint_zero_step_panics() {
    interpolate_joint(90, 100, 0);
}

#[test]
#[should_panic(expected = "step_size must be > 0")]
fn interpolate_joint_negative_step_panics() {
    interpolate_joint(90, 100, -5);
}

#[test]
#[should_panic(expected = "step_size must be > 0")]
fn interpolate_all_zero_step_panics() {
    let config = InterpolationConfig {
        step_size: 0,
        delay_ms: 0,
    };
    interpolate_all(&[90; 6], &[100; 6], &config);
}

#[test]
fn interpolate_joint_extreme_values_no_overflow() {
    let steps = interpolate_joint(-170, i32::MAX, 1_000_000_000);
    assert!(!steps.is_empty());
    assert_eq!(*steps.last().unwrap(), i32::MAX);
    assert_eq!(steps.len(), 3);
    assert_eq!(steps[0], 999_999_830);
    assert_eq!(steps[1], 1_999_999_830);

    let steps = interpolate_joint(i32::MAX, i32::MIN, i32::MAX);
    assert_eq!(*steps.last().unwrap(), i32::MIN);
    assert_eq!(steps.len(), 3);
}
