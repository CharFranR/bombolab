//! Unit tests for `quaternion_tests`.

use super::*;

fn approx_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-10
}

fn quat_approx_eq(q1: &Quaternion, q2: &Quaternion) -> bool {
    approx_eq(q1.a, q2.a) && approx_eq(q1.b, q2.b) && approx_eq(q1.c, q2.c) && approx_eq(q1.d, q2.d)
}

#[test]
fn test_new() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    assert_eq!(q.a, 1.0);
    assert_eq!(q.b, 2.0);
    assert_eq!(q.c, 3.0);
    assert_eq!(q.d, 4.0);
}

#[test]
fn test_identity() {
    let q = Quaternion::identity();
    assert!(quat_approx_eq(&q, &Quaternion::new(1.0, 0.0, 0.0, 0.0)));
}

#[test]
fn test_zero() {
    let q = Quaternion::zero();
    assert!(quat_approx_eq(&q, &Quaternion::new(0.0, 0.0, 0.0, 0.0)));
}

#[test]
fn test_display() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    assert_eq!(q.to_string(), "(1 + 2i + 3j + 4k)");
}

#[test]
fn test_norm_sq() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    assert!(approx_eq(q.norm_sq(), 30.0));
}

#[test]
fn test_norm() {
    let q = Quaternion::new(0.0, 3.0, 4.0, 0.0);
    assert!(approx_eq(q.norm(), 5.0));
}

#[test]
fn test_normalize() {
    let q = Quaternion::new(0.0, 3.0, 4.0, 0.0);
    let n = q.normalize();
    assert!(approx_eq(n.norm(), 1.0));
    assert!(approx_eq(n.b, 0.6));
    assert!(approx_eq(n.c, 0.8));
}

#[test]
fn test_normalize_identity() {
    let q = Quaternion::identity();
    assert!(quat_approx_eq(&q.normalize(), &q));
}

#[test]
fn test_conjugate() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let c = q.conjugate();
    assert!(quat_approx_eq(&c, &Quaternion::new(1.0, -2.0, -3.0, -4.0)));
}

#[test]
fn test_inverse() {
    let q = Quaternion::new(1.0, 2.0, 0.0, 0.0);
    let inv = q.inverse();
    let product = solve_multiply(&[q.clone(), inv]);
    assert!(quat_approx_eq(&product, &Quaternion::identity()));
}

#[test]
fn test_add_two() {
    let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let q2 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
    let result = solve_add(&[q1, q2]);
    assert!(quat_approx_eq(
        &result,
        &Quaternion::new(6.0, 8.0, 10.0, 12.0)
    ));
}

#[test]
fn test_add_empty() {
    let result = solve_add(&[]);
    assert!(quat_approx_eq(&result, &Quaternion::zero()));
}

#[test]
fn test_subtract_two() {
    let q1 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
    let q2 = Quaternion::new(1.0, 2.0, 3.0, 4.0);

    let result = solve_subtract(&[q1, q2]);
    assert!(quat_approx_eq(
        &result,
        &Quaternion::new(-6.0, -8.0, -10.0, -12.0)
    ));
}

#[test]
fn test_multiply_identity() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let result = solve_multiply(&[q.clone(), Quaternion::identity()]);
    assert!(quat_approx_eq(&result, &q));
}

#[test]
fn test_multiply_two() {
    let q1 = Quaternion::new(1.0, 1.0, 0.0, 0.0);
    let q2 = Quaternion::new(1.0, 0.0, 0.0, 1.0);
    let result = solve_multiply(&[q1, q2]);

    assert!(approx_eq(result.a, 1.0));
    assert!(approx_eq(result.b, 1.0));
    assert!(approx_eq(result.c, -1.0));
    assert!(approx_eq(result.d, 1.0));
}

#[test]
fn test_divide_single() {
    let q = Quaternion::new(1.0, 2.0, 0.0, 0.0);

    let result = solve_divide(&[q.clone()]);
    assert!(quat_approx_eq(&result, &q.inverse()));
}

#[test]
fn test_divide_inverse() {
    let q = Quaternion::new(1.0, 2.0, 0.0, 0.0);
    let result = solve_divide(&[Quaternion::identity(), q.clone()]);
    let expected = q.inverse();
    assert!(quat_approx_eq(&result, &expected));
}

#[test]
fn test_add_multiple() {
    let q = Quaternion::new(1.0, 1.0, 1.0, 1.0);
    let result = solve_add(&[q.clone(), q.clone(), q.clone()]);
    assert!(quat_approx_eq(
        &result,
        &Quaternion::new(3.0, 3.0, 3.0, 3.0)
    ));
}

#[test]
fn test_multiply_associativity() {
    let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let q2 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
    let q3 = Quaternion::new(9.0, 10.0, 11.0, 12.0);
    let r1 = solve_multiply(&[solve_multiply(&[q1.clone(), q2.clone()]), q3.clone()]);
    let r2 = solve_multiply(&[q1, q2, q3]);
    assert!(quat_approx_eq(&r1, &r2));
}

#[test]
fn test_from_rotation_matrix_identity() {
    let r = Mat3::identity();
    let q = Quaternion::from_rotation_matrix(&r);
    assert!(quat_approx_eq(&q, &Quaternion::new(1.0, 0.0, 0.0, 0.0)));
}

#[test]
fn test_from_rotation_matrix_rotx_90() {
    let r = Mat3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
    let q = Quaternion::from_rotation_matrix(&r);

    let h = std::f64::consts::FRAC_1_SQRT_2;
    assert!((q.a - h).abs() < 1e-9);
    assert!((q.b + h).abs() < 1e-9);
    assert!(q.c.abs() < 1e-9);
    assert!(q.d.abs() < 1e-9);
    assert!((q.norm() - 1.0).abs() < 1e-9);
}

#[test]
fn test_dual_quaternion_pose_reconstruction() {
    let r = Mat3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
    let t = Vec3::new(140.0, -15.0, 205.0);
    let dq = DualQuaternion::from_pose(&r, &t);
    let t_rec = dq.translation();
    assert!((t_rec.x - 140.0).abs() < 1e-9);
    assert!((t_rec.y + 15.0).abs() < 1e-9);
    assert!((t_rec.z - 205.0).abs() < 1e-9);
    let h = std::f64::consts::FRAC_1_SQRT_2;
    assert!((dq.real.a - h).abs() < 1e-9);
    assert!((dq.real.b + h).abs() < 1e-9);
}
