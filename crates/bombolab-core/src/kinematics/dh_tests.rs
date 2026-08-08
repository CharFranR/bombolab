//! Unit tests for `dh_tests`.

use super::*;

const PI: f64 = std::f64::consts::PI;
const FRAC_PI_2: f64 = std::f64::consts::FRAC_PI_2;
const EPS: f64 = 1e-10;

fn approx(a: f64, b: f64) -> bool {
    (a - b).abs() < EPS
}

fn matrix_approx(a: &Mat4, b: &Mat4) -> bool {
    a.iter().zip(b.iter()).all(|(x, y)| approx(*x, *y))
}

#[test]
fn rotation_z_90() {
    let m = compute_a_matrix(DHParameter::new(0.0, 0.0, 0.0, FRAC_PI_2));
    assert!(approx(m[(0, 0)], 0.0));
    assert!(approx(m[(0, 1)], -1.0));
    assert!(approx(m[(1, 0)], 1.0));
    assert!(approx(m[(1, 1)], 0.0));
}

#[test]
fn translation_x() {
    let m = compute_a_matrix(DHParameter::new(0.0, 5.0, 0.0, 0.0));
    assert!(approx(m[(0, 3)], 5.0));
    assert!(approx(m[(1, 3)], 0.0));
    assert!(approx(m[(2, 3)], 0.0));
}

#[test]
fn rotation_x_90() {
    let m = compute_a_matrix(DHParameter::new(FRAC_PI_2, 0.0, 0.0, 0.0));
    assert!(approx(m[(0, 0)], 1.0));
    assert!(approx(m[(1, 1)], 0.0));
    assert!(approx(m[(1, 2)], -1.0));
    assert!(approx(m[(2, 1)], 1.0));
    assert!(approx(m[(2, 2)], 0.0));
}

#[test]
fn identity_params() {
    let m = compute_a_matrix(DHParameter::new(0.0, 0.0, 0.0, 0.0));
    assert!(matrix_approx(&m, &Mat4::identity()));
}

#[test]
fn solve_empty_table() {
    let sol = solve(&[]);
    assert!(matrix_approx(&sol.final_transform, &Mat4::identity()));
    assert!(sol.a_matrices.is_empty());
    assert!(sol.intermediates.is_empty());
}

#[test]
fn solve_single_joint() {
    let table = vec![DHParameter::new(0.0, 1.0, 2.0, 0.0)];
    let sol = solve(&table);
    assert_eq!(sol.a_matrices.len(), 1);
    assert!(matrix_approx(&sol.intermediates[0], &sol.final_transform));
}

#[test]
fn solve_two_joints_planar_2r() {
    let table = vec![
        DHParameter::new(0.0, 1.0, 0.0, FRAC_PI_2),
        DHParameter::new(0.0, 1.0, 0.0, 0.0),
    ];
    let sol = solve(&table);
    let p = sol.translation();
    assert!(approx(p.x, 0.0));
    assert!(approx(p.y, 2.0));
    assert!(approx(p.z, 0.0));
}

#[test]
fn intermediates_are_cumulative() {
    let table = vec![
        DHParameter::new(0.0, 1.0, 0.0, 0.0),
        DHParameter::new(0.0, 1.0, 0.0, 0.0),
        DHParameter::new(0.0, 1.0, 0.0, 0.0),
    ];
    let sol = solve(&table);

    assert!(matrix_approx(&sol.intermediates[0], &sol.a_matrices[0]));

    let expected_12 = sol.a_matrices[0] * sol.a_matrices[1];
    assert!(matrix_approx(&sol.intermediates[1], &expected_12));

    let expected_123 = expected_12 * sol.a_matrices[2];
    assert!(matrix_approx(&sol.intermediates[2], &expected_123));
}

#[test]
fn display_doesnt_panic() {
    let table = vec![
        DHParameter::new(0.0, 1.0, 0.5, 0.0),
        DHParameter::new(PI, 0.5, 0.0, FRAC_PI_2),
    ];
    let sol = solve(&table);
    let output = format!("{}", sol);
    assert!(output.contains("TABLA DH"));
    assert!(output.contains("A1"));
    assert!(output.contains("FRAMES"));
    assert!(output.contains("POSE FINAL"));
}
