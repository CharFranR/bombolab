//! Unit tests for `ik_orientation_tests`.

use super::*;
use crate::math::Rot3;
use crate::robot::fabri_creator;

fn get_rot3(iso: &Iso3) -> Rot3 {
    iso.rotation.to_rotation_matrix()
}

const TOL: f64 = 1e-10;

fn make_robot() -> Robot {
    fabri_creator()
}

fn make_base() -> Iso3 {
    crate::robot::base_transform()
}

fn q_to_robot(robot: &Robot, q: &[f64; 5]) -> Robot {
    build_robot(robot, q)
}

#[test]
fn test_home_pose_extracts_zero() {
    let robot = make_robot();
    let base = make_base();
    let solver = OrientationSolver::new(1e-6);

    let q = [0.0; 5];
    let robot_q = q_to_robot(&robot, &q);
    let (frames, effector) = forward_kinematics(base, &robot_q);
    let r03 = get_rot3(&frames[2]);
    let r_target = get_rot3(&effector);

    let result = solver.solve(&r03, &r_target, &robot);
    assert!(result.is_ok(), "home debería ser alcanzable");

    let [q4, q5] = result.unwrap();
    assert!(q4.abs() < TOL, "q4 en home debería ser 0, got {:.2e}", q4);
    assert!(q5.abs() < TOL, "q5 en home debería ser 0, got {:.2e}", q5);
}

#[test]
fn test_random_configurations_reconstructed() {
    let robot = make_robot();
    let base = make_base();
    let solver = OrientationSolver::new(1e-6);

    let mut seed: u64 = 42;
    let n_samples = 100;

    let mut max_q4_err: f64 = 0.0;
    let mut max_q5_err: f64 = 0.0;
    let mut max_reconstruction_err: f64 = 0.0;

    for _ in 0..n_samples {
        let q: [f64; 5] = std::array::from_fn(|i| {
            let lo = robot.segments[i].joint.value_min.max(-2.0);
            let hi = robot.segments[i].joint.value_max.min(2.0);

            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let r = (seed as f64) / (u64::MAX as f64);
            lo + r * (hi - lo)
        });

        let robot_q = q_to_robot(&robot, &q);
        let (frames, effector) = forward_kinematics(base, &robot_q);
        let r03 = get_rot3(&frames[2]);
        let r_target = get_rot3(&effector);

        let result = solver.solve(&r03, &r_target, &robot);
        assert!(
            result.is_ok(),
            "q = [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}] debería ser alcanzable",
            q[0],
            q[1],
            q[2],
            q[3],
            q[4]
        );

        let [q4, q5] = result.unwrap();

        let q4_err = (q4 - q[3]).abs() % (2.0 * std::f64::consts::PI);
        let q4_err = q4_err.min(2.0 * std::f64::consts::PI - q4_err);
        let q5_err = (q5 - q[4]).abs() % (2.0 * std::f64::consts::PI);
        let q5_err = q5_err.min(2.0 * std::f64::consts::PI - q5_err);

        max_q4_err = max_q4_err.max(q4_err);
        max_q5_err = max_q5_err.max(q5_err);

        assert!(
            q4_err < 1e-12,
            "error q4 = {:.2e} para original={:.6}, extraído={:.6}",
            q4_err,
            q[3],
            q4
        );
        assert!(
            q5_err < 1e-12,
            "error q5 = {:.2e} para original={:.6}, extraído={:.6}",
            q5_err,
            q[4],
            q5
        );

        let (s4, c4) = q4.sin_cos();
        let (s5, c5) = q5.sin_cos();
        let r35_reconstructed = Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
            c5,
            -s5,
            0.0,
            -s4 * s5,
            -s4 * c5,
            -c4,
            c4 * s5,
            c4 * c5,
            -s4,
        ));
        let r_reconstructed = r03 * r35_reconstructed;
        let diff = (r_reconstructed.matrix() - r_target.matrix()).norm();
        max_reconstruction_err = max_reconstruction_err.max(diff);

        assert!(diff < 1e-12, "error reconstrucción = {:.2e}", diff);
    }

    eprintln!("=== OrientationSolver: random configs ===");
    eprintln!("Muestras: {n_samples}");
    eprintln!("Máx error q4: {:.2e} rad", max_q4_err);
    eprintln!("Máx error q5: {:.2e} rad", max_q5_err);
    eprintln!("Máx error reconstrucción: {:.2e}", max_reconstruction_err);
}

#[test]
fn test_unreachable_orientation_detected() {
    let robot = make_robot();
    let base = make_base();

    let solver = OrientationSolver::new(1e-10);

    let mut seed: u64 = 1234;
    let n_samples = 50;

    for _ in 0..n_samples {
        let mut q: [f64; 5] = [0.0; 5];
        for i in 0..3 {
            let lo = robot.segments[i].joint.value_min.max(-2.0);
            let hi = robot.segments[i].joint.value_max.min(2.0);
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let r = (seed as f64) / (u64::MAX as f64);
            q[i] = lo + r * (hi - lo);
        }

        let robot_q = q_to_robot(&robot, &q);
        let (frames, _effector) = forward_kinematics(base, &robot_q);
        let r03 = get_rot3(&frames[2]);

        let r_y = Rot3::from_axis_angle(&nalgebra::Unit::new_normalize(Vec3::y()), 0.2);

        let r_target_perturbed = r03 * r_y;

        let result = solver.solve(&r03, &r_target_perturbed, &robot);
        assert!(
            result.is_err(),
            "debería rechazar orientación con rotación Y pura en R35"
        );
        match result {
            Err(OrientationError::UnreachableOrientation { r35_02, tolerance }) => {
                assert!(
                    r35_02 > tolerance,
                    "r35_02={:.2e} debería exceder tol={:.2e}",
                    r35_02,
                    tolerance
                );
            }
            _ => unreachable!(),
        }
    }
}

#[test]
fn test_vary_q4_reconstructed() {
    let robot = make_robot();
    let base = make_base();
    let solver = OrientationSolver::new(1e-6);

    let mut seed: u64 = 77;

    let q5_vals = [-0.8, 0.0, 0.8];
    let q4_vals = [-1.2, -0.7, -0.2, 0.0, 0.3, 0.8, 1.2];

    for _ in 0..20 {
        let mut q_base: [f64; 5] = [0.0; 5];
        for i in 0..3 {
            let lo = robot.segments[i].joint.value_min.max(-2.0);
            let hi = robot.segments[i].joint.value_max.min(2.0);
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let r = (seed as f64) / (u64::MAX as f64);
            q_base[i] = lo + r * (hi - lo);
        }

        for &q5 in &q5_vals {
            q_base[4] = q5;
            for &q4 in &q4_vals {
                q_base[3] = q4;

                let robot_q = q_to_robot(&robot, &q_base);
                let (frames, effector) = forward_kinematics(base, &robot_q);
                let r03 = get_rot3(&frames[2]);
                let r_target = get_rot3(&effector);

                let result = solver.solve(&r03, &r_target, &robot);
                assert!(result.is_ok(), "q4={:.4}, q5={:.4} falló", q4, q5);

                let [q4_out, q5_out] = result.unwrap();
                let err4 = (q4_out - q4).abs() % (2.0 * std::f64::consts::PI);
                let err4 = err4.min(2.0 * std::f64::consts::PI - err4);
                let err5 = (q5_out - q5).abs() % (2.0 * std::f64::consts::PI);
                let err5 = err5.min(2.0 * std::f64::consts::PI - err5);

                assert!(err4 < 1e-12, "q4 error: {:.2e}", err4);
                assert!(err5 < 1e-12, "q5 error: {:.2e}", err5);
            }
        }
    }
}
