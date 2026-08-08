//! Unit tests for `ik_full_tests`.

use super::*;
use crate::robot::fabri_creator;

fn make_robot() -> Robot {
    fabri_creator()
}

fn make_base() -> Iso3 {
    crate::robot::base_transform()
}

fn make_tool() -> Iso3 {
    *crate::robot::ToolFrame::marker_perpendicular().pose()
}

fn get_rot3(iso: &Iso3) -> Rot3 {
    iso.rotation.to_rotation_matrix()
}

fn position_error_for_q(
    robot: &Robot,
    q: &[f64],
    target_pos: &[f64; 3],
    base: &Iso3,
    tool: &Iso3,
) -> f64 {
    let robot_q = build_robot(robot, q);
    let (_frames, effector) = forward_kinematics(*base, &robot_q);
    let tool_pose = effector * tool;
    let p_ee = tool_pose.translation.vector;
    let target_v = Vec3::new(target_pos[0], target_pos[1], target_pos[2]);
    (target_v - p_ee).norm()
}

fn orientation_error_for_q(robot: &Robot, q: &[f64], target_rot: &Rot3, base: &Iso3) -> f64 {
    let robot_q = build_robot(robot, q);
    let (_frames, effector) = forward_kinematics(*base, &robot_q);
    let r_actual = get_rot3(&effector);
    (r_actual.matrix() - target_rot.matrix()).norm()
}

fn lcg(seed: &mut u64) -> f64 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    (*seed as f64) / (u64::MAX as f64)
}

fn rand_range(seed: &mut u64, lo: f64, hi: f64) -> f64 {
    lo + lcg(seed) * (hi - lo)
}

#[test]
fn test_full_ik_home_pose() {
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let orient_solver = OrientationSolver::new(1e-2);

    let q_home = [0.0; 5];
    let robot_home = build_robot(&robot, &q_home);
    let (_frames, effector) = forward_kinematics(base, &robot_home);
    let target_pos = [
        effector.translation.x,
        effector.translation.y,
        effector.translation.z,
    ];
    let target_rot = get_rot3(&effector);

    let q_init = vec![0.0; 5];
    let result = solve_full_ik(
        &pos_solver,
        &orient_solver,
        &target_pos,
        &target_rot,
        &q_init,
        &robot,
        &base,
        &tool,
    );
    assert!(result.is_ok(), "home debería ser alcanzable: {result:?}");

    let q = result.unwrap();

    let pos_err = position_error_for_q(&robot, &q, &target_pos, &base, &tool);
    assert!(pos_err < 5.0, "error posición home: {:.3}mm", pos_err);

    let orient_err = orientation_error_for_q(&robot, &q, &target_rot, &base);
    assert!(
        orient_err < 1e-1,
        "error orientación home: {:.2e}",
        orient_err
    );
}

#[test]
fn test_full_ik_random_configs() {
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let orient_solver = OrientationSolver::new(1e-6);

    let mut seed: u64 = 42;
    let n_samples = 50;
    let mut max_pos_err: f64 = 0.0;
    let mut max_orient_err: f64 = 0.0;

    for _ in 0..n_samples {
        let mut q: [f64; 5] = [0.0; 5];
        for i in 0..5 {
            let lo = robot.segments[i].joint.value_min.max(-1.3);
            let hi = robot.segments[i].joint.value_max.min(1.3);
            q[i] = rand_range(&mut seed, lo, hi);
        }

        let robot_q = build_robot(&robot, &q);
        let (_frames, effector) = forward_kinematics(base, &robot_q);
        let tool_pose = effector * tool;
        let target_pos: [f64; 3] = [
            tool_pose.translation.vector.x,
            tool_pose.translation.vector.y,
            tool_pose.translation.vector.z,
        ];
        let target_rot = get_rot3(&effector);

        let q_init = q.to_vec();
        let result = solve_full_ik(
            &pos_solver,
            &orient_solver,
            &target_pos,
            &target_rot,
            &q_init,
            &robot,
            &base,
            &tool,
        );
        assert!(
            result.is_ok(),
            "q={:.3?} debería ser alcanzable: {result:?}",
            q
        );

        let q_solved = result.unwrap();

        let pos_err = position_error_for_q(&robot, &q_solved, &target_pos, &base, &tool);
        max_pos_err = max_pos_err.max(pos_err);
        assert!(
            pos_err < 10.0,
            "error posición = {:.3}mm para q_target={:.3?}",
            pos_err,
            q
        );

        let orient_err = orientation_error_for_q(&robot, &q_solved, &target_rot, &base);
        max_orient_err = max_orient_err.max(orient_err);
        assert!(
            orient_err < 1e-10,
            "error orientación = {:.2e} para q_target={:.3?}",
            orient_err,
            q
        );
    }

    eprintln!("=== Full IK: random configs ===");
    eprintln!("Muestras: {n_samples}");
    eprintln!("Máx error posición: {:.3}mm", max_pos_err);
    eprintln!("Máx error orientación: {:.2e}", max_orient_err);
}

#[test]
fn test_full_ik_unreachable_orientation() {
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let orient_solver = OrientationSolver::new(1e-10);

    let target_pos = [236.0, 0.0, 314.0];

    let q_home = [0.0; 5];
    let robot_home = build_robot(&robot, &q_home);
    let (_frames, effector) = forward_kinematics(base, &robot_home);
    let r05_home = get_rot3(&effector);

    let r_y = Rot3::from_axis_angle(&nalgebra::Unit::new_normalize(Vec3::y()), 0.3);
    let target_rot = r05_home * r_y;

    let q_init = vec![0.0; 5];
    let result = solve_full_ik(
        &pos_solver,
        &orient_solver,
        &target_pos,
        &target_rot,
        &q_init,
        &robot,
        &base,
        &tool,
    );

    assert!(result.is_err(), "debería rechazar orientación inalcanzable");
    match result {
        Err(IkError::UnreachableOrientation { r35_02, tolerance }) => {
            assert!(
                r35_02 > tolerance,
                "r35_02={:.2e} debería exceder tol={:.2e}",
                r35_02,
                tolerance
            );
            eprintln!(" Orientación inalcanzable detectada: r35_02={:.2e}", r35_02);
        }
        Err(other) => panic!("error inesperado: {other}"),
        Ok(_) => unreachable!(),
    }
}

#[test]
fn test_position_solver_unchanged() {
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);

    let target = [236.0, 0.0, 314.0];
    let result = pos_solver.solve_position(&target, &[0.0; 5], &robot, &base, &tool);
    assert!(result.is_ok());
    let q = result.unwrap();
    let err = position_error_for_q(&robot, &q, &target, &base, &tool);
    assert!(err < 2.0, "home position error: {:.3}", err);

    let target = [200.0, 0.0, 280.0];
    let result = pos_solver.solve_position(&target, &[0.0; 5], &robot, &base, &tool);
    assert!(result.is_ok());
    let q = result.unwrap();
    let err = position_error_for_q(&robot, &q, &target, &base, &tool);
    assert!(err < 10.0, "reachable position error: {:.3}", err);

    let target_a = [200.0, 20.0, 280.0];
    let q_a = pos_solver
        .solve_position(&target_a, &[0.0; 5], &robot, &base, &tool)
        .unwrap();
    let target_b = [210.0, 10.0, 270.0];
    let result = pos_solver.solve_position(&target_b, &q_a, &robot, &base, &tool);
    assert!(result.is_ok());
    let q_b = result.unwrap();
    let err = position_error_for_q(&robot, &q_b, &target_b, &base, &tool);
    assert!(err < 10.0, "tracking error: {:.3}", err);

    let target = [5000.0, 5000.0, 5000.0];
    let result = pos_solver.solve_position(&target, &[0.0; 5], &robot, &base, &tool);
    assert!(matches!(result, Err(IkError::MaxIterationsReached { .. })));
}

#[test]
fn diagnostic_drawing_pose_roundtrip() {
    use crate::kinematics::pose_generator::PoseGenerator;

    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let orient_solver = OrientationSolver::new(1e-6);

    let test_positions: [[f64; 3]; 5] = [
        [200.0, 0.0, 80.0],
        [150.0, 50.0, 80.0],
        [150.0, -50.0, 80.0],
        [250.0, 0.0, 100.0],
        [180.0, 30.0, 70.0],
    ];

    for &pos in &test_positions {
        let target = PoseGenerator::drawing_pose(pos);
        let q_init = vec![0.0; 5];

        let result = solve_full_ik(
            &pos_solver,
            &orient_solver,
            &target.position,
            &target.rotation,
            &q_init,
            &robot,
            &base,
            &tool,
        );

        eprintln!();
        eprintln!("═══════════════════════════════════════════════");
        eprintln!(
            "Pose de dibujo: ({:.0}, {:.0}, {:.0}) mm",
            pos[0], pos[1], pos[2]
        );

        match result {
            Err(e) => {
                eprintln!(" IK falló: {e}");
                continue;
            }
            Ok(q) => {
                eprintln!(
                    " Solución IK: q = [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}]",
                    q[0], q[1], q[2], q[3], q[4]
                );

                let r_target = target.rotation;
                eprintln!();
                eprintln!("R_target (PoseGenerator):");
                let rt = r_target.matrix();
                for row in 0..3 {
                    eprintln!(
                        "  [{:>8.4} {:>8.4} {:>8.4}]",
                        rt[(row, 0)],
                        rt[(row, 1)],
                        rt[(row, 2)]
                    );
                }
                eprintln!(
                    "  → X5 (marcador) en mundo: [{:.4}, {:.4}, {:.4}]",
                    rt[(0, 0)],
                    rt[(1, 0)],
                    rt[(2, 0)]
                );

                let robot_q = build_robot(&robot, &q);
                let (_frames, effector) = forward_kinematics(base, &robot_q);
                let r05 = get_rot3(&effector);
                let r05_m = r05.matrix();
                eprintln!();
                eprintln!("R05 (FK desde solución IK):");
                for row in 0..3 {
                    eprintln!(
                        "  [{:>8.4} {:>8.4} {:>8.4}]",
                        r05_m[(row, 0)],
                        r05_m[(row, 1)],
                        r05_m[(row, 2)]
                    );
                }
                eprintln!(
                    "  → X5 real en mundo: [{:.4}, {:.4}, {:.4}]",
                    r05_m[(0, 0)],
                    r05_m[(1, 0)],
                    r05_m[(2, 0)]
                );

                let tool_pose = effector * tool;
                let tcp = tool_pose.translation.vector;
                eprintln!();
                eprintln!(
                    "TCP real (con tool_transform): ({:.2}, {:.2}, {:.2}) mm",
                    tcp.x, tcp.y, tcp.z
                );
                eprintln!(
                    "TCP objetivo: ({:.0}, {:.0}, {:.0}) mm",
                    pos[0], pos[1], pos[2]
                );
                let pos_err = (Vec3::new(pos[0], pos[1], pos[2]) - tcp).norm();
                eprintln!("Error posición: {:.4} mm", pos_err);

                let r_error = r05.transpose() * r_target;
                let re = r_error.matrix();
                eprintln!();
                eprintln!("R_error = R05^T · R_target (≈ I si IK correcta):");
                for row in 0..3 {
                    eprintln!(
                        "  [{:>8.4} {:>8.4} {:>8.4}]",
                        re[(row, 0)],
                        re[(row, 1)],
                        re[(row, 2)]
                    );
                }

                let angle_err = r_error.angle();
                let frob_err = (re - nalgebra::Matrix3::<f64>::identity()).norm();
                eprintln!(
                    "Error angular: {:.2e} rad ({:.6}°)",
                    angle_err,
                    angle_err.to_degrees()
                );
                eprintln!("Error Frobenius: {:.2e}", frob_err);

                if angle_err < 1e-6 {
                    eprintln!("R_error ≈ I → IK correcta. El bug está en el render visual.");
                } else {
                    eprintln!("R_error NO es I → bug en la integración IK.");
                }
            }
        }
    }
    eprintln!();
    eprintln!("═══════════════════════════════════════════════");
    eprintln!("Diagnóstico completado.");
}

#[test]
fn test_solve_drawing_ik_centered() {
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let orient_solver = OrientationSolver::new(1e-6);

    let pos = [200.0, 0.0, 80.0];
    let result = solve_drawing_ik(
        &pos_solver,
        &orient_solver,
        &pos,
        &[0.0; 5],
        &robot,
        &base,
        &tool,
    );
    assert!(result.is_ok(), "centrada debe funcionar: {result:?}");
    let q = result.unwrap();

    let robot_q = build_robot(&robot, &q);
    let (_frames, effector) = forward_kinematics(base, &robot_q);
    let r05 = get_rot3(&effector);

    let x5 = r05.matrix().column(0);
    assert!((x5.x).abs() < 1e-6, "X5_x ≈ 0, got {}", x5.x);
    assert!((x5.y).abs() < 1e-6, "X5_y ≈ 0, got {}", x5.y);
    assert!((x5.z + 1.0).abs() < 1e-6, "X5_z ≈ -1, got {}", x5.z);

    eprintln!(
        "solve_drawing_ik centrada: q=[{:.4},{:.4},{:.4},{:.4},{:.4}]",
        q[0], q[1], q[2], q[3], q[4]
    );
}

#[test]
fn test_solve_drawing_ik_lateral() {
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.1, 0.5);
    let orient_solver = OrientationSolver::new(1e-6);

    let test_positions: [[f64; 3]; 6] = [
        [200.0, 50.0, 80.0],
        [200.0, 100.0, 80.0],
        [200.0, -50.0, 80.0],
        [200.0, -100.0, 80.0],
        [250.0, 50.0, 90.0],
        [150.0, 80.0, 75.0],
    ];

    let all_ok: bool = true;
    for &pos in &test_positions {
        let result = solve_drawing_ik(
            &pos_solver,
            &orient_solver,
            &pos,
            &[0.0; 5],
            &robot,
            &base,
            &tool,
        );

        match result {
            Err(e) => {
                eprintln!("({:.0},{:.0},{:.0}) rechazada: {e}", pos[0], pos[1], pos[2]);
            }
            Ok(q) => {
                let robot_q = build_robot(&robot, &q);
                let (_frames, effector) = forward_kinematics(base, &robot_q);
                let r05 = get_rot3(&effector);
                let x5 = r05.matrix().column(0);
                let x5_down = (x5.z + 1.0).abs();

                assert!(
                    x5_down < 1e-6,
                    "({:.0},{:.0},{:.0}): X5_z ≈ -1, got {:.4} (q1={:.1}°)",
                    pos[0],
                    pos[1],
                    pos[2],
                    x5.z,
                    q[0].to_degrees()
                );
                eprintln!(
                    " ({:.0},{:.0},{:.0}) q1={:.1}° X5=[{:.3},{:.3},{:.3}]",
                    pos[0],
                    pos[1],
                    pos[2],
                    q[0].to_degrees(),
                    x5.x,
                    x5.y,
                    x5.z
                );
            }
        }
    }
    assert!(all_ok, "al menos una pose lateral falló");
}

#[test]
fn test_solve_drawing_ik_vs_constant() {
    use crate::kinematics::pose_generator::PoseGenerator;

    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let pos_solver = IkSolver::new(200, 1.0, 0.1, 0.5);
    let orient_solver = OrientationSolver::new(1e-6);

    let pos = [200.0, 80.0, 80.0];

    let const_pose = PoseGenerator::drawing_pose(pos);
    let const_result = solve_full_ik(
        &pos_solver,
        &orient_solver,
        &pos,
        &const_pose.rotation,
        &[0.0; 5],
        &robot,
        &base,
        &tool,
    );
    assert!(
        const_result.is_err(),
        "constante debería fallar para posición lateral"
    );

    let adapt_result = solve_drawing_ik(
        &pos_solver,
        &orient_solver,
        &pos,
        &[0.0; 5],
        &robot,
        &base,
        &tool,
    );
    assert!(
        adapt_result.is_ok(),
        "adaptativa debería funcionar para posición lateral"
    );

    eprintln!("Constante, Adaptativa para (200, 80, 80)");
}

fn constrained_tcp(robot: &Robot, base: Iso3, q13: &[f64; 3]) -> Vec3 {
    let q = [q13[0], q13[1], q13[2], 0.0, -(q13[1] + q13[2])];
    let robot_q = build_robot(robot, &q);
    let (frames, _) = forward_kinematics(base, &robot_q);
    let tool = *crate::robot::ToolFrame::marker_perpendicular().pose();
    (frames.last().unwrap() * tool).translation.vector
}

#[test]
fn test_reduced_jacobian_matches_fd() {
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let q13 = [
        (-3.36_f64).to_radians(),
        26.99_f64.to_radians(),
        28.37_f64.to_radians(),
    ];

    let q_full = [q13[0], q13[1], q13[2], 0.0, -(q13[1] + q13[2])];
    let robot_q = build_robot(&robot, &q_full);
    let (frames, _) = forward_kinematics(base, &robot_q);
    let p_ee = (frames.last().unwrap() * tool).translation.vector;

    let j_full = position_jacobian(&robot_q, &frames, &p_ee, &base, 5);

    let mut jr_analytic = [Vec3::zeros(); 3];
    for r in 0..3 {
        jr_analytic[0][r] = j_full[(r, 0)];
        jr_analytic[1][r] = j_full[(r, 1)] - j_full[(r, 4)];
        jr_analytic[2][r] = j_full[(r, 2)] - j_full[(r, 4)];
    }

    let d = 1e-6;
    for i in 0..3 {
        let mut qp = q13;
        qp[i] += d;
        let mut qm = q13;
        qm[i] -= d;
        let fd =
            (constrained_tcp(&robot, base, &qp) - constrained_tcp(&robot, base, &qm)) / (2.0 * d);
        let diff = (Vec3::new(jr_analytic[i][0], jr_analytic[i][1], jr_analytic[i][2]) - fd).norm();
        assert!(
            diff < 1e-5,
            "reduced jacobian col {i} mismatch vs FD: |diff| = {diff:.2e}"
        );
    }
}

#[test]
fn test_drawing_plane_demo_corner() {
    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();
    let target = [175.0, -25.0, 80.0];

    let q = solve_drawing_plane_ik(&solver, &target, &[0.0; 5], &robot, &base, &tool)
        .expect("demo corner must be solvable in the drawing manifold");

    assert!((q[3]).abs() < 1e-12, "q4 must be 0, got {}", q[3]);
    assert!(
        (q[4] + (q[1] + q[2])).abs() < 1e-12,
        "q5 must be −(q2+q3), got q5={} q23={}",
        q[4],
        q[1] + q[2]
    );

    let robot_q = build_robot(&robot, &q);
    let (frames, _) = forward_kinematics(base, &robot_q);
    let p = (frames.last().unwrap() * tool).translation.vector;
    let err = (Vec3::new(target[0], target[1], target[2]) - p).norm();
    assert!(err < 2.0, "TCP error must be < 2mm, got {err:.3}mm");

    let y5 = frames[4].rotation * Vec3::y();
    assert!(
        (y5 - Vec3::new(0.0, 0.0, -1.0)).norm() < 1e-9,
        "marker must point down, got Y5 = {y5}"
    );
}

#[test]
fn test_drawing_plane_outside_workspace() {
    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let robot = make_robot();
    let base = make_base();
    let tool = make_tool();

    let folded = [
        0.0_f64,
        60.0_f64.to_radians(),
        60.0_f64.to_radians(),
        0.0,
        -120.0_f64.to_radians(),
    ];
    let robot_q = build_robot(&robot, &folded);
    let (frames, _) = forward_kinematics(base, &robot_q);
    let p = (frames.last().unwrap() * tool).translation.vector;
    let target = [p.x, p.y, p.z];

    let result = solve_drawing_plane_ik(&solver, &target, &[0.0; 5], &robot, &base, &tool);
    assert!(
        matches!(result, Err(IkError::DrawingConstraintViolated { .. }))
            || matches!(result, Err(IkError::MaxIterationsReached { .. })),
        "folded target must not converge in the drawing manifold, got {result:?}"
    );
}
