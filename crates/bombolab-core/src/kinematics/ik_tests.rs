//! Unit tests for `ik_tests`.

use super::*;
use crate::robot::fabri_creator;

fn make_test() -> (IkSolver, Robot, Iso3, Iso3) {
    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let robot = fabri_creator();
    let base = crate::robot::base_transform();
    let tool = *crate::robot::ToolFrame::marker_perpendicular().pose();
    (solver, robot, base, tool)
}

#[test]
fn solve_home_pose() {
    let (solver, robot, base, tool) = make_test();

    use crate::kinematics::forward::forward_kinematics;
    let home_q = [0.0_f64; 5];
    let robot_home = build_robot(&robot, &home_q);
    let (frames, _last) = forward_kinematics(base, &robot_home);
    let tool_tip = frames.last().unwrap() * tool;
    let target = [
        tool_tip.translation.x,
        tool_tip.translation.y,
        tool_tip.translation.z,
    ];
    let q_init = vec![0.0; 5];

    let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
    assert!(result.is_ok(), "IK should converge at home position");

    let q = result.unwrap();
    let robot_q = build_robot(&robot, &q);
    let (frames_q, _) = forward_kinematics(base, &robot_q);
    let tip_q = frames_q.last().unwrap() * tool;
    let err = (tip_q.translation.vector - tool_tip.translation.vector).norm();
    assert!(
        err < 2.0,
        "Position error at home: {:.3}mm (should be < 2mm)",
        err
    );
}

#[test]
fn solve_q_init_length_validated() {
    let (solver, robot, base, tool) = make_test();
    let target = [200.0, 0.0, 280.0];

    let empty: Vec<f64> = vec![];
    let result = solver.solve_position(&target, &empty, &robot, &base, &tool);
    assert!(matches!(
        result,
        Err(IkError::InvalidInitLength {
            expected: 5,
            got: 0
        })
    ));

    let short = vec![0.0; 3];
    let result = solver.solve_position(&target, &short, &robot, &base, &tool);
    assert!(matches!(
        result,
        Err(IkError::InvalidInitLength {
            expected: 5,
            got: 3
        })
    ));

    let long = vec![0.0; 7];
    let result = solver.solve_position(&target, &long, &robot, &base, &tool);
    assert!(matches!(
        result,
        Err(IkError::InvalidInitLength {
            expected: 5,
            got: 7
        })
    ));
}

#[test]
fn position_jacobian_finite_differences_fabri() {
    let (_, robot, base, tool) = make_test();
    use crate::kinematics::forward::forward_kinematics;

    let eps = 1e-8;
    let tol = 1e-4;

    let configs: [[f64; 5]; 4] = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.3, -0.5, 0.7, 0.4, -0.4],
        [-0.2, 0.4, -0.3, 0.5, 0.1],
        [0.0, 0.8, -0.6, 0.2, -0.2],
    ];

    for q in configs.iter() {
        let robot_q = build_robot(&robot, q);
        let (frames, _) = forward_kinematics(base, &robot_q);
        let tool_pose = frames.last().unwrap() * tool;
        let p_ee = tool_pose.translation.vector;
        let j_ana = position_jacobian(&robot_q, &frames, &p_ee, &base, 5);

        for col in 0..5 {
            let mut q_plus = *q;
            let mut q_minus = *q;
            q_plus[col] += eps;
            q_minus[col] -= eps;

            let robot_plus = build_robot(&robot, &q_plus);
            let (frames_plus, _) = forward_kinematics(base, &robot_plus);
            let p_plus = (frames_plus.last().unwrap() * tool).translation.vector;

            let robot_minus = build_robot(&robot, &q_minus);
            let (frames_minus, _) = forward_kinematics(base, &robot_minus);
            let p_minus = (frames_minus.last().unwrap() * tool).translation.vector;

            let dp = (p_plus - p_minus) / (2.0 * eps);

            for row in 0..3 {
                let num = dp[row];
                let ana = j_ana[(row, col)];
                assert!(
                    (num - ana).abs() < tol,
                    "config {q:?}, col {col}, row {row}: \
                         numerical = {num:.6e}, analytical = {ana:.6e}, diff = {}",
                    (num - ana).abs()
                );
            }
        }

        if *q == [0.0; 5] {
            let twist_col = j_ana.fixed_view::<3, 1>(0, 3).into_owned();
            assert!(
                twist_col.norm() < 1e-6,
                "FABRI twist linear column at home should be ~0, got {twist_col:?}"
            );
        }
    }
}

#[test]
fn solve_reachable_pose() {
    let (solver, robot, base, tool) = make_test();

    let target = [200.0, 0.0, 280.0];
    let q_init = vec![0.0; 5];

    let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
    assert!(result.is_ok(), "IK debería converger: {result:?}");

    let q = result.unwrap();
    let robot_q = build_robot(&robot, &q);
    let err = position_error(&robot_q, &target, &base, &tool);
    assert!(err < 10.0, "error debería ser <10mm, got {err:.3}");
}

#[test]
fn solve_upward() {
    let (solver, robot, base, tool) = make_test();

    let target = [150.0, 0.0, 400.0];
    let q_init = vec![0.0; 5];

    let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
    assert!(result.is_ok(), "IK debería converger: {result:?}");

    let q = result.unwrap();
    let robot_q = build_robot(&robot, &q);
    let err = position_error(&robot_q, &target, &base, &tool);
    assert!(err < 10.0, "error debería ser <10mm, got {err:.3}");
}

#[test]
fn solve_tracking() {
    let (solver, robot, base, tool) = make_test();

    let target_a = [200.0, 20.0, 280.0];
    let q_a = solver
        .solve_position(&target_a, &[0.0; 5], &robot, &base, &tool)
        .unwrap();

    let target_b = [210.0, 10.0, 270.0];
    let q_b = solver
        .solve_position(&target_b, &q_a, &robot, &base, &tool)
        .unwrap();

    let robot_q = build_robot(&robot, &q_b);
    let err = position_error(&robot_q, &target_b, &base, &tool);
    assert!(err < 10.0, "tracking error: {err:.3}");
}

#[test]
fn solve_unreachable_returns_max_iterations() {
    let (solver, robot, base, tool) = make_test();

    let target = [5000.0, 5000.0, 5000.0];
    let q_init = vec![0.0; 5];

    let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
    assert!(matches!(result, Err(IkError::MaxIterationsReached { .. })));
}
