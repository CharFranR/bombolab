use super::*;
use crate::math::Vec3;
use crate::robot::{Joint, Segment, fabri_creator};

fn make_base() -> Iso3 {
    crate::robot::base_transform()
}

fn constrained_tcp(robot: &Robot, base: Iso3, q13: &[f64; 3]) -> Vec3 {
    let q = [q13[0], q13[1], q13[2], 0.0, -(q13[1] + q13[2])];
    let segments: Vec<Segment> = robot
        .segments
        .iter()
        .zip(q.iter())
        .map(|(seg, &val)| {
            let joint = Joint::new(
                seg.joint.joint_type,
                val,
                seg.joint.value_max,
                seg.joint.value_min,
            );
            Segment::new(joint, seg.dh)
        })
        .collect();
    let robot_q = crate::robot::Robot::with_directions(
        segments,
        robot.home_pose.clone(),
        robot.servo_offsets.clone(),
        robot.servo_directions.clone(),
    )
    .with_tool(robot.tool().clone());
    let (frames, _) = crate::kinematics::forward::forward_kinematics(base, &robot_q);
    (frames.last().unwrap() * robot_q.tool().pose())
        .translation
        .vector
}

#[test]
fn test_reduced_jacobian_matches_fd() {
    let robot = fabri_creator();
    let base = make_base();
    let tool = *robot.tool().pose();
    let q13 = [
        (-3.36_f64).to_radians(),
        26.99_f64.to_radians(),
        28.37_f64.to_radians(),
    ];
    let q_full = [q13[0], q13[1], q13[2], 0.0, -(q13[1] + q13[2])];
    let jr = reduced_jacobian(&robot, &q_full, &base, &tool);
    let d = 1e-6;
    for i in 0..3 {
        let mut qp = q13;
        qp[i] += d;
        let mut qm = q13;
        qm[i] -= d;
        let fd =
            (constrained_tcp(&robot, base, &qp) - constrained_tcp(&robot, base, &qm)) / (2.0 * d);
        let diff = (Vec3::new(jr[(0, i)], jr[(1, i)], jr[(2, i)]) - fd).norm();
        assert!(
            diff < 1e-5,
            "reduced jacobian col {i} mismatch vs FD: |diff| = {diff:.2e}"
        );
    }
}

#[test]
fn test_waypoint_metrics_well_conditioned() {
    let robot = fabri_creator();
    let base = make_base();
    let tool = *robot.tool().pose();
    let q = [
        0.07505704715318309,
        1.0193783305962196,
        0.49389841131011203,
        0.0,
        -1.5132767419063318,
    ];
    let jr = reduced_jacobian(&robot, &q, &base, &tool);
    let m = waypoint_metrics(&jr);
    assert!(m.sigma_min > 50.0, "sigma_min = {}", m.sigma_min);
    assert!(m.kappa < 20.0, "kappa = {}", m.kappa);
    assert!(m.yoshikawa > 0.0, "yoshikawa = {}", m.yoshikawa);
    assert!(m.sv[0] > 0.0 && m.sv[2] >= m.sv[0], "sv = {:?}", m.sv);
}

#[test]
fn test_waypoint_metrics_degenerate_fold() {
    let robot = fabri_creator();
    let base = make_base();
    let tool = *robot.tool().pose();
    let q = [
        0.0_f64,
        90.0_f64.to_radians(),
        90.0_f64.to_radians(),
        0.0,
        (-180.0_f64).to_radians(),
    ];
    let jr = reduced_jacobian(&robot, &q, &base, &tool);
    let m = waypoint_metrics(&jr);
    assert!(m.sigma_min < 1e-3, "sigma_min = {}", m.sigma_min);
    assert!(m.yoshikawa < 1e-3, "yoshikawa = {}", m.yoshikawa);
}

#[test]
fn test_sigma_min_descent_toward_fold() {
    let robot = fabri_creator();
    let base = make_base();
    let tool = *robot.tool().pose();
    let q23s = [115.0_f64, 135.0, 150.0, 165.0];
    let mut prev = f64::INFINITY;
    for a in q23s {
        let half = a.to_radians() / 2.0;
        let q = [0.0, half, half, 0.0, -a.to_radians()];
        let jr = reduced_jacobian(&robot, &q, &base, &tool);
        let m = waypoint_metrics(&jr);
        assert!(
            m.sigma_min < prev,
            "sigma_min not monotonic at q23={a}: {} !< {prev}",
            m.sigma_min
        );
        prev = m.sigma_min;
    }
}

#[test]
fn test_gate_empty_path() {
    let robot = fabri_creator();
    let base = make_base();
    let report = analyze_path(&robot, &base, &[], &SingularityThresholds::default());
    assert_eq!(report.sampled, 0);
    assert!(report.worst.is_empty());
}

#[test]
fn test_gate_warm_start_chain() {
    let robot = fabri_creator();
    let base = make_base();
    let p1 = [310.0, 0.0, 120.0];
    let p2 = [320.0, 0.0, 120.0];
    let p3 = [330.0, 0.0, 120.0];
    let commands = vec![
        MotionCommand::MoveLinear {
            target: p1,
            speed: 1.0,
        },
        MotionCommand::MoveLinear {
            target: p2,
            speed: 1.0,
        },
        MotionCommand::MoveLinear {
            target: p3,
            speed: 1.0,
        },
    ];
    let report = analyze_path(&robot, &base, &commands, &SingularityThresholds::default());
    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let tool = *robot.tool().pose();
    let q1 = solve_drawing_plane_ik(&solver, &p1, &robot.kinematic_home(), &robot, &base, &tool)
        .expect("p1 reachable from kinematic home");
    let expected = solve_drawing_plane_ik(&solver, &p2, &q1, &robot, &base, &tool)
        .expect("p2 reachable from p1 warm start");
    let wp = report
        .worst
        .iter()
        .find(|w| w.target == p2)
        .expect("p2 must appear in the report");
    assert_eq!(wp.level, SingularityLevel::Ok);
    for (a, b) in wp.q.iter().zip(&expected) {
        assert!(
            (a - b).abs() < 0.1,
            "p2 q must stay on the warm-started branch: {a} vs {b}"
        );
    }
    for target in [p2, p3] {
        let wp = report
            .worst
            .iter()
            .find(|w| w.target == target)
            .expect("waypoint must appear in the report");
        let robot_q = build_robot(&robot, &wp.q);
        let (frames, _) = crate::kinematics::forward::forward_kinematics(base, &robot_q);
        let p = (frames.last().unwrap() * tool).translation.vector;
        let err = (Vec3::new(target[0], target[1], target[2]) - p).norm();
        assert!(
            err < 2.0,
            "reported q must land on target, err = {err:.3}mm"
        );
    }
    let wp3 = report
        .worst
        .iter()
        .find(|w| w.target == p3)
        .expect("p3 must appear in the report");
    assert_eq!(
        wp3.level,
        SingularityLevel::Warn,
        "boundary point is near-singular"
    );
}

#[test]
fn test_gate_10k_path_capped() {
    let robot = fabri_creator();
    let base = make_base();
    let mut commands = Vec::new();
    let mut x = 180.0;
    let mut dir = 4.0;
    for _ in 0..10_000u32 {
        commands.push(MotionCommand::MoveLinear {
            target: [x, 0.0, 80.0],
            speed: 1.0,
        });
        if x + dir > 300.0 || x + dir < 180.0 {
            dir = -dir;
        }
        x += dir;
    }
    let report = analyze_path(&robot, &base, &commands, &SingularityThresholds::default());
    assert_eq!(
        report.sampled, 5000,
        "sampling cap must bound the solved points"
    );
    assert_eq!(
        report.worst.len(),
        10,
        "well-conditioned path fills worst-N"
    );
}

#[test]
fn test_gate_custom_thresholds() {
    let robot = fabri_creator();
    let base = make_base();
    let target = [320.0, 0.0, 120.0];
    let commands = vec![MotionCommand::MoveLinear { target, speed: 1.0 }];
    let defaults = analyze_path(&robot, &base, &commands, &SingularityThresholds::default());
    let wp = defaults
        .worst
        .iter()
        .find(|w| w.target == target)
        .expect("target must appear under default thresholds");
    assert_eq!(
        wp.level,
        SingularityLevel::Ok,
        "sigma_min 36.6 is above warn 25"
    );
    let warn = SingularityThresholds {
        warn_sigma_min: 40.0,
        warn_kappa: 20.0,
        block_sigma_min: 5.0,
        block_kappa: 100.0,
    };
    let warn_report = analyze_path(&robot, &base, &commands, &warn);
    let wp = warn_report
        .worst
        .iter()
        .find(|w| w.target == target)
        .expect("target must appear under warn thresholds");
    assert_eq!(wp.level, SingularityLevel::Warn);
    let block = SingularityThresholds {
        warn_sigma_min: 40.0,
        warn_kappa: 20.0,
        block_sigma_min: 40.0,
        block_kappa: 100.0,
    };
    let block_report = analyze_path(&robot, &base, &commands, &block);
    let wp = block_report
        .worst
        .iter()
        .find(|w| w.target == target)
        .expect("target must appear under block thresholds");
    assert_eq!(wp.level, SingularityLevel::Block);
}

#[test]
fn test_gate_ik_non_convergence_blocks() {
    let robot = fabri_creator();
    let base = make_base();
    let commands = vec![MotionCommand::MoveLinear {
        target: [500.0, 0.0, 100.0],
        speed: 1.0,
    }];
    let report = analyze_path(&robot, &base, &commands, &SingularityThresholds::default());
    assert_eq!(report.sampled, 1);
    let wp = &report.worst[0];
    assert_eq!(wp.level, SingularityLevel::Block);
    assert_eq!(wp.reason, GateReason::IkNonConvergence);
}

#[test]
fn test_gate_deterministic() {
    let robot = fabri_creator();
    let base = make_base();
    let mut commands = Vec::new();
    for i in 0..50u32 {
        let theta = i as f64 * 0.07;
        commands.push(MotionCommand::MoveLinear {
            target: [220.0 * theta.cos(), 180.0 * theta.sin(), 90.0],
            speed: 1.0,
        });
    }
    let a = analyze_path(&robot, &base, &commands, &SingularityThresholds::default());
    let b = analyze_path(&robot, &base, &commands, &SingularityThresholds::default());
    assert_eq!(a.sampled, b.sampled);
    assert_eq!(a.worst.len(), b.worst.len());
    for (wa, wb) in a.worst.iter().zip(&b.worst) {
        assert_eq!(wa.index, wb.index);
        assert_eq!(
            wa.metrics.sigma_min.to_bits(),
            wb.metrics.sigma_min.to_bits()
        );
        assert_eq!(wa.level, wb.level);
    }
}
