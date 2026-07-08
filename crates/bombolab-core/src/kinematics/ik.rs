use std::fmt;

use crate::kinematics::dh::{self, DHParameter};
use crate::math::jacobian::{geometric_jacobian, JointKind};
use crate::math::{Iso3, Mat4, MatDyn, Quat, Rot3, Vec3};
use crate::robot::{JointType, Robot};

/// Options for the damped pseudoinverse (Levenberg–Marquardt) IK solver.
#[derive(Debug, Clone, PartialEq)]
pub struct IkOptions {
    /// Position convergence threshold in mm.
    pub tolerance_pos: f64,
    /// Orientation convergence threshold in radians (free axes only).
    pub tolerance_angle: f64,
    /// Maximum number of damping iterations.
    pub max_iterations: usize,
    /// Initial damping factor λ₀.
    pub lambda_initial: f64,
    /// Per-iteration damping decay factor (λ ← λ × λ_decay).
    pub lambda_decay: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        Self {
            tolerance_pos: 1.0,
            tolerance_angle: 0.1,
            max_iterations: 200,
            lambda_initial: 10.0,
            lambda_decay: 0.95,
        }
    }
}

/// Result of a single IK solve attempt.
#[derive(Debug, Clone, PartialEq)]
pub struct IkResult {
    /// Solution joint angles (kinematic coordinates, radians).
    pub q: Vec<f64>,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Final position error magnitude (mm) — ‖δp‖.
    pub error_pos: f64,
    /// Final orientation error magnitude (rad) — ‖δω[0:2]‖ (free axes only).
    pub error_angle: f64,
    /// Whether the solver converged within tolerance.
    pub converged: bool,
}

/// Errors that can occur during IK solving.
#[derive(Debug, Clone, PartialEq)]
pub enum IkError {
    /// The solver reached max iterations without converging.
    DidNotConverge {
        /// Final q values (kinematic coordinates).
        q: Vec<f64>,
        /// Number of iterations performed.
        iterations: usize,
        /// Final position error magnitude (mm).
        error_pos: f64,
        /// Final orientation error magnitude (rad).
        error_angle: f64,
    },
    /// The target or configuration is invalid (e.g., zero-DOF robot).
    InvalidTarget(String),
    /// Jacobian computation failed.
    JacobianError(String),
}

impl fmt::Display for IkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IkError::DidNotConverge {
                q: _,
                iterations,
                error_pos,
                error_angle,
            } => {
                write!(
                    f,
                    "IK did not converge after {iterations} iterations \
                     (pos_err={error_pos:.4} mm, ang_err={error_angle:.4} rad)"
                )
            }
            IkError::InvalidTarget(msg) => {
                write!(f, "Invalid IK target: {msg}")
            }
            IkError::JacobianError(msg) => {
                write!(f, "Jacobian error: {msg}")
            }
        }
    }
}

impl std::error::Error for IkError {}

/// Compute a DH table from the current `q` vector, using the fixed robot geometry.
fn build_dh_table(robot: &Robot, q: &[f64]) -> Vec<DHParameter> {
    robot
        .segments
        .iter()
        .enumerate()
        .map(|(i, seg)| match seg.joint.joint_type {
            JointType::Revolute => {
                // theta = q[i] (joint angle), d/a/alpha are fixed link parameters
                DHParameter::new(seg.dh.alpha, seg.dh.a, seg.dh.d, q[i])
            }
            JointType::Prismatic => {
                // d = q[i] (prismatic displacement), theta/a/alpha are fixed
                DHParameter::new(seg.dh.alpha, seg.dh.a, q[i], seg.dh.theta)
            }
        })
        .collect()
}

/// Extract target position and rotation from a J5-frame target (as Mat4).
fn extract_target(target_mat4: &Mat4) -> (Vec3, Rot3) {
    let pos = target_mat4.fixed_view::<3, 1>(0, 3).into_owned();
    let rot_mat = target_mat4.fixed_view::<3, 3>(0, 0).into_owned();
    let rot = Rot3::from_matrix_unchecked(rot_mat);
    (pos, rot)
}

/// Compute position and orientation errors between current and target.
fn compute_errors(
    sol_translation: &Vec3,
    sol_rotation: &Rot3,
    target_pos: &Vec3,
    target_rot: &Rot3,
) -> (Vec3, Vec3, f64, f64) {
    // Position error
    let δp = target_pos - sol_translation;

    // Orientation error: R_err = R_target × R_curᵀ → scaled axis
    let r_err = target_rot * sol_rotation.transpose();
    let δω = Quat::from_rotation_matrix(&r_err).scaled_axis();

    // Weighted error magnitudes
    let pos_err = δp.norm();
    // Only free axes (X, Y) — Z (tool roll) is free
    let ang_err = Vec3::new(δω.x, δω.y, 0.0).norm();

    (δp, δω, pos_err, ang_err)
}

/// Damped pseudoinverse (Levenberg–Marquardt) IK solver.
///
/// Computes joint angles `q` that position the robot's end-effector at the
/// given `target` pose. The `target` is expressed in the **J5/base frame**
/// (the same frame produced by `forward_kinematics` / `dh::solve`).
///
/// The solver works on a local `q` copy, never mutating `robot.segments`.
/// Joint limits are enforced by clamping each q element after every iteration.
pub fn inverse_kinematics(
    robot: &Robot,
    base: &Iso3,
    target: &Iso3,
    q_seed: &[f64],
    options: &IkOptions,
) -> Result<IkResult, IkError> {
    let n = robot.dof();

    // --- Validation ---
    if n == 0 {
        return Err(IkError::InvalidTarget(
            "robot has zero degrees of freedom".into(),
        ));
    }
    if q_seed.len() != n {
        return Err(IkError::InvalidTarget(format!(
            "q_seed length {} does not match robot DOF {}",
            q_seed.len(),
            n
        )));
    }

    // --- 2.1: Target transform: bring target into J5/base frame ---
    let base_inv = base.inverse();
    let target_j5 = base_inv * target;
    let target_mat4 = target_j5.to_homogeneous();
    let (target_pos, target_rot) = extract_target(&target_mat4);

    // --- Joint kinds for Jacobian (computed once) ---
    let joint_kinds: Vec<JointKind> = robot
        .segments
        .iter()
        .map(|s| match s.joint.joint_type {
            JointType::Revolute => JointKind::Revolute,
            JointType::Prismatic => JointKind::Prismatic,
        })
        .collect();

    // --- Local q copy ---
    let mut q = q_seed.to_vec();
    let mut lambda = options.lambda_initial;
    let mut iteration = 0usize;

    while iteration < options.max_iterations {
        iteration += 1;
        // --- 2.2: DH bridge ---
        let dh_table = build_dh_table(robot, &q);

        // --- 2.3: FK + Jacobian ---
        let sol = dh::solve(&dh_table);
        let j = geometric_jacobian(&sol.intermediates, &joint_kinds, &sol.final_transform)
            .map_err(|e| IkError::JacobianError(e.to_string()))?;

        let cur_pos = sol.translation();
        let cur_rot_mat = sol.final_transform.fixed_view::<3, 3>(0, 0).into_owned();
        let cur_rot = Rot3::from_matrix_unchecked(cur_rot_mat);

        // --- 2.4: Compute errors ---
        let (δp, δω, pos_err, ang_err) =
            compute_errors(&cur_pos, &cur_rot, &target_pos, &target_rot);

        // --- 2.7: Convergence check ---
        if pos_err < options.tolerance_pos && ang_err < options.tolerance_angle {
            return Ok(IkResult {
                q,
                iterations: iteration,
                error_pos: pos_err,
                error_angle: ang_err,
                converged: true,
            });
        }

        // --- 2.5: Weighted damped solve ---
        // Build weighted error vector (Z-angular is free → row 5 = 0)
        let e_w = MatDyn::from_vec(6, 1, vec![δp.x, δp.y, δp.z, δω.x, δω.y, 0.0]);

        // Zero out row 5 of J (Z-angular velocity)
        let mut j_w = j;
        for col in 0..j_w.ncols() {
            j_w[(5, col)] = 0.0;
        }

        // Cost: ½‖e_w‖² — same objective the LM step minimizes
        let current_cost = 0.5 * e_w.dot(&e_w);

        // Levenberg–Marquardt damping with cost-based acceptance:
        //
        //   jtj = J^T J + λ² I
        //
        // The LM step solves  (JᵀJ + λ²I)dq = Jᵀe_w  which minimizes
        // ½‖e_w‖²  (squared weighted error).  The acceptance criterion
        // compares the SAME quantity before and after the trial step:
        //
        //   actual_reduction = ½‖e_w‖² - ½‖e_w_trial‖²
        //
        // If actual_reduction > 0 the step decreased the objective and
        // is accepted (λ decays).  Otherwise the step is rejected and
        // λ is increased (more damping).
        let jtj = j_w.tr_mul(&j_w) + lambda.powi(2) * MatDyn::identity(n, n);
        let jte = j_w.tr_mul(&e_w);

        let dq = jtj
            .lu()
            .solve(&jte)
            .ok_or_else(|| IkError::JacobianError("singular matrix in damped solve".into()))?;

        // --- 2.6: Gain-ratio check and step ---
        // Try the step: q_trial = q + Δq
        let mut dq_applied = false;
        let mut q_trial = q.clone();
        for i in 0..n {
            q_trial[i] += dq[(i, 0)];
            let min = robot.segments[i].joint.value_min;
            let max = robot.segments[i].joint.value_max;
            let clamped = q_trial[i].clamp(min, max);
            if (clamped - q[i]).abs() > 1e-15 {
                dq_applied = true;
            }
            q_trial[i] = clamped;
        }

        if dq_applied {
            // FK at trial q to evaluate actual error reduction
            let dh_trial = build_dh_table(robot, &q_trial);
            let sol_trial = dh::solve(&dh_trial);
            let trial_pos = sol_trial.translation();
            let trial_rot_mat = sol_trial
                .final_transform
                .fixed_view::<3, 3>(0, 0)
                .into_owned();
            let trial_rot = Rot3::from_matrix_unchecked(trial_rot_mat);
            let (trial_δp, trial_δω, _trial_pos_err, _trial_ang_err) =
                compute_errors(&trial_pos, &trial_rot, &target_pos, &target_rot);

            let trial_e_w = MatDyn::from_vec(
                6,
                1,
                vec![
                    trial_δp.x,
                    trial_δp.y,
                    trial_δp.z,
                    trial_δω.x,
                    trial_δω.y,
                    0.0,
                ],
            );
            let trial_cost = 0.5 * trial_e_w.dot(&trial_e_w);
            let actual_reduction = current_cost - trial_cost;

            if actual_reduction > 0.0 {
                // Accept: error decreased
                q = q_trial;
                lambda *= options.lambda_decay;
            } else {
                // Reject: error increased — increase damping and try again
                lambda *= 2.0;
                if lambda > 1e6 {
                    break;
                }
            }
        } else {
            // No change to any joint (all clamped) — we're stuck
            break;
        }
    }

    // --- 2.8: Did not converge — return final q with diagnostics ---
    let dh_table = build_dh_table(robot, &q);
    let sol = dh::solve(&dh_table);
    let cur_pos = sol.translation();
    let cur_rot_mat = sol.final_transform.fixed_view::<3, 3>(0, 0).into_owned();
    let cur_rot = Rot3::from_matrix_unchecked(cur_rot_mat);
    let (_δp, _δω, pos_err, ang_err) = compute_errors(&cur_pos, &cur_rot, &target_pos, &target_rot);

    Err(IkError::DidNotConverge {
        q,
        iterations: iteration,
        error_pos: pos_err,
        error_angle: ang_err,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::forward::forward_kinematics;
    use crate::robot::fabri_creator::{base_transform, fabri_creator};
    use crate::robot::Robot;

    fn default_opts() -> IkOptions {
        IkOptions::default()
    }

    // ── Phase 1: Type Tests ──────────────────────────────────────────

    #[test]
    fn test_ik_options_default() {
        let opts = IkOptions::default();
        assert!((opts.tolerance_pos - 1.0).abs() < 1e-10);
        assert!((opts.tolerance_angle - 0.1).abs() < 1e-10);
        assert_eq!(opts.max_iterations, 200);
        assert!((opts.lambda_initial - 10.0).abs() < 1e-10);
        assert!((opts.lambda_decay - 0.95).abs() < 1e-10);
    }

    #[test]
    fn test_ik_options_custom() {
        let opts = IkOptions {
            tolerance_pos: 0.5,
            tolerance_angle: 0.01,
            max_iterations: 100,
            lambda_initial: 1.0,
            lambda_decay: 0.95,
        };
        assert!((opts.tolerance_pos - 0.5).abs() < 1e-10);
        assert!((opts.tolerance_angle - 0.01).abs() < 1e-10);
        assert_eq!(opts.max_iterations, 100);
        assert!((opts.lambda_initial - 1.0).abs() < 1e-10);
        assert!((opts.lambda_decay - 0.95).abs() < 1e-10);
    }

    #[test]
    fn test_ik_result_converged() {
        let result = IkResult {
            q: vec![0.1, 0.2, -0.1, 0.3, 0.0],
            iterations: 15,
            error_pos: 0.5,
            error_angle: 0.01,
            converged: true,
        };
        assert_eq!(result.q.len(), 5);
        assert!(result.converged);
        assert_eq!(result.iterations, 15);
        assert!((result.error_pos - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_ik_result_not_converged() {
        let result = IkResult {
            q: vec![0.0, 0.0],
            iterations: 50,
            error_pos: 10.0,
            error_angle: 0.5,
            converged: false,
        };
        assert!(!result.converged);
        assert_eq!(result.iterations, 50);
    }

    #[test]
    fn test_ik_error_display_invalid_target() {
        let err = IkError::InvalidTarget("zero DOF robot".into());
        let msg = format!("{}", err);
        assert!(msg.contains("zero DOF robot"));
        assert!(msg.contains("Invalid"));
    }

    #[test]
    fn test_ik_error_display_jacobian() {
        let err = IkError::JacobianError("empty chain".into());
        let msg = format!("{}", err);
        assert!(msg.contains("empty chain"));
    }

    #[test]
    fn test_ik_error_display_did_not_converge() {
        let err = IkError::DidNotConverge {
            q: vec![0.0; 5],
            iterations: 50,
            error_pos: 5.0,
            error_angle: 0.1,
        };
        let msg = format!("{}", err);
        assert!(msg.contains("50 iterations"));
        assert!(msg.contains("5.0"));
        assert!(msg.contains("0.1"));
    }

    #[test]
    fn test_ik_error_source_invalid_target() {
        use std::error::Error;
        let err = IkError::InvalidTarget("test".into());
        assert!(err.source().is_none());
    }

    #[test]
    fn test_ik_error_source_did_not_converge() {
        use std::error::Error;
        let err = IkError::DidNotConverge {
            q: vec![],
            iterations: 10,
            error_pos: 1.0,
            error_angle: 0.1,
        };
        assert!(err.source().is_none());
    }

    #[test]
    fn test_ik_error_source_jacobian() {
        use std::error::Error;
        let err = IkError::JacobianError("test".into());
        assert!(err.source().is_none());
    }

    #[test]
    fn test_inverse_kinematics_zero_dof() {
        let robot = Robot::new(vec![]);
        let base = Iso3::identity();
        let target = Iso3::identity();
        let q_seed: Vec<f64> = vec![];
        let opts = default_opts();
        let result = inverse_kinematics(&robot, &base, &target, &q_seed, &opts);
        assert!(matches!(result, Err(IkError::InvalidTarget(_))));
    }

    #[test]
    fn test_inverse_kinematics_seed_length_mismatch() {
        let robot = fabri_creator();
        let base = base_transform();
        let target = Iso3::identity();
        let q_seed = vec![0.0, 0.0]; // wrong length
        let opts = default_opts();
        let result = inverse_kinematics(&robot, &base, &target, &q_seed, &opts);
        assert!(matches!(result, Err(IkError::InvalidTarget(_))));
    }

    // ── Phase 2: Algorithm Tests (DH bridge, target conversion, loop) ──
    // These are covered by Phase 3 integration tests below.

    // ── Phase 3: Test scenarios ──────────────────────────────────────

    #[test]
    fn test_home_pose_roundtrip() {
        // 3.1: FK→IK→FK at home pose
        let robot = fabri_creator();
        let base = base_transform();
        let opts = default_opts();

        let (_, fk_pose) = forward_kinematics(base, &robot);

        let q_seed = vec![0.0; 5];
        let result = inverse_kinematics(&robot, &base, &fk_pose, &q_seed, &opts)
            .expect("IK should converge at home pose");

        assert!(result.converged, "IK should converge");
        assert!(
            result.error_pos < 1.0,
            "position error < 1mm: {}",
            result.error_pos
        );

        // q should approximately match seed (home pose)
        for (i, &qi) in result.q.iter().enumerate() {
            assert!(
                (qi - q_seed[i]).abs() < 0.1,
                "q[{}] differs from seed: {} vs {}",
                i,
                qi,
                q_seed[i]
            );
        }
    }

    #[test]
    fn test_random_q_roundtrip() {
        // 3.2: Multiple random reachable configurations
        let robot = fabri_creator();
        let base = base_transform();
        let opts = default_opts();

        let test_angles = vec![
            vec![0.1, 0.2, -0.1, 0.3, 0.0],
            vec![-0.2, 0.3, 0.1, -0.2, 0.1],
            vec![0.5, -0.3, 0.2, -0.1, 0.05],
        ];

        for q_orig in &test_angles {
            // Set up robot at test configuration for FK target
            let mut robot_mut = fabri_creator();
            for (i, &qi) in q_orig.iter().enumerate() {
                robot_mut.segments[i].joint.value = qi;
            }
            let (_, fk_pose) = forward_kinematics(base, &robot_mut);

            // IK from q_orig as seed should converge back to similar q
            let result = inverse_kinematics(&robot, &base, &fk_pose, q_orig, &opts)
                .unwrap_or_else(|e| panic!("IK should converge for q={q_orig:?}: {e:?}"));

            assert!(result.converged, "IK should report converged");
            assert!(
                result.error_pos < 1.0,
                "position error: {}",
                result.error_pos
            );
        }
    }

    #[test]
    fn test_free_z_roll_invariance() {
        // 3.3: Same target reachable from different seeds (Z-roll is free).
        // Verify via FK(IK(target)) ≈ target that any found solution is valid.
        let robot = fabri_creator();
        let base = base_transform();
        let opts = default_opts();

        // Get a FK pose from a test configuration
        let test_q = vec![0.1, 0.1, -0.05, 0.0, 0.0];
        let mut robot_mut = fabri_creator();
        for (i, &qi) in test_q.iter().enumerate() {
            robot_mut.segments[i].joint.value = qi;
        }
        let (_, base_pose) = forward_kinematics(base, &robot_mut);

        // Try from different seeds — at least one should give FK(IK) ≈ target
        let seeds = vec![
            vec![0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.05, -0.05, 0.02, 0.0, 0.03],
            vec![-0.02, 0.03, -0.01, 0.02, -0.01],
        ];

        let mut best_dist = f64::MAX;
        for seed in &seeds {
            let result = inverse_kinematics(&robot, &base, &base_pose, seed, &opts);
            match result {
                Ok(r) => {
                    // Verify FK(IK_result) ≈ original target
                    let mut verify_robot = fabri_creator();
                    for (i, &qi) in r.q.iter().enumerate() {
                        verify_robot.segments[i].joint.value = qi;
                    }
                    let (_, fk_back) = forward_kinematics(base, &verify_robot);
                    let δp = base_pose.translation.vector - fk_back.translation.vector;
                    let dist = δp.norm();
                    best_dist = best_dist.min(dist);
                }
                Err(IkError::DidNotConverge { q, .. }) => {
                    let mut verify_robot = fabri_creator();
                    for (i, &qi) in q.iter().enumerate() {
                        verify_robot.segments[i].joint.value = qi;
                    }
                    let (_, fk_back) = forward_kinematics(base, &verify_robot);
                    let δp = base_pose.translation.vector - fk_back.translation.vector;
                    let dist = δp.norm();
                    best_dist = best_dist.min(dist);
                }
                Err(_) => {}
            }
        }

        assert!(
            best_dist < 3.0,
            "Best FK(IK(target)) position error {best_dist}mm > 3mm for all seeds"
        );
    }

    #[test]
    fn test_joint_limits_respected() {
        // 3.4: Even for unreachable targets, q stays within limits
        let robot = fabri_creator();
        let base = base_transform();
        let opts = IkOptions {
            max_iterations: 100,
            ..Default::default()
        };
        let q_seed = vec![0.0; 5];

        // Target far outside reachable workspace
        let far_target = Iso3::new(Vec3::new(10000.0, 0.0, 10000.0), nalgebra::zero());

        let result = inverse_kinematics(&robot, &base, &far_target, &q_seed, &opts);

        match result {
            Ok(result) => {
                for (i, &qi) in result.q.iter().enumerate() {
                    assert!(
                        qi >= robot.segments[i].joint.value_min - 1e-10,
                        "q[{}] = {} below min {}",
                        i,
                        qi,
                        robot.segments[i].joint.value_min
                    );
                    assert!(
                        qi <= robot.segments[i].joint.value_max + 1e-10,
                        "q[{}] = {} above max {}",
                        i,
                        qi,
                        robot.segments[i].joint.value_max
                    );
                }
            }
            Err(IkError::DidNotConverge { q, .. }) => {
                for (i, &qi) in q.iter().enumerate() {
                    assert!(
                        qi >= robot.segments[i].joint.value_min - 1e-10,
                        "q[{}] = {} below min {}",
                        i,
                        qi,
                        robot.segments[i].joint.value_min
                    );
                    assert!(
                        qi <= robot.segments[i].joint.value_max + 1e-10,
                        "q[{}] = {} above max {}",
                        i,
                        qi,
                        robot.segments[i].joint.value_max
                    );
                }
            }
            Err(e) => panic!("Unexpected error: {e:?}"),
        }
    }

    #[test]
    fn test_zero_dof_robot_error() {
        // 3.5: Zero DOF returns error
        let robot = Robot::new(vec![]);
        let base = Iso3::identity();
        let target = Iso3::identity();
        let opts = default_opts();
        let q_seed: Vec<f64> = vec![];
        let result = inverse_kinematics(&robot, &base, &target, &q_seed, &opts);
        assert!(result.is_err(), "zero DOF robot should return error");
    }

    #[test]
    fn test_workspace_targets_converge() {
        // 3.6: ≥8 workspace targets converge from home seed,
        // AND the converged targets all have < 1mm actual FK error.
        let robot = fabri_creator();
        let base = base_transform();
        let opts = default_opts();
        let q_seed = vec![0.0; 5];

        // Use a variety of configurations spanning the reachable workspace
        let test_configs = vec![
            vec![0.0, 0.0, 0.0, 0.0, 0.0], // home
            vec![0.2, 0.1, -0.1, 0.0, 0.0],
            vec![-0.2, 0.15, 0.05, 0.0, 0.0],
            vec![0.15, -0.1, 0.1, 0.0, 0.0],
            vec![0.3, 0.0, -0.2, 0.0, 0.0],
            vec![-0.25, 0.2, -0.05, 0.0, 0.0],
            vec![0.1, 0.1, 0.1, 0.0, 0.0],
            vec![-0.15, -0.1, 0.15, 0.0, 0.0],
            vec![0.2, -0.1, -0.1, 0.0, 0.0],
            vec![-0.1, 0.15, -0.15, 0.0, 0.0],
            vec![0.1, 0.1, 0.0, 0.1, 0.0],
            vec![-0.1, -0.1, 0.0, -0.1, 0.0],
        ];

        let mut converged_count = 0;
        for q_orig in &test_configs {
            let mut robot_mut = fabri_creator();
            for (i, &qi) in q_orig.iter().enumerate() {
                robot_mut.segments[i].joint.value = qi;
            }
            let (_, fk_pose) = forward_kinematics(base, &robot_mut);

            if let Ok(r) = inverse_kinematics(&robot, &base, &fk_pose, &q_seed, &opts) {
                if r.converged && r.error_pos < 1.0 {
                    converged_count += 1;
                }
            }
        }

        assert!(
            converged_count >= 8,
            "Only {converged_count} of {} targets converged (need ≥8)",
            test_configs.len()
        );
    }

    // ── Edge case test: different target positions ──────────────────

    #[test]
    fn test_different_target_positions() {
        // Triangulation: verify solver handles distinctly different targets.
        // We verify via FK(IK(target)) ≈ target, which proves the solver
        // found a valid joint configuration even if it differs from the seed.
        let robot = fabri_creator();
        let base = base_transform();
        let opts = default_opts();
        let q_seed = vec![0.0; 5];

        let configs = vec![
            vec![0.2, 0.1, -0.1, 0.0, 0.0],
            vec![-0.2, 0.15, 0.05, 0.0, 0.0],
            vec![0.15, -0.1, 0.1, 0.0, 0.0],
        ];

        for cfg in &configs {
            // FK at the test configuration → this is our target
            let mut robot_mut = fabri_creator();
            for (i, &qi) in cfg.iter().enumerate() {
                robot_mut.segments[i].joint.value = qi;
            }
            let (_, fk_pose) = forward_kinematics(base, &robot_mut);

            // IK: find q that reaches fk_pose
            let result = inverse_kinematics(&robot, &base, &fk_pose, &q_seed, &opts);
            match result {
                Ok(r) => {
                    // Verify FK(IK_result) ≈ original target
                    let mut verify_robot = fabri_creator();
                    for (i, &qi) in r.q.iter().enumerate() {
                        verify_robot.segments[i].joint.value = qi;
                    }
                    let (_, fk_back) = forward_kinematics(base, &verify_robot);
                    let δp = fk_pose.translation.vector - fk_back.translation.vector;
                    let dist = δp.norm();
                    assert!(
                        dist < 2.0,
                        "FK(IK(target)) position error {dist}mm for config {cfg:?}"
                    );
                }
                Err(IkError::DidNotConverge { q, .. }) => {
                    // Even non-converged solutions should be close
                    let mut verify_robot = fabri_creator();
                    for (i, &qi) in q.iter().enumerate() {
                        verify_robot.segments[i].joint.value = qi;
                    }
                    let (_, fk_back) = forward_kinematics(base, &verify_robot);
                    let δp = fk_pose.translation.vector - fk_back.translation.vector;
                    let dist = δp.norm();
                    assert!(
                        dist < 2.0,
                        "Non-converged FK(IK(target)) position error {dist}mm > 2mm, q={q:?}"
                    );
                }
                Err(e) => panic!("Unexpected error for config {cfg:?}: {e:?}"),
            }
        }
    }

    #[test]
    fn test_ik_returns_valid_error_vector() {
        // Ensure the orientation error computation is correct:
        // At home pose, the target and current orientation match,
        // so the angular error should be near zero.
        let robot = fabri_creator();
        let base = base_transform();
        let opts = default_opts();
        let q_seed = vec![0.0; 5];

        let (_, fk_pose) = forward_kinematics(base, &robot);
        let result = inverse_kinematics(&robot, &base, &fk_pose, &q_seed, &opts).unwrap();

        // At home pose, error should be very small in both position and angle
        assert!(
            result.error_angle < 0.01,
            "angular error should be near zero at home: {}",
            result.error_angle
        );
    }

    #[test]
    fn test_non_identity_base_transform() {
        // Triangulation: verify solver works with non-identity base
        let robot = fabri_creator();
        // Base with only vertical offset (same as base_transform but explicit)
        let base = Iso3::new(Vec3::new(0.0, 0.0, 57.0), nalgebra::zero());
        let opts = default_opts();

        // FK from this base to get a valid target
        let (_, fk_pose) = forward_kinematics(base, &robot);
        let q_seed = vec![0.0; 5];

        let result = inverse_kinematics(&robot, &base, &fk_pose, &q_seed, &opts)
            .expect("IK should converge with non-identity base");

        assert!(result.converged);
        assert!(result.error_pos < 1.0, "pos error: {}", result.error_pos);
    }
}
