use std::fmt;

use crate::math::{Mat4, MatDyn, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
    /// Twist joint rotates around the X axis of the previous frame
    /// (first column of R_{i-1}) rather than Z.
    Twist,
}

#[derive(Debug, Clone, PartialEq)]
pub enum JacobianError {
    EmptyChain,
    JointKindMismatch { intermediates: usize, kinds: usize },
}

impl fmt::Display for JacobianError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JacobianError::EmptyChain => write!(f, "chain cannot be empty"),
            JacobianError::JointKindMismatch {
                intermediates,
                kinds,
            } => {
                write!(
                    f,
                    "expected {kinds} joint kinds, got {intermediates} transforms"
                )
            }
        }
    }
}

impl std::error::Error for JacobianError {}

/// Returns a `6 × n` geometric Jacobian.
/// Top 3 rows = linear velocity, bottom 3 = angular velocity.
///
/// For a revolute joint `i`:   `J_i = [z_i × (p_ee − p_i); z_i]`
/// For a prismatic joint `i`:  `J_i = [z_i; 0]`
/// For a twist joint `i`:      `J_i = [0; x_i]`
///   where x_i is the first column of the rotation matrix (X axis).
///   The linear component is zero because the frame origin of a
///   Twist joint (Iso3::from_parts with constant translation)
///   does not move when the joint rotates.
pub fn geometric_jacobian(
    intermediates: &[Mat4],
    joint_kinds: &[JointKind],
    end_effector: &Mat4,
) -> Result<MatDyn, JacobianError> {
    if intermediates.is_empty() {
        return Err(JacobianError::EmptyChain);
    }
    if intermediates.len() != joint_kinds.len() {
        return Err(JacobianError::JointKindMismatch {
            intermediates: intermediates.len(),
            kinds: joint_kinds.len(),
        });
    }

    let n = intermediates.len();
    let p_ee = end_effector.fixed_view::<3, 1>(0, 3).into_owned();
    let mut jacobian = MatDyn::zeros(6, n);

    for (i, kind) in joint_kinds.iter().enumerate() {
        // Use frame BEFORE the joint (frame i-1) for the geometric Jacobian:
        //   - Revolute/Prismatic: joint i rotates about Z_{i-1}
        //   - Twist: joint i rotates about X_{i-1}
        //   - For i=0, the base frame (identity) is used.
        let (axis, p_i) = if i == 0 {
            let ax = match kind {
                JointKind::Twist => Vec3::x(),
                _ => Vec3::z(),
            };
            (ax, Vec3::zeros())
        } else {
            let prev = &intermediates[i - 1];
            let ax = match kind {
                JointKind::Twist => prev.fixed_view::<3, 1>(0, 0).into_owned(),
                _ => prev.fixed_view::<3, 1>(0, 2).into_owned(),
            };
            let p = prev.fixed_view::<3, 1>(0, 3).into_owned();
            (ax, p)
        };

        let linear = match kind {
            JointKind::Revolute => axis.cross(&(p_ee - p_i)),
            JointKind::Prismatic => axis,
            JointKind::Twist => Vec3::zeros(),
        };
        let angular = match kind {
            JointKind::Revolute | JointKind::Twist => axis,
            JointKind::Prismatic => Vec3::zeros(),
        };

        jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&linear);
        jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&angular);
    }

    Ok(jacobian)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::dh::{DHParameter, solve};
    use crate::math::{Iso3, Mat4};

    const EPS: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) {
        assert!((a - b).abs() < EPS, "expected {b}, got {a}");
    }

    fn assert_jacobian_eq(j: &MatDyn, expected: &[f64], ncols: usize) {
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), ncols);
        for col in 0..ncols {
            for row in 0..6 {
                approx_eq(j[(row, col)], expected[col * 6 + row]);
            }
        }
    }

    #[test]
    fn planar_2r_at_zero() {
        let table = vec![
            DHParameter::new(0.0, 1.0, 0.0, 0.0),
            DHParameter::new(0.0, 1.0, 0.0, 0.0),
        ];
        let sol = solve(&table);
        let j = geometric_jacobian(
            &sol.intermediates,
            &[JointKind::Revolute, JointKind::Revolute],
            &sol.final_transform,
        )
        .unwrap();
        assert_jacobian_eq(
            &j,
            &[0.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            2,
        );
    }

    #[test]
    fn planar_2r_at_j1_ninety() {
        let table = vec![
            DHParameter::new(0.0, 1.0, 0.0, std::f64::consts::FRAC_PI_2),
            DHParameter::new(0.0, 1.0, 0.0, 0.0),
        ];
        let sol = solve(&table);
        let j = geometric_jacobian(
            &sol.intermediates,
            &[JointKind::Revolute, JointKind::Revolute],
            &sol.final_transform,
        )
        .unwrap();
        assert_jacobian_eq(
            &j,
            &[-2.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            2,
        );
    }

    #[test]
    fn single_joint_at_origin() {
        let table = vec![DHParameter::new(0.0, 0.0, 0.0, 0.0)];
        let sol = solve(&table);
        let j = geometric_jacobian(
            &sol.intermediates,
            &[JointKind::Revolute],
            &sol.final_transform,
        )
        .unwrap();
        assert_jacobian_eq(&j, &[0.0, 0.0, 0.0, 0.0, 0.0, 1.0], 1);
    }

    #[test]
    fn single_prismatic_at_origin() {
        let table = vec![DHParameter::new(0.0, 0.0, 0.0, 0.0)];
        let sol = solve(&table);
        let j = geometric_jacobian(
            &sol.intermediates,
            &[JointKind::Prismatic],
            &sol.final_transform,
        )
        .unwrap();
        assert_jacobian_eq(&j, &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0], 1);
    }

    #[test]
    fn prismatic_and_revolute() {
        let table = vec![
            DHParameter::new(0.0, 0.0, 0.0, 0.0),
            DHParameter::new(0.0, 1.0, 0.0, 0.0),
        ];
        let sol = solve(&table);
        let j = geometric_jacobian(
            &sol.intermediates,
            &[JointKind::Prismatic, JointKind::Revolute],
            &sol.final_transform,
        )
        .unwrap();
        assert_jacobian_eq(
            &j,
            &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            2,
        );
    }

    #[test]
    fn fabri_creator_home_pose() {
        // Uses the actual fabri_creator() robot via forward_kinematics(),
        // with JointKind::Twist for joint 4.
        use crate::kinematics::forward::forward_kinematics;
        use crate::robot::fabri_creator::fabri_creator;

        let robot = fabri_creator();
        let base = Iso3::identity();
        let (frames, ee) = forward_kinematics(base, &robot);

        let mats: Vec<Mat4> = frames.iter().map(|iso| iso.to_matrix()).collect();
        let ee_mat = ee.to_matrix();
        let kinds = [
            JointKind::Revolute,
            JointKind::Revolute,
            JointKind::Revolute,
            JointKind::Twist,
            JointKind::Revolute,
        ];

        let j = geometric_jacobian(&mats, &kinds, &ee_mat).unwrap();
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 5);
        assert!(j.iter().all(|v| v.is_finite()));

        // Home position from forward_kinematics:
        // p_ee = (140, -15, 205) without base/tool
        let p_ee = ee_mat.fixed_view::<3, 1>(0, 3);
        assert!((p_ee[(0, 0)] - 140.0).abs() < 1e-10, "home x: {}", p_ee[(0, 0)]);
        assert!((p_ee[(1, 0)] - -15.0).abs() < 1e-10, "home y: {}", p_ee[(1, 0)]);
        assert!((p_ee[(2, 0)] - 205.0).abs() < 1e-10, "home z: {}", p_ee[(2, 0)]);

        // Verify J_ee structure at home
        // Column 1: z0 × (p_ee − p0), z0
        approx_eq(j[(0, 0)], 15.0);
        approx_eq(j[(1, 0)], 140.0);
        approx_eq(j[(2, 0)], 0.0);
        approx_eq(j[(5, 0)], 1.0);
        // Column 4 (Twist): linear=0 (frame origin does not move), angular=x3
        approx_eq(j[(0, 3)], 0.0);
        approx_eq(j[(1, 3)], 0.0);
        approx_eq(j[(2, 3)], 0.0);
        approx_eq(j[(3, 3)], 1.0);
    }

    /// Finite-difference validation of the geometric Jacobian using the
    /// actual fabri_creator() robot via forward_kinematics(). Joint 4 uses
    /// JointKind::Twist (X-axis rotation).
    #[test]
    fn fabri_creator_jacobian_finite_differences() {
        use crate::kinematics::forward::forward_kinematics;
        use crate::robot::fabri_creator::fabri_creator;

        let kinds = [
            JointKind::Revolute,
            JointKind::Revolute,
            JointKind::Revolute,
            JointKind::Twist,
            JointKind::Revolute,
        ];

        let eps = 1e-8;
        let tol = 1e-5;

        let configs: [[f64; 5]; 5] = [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [0.3, -0.5, 0.7, 0.2, -0.4],
            [-0.2, 0.4, -0.3, 0.5, 0.1],
            [0.0, 0.8, -0.6, 0.0, 0.0],
            [0.5, 0.0, 0.0, 0.3, -0.2],
        ];

        for (ci, q) in configs.iter().enumerate() {
            let mut robot = fabri_creator();
            for (seg, &qi) in robot.segments.iter_mut().zip(q.iter()) {
                seg.joint.value = qi;
            }

            let base = Iso3::identity();
            let (frames, ee) = forward_kinematics(base, &robot);
            let mats: Vec<Mat4> = frames.iter().map(|iso| iso.to_matrix()).collect();
            let ee_mat = ee.to_matrix();
            let p_ee = ee_mat.fixed_view::<3, 1>(0, 3).into_owned();
            let r_ee = ee_mat.fixed_view::<3, 3>(0, 0).into_owned();

            let j_ana = geometric_jacobian(&mats, &kinds, &ee_mat).unwrap();

            for col in 0..5 {
                let mut robot_pert = fabri_creator();
                for (k, &qk) in q.iter().enumerate() {
                    robot_pert.segments[k].joint.value = if k == col { qk + eps } else { qk };
                }
                let (_frames_pert, ee_pert) = forward_kinematics(base, &robot_pert);
                let ee_pert_mat = ee_pert.to_matrix();
                let p_pert = ee_pert_mat.fixed_view::<3, 1>(0, 3).into_owned();

                // Linear velocity
                let dp = (p_pert - p_ee) / eps;
                for row in 0..3 {
                    let num = dp[row];
                    let ana = j_ana[(row, col)];
                    assert!(
                        (num - ana).abs() < tol,
                        "config {ci}, col {col}, linear row {row}: \
                         numerical = {num:.12e}, analytical = {ana:.12e}, diff = {}",
                        (num - ana).abs()
                    );
                }

                // Angular velocity
                let r_pert = ee_pert_mat.fixed_view::<3, 3>(0, 0).into_owned();
                let r_rel = r_pert * r_ee.transpose();
                let wx = (r_rel[(2, 1)] - r_rel[(1, 2)]) / (2.0 * eps);
                let wy = (r_rel[(0, 2)] - r_rel[(2, 0)]) / (2.0 * eps);
                let wz = (r_rel[(1, 0)] - r_rel[(0, 1)]) / (2.0 * eps);

                for row in 0..3 {
                    let num = [wx, wy, wz][row];
                    let ana = j_ana[(3 + row, col)];
                    assert!(
                        (num - ana).abs() < tol,
                        "config {ci}, col {col}, angular row {row}: \
                         numerical = {num:.12e}, analytical = {ana:.12e}, diff = {}",
                        (num - ana).abs()
                    );
                }
            }
        }
    }

    #[test]
    fn error_on_empty_chain() {
        let result = geometric_jacobian(&[], &[], &Mat4::identity());
        assert_eq!(result, Err(JacobianError::EmptyChain));
    }

    #[test]
    fn error_on_mismatched_lengths() {
        let table = vec![DHParameter::new(0.0, 1.0, 0.0, 0.0)];
        let sol = solve(&table);
        let result = geometric_jacobian(
            &sol.intermediates,
            &[JointKind::Revolute, JointKind::Revolute],
            &sol.final_transform,
        );
        assert_eq!(
            result,
            Err(JacobianError::JointKindMismatch {
                intermediates: 1,
                kinds: 2
            })
        );
    }
}
