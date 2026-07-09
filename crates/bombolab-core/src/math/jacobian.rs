use std::fmt;

use crate::math::{Mat4, MatDyn, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
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
/// For a revolute joint `i`: `J_i = [z_i × (p_ee − p_i); z_i]`
/// For a prismatic joint `i`: `J_i = [z_i; 0]`
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
        //   - Joint i rotates about Z_{i-1}
        //   - For i=0, the base frame (identity) is used.
        let (z_i, p_i) = if i == 0 {
            (Vec3::z(), Vec3::zeros())
        } else {
            let prev = &intermediates[i - 1];
            (
                prev.fixed_view::<3, 1>(0, 2).into_owned(),
                prev.fixed_view::<3, 1>(0, 3).into_owned(),
            )
        };

        let linear = match kind {
            JointKind::Revolute => z_i.cross(&(p_ee - p_i)),
            JointKind::Prismatic => z_i,
        };
        let angular = match kind {
            JointKind::Revolute => z_i,
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
        let table = vec![
            DHParameter::new(std::f64::consts::FRAC_PI_2, 15.0, 68.5, 0.0),
            DHParameter::new(0.0, 0.0, 162.0, std::f64::consts::FRAC_PI_2),
            DHParameter::new(
                std::f64::consts::FRAC_PI_2,
                0.0,
                0.0,
                std::f64::consts::FRAC_PI_2,
            ),
            DHParameter::new(-std::f64::consts::FRAC_PI_2, 0.0, 155.0, 0.0),
            DHParameter::new(std::f64::consts::FRAC_PI_2, 35.0, 0.0, 0.0),
        ];
        let sol = solve(&table);
        let j = geometric_jacobian(
            &sol.intermediates,
            &[JointKind::Revolute; 5],
            &sol.final_transform,
        )
        .unwrap();
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 5);
        assert!(j.iter().all(|v| v.is_finite()));
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
