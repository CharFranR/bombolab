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
    use crate::math::FRAC_PI_2;

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
        // FABRI Creator DH table (corregida: a₄=35) — misma configuración
        // que `fabri_creator_jacobian_finite_differences` en home.
        let table = vec![
            DHParameter::new(-FRAC_PI_2, 15.0, 95.0, 0.0),
            DHParameter::new(0.0, 0.0, 162.0, 0.0),
            DHParameter::new(-FRAC_PI_2, 111.0, 0.0, 0.0),
            DHParameter::new(FRAC_PI_2, 35.0, 0.0, 0.0),
            DHParameter::new(0.0, 0.0, 0.0, 0.0),
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

        // Verify home-pose FK position (sin base transform: empieza en origen).
        // Arm extends along +X (15+111+35=161), offset Y from d₂=162, Z from d₁=95.
        let p = sol.translation();
        assert!((p.x - 161.0).abs() < 1e-10, "home x: {}", p.x);
        assert!((p.y - 162.0).abs() < 1e-10, "home y: {}", p.y);
        assert!((p.z - 95.0).abs() < 1e-10, "home z: {}", p.z);
    }

    /// Finite-difference validation of the geometric Jacobian.
    ///
    /// For each column i, perturbs θ_i by ε and computes the numerical Jacobian
    /// as (FK(q + ε·e_i) - FK(q)) / ε. Compares each element against the
    /// analytical `geometric_jacobian`. Runs at home pose and several arbitrary
    /// configurations.
    ///
    /// Uses the corrected FABRI Creator DH table (a₄=35).
    #[test]
    fn fabri_creator_jacobian_finite_differences() {

        // FABRI Creator: Standard DH (corrected: a₄=35)
        // DHParameter::new(alpha, a, d, theta)
        let make_table = |q: &[f64; 5]| -> Vec<DHParameter> {
            vec![
                DHParameter::new(-FRAC_PI_2, 15.0, 95.0, q[0]),
                DHParameter::new(0.0, 0.0, 162.0, q[1]),
                DHParameter::new(-FRAC_PI_2, 111.0, 0.0, q[2]),
                DHParameter::new(FRAC_PI_2, 35.0, 0.0, q[3]),
                DHParameter::new(0.0, 0.0, 0.0, q[4]),
            ]
        };

        let eps = 1e-8;
        // Forward-difference tolerance: O(ε) truncation + O(δ/ε) rounding.
        // Link lengths up to 162mm → linear J entries O(100),
        // so truncation ~ O(100·ε) ≈ 1e-6, tol = 1e-5 is safe.
        let tol = 1e-5;

        // Test configurations: home + 4 arbitrary poses
        let configs: [[f64; 5]; 5] = [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [0.3, -0.5, 0.7, 0.2, -0.4],
            [-0.2, 0.4, -0.3, 0.5, 0.1],
            [0.0, 0.8, -0.6, 0.0, 0.0],
            [0.5, 0.0, 0.0, 0.3, -0.2],
        ];

        for (ci, q) in configs.iter().enumerate() {
            let table = make_table(q);
            let sol = solve(&table);
            let p_ee = sol.translation();
            let r_ee = sol.rotation();

            let j_ana = geometric_jacobian(
                &sol.intermediates,
                &[JointKind::Revolute; 5],
                &sol.final_transform,
            )
            .unwrap();

            for col in 0..5 {
                let mut q_pert = *q;
                q_pert[col] += eps;
                let tab_pert = make_table(&q_pert);
                let sol_pert = solve(&tab_pert);

                // --- Linear velocity: (p_pert - p_ee) / ε ---
                let dp = (sol_pert.translation() - p_ee) / eps;

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

                // --- Angular velocity: extract ω from ΔR = R_pert · R_eeᵀ ---
                // For small ε, ΔR ≈ I + [ω]× · ε, so ω · ε ≈ skew(ΔR)
                let r_rel = sol_pert.rotation() * r_ee.transpose();
                let wx = (r_rel[(2, 1)] - r_rel[(1, 2)]) / (2.0 * eps);
                let wy = (r_rel[(0, 2)] - r_rel[(2, 0)]) / (2.0 * eps);
                let wz = (r_rel[(1, 0)] - r_rel[(0, 1)]) / (2.0 * eps);
                let omega = Vec3::new(wx, wy, wz);

                for row in 0..3 {
                    let num = omega[row];
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
