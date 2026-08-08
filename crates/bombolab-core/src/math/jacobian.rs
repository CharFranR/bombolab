use std::fmt;

use crate::math::{Mat4, MatDyn, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
    
    
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
            JointKind::Twist => {
                
                
                
                
                
                
                let p_twist = intermediates[i].fixed_view::<3, 1>(0, 3).into_owned();
                axis.cross(&(p_ee - p_twist))
            }
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

        
        
        let p_ee = ee_mat.fixed_view::<3, 1>(0, 3);
        assert!(
            (p_ee[(0, 0)] - 140.0).abs() < 1e-10,
            "home x: {}",
            p_ee[(0, 0)]
        );
        assert!(
            (p_ee[(1, 0)] - -15.0).abs() < 1e-10,
            "home y: {}",
            p_ee[(1, 0)]
        );
        assert!(
            (p_ee[(2, 0)] - 205.0).abs() < 1e-10,
            "home z: {}",
            p_ee[(2, 0)]
        );

        
        
        approx_eq(j[(0, 0)], 15.0);
        approx_eq(j[(1, 0)], 140.0);
        approx_eq(j[(2, 0)], 0.0);
        approx_eq(j[(5, 0)], 1.0);
        
        approx_eq(j[(0, 3)], 0.0);
        approx_eq(j[(1, 3)], 0.0);
        approx_eq(j[(2, 3)], 0.0);
        approx_eq(j[(3, 3)], 1.0);
    }

    
    
    
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
    fn twist_with_non_axial_offset_finite_differences() {
        use crate::kinematics::forward::forward_kinematics;
        use crate::robot::{DHParams, Joint, JointType, Robot, Segment};

        
        
        
        
        let make_robot = |q1: f64, q2: f64| {
            Robot::new(vec![
                Segment::new(
                    Joint::new(JointType::Twist, q1, 2.0, -2.0),
                    DHParams::new(0.0, 0.0, 0.0, 0.0),
                ),
                Segment::new(
                    Joint::new(JointType::Revolute, q2, 2.0, -2.0),
                    DHParams::new(0.0, 50.0, 0.0, 0.0),
                ),
            ])
        };
        let robot = make_robot(0.3, -0.5);

        let base = Iso3::identity();
        let (frames, ee) = forward_kinematics(base, &robot);
        let mats: Vec<Mat4> = frames.iter().map(|iso| iso.to_matrix()).collect();
        let ee_mat = ee.to_matrix();
        let kinds = [JointKind::Twist, JointKind::Revolute];

        let j_ana = geometric_jacobian(&mats, &kinds, &ee_mat).unwrap();

        
        let twist_lin = j_ana.fixed_view::<3, 1>(0, 0).into_owned();
        assert!(
            twist_lin.norm() > 1.0,
            "twist linear column must be non-zero for off-axis TCP, got {twist_lin:?}"
        );

        
        let eps = 1e-8;
        let tol = 1e-5;
        let p_ee = ee_mat.fixed_view::<3, 1>(0, 3).into_owned();
        let r_ee = ee_mat.fixed_view::<3, 3>(0, 0).into_owned();

        let mut q = [0.3, -0.5];
        for col in 0..2 {
            q[col] += eps;
            let robot_pert = make_robot(q[0], q[1]);
            let (_fp, ee_p) = forward_kinematics(base, &robot_pert);
            let ee_p_mat = ee_p.to_matrix();
            let p_plus = ee_p_mat.fixed_view::<3, 1>(0, 3).into_owned();
            let r_plus = ee_p_mat.fixed_view::<3, 3>(0, 0).into_owned();

            q[col] -= 2.0 * eps;
            let robot_pert = make_robot(q[0], q[1]);
            let (_fm, ee_m) = forward_kinematics(base, &robot_pert);
            let ee_m_mat = ee_m.to_matrix();
            let p_minus = ee_m_mat.fixed_view::<3, 1>(0, 3).into_owned();
            let r_minus = ee_m_mat.fixed_view::<3, 3>(0, 0).into_owned();
            q[col] += eps;

            
            let dp = (p_plus - p_minus) / (2.0 * eps);
            for row in 0..3 {
                let num = dp[row];
                let ana = j_ana[(row, col)];
                assert!(
                    (num - ana).abs() < tol,
                    "col {col}, linear row {row}: numerical = {num:.6e}, \
                     analytical = {ana:.6e}",
                );
            }

            
            let r_rel_p = r_plus * r_ee.transpose();
            let wx_p = (r_rel_p[(2, 1)] - r_rel_p[(1, 2)]) / 2.0;
            let wy_p = (r_rel_p[(0, 2)] - r_rel_p[(2, 0)]) / 2.0;
            let wz_p = (r_rel_p[(1, 0)] - r_rel_p[(0, 1)]) / 2.0;
            let r_rel_m = r_minus * r_ee.transpose();
            let wx_m = (r_rel_m[(2, 1)] - r_rel_m[(1, 2)]) / 2.0;
            let wy_m = (r_rel_m[(0, 2)] - r_rel_m[(2, 0)]) / 2.0;
            let wz_m = (r_rel_m[(1, 0)] - r_rel_m[(0, 1)]) / 2.0;

            let domega = [
                (wx_p - wx_m) / (2.0 * eps),
                (wy_p - wy_m) / (2.0 * eps),
                (wz_p - wz_m) / (2.0 * eps),
            ];
            for row in 0..3 {
                let num = domega[row];
                let ana = j_ana[(3 + row, col)];
                assert!(
                    (num - ana).abs() < tol,
                    "col {col}, angular row {row}: numerical = {num:.6e}, \
                     analytical = {ana:.6e}",
                );
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
