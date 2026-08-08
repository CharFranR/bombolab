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
#[path = "jacobian_tests.rs"]
mod jacobian_tests;
