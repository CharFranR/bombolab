use crate::math::Rot3;

pub struct TargetPose {
    pub position: [f64; 3],
    pub rotation: Rot3,
}

pub struct PoseGenerator;

impl PoseGenerator {
    pub fn drawing_pose(position: [f64; 3]) -> TargetPose {
        TargetPose {
            position,

            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0,
            )),
        }
    }

    pub fn drawing_pose_adaptive(position: [f64; 3], q1: f64) -> TargetPose {
        let (sq1, cq1) = q1.sin_cos();
        TargetPose {
            position,
            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                0.0, -cq1, -sq1, 0.0, -sq1, cq1, -1.0, 0.0, 0.0,
            )),
        }
    }

    pub fn drawing_pose_v2(position: [f64; 3], q1: f64) -> TargetPose {
        let (sq1, cq1) = q1.sin_cos();
        TargetPose {
            position,
            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                cq1, 0.0, -sq1, sq1, 0.0, cq1, 0.0, -1.0, 0.0,
            )),
        }
    }
}

#[cfg(test)]
#[path = "pose_generator_tests.rs"]
mod pose_generator_tests;
