use crate::math::{Iso3, Quat, Rot3, Tras, Vec3};

use crate::robot::{JointType, Robot, Segment};

pub fn matrix_from_segment(segment: &Segment) -> Iso3 {
    match segment.joint.joint_type {
        JointType::Twist => {
            let (_, d, a, alpha) = segment.dh_params();
            let rot_x = Rot3::from_axis_angle(&Vec3::x_axis(), alpha);
            let rotation = Quat::from_rotation_matrix(&rot_x);
            let translation = Tras::new(a, d, 0.0);
            Iso3::from_parts(translation, rotation)
        }
        JointType::Revolute | JointType::Prismatic => {
            let (theta, d, a, alpha) = segment.dh_params();
            let rot_z = Rot3::from_axis_angle(&Vec3::z_axis(), theta);
            let rot_x = Rot3::from_axis_angle(&Vec3::x_axis(), alpha);
            let rotation = Quat::from_rotation_matrix(&(rot_z * rot_x));
            let translation = Tras::new(a * theta.cos(), a * theta.sin(), d);
            Iso3::from_parts(translation, rotation)
        }
    }
}

pub fn forward_kinematics(base: Iso3, robot: &Robot) -> (Vec<Iso3>, Iso3) {
    let mut frames = Vec::new();
    let mut current = base;

    for segment in &robot.segments {
        current *= matrix_from_segment(segment);
        frames.push(current);
    }

    (frames, current)
}

#[cfg(test)]
#[path = "forward_tests.rs"]
mod forward_tests;
