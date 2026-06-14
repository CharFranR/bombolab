use nalgebra::{Isometry3, Rotation3, Translation3, UnitQuaternion, Vector3};

use crate::domain::Segment;
use crate::domain::Robot;

pub fn matrix_from_segment(segment: &Segment) -> Isometry3<f64> {
    let (theta, d, a, alpha) = segment.dh_params();

    let rot_z = Rotation3::from_axis_angle(&Vector3::z_axis(), theta);
    let rot_x = Rotation3::from_axis_angle(&Vector3::x_axis(), alpha);
    let rotation = UnitQuaternion::from_rotation_matrix(&(rot_z * rot_x));
    let translation = Translation3::new(a * theta.cos(), a * theta.sin(), d);

    Isometry3::from_parts(translation, rotation)
}


pub fn forward_kinematics(base: Isometry3<f64>, robot: &Robot) -> (Vec<Isometry3<f64>>, Isometry3<f64>) {
    let mut frames = Vec::new();
    let mut current = base;

    for segment in &robot.segments {
        current = current * matrix_from_segment(segment);
        frames.push(current);
    }

    (frames, current)
}