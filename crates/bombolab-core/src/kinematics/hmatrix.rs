use nalgebra::{Isometry3, Rotation3, Unit, Vector3};

/// Describes an end-effector movement: rotation around an axis + translation.
pub struct Movement {
    pub translation: Vector3<f64>,
    pub angles: f64,
    pub axis: Vector3<f64>,
    pub isometry: bool,
}

pub fn rotation_and_translation(
    axis: Vector3<f64>,
    angle: f64,
    translation: Vector3<f64>,
) -> Isometry3<f64> {
    let axis_unit = Unit::new_normalize(axis);
    let axisangle = axis_unit.as_ref() * angle;
    Isometry3::new(translation, axisangle)
}

pub fn translation_and_rotation(
    axis: Vector3<f64>,
    angle: f64,
    translation: Vector3<f64>,
) -> Isometry3<f64> {
    let axis_unit = Unit::new_normalize(axis);
    let axisangle = axis_unit.as_ref() * angle;
    let rotation = Rotation3::from_axis_angle(&axis_unit, angle);
    Isometry3::new(rotation * translation, axisangle)
}

pub fn make_movement(
    initial: Isometry3<f64>,
    movements: &[Movement],
) -> (Vec<Isometry3<f64>>, Isometry3<f64>) {
    let mut trajectory: Vec<Isometry3<f64>> = Vec::new();
    let mut current = initial;

    for movement in movements {
        let step = if movement.isometry {
            rotation_and_translation(movement.axis, movement.angles, movement.translation)
        } else {
            translation_and_rotation(movement.axis, movement.angles, movement.translation)
        };

        current = step * current;
        trajectory.push(current);
    }

    (trajectory, current)
}
