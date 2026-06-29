use crate::math::{Iso3, UnitVec, Rot3, Vec3};

pub struct Movement {
    pub translation: Vec3,
    pub angles: f64,
    pub axis: Vec3,
    pub isometry: bool,
}

// R T  =  T * R
// 0 1
pub fn rotation_and_translation( axis: Vec3, angle: f64, translation: Vec3) -> Iso3 {
    let axis_unit = UnitVec::new_normalize(axis);
    let axisangle = axis_unit.as_ref() * angle;
    Iso3::new(translation, axisangle)
}

// R RT  =  R * T
// 0 1
pub fn translation_and_rotation(axis: Vec3, angle: f64, translation: Vec3) -> Iso3 {
    let axis_unit = UnitVec::new_normalize(axis);
    let rotation = Rot3::from_axis_angle(&axis_unit, angle);
    Iso3::new(rotation * translation, Vec3::zeros())
}

pub fn make_movement(initial: Iso3, movements: &[Movement]) -> (Vec<Iso3>, Iso3) {
    let mut trajectory: Vec<Iso3> = Vec::new();
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
