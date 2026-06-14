use core::f32;

use nalgebra::{Isometry, Isometry3, Rotation3, Unit, Vector3};

pub struct Movement {
    pub traslation: Vector3<f64>,
    pub angles: f64,
    pub axis: Vector3<f64>,
    pub isommetry: bool
}

pub fn rotation_and_traslation (axis: Vector3<f64>, angle: f64, translation: Vector3<f64>) -> Isometry3<f64>{

    let axis_unit = Unit::new_normalize(axis);  
    let axisangle = axis_unit.as_ref() * angle;  
    Isometry3::new(translation, axisangle)

}

pub fn translation_and_rotation (axis: Vector3<f64>, angle: f64, translation: Vector3<f64>) -> Isometry3<f64> {
    let axis_unit = Unit::new_normalize(axis); 
    let axisangle = axis_unit.as_ref() * angle;
    let rotation = Rotation3::from_axis_angle(&axis_unit, angle);

    Isometry3::new(rotation * translation, axisangle)
}

pub fn make_movement(movements: Vec<Movement>){

    let mut  matrix_order: Vec<Isometry3<f64>> = Vec::new();
    let mut final_matrix = Isometry3::identity();

    for movement in movements.iter() {
        if movement.isommetry == true {
            let result = rotation_and_traslation (movement.axis, movement.angles, movement.traslation);
            matrix_order.push(result);
            final_matrix = final_matrix * result;
        }

        if movement.isommetry == false {
            let result = translation_and_rotation (movement.axis, movement.angles, movement.traslation);
            matrix_order.push(result);
            final_matrix = final_matrix * result;
        }
    }

}