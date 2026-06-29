pub mod constants;
pub mod dh;
pub mod quaternion;
pub mod isometry;

pub use constants::{DEG_TO_RAD, EPS, FRAC_PI_2, FRAC_PI_4, PI, RAD_TO_DEG, TAU};
pub use dh::{DHParameter, DHSolution, compute_a_matrix, solve};
pub use quaternion::{Quaternion, solve_add, solve_subtract, solve_multiply, solve_divide};
pub use isometry::{Movement, rotation_and_translation, translation_and_rotation, make_movement};

// Alias que apuntan a nalgebra
pub type Vec3 = nalgebra::Vector3<f64>;
pub type Tras = nalgebra::Translation3<f64>;
pub type Iso3 = nalgebra::Isometry3<f64>;
pub type Rot3 = nalgebra::Rotation3<f64>;
pub type UnitVec = nalgebra::Unit<nalgebra::Vector3<f64>>;
pub type Quat = nalgebra::UnitQuaternion<f64>;
pub type Mat3 = nalgebra::Matrix3<f64>;
pub type Mat4 = nalgebra::Matrix4<f64>;