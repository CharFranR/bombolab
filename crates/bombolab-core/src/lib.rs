pub mod kinematics;
pub mod math;
pub mod robot;

pub use kinematics::{forward_kinematics, matrix_from_segment};
pub use math::{DHParameter, DHSolution, DEG_TO_RAD, PI, RAD_TO_DEG, compute_a_matrix, solve};
pub use nalgebra::Isometry3;
pub use robot::{DHParams, Error, Joint, JointType, Result, Robot, Segment};
