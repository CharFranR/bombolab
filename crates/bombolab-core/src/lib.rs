pub mod kinematics;
pub mod math;
pub mod robot;

pub use kinematics::{forward_kinematics, matrix_from_segment};
pub use math::{
    DEG_TO_RAD, DHParameter, DHSolution, Iso3, PI, RAD_TO_DEG, compute_a_matrix, solve,
};
pub use robot::{DHParams, Error, Joint, JointType, Result, Robot, Segment};
