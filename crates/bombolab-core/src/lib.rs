pub mod kinematics;
pub mod math;
pub mod robot;

pub use kinematics::{
    DHParameter, DHSolution, compute_a_matrix, forward_kinematics, matrix_from_segment, solve,
};
pub use math::{DEG_TO_RAD, Iso3, PI, RAD_TO_DEG};
pub use robot::{DHParams, Error, Joint, JointType, Result, Robot, Segment};
