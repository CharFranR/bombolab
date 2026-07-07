pub mod communication;
pub mod kinematics;
pub mod math;
pub mod robot;

pub use kinematics::{
    compute_a_matrix, forward_kinematics, inverse_kinematics, matrix_from_segment, solve,
    DHParameter, DHSolution, IkError, IkOptions, IkResult,
};
pub use math::{DEG_TO_RAD, Iso3, PI, RAD_TO_DEG};
pub use robot::fabri_creator::{base_transform, fabri_creator, tool_transform};
pub use robot::{DHParams, Error, Joint, JointType, Result, Robot, Segment};
