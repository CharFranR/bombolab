pub mod communication;
pub mod kinematics;
pub mod math;
pub mod robot;

pub use communication::{ServoCommand, ServoMapper};
pub use kinematics::{
    compute_a_matrix, forward_kinematics, matrix_from_segment, solve, DHParameter, DHSolution,
    IkError, IkSolver,
};
pub use math::{DEG_TO_RAD, Iso3, PI, RAD_TO_DEG};
pub use robot::{base_transform, fabri_creator, tool_transform, DHParams, Error, Joint, JointType, Result, Robot, Segment};
