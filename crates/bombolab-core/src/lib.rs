#[cfg(feature = "serial")]
pub mod communication;

pub mod kinematics;
pub mod math;
pub mod robot;

#[cfg(feature = "serial")]
pub use communication::{ServoCommand, ServoMapper};
pub use kinematics::{
    compute_a_matrix, forward_kinematics, matrix_from_segment, pose_generator, solve, solve_drawing_ik,
    solve_drawing_ik_v2, solve_full_ik, DHParameter, DHSolution, IkError, IkSolver, OrientationError,
    OrientationSolver, PoseGenerator, TargetPose,
};
pub use math::{DEG_TO_RAD, Iso3, PI, RAD_TO_DEG, Rot3};
pub use robot::{base_transform, fabri_creator, tool_transform, DHParams, Error, Joint, JointType, Result, Robot, Segment};
