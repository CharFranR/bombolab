#[cfg(feature = "serial")]
pub mod communication;

pub mod kinematics;
pub mod math;
pub mod robot;

#[cfg(feature = "serial")]
pub use communication::{ServoCommand, ServoMapper};
pub use kinematics::{
    DHParameter, DHSolution, IkError, IkSolver, OrientationError, OrientationSolver, PoseGenerator,
    TargetPose, compute_a_matrix, forward_kinematics, matrix_from_segment, pose_generator, solve,
    solve_drawing_ik, solve_drawing_ik_v2, solve_full_ik,
};
pub use math::{
    DEG_TO_RAD, DualQuaternion, Iso3, LinkParams, PI, RAD_TO_DEG, Rot3, gravity_vector,
    inertia_matrix,
};
pub use robot::{
    DHParams, Error, Joint, JointType, Result, Robot, Segment, base_transform, fabri_creator,
    tool_transform,
};
