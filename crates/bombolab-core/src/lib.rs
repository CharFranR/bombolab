#[cfg(feature = "serial")]
pub mod communication;

pub mod kinematics;
pub mod math;
pub mod robot;
pub mod trajectory;

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
#[allow(deprecated)]
pub use robot::tool_transform;
pub use robot::{
    DHParams, Error, Joint, JointType, Result, Robot, Segment, ToolFrame, base_transform,
    fabri_creator,
};
pub use trajectory::{MAX_DT, MotionCommand, MotionPlayer, PlayerState, TrajectoryPlanner};
