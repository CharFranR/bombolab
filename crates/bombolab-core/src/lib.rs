pub mod kinematics;
pub mod math;
pub mod robot;
pub mod trajectory;

pub use kinematics::{
    DHParameter, DHSolution, GateReason, GateReport, GateWaypoint, IkError, IkSolver,
    OrientationError, OrientationSolver, PoseGenerator, SingularityLevel, SingularityThresholds,
    TargetPose, WaypointMetrics, WorkspaceMode, WorkspaceSampler, WorkspaceStats, analyze_path,
    classify, compute_a_matrix, forward_kinematics, matrix_from_segment, pose_generator,
    reduced_jacobian, solve, solve_drawing_ik, solve_drawing_ik_v2, solve_drawing_plane_ik,
    solve_full_ik, waypoint_metrics,
};
pub use math::{
    DEG_TO_RAD, DualQuaternion, Iso3, LinkParams, PI, RAD_TO_DEG, Rot3, Xoshiro256StarStar,
    gravity_vector, inertia_matrix,
};
pub use robot::{
    DHParams, Error, Joint, JointType, Result, Robot, Segment, ToolFrame, base_transform,
    fabri_creator,
};
pub use trajectory::{MAX_DT, MotionCommand, MotionPlayer, PlayerState, TrajectoryPlanner};
