pub mod dh;
pub mod forward;
pub mod ik;
pub mod init;
pub mod pose_generator;
pub mod singularity;
pub mod workspace;

pub use dh::{DHParameter, DHSolution, compute_a_matrix, solve};
pub use forward::{forward_kinematics, matrix_from_segment};
pub use ik::{
    DrawingConfiguration, IkError, IkSolver, OrientationError, OrientationSolver, solve_drawing_ik,
    solve_drawing_ik_v2, solve_drawing_plane_ik, solve_full_ik,
};
pub use pose_generator::{PoseGenerator, TargetPose};
pub use singularity::{
    GateReason, GateReport, GateWaypoint, SingularityLevel, SingularityThresholds, WaypointMetrics,
    analyze_path, classify, reduced_jacobian, waypoint_metrics,
};
pub use workspace::{WorkspaceMode, WorkspaceSampler, WorkspaceStats};
