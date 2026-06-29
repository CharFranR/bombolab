pub mod kinematics;
pub mod math;
pub mod robot;

pub use kinematics::{
    DHParameter, DHSolution, DHSolver, GeometricLink, compute_a_matrix, forward_kinematics,
    generate_dh_table, matrix_from_segment,
};
pub use math::{DEG_TO_RAD, PI, RAD_TO_DEG};
pub use nalgebra::Isometry3;
pub use robot::{DHParams, Error, Joint, JointType, MathError, Result, Robot, Segment};
