pub mod constants;
pub mod dh;
pub mod quaternion;

pub use constants::{DEG_TO_RAD, EPS, FRAC_PI_2, FRAC_PI_4, PI, RAD_TO_DEG, TAU};
pub use dh::{DHParameter, DHSolution, compute_a_matrix, solve};
pub use quaternion::{Quaternion, solve_add, solve_subtract, solve_multiply, solve_divide};
