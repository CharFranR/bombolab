/// Pi constant (alias for std::f64::consts::PI)
pub const PI: f64 = std::f64::consts::PI;

/// Degrees to radians conversion factor
pub const DEG_TO_RAD: f64 = PI / 180.0;

/// Radians to degrees conversion factor
pub const RAD_TO_DEG: f64 = 180.0 / PI;

/// Half pi (PI / 2)
pub const FRAC_PI_2: f64 = std::f64::consts::FRAC_PI_2;

/// Quarter pi (PI / 4)
pub const FRAC_PI_4: f64 = std::f64::consts::FRAC_PI_4;

/// Machine epsilon for f64 comparisons
pub const EPS: f64 = f64::EPSILON;

/// Two times pi
pub const TAU: f64 = std::f64::consts::TAU;
