use std::fmt;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    JointCountMismatch { expected: usize, got: usize },
    IndexOutOfBounds { index: usize, len: usize },
    JointValueOutOfLimits { value: f64, min: f64, max: f64 },
}

#[derive(Debug, Clone, PartialEq)]
pub enum MathError {
    ZeroVectorNormalization,
    ZeroQuaternionNormalization,
    ZeroQuaternionInverse { norm_sq: f64 },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::JointCountMismatch { expected, got } => {
                write!(f, "expected {} joints, got {}", expected, got)
            }
            Error::IndexOutOfBounds { index, len } => {
                write!(
                    f,
                    "index {} out of bounds, robot has {} segments",
                    index, len
                )
            }
            Error::JointValueOutOfLimits { value, min, max } => {
                write!(f, "value {} out of limits [{}, {}]", value, min, max)
            }
        }
    }
}

impl std::error::Error for Error {}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MathError::ZeroVectorNormalization => {
                write!(f, "cannot normalize a zero-length vector")
            }
            MathError::ZeroQuaternionNormalization => {
                write!(f, "cannot normalize a zero quaternion")
            }
            MathError::ZeroQuaternionInverse { norm_sq } => {
                write!(
                    f,
                    "cannot invert a zero quaternion (norm² = {})",
                    norm_sq
                )
            }
        }
    }
}

impl std::error::Error for MathError {}
