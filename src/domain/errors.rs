use std::fmt;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    JointCountMismatch { expected: usize, got: usize },
    IndexOutOfBounds { index: usize, len: usize },
    JointValueOutOfLimits { value: f64, min: f64, max: f64 },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::JointCountMismatch { expected, got } => {
                write!(f, "se esperaban {} joints, se recibieron {}", expected, got)
            }
            Error::IndexOutOfBounds { index, len } => {
                write!(
                    f,
                    "índice {} fuera de rango, el robot tiene {} segments",
                    index, len
                )
            }
            Error::JointValueOutOfLimits { value, min, max } => {
                write!(f, "valor {} fuera de los límites [{}, {}]", value, min, max)
            }
        }
    }
}

impl std::error::Error for Error {}
