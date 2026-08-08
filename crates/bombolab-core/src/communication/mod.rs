//! | Pin | Servo | Joint   | Notes        |
//! |-----|-------|---------|--------------|
//! | A1  | S1    | J1      | Base yaw     |
//! | A0  | S2    | J2      | Shoulder     |
//! | A2  | S3    | J3      | Elbow        |
//! | A4  | S4    | J4      | Wrist roll   |
//! | 13  | S5    | J5      | Wrist pitch  |
//! | A5  | S6    | Gripper | —            |

#[cfg(feature = "serial")]
pub mod arduino_nano;
pub mod command;
pub mod interpolation;
pub mod mapper;

use std::fmt;

#[cfg(feature = "serial")]
pub use arduino_nano::ArduinoNano;
pub use command::ServoCommand;
pub use interpolation::{
    InterpolationConfig, interpolate_all, interpolate_all_command, interpolate_joint,
};
pub use mapper::ServoMapper;

pub const BAUD_RATE: u32 = 115_200;

pub const JOINT_COUNT: usize = 6;

pub const ANGLE_MIN: i32 = 5;

pub const ANGLE_MAX: i32 = 175;

pub const READ_TIMEOUT_MS: u64 = 1000;

#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionError {
    PortNotFound { port: String },
    OpenFailed { port: String, source: String },
    WriteFailed { port: String, source: String },
    ReadFailed { port: String, source: String },
    Timeout { port: String, ms: u64 },
    InvalidResponse { port: String, response: String },
}

impl fmt::Display for ConnectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConnectionError::PortNotFound { port } => {
                write!(f, "port not found: {}", port)
            }
            ConnectionError::OpenFailed { port, source } => {
                write!(f, "failed to open {}: {}", port, source)
            }
            ConnectionError::WriteFailed { port, source } => {
                write!(f, "write to {} failed: {}", port, source)
            }
            ConnectionError::ReadFailed { port, source } => {
                write!(f, "read from {} failed: {}", port, source)
            }
            ConnectionError::Timeout { port, ms } => {
                write!(f, "read timeout on {} after {}ms", port, ms)
            }
            ConnectionError::InvalidResponse { port, response } => {
                write!(f, "invalid response from {}: {:?}", port, response)
            }
        }
    }
}

impl std::error::Error for ConnectionError {}

#[cfg(test)]
mod tests;
