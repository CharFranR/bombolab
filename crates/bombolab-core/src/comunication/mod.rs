pub mod arduino_nano;
pub mod interpolation;

use std::fmt;

pub use arduino_nano::ArduinoNano;
pub use interpolation::{InterpolationConfig, interpolate_all, interpolate_joint};

pub const BAUD_RATE: u32 = 115_200;
pub const JOINT_COUNT: usize = 6;
pub const ANGLE_MIN: i32 = 10;
pub const ANGLE_MAX: i32 = 170;
pub const READ_TIMEOUT_MS: u64 = 1000;

#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionError {
    PortNotFound { port: String },
    OpenFailed { port: String, source: String },
    WriteFailed { port: String, source: String },
    ReadFailed { port: String, source: String },
    Timeout { port: String, ms: u64 },
    InvalidResponse { port: String, response: String },
    AnglesOutOfRange { values: Vec<i32>, min: i32, max: i32 },
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
            ConnectionError::AnglesOutOfRange { values, min, max } => {
                write!(
                    f,
                    "angles {:?} out of range [{}, {}]",
                    values, min, max
                )
            }
        }
    }
}

impl std::error::Error for ConnectionError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_port_not_found() {
        let err = ConnectionError::PortNotFound {
            port: "/dev/ttyUSB0".into(),
        };
        assert_eq!(err.to_string(), "port not found: /dev/ttyUSB0");
    }

    #[test]
    fn display_open_failed() {
        let err = ConnectionError::OpenFailed {
            port: "COM3".into(),
            source: "permission denied".into(),
        };
        assert_eq!(err.to_string(), "failed to open COM3: permission denied");
    }

    #[test]
    fn display_write_failed() {
        let err = ConnectionError::WriteFailed {
            port: "/dev/ttyACM0".into(),
            source: "broken pipe".into(),
        };
        assert_eq!(
            err.to_string(),
            "write to /dev/ttyACM0 failed: broken pipe"
        );
    }

    #[test]
    fn display_read_failed() {
        let err = ConnectionError::ReadFailed {
            port: "COM3".into(),
            source: "device not connected".into(),
        };
        assert_eq!(
            err.to_string(),
            "read from COM3 failed: device not connected"
        );
    }

    #[test]
    fn display_timeout() {
        let err = ConnectionError::Timeout {
            port: "/dev/ttyUSB0".into(),
            ms: 1000,
        };
        assert_eq!(err.to_string(), "read timeout on /dev/ttyUSB0 after 1000ms");
    }

    #[test]
    fn display_invalid_response() {
        let err = ConnectionError::InvalidResponse {
            port: "COM3".into(),
            response: "GARBAGE\n".into(),
        };
        assert_eq!(
            err.to_string(),
            "invalid response from COM3: \"GARBAGE\\n\""
        );
    }

    #[test]
    fn display_angles_out_of_range() {
        let err = ConnectionError::AnglesOutOfRange {
            values: vec![5, 180, 90],
            min: 10,
            max: 170,
        };
        assert_eq!(
            err.to_string(),
            "angles [5, 180, 90] out of range [10, 170]"
        );
    }

    #[test]
    fn error_is_std_error() {
        let err: Box<dyn std::error::Error> = Box::new(ConnectionError::PortNotFound {
            port: "test".into(),
        });
        assert!(err.to_string().contains("port not found"));
    }
}
