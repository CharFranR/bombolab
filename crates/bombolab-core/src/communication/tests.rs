//! Unit tests for `tests`.

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
    assert_eq!(err.to_string(), "write to /dev/ttyACM0 failed: broken pipe");
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
fn error_is_std_error() {
    let err: Box<dyn std::error::Error> = Box::new(ConnectionError::PortNotFound {
        port: "test".into(),
    });
    assert!(err.to_string().contains("port not found"));
}
