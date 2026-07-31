use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

use serialport::SerialPort;

use super::command::ServoCommand;
use super::{BAUD_RATE, ConnectionError, READ_TIMEOUT_MS};

/// Arduino Nano serial connection.
///
/// Wraps a serial port handle (inside a persistent `BufReader`) and provides
/// angle transmission with OK/ERR response verification. Uses the crate-level
/// constants `BAUD_RATE`, `JOINT_COUNT`, and `READ_TIMEOUT_MS`.
///
/// The `BufReader` lives for the whole connection so bytes buffered after a
/// partial read (e.g. a timeout mid-response) are kept for the next
/// `read_response` call — a fresh reader per call would drop them and
/// permanently shift every subsequent response by one line.
pub struct ArduinoNano {
    reader: BufReader<Box<dyn SerialPort>>,
    port_name: String,
}

impl ArduinoNano {
    pub fn list_ports() -> Result<Vec<String>, ConnectionError> {
        let ports = serialport::available_ports().map_err(|e| ConnectionError::ReadFailed {
            port: "<system>".into(),
            source: e.to_string(),
        })?;
        Ok(ports.into_iter().map(|p| p.port_name).collect())
    }

    /// Connect to a serial port at 115200 baud.
    pub fn connect(port_name: &str) -> Result<Self, ConnectionError> {
        let port = serialport::new(port_name, BAUD_RATE)
            .timeout(Duration::from_millis(READ_TIMEOUT_MS))
            .open()
            .map_err(|e| ConnectionError::OpenFailed {
                port: port_name.into(),
                source: e.to_string(),
            })?;

        Ok(Self {
            reader: BufReader::new(port),
            port_name: port_name.into(),
        })
    }

    /// Send a `ServoCommand` as comma-separated values: `a1,a2,a3,a4,a5,g\n`
    ///
    /// Delegates to `ServoCommand::to_wire()` which produces the same wire format
    /// as the original `send_angles()` — no protocol breakage.
    pub fn send(&mut self, cmd: &ServoCommand) -> Result<(), ConnectionError> {
        let msg = cmd.to_wire();
        let port = self.reader.get_mut();
        port.write_all(msg.as_bytes())
            .map_err(|e| ConnectionError::WriteFailed {
                port: self.port_name.clone(),
                source: e.to_string(),
            })?;
        port.flush().map_err(|e| ConnectionError::WriteFailed {
            port: self.port_name.clone(),
            source: e.to_string(),
        })?;
        Ok(())
    }

    /// Read one line from Arduino (expects "OK" or "ERR").
    ///
    /// Uses the connection-wide `BufReader`, so bytes buffered after a partial
    /// read are preserved for the next call.
    pub fn read_response(&mut self) -> Result<String, ConnectionError> {
        let mut line = String::new();
        self.reader
            .read_line(&mut line)
            .map_err(|e| ConnectionError::ReadFailed {
                port: self.port_name.clone(),
                source: e.to_string(),
            })?;
        Ok(line.trim().to_string())
    }

    /// Send a `ServoCommand` and wait for "OK" response.
    pub fn send_and_verify(&mut self, cmd: &ServoCommand) -> Result<(), ConnectionError> {
        self.send(cmd)?;
        let response = self.read_response()?;
        match response.as_str() {
            "OK" => Ok(()),
            "ERR" => Err(ConnectionError::InvalidResponse {
                port: self.port_name.clone(),
                response,
            }),
            other => Err(ConnectionError::InvalidResponse {
                port: self.port_name.clone(),
                response: other.to_string(),
            }),
        }
    }

    /// Flush and close the port.
    pub fn disconnect(&mut self) -> Result<(), ConnectionError> {
        self.reader
            .get_mut()
            .flush()
            .map_err(|e| ConnectionError::WriteFailed {
                port: self.port_name.clone(),
                source: e.to_string(),
            })?;
        Ok(())
    }
}
