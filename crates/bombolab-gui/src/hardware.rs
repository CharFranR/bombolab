// ---------------------------------------------------------------------------
// Módulo hardware — Abstracción de comunicación con el robot físico.
//
// Define un trait `RobotController` que cualquier implementación real (Serial,
// TCP/IP, etc.) debe cumplir, y una implementación `MockRobotController` que
// permite probar la UI sin tener el hardware conectado.
//
// TODO: Cuando se tenga el hardware real, crear una implementación que use la
//       crate `serialport` (ej. `SerialRobotController`) y reemplazar el mock
//       en `AppState::new()`.
// ---------------------------------------------------------------------------

/// Trait que abstrae la comunicación serie con el robot físico.
///
/// Cada método retorna `Result` para poder propagar errores de conexión,
/// timeout o protocolo sin bloquear la UI.
pub trait RobotController {
    /// Abre la conexión con el puerto serie del robot físico.
    ///
    /// TODO: Aquí se configurará el puerto real:
    ///       let port = serialport::new("/dev/ttyUSB0", 115200)
    ///           .timeout(Duration::from_millis(100))
    ///           .open()?;
    fn connect(&mut self) -> Result<(), String>;

    /// Cierra la conexión serie de forma limpia.
    fn disconnect(&mut self) -> Result<(), String>;

    /// Lee los ángulos actuales de cada articulación desde el hardware.
    ///
    /// El vector resultante tiene un elemento por articulación (grados).
    ///
    /// TODO: Reemplazar con lectura real:
    ///       let mut buf = [0u8; 32];
    ///       port.read(&mut buf)?;
    ///       parse_angles(&buf)
    fn read_angles(&mut self) -> Result<Vec<f32>, String>;

    /// Envía ángulos objetivo al robot para que ejecute el movimiento.
    ///
    /// `angles` debe tener tantos elementos como articulaciones tenga el robot.
    ///
    /// TODO: Reemplazar con escritura real:
    ///       let msg = format!("{} {} {} {}\n", a1, a2, a3, a4);
    ///       port.write(msg.as_bytes())?;
    fn send_angles(&mut self, angles: &[f32]) -> Result<(), String>;
}

// ---------------------------------------------------------------------------
// Implementación Mock (simulada)
// ---------------------------------------------------------------------------

/// Implementación simulada de `RobotController` para desarrollo y pruebas.
///
/// Almacena internamente los ángulos que se envían con `send_angles` y los
/// devuelve en `read_angles`, simulando un robot ideal que siempre alcanza
/// la posición objetivo instantáneamente.
///
/// # Ejemplo
/// ```
/// use bombolab_gui::hardware::{RobotController, MockRobotController};
///
/// let mut ctrl = MockRobotController::new(4);
/// assert!(ctrl.connect().is_ok());
/// assert!(ctrl.send_angles(&[10.0, 20.0, 30.0, 40.0]).is_ok());
/// let angles = ctrl.read_angles().unwrap();
/// assert_eq!(angles, vec![10.0, 20.0, 30.0, 40.0]);
/// ```
pub struct MockRobotController {
    /// Indica si el "puerto serie" simulado está abierto.
    connected: bool,
    /// Últimos ángulos recibidos (simula la lectura de encoders).
    angles: Vec<f32>,
}

impl MockRobotController {
    /// Crea un nuevo controlador mock con `num_joints` articulaciones.
    ///
    /// Inicialmente desconectado y con todos los ángulos en 0.0.
    pub fn new(num_joints: usize) -> Self {
        Self {
            connected: false,
            angles: vec![0.0; num_joints],
        }
    }
}

impl RobotController for MockRobotController {
    /// Simula la apertura del puerto serie. Siempre tiene éxito.
    fn connect(&mut self) -> Result<(), String> {
        // TODO: En la versión real, aquí se abriría el puerto:
        //
        //   use serialport::prelude::*;
        //   let mut settings = SerialPortSettings {
        //       baud_rate: 115200,
        //       data_bits: DataBits::Eight,
        //       flow_control: FlowControl::None,
        //       parity: Parity::None,
        //       stop_bits: StopBits::One,
        //       timeout: Duration::from_millis(100),
        //   };
        //   let port = serialport::open_with_settings("/dev/ttyUSB0", &settings)?;
        //   self.port = Some(port);
        self.connected = true;
        Ok(())
    }

    /// Simula el cierre del puerto serie.
    fn disconnect(&mut self) -> Result<(), String> {
        // TODO: En la versión real:
        //   if let Some(port) = self.port.take() {
        //       drop(port);
        //   }
        self.connected = false;
        Ok(())
    }

    /// Devuelve los ángulos almacenados internamente (simulación).
    ///
    /// Falla con un mensaje descriptivo si no hay conexión activa.
    fn read_angles(&mut self) -> Result<Vec<f32>, String> {
        if !self.connected {
            return Err("Robot no conectado. Presione 'Conectar' primero.".to_string());
        }
        // TODO: Reemplazar con lectura real desde el buffer serie.
        //
        //   let mut buf = [0u8; 64];
        //   let n = self.port.as_mut().unwrap().read(&mut buf)?;
        //   let line = String::from_utf8_lossy(&buf[..n]);
        //   self.angles = parse_angles_line(&line)?;
        Ok(self.angles.clone())
    }

    /// Almacena los ángulos recibidos (simula envío al robot).
    ///
    /// Valida que la cantidad de ángulos coincida con la configurada.
    fn send_angles(&mut self, angles: &[f32]) -> Result<(), String> {
        if !self.connected {
            return Err("Robot no conectado. Presione 'Conectar' primero.".to_string());
        }
        // Validar cantidad de articulaciones
        if angles.len() != self.angles.len() {
            return Err(format!(
                "Número de articulaciones incorrecto: esperaba {}, recibí {}",
                self.angles.len(),
                angles.len()
            ));
        }
        // TODO: En la versión real se escribirían los ángulos al puerto serie:
        //
        //   let msg = angles
        //       .iter()
        //       .map(|a| format!("{:.2}", a))
        //       .collect::<Vec<_>>()
        //       .join(" ");
        //   let msg = format!("{}\n", msg);
        //   self.port.as_mut().unwrap().write(msg.as_bytes())?;
        self.angles = angles.to_vec();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_connect_disconnect() {
        let mut ctrl = MockRobotController::new(4);
        assert!(ctrl.read_angles().is_err()); // desconectado
        assert!(ctrl.connect().is_ok());
        assert!(ctrl.read_angles().is_ok());  // conectado
        assert!(ctrl.disconnect().is_ok());
        assert!(ctrl.read_angles().is_err()); // desconectado de nuevo
    }

    #[test]
    fn test_mock_send_and_read_angles() {
        let mut ctrl = MockRobotController::new(4);
        ctrl.connect().unwrap();

        let target = vec![10.0, 20.0, 30.0, 40.0];
        assert!(ctrl.send_angles(&target).is_ok());

        let read = ctrl.read_angles().unwrap();
        assert_eq!(read, target);
    }

    #[test]
    fn test_mock_wrong_number_of_angles() {
        let mut ctrl = MockRobotController::new(4);
        ctrl.connect().unwrap();

        // Enviar 3 ángulos en un robot de 4 articulaciones → error
        let result = ctrl.send_angles(&[1.0, 2.0, 3.0]);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .contains("Número de articulaciones incorrecto"));
    }

    #[test]
    fn test_mock_disconnected_errors() {
        let mut ctrl = MockRobotController::new(2);
        assert!(ctrl.send_angles(&[1.0, 2.0]).is_err());
        assert!(ctrl.read_angles().is_err());
    }
}
