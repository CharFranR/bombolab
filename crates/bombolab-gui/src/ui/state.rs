// ---------------------------------------------------------------------------
// state.rs — Estado global de la aplicación.
//
// Contiene todos los structs que persisten entre frames de egui, incluyendo
// la definición de robots para simulación y el estado del robot físico.
// ---------------------------------------------------------------------------

use bombolab_core::{fabri_creator, DHParams, Joint, JointType, Robot, Segment};

use crate::hardware::{MockRobotController, RobotController};

// ---------------------------------------------------------------------------
// Modelo de datos para la UI (Simulación)
// ---------------------------------------------------------------------------

/// Representa un segmento (articulación + DH) en la UI de edición.
///
/// Los valores angulares se almacenan en grados para facilitar la edición
/// con sliders, y se convierten a radianes al construir el modelo de dominio.
pub struct SegmentUi {
    pub joint_type: JointType,
    pub theta: f64,
    pub d: f64,
    pub a: f64,
    pub alpha: f64,
}

impl SegmentUi {
    /// Crea un segmento revolute con valores por defecto.
    pub fn new_revolute() -> Self {
        Self {
            joint_type: JointType::Revolute,
            theta: 0.0,
            d: 0.0,
            a: 1.0,
            alpha: 0.0,
        }
    }

    /// Convierte este SegmentUi a un Segment de dominio (bombolab-core).
    ///
    /// `joint_value` es el valor actual de la articulación en radianes (para
    /// revolutes) o metros (para prismatics).
    ///
    /// Los valores angulares (`theta`, `alpha`) se almacenan en grados en la UI
    /// y se convierten a radianes al construir `DHParams`.
    pub fn to_segment(&self, joint_value: f64) -> Segment {
        let joint = Joint::new(self.joint_type, joint_value, 0.0, 0.0);
        let dh = DHParams::new(
            self.theta.to_radians(),
            self.d,
            self.a,
            self.alpha.to_radians(),
        );
        Segment::new(joint, dh)
    }
}

/// Definición completa de un robot para la simulación.
pub struct RobotDef {
    pub name: String,
    pub segments: Vec<SegmentUi>,
}

impl RobotDef {
    /// Crea un robot vacío con un nombre dado.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            segments: Vec::new(),
        }
    }

    /// Crea un RobotDef con los parámetros DH del robot FABRI Creator (5 DOF).
    ///
    /// Los valores angulares se almacenan en grados (formato de la UI).
    /// Los valores lineales están en mm (unidades del fabricante).
    ///
    /// Lee la configuración desde `bombolab_core::fabri_creator()` para
    /// mantener una única fuente de verdad — la GUI SIEMPRE refleja el
    /// robot canónico.
    ///
    /// Tabla DH (estándar):
    /// | i | α     | a    | d    | θ |
    /// |---|-------|------|------|---|
    /// | 1 | -90°  | 15   | 95   | 0 |
    /// | 2 |   0°  |  0   | 162  | 0 |
    /// | 3 | -90°  | 111  |  0   | 0 |
    /// | 4 |  90°  | 35   |  0   | 0 |
    /// | 5 |   0°  |  0   |  0   | 0 |
    pub fn fabri_creator() -> Self {
        let core = fabri_creator();
        let segments = core
            .segments
            .iter()
            .map(|seg| SegmentUi {
                joint_type: seg.joint.joint_type,
                theta: seg.dh.theta.to_degrees(),
                d: seg.dh.d,
                a: seg.dh.a,
                alpha: seg.dh.alpha.to_degrees(),
            })
            .collect();

        Self {
            name: "FABRI Creator".to_string(),
            segments,
        }
    }

    /// Número de grados de libertad (cantidad de segmentos).
    pub fn dof(&self) -> usize {
        self.segments.len()
    }

    /// Convierte este RobotDef a un `Robot` de dominio para cálculos FK.
    ///
    /// Todos los joint values se inicializan en 0 (q=0 — home cinemático).
    pub fn to_robot(&self) -> Robot {
        let segments: Vec<Segment> = self.segments.iter().map(|s| s.to_segment(0.0)).collect();
        Robot::new(segments)
    }

    /// Convierte este RobotDef a un `Robot` de dominio con valores articulares
    /// específicos.
    ///
    /// `q_deg` — ángulos en **grados** (uno por segmento).
    /// Cada valor se convierte a radianes y se asigna como joint value del segmento.
    pub fn to_robot_with_joints(&self, q_deg: &[f64]) -> Robot {
        let segments: Vec<Segment> = self
            .segments
            .iter()
            .zip(q_deg.iter())
            .map(|(seg_ui, angle_deg)| seg_ui.to_segment((*angle_deg).to_radians()))
            .collect();
        Robot::new(segments)
    }
}

// ---------------------------------------------------------------------------
// Navegación entre vistas de simulación
// ---------------------------------------------------------------------------

/// Vista actual del panel lateral izquierdo en modo simulación.
#[derive(PartialEq)]
pub enum PanelView {
    /// Pantalla principal con resumen y acciones rápidas.
    Main,
    /// Lista de robots definidos para seleccionar/editar.
    RobotList,
    /// Editor de parámetros DH del robot en el índice dado.
    RobotEditor(usize),
    /// Vista de movimientos / planificación de trayectorias.
    Movements,
}

// ---------------------------------------------------------------------------
// Modo general de la aplicación
// ---------------------------------------------------------------------------

/// Modo activo de la aplicación.
///
/// Determina qué contenido se muestra en el panel lateral y de dónde se
/// toman los datos para el viewport 3D.
#[derive(PartialEq)]
pub enum AppMode {
    /// Modo simulación: editor DH, cinemática directa, planificación.
    Simulation,
    /// Modo robot físico: conexión serie, telemetría, control en tiempo real.
    PhysicalRobot,
}

// ---------------------------------------------------------------------------
// Estado del robot físico
// ---------------------------------------------------------------------------

/// Estado del brazo robótico físico y su telemetría.
///
/// El número de articulaciones se determina en tiempo de ejecución según
/// el hardware conectado. Inicialmente se configura con 4 DOF por defecto.
pub struct PhysicalRobotState {
    /// Indica si hay una conexión activa con el hardware.
    pub connected: bool,
    /// Ángulos actuales de las articulaciones en grados (telemetría).
    ///
    /// Vector de longitud dinámica. Cada elemento es una articulación.
    /// El tamaño se ajusta automáticamente al seleccionar un modelo cinemático.
    pub angles: Vec<f32>,
    /// Índice dentro de `AppState.robots[]` del modelo cinemático a usar.
    ///
    /// Si es `None`, el viewport muestra un placeholder.
    /// Al seleccionar un modelo, `angles` se redimensiona a su DOF.
    pub model_index: Option<usize>,
    /// Mensaje de error de la última operación de conexión/lectura/envío.
    pub connection_error: Option<String>,
    /// Bandera para solicitar una lectura de telemetría en el próximo frame.
    pub pending_read: bool,
    /// Bandera para solicitar un envío de ángulos en el próximo frame.
    pub pending_send: bool,
}

impl PhysicalRobotState {
    /// Crea un estado inicial con `num_joints` articulaciones en 0° y desconectado.
    pub fn new(num_joints: usize) -> Self {
        Self {
            connected: false,
            angles: vec![0.0; num_joints],
            model_index: None,
            connection_error: None,
            pending_read: false,
            pending_send: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Estado global de la aplicación
// ---------------------------------------------------------------------------

/// Estado raíz de la aplicación, compartido entre todos los frames de egui.
pub struct AppState {
    // ─── Simulación ───
    /// Vista activa del panel de simulación.
    pub view: PanelView,
    /// Lista de robots definidos por el usuario.
    pub robots: Vec<RobotDef>,
    /// Índice del robot seleccionado actualmente (si hay).
    pub selected_robot: Option<usize>,
    /// Indica si la ventana de detalles de transformación está abierta.
    pub show_details: bool,
    /// Ángulos articulares actuales en el modo simulación (grados).
    /// Se redimensiona al DOF del robot seleccionado.
    pub sim_angles: Vec<f64>,

    // ─── General ───
    /// Modo activo de la aplicación (Simulación | Robot Físico).
    pub mode: AppMode,

    // ─── Robot Físico ───
    /// Estado del robot físico (conexión, telemetría, etc.).
    pub physical_robot: PhysicalRobotState,
    /// Controlador de hardware — `MockRobotController` (offline) o
    /// `SerialRobotController` (hardware real vía `ArduinoNano`).
    pub robot_controller: Box<dyn RobotController>,
}

impl AppState {
    /// Crea un estado inicial con valores por defecto.
    pub fn new() -> Self {
        Self {
            view: PanelView::Main,
            robots: Vec::new(),
            selected_robot: None,
            show_details: false,
            sim_angles: Vec::new(),
            mode: AppMode::Simulation,
            physical_robot: PhysicalRobotState::new(4),
            robot_controller: Box::new(MockRobotController::new(4)),
        }
    }
}
