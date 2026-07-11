// ---------------------------------------------------------------------------
// state.rs — Estado global de la aplicación.
//
// Contiene todos los structs que persisten entre frames de egui, incluyendo
// la definición de robots para simulación y el estado del robot físico.
// ---------------------------------------------------------------------------

use bombolab_core::{DHParams, Joint, JointType, Robot, Segment};

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
    pub fn to_segment(&self, joint_value: f64) -> Segment {
        let joint = Joint::new(self.joint_type, joint_value, 0.0, 0.0);
        let dh = DHParams::new(self.theta, self.d, self.a, self.alpha);
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

    /// Número de grados de libertad (cantidad de segmentos).
    pub fn dof(&self) -> usize {
        self.segments.len()
    }

    /// Convierte este RobotDef a un `Robot` de dominio para cálculos FK.
    pub fn to_robot(&self) -> Robot {
        let segments: Vec<Segment> = self.segments.iter().map(|s| s.to_segment(0.0)).collect();
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

    // ─── General ───
    /// Modo activo de la aplicación (Simulación | Robot Físico).
    pub mode: AppMode,

    // ─── Robot Físico ───
    /// Estado del robot físico (conexión, telemetría, etc.).
    pub physical_robot: PhysicalRobotState,
    /// Controlador de hardware (mock por ahora).
    ///
    /// TODO: Reemplazar `MockRobotController` por `SerialRobotController`
    ///       cuando se implemente la comunicación serie real.
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
            mode: AppMode::Simulation,
            physical_robot: PhysicalRobotState::new(4),
            robot_controller: Box::new(MockRobotController::new(4)),
        }
    }
}
