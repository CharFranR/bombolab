// ---------------------------------------------------------------------------
// main_page.rs — Punto de entrada del renderizado de la UI.
//
// Organiza la ventana en tres áreas principales:
//   1. Barra superior (título)
//   2. Panel lateral izquierdo con pestañas [Simulación | Robot Físico]
//   3. Panel central (viewport 3D isométrico)
//
// El panel lateral cambia completamente según la pestaña activa, y el
// viewport 3D decide de dónde tomar los datos según el modo.
// ---------------------------------------------------------------------------

use bombolab_core::{base_transform, forward_kinematics, tool_transform, JointType};

use crate::ui::state::{AppMode, PanelView, RobotDef, SegmentUi};
use crate::ui::viewport::{draw_robot_skeleton, Point3D};
// ---------------------------------------------------------------------------
// Render principal (llamado desde lib.rs → main.rs)
// ---------------------------------------------------------------------------

/// Renderiza todos los elementos de la ventana principal.
///
/// No modifica la firma para mantener compatibilidad con `main.rs`, que
/// ya llama a `bombolab_gui::render(ui, &mut self.state)`.
pub fn render(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    // ────────────────────────────────────────────────────────────────────────
    // 1. Barra superior (título)
    // ────────────────────────────────────────────────────────────────────────
    egui::Panel::top("top_bar").show_inside(ui, |ui| {
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.heading("Bombolab");
            ui.separator();
            ui.label("Forward Kinematics Visualizer");
        });
        ui.add_space(4.0);
    });

    // ────────────────────────────────────────────────────────────────────────
    // 2. Panel lateral izquierdo (con pestañas)
    // ────────────────────────────────────────────────────────────────────────
    egui::Panel::left("side_panel")
        .default_size(280.0)
        .show_inside(ui, |ui| {
            // 2a. Pestañas en la parte superior del panel
            ui.horizontal(|ui| {
                let sim_selected = state.mode == AppMode::Simulation;
                if ui
                    .selectable_label(sim_selected, "  Simulación  ")
                    .clicked()
                {
                    state.mode = AppMode::Simulation;
                }

                let phys_selected = state.mode == AppMode::PhysicalRobot;
                if ui
                    .selectable_label(phys_selected, "  Robot Físico  ")
                    .clicked()
                {
                    state.mode = AppMode::PhysicalRobot;
                }
            });
            ui.separator();

            // 2b. Contenido según la pestaña activa
            match state.mode {
                AppMode::Simulation => render_simulation_panel(ui, state),
                AppMode::PhysicalRobot => render_physical_panel(ui, state),
            }
        });

    // ────────────────────────────────────────────────────────────────────────
    // 3. Panel central (viewport 3D)
    // ────────────────────────────────────────────────────────────────────────
    egui::CentralPanel::default().show_inside(ui, |ui| {
        let rect = ui.available_rect_before_wrap();
        let painter = ui.painter();

        // Fondo oscuro tipo viewport 3D
        painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(30, 30, 30));

        // Calcular los puntos 3D según el modo activo
        let points: Vec<Point3D> = match state.mode {
            AppMode::Simulation => compute_simulation_points(state),
            AppMode::PhysicalRobot => compute_physical_robot_points(state),
        };

        // Decidir si hay datos suficientes para dibujar
        let has_valid_data = {
            let non_zero = points
                .iter()
                .any(|p| p.x != 0.0 || p.y != 0.0 || p.z != 0.0);
            points.len() >= 2 && non_zero
        };

        if has_valid_data {
            // ── Renderizar esqueleto 3D ──
            draw_robot_skeleton(
                &painter,
                rect,
                &points,
                egui::Color32::from_rgb(255, 200, 50),   // Color articulaciones (amarillo)
                egui::Color32::from_rgb(220, 180, 60),   // Color eslabones (oro)
                3.0,                                      // Grosor de líneas (px)
                6.0,                                      // Radio de círculos (px)
            );
        } else {
            // ── Placeholder ──
            let msg = match state.mode {
                AppMode::Simulation => {
                    if state.selected_robot.is_none() {
                        "3D Viewport\nSeleccione o cree un robot en 'Simulación'"
                    } else {
                        "3D Viewport\nRobot sin segmentos definidos"
                    }
                }
                AppMode::PhysicalRobot => {
                    "3D Viewport\nConecte el robot físico para ver la telemetría"
                }
            };
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                msg,
                egui::FontId::proportional(16.0),
                egui::Color32::from_rgb(100, 100, 100),
            );
        }
    });

    // ────────────────────────────────────────────────────────────────────────
    // 4. Ventana de detalles (solo en modo simulación)
    // ────────────────────────────────────────────────────────────────────────
    if state.show_details && state.mode == AppMode::Simulation {
        let mut open = state.show_details;
        egui::Window::new("Transformation Details")
            .open(&mut open)
            .default_width(500.0)
            .default_height(400.0)
            .resizable(true)
            .scroll([true, true])
            .show(ui, |ui| {
                render_details(ui, state);
            });
        state.show_details = open;
    }
}

// ---------------------------------------------------------------------------
// Panel de simulación
// ---------------------------------------------------------------------------

/// Renderiza el contenido del panel lateral en modo simulación.
///
/// Delega en las funciones existentes según la vista activa (`PanelView`).
fn render_simulation_panel(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    match &state.view {
        PanelView::Main => render_main(ui, state),
        PanelView::RobotList => render_robot_list(ui, state),
        PanelView::RobotEditor(idx) => {
            let idx = *idx;
            render_robot_editor(ui, state, idx);
        }
        PanelView::Movements => render_movements(ui, state),
    }
}

// ---------------------------------------------------------------------------
// Panel de robot físico
// ---------------------------------------------------------------------------

/// Renderiza el contenido del panel lateral en modo robot físico.
///
/// Incluye:
///   - Controles de conexión serie (Conectar / Desconectar)
///   - Indicador de estado de conexión
///   - Sección de telemetría con sliders para cada articulación
///   - Botones para leer y enviar ángulos
fn render_physical_panel(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    ui.add_space(8.0);
    ui.heading("Robot Físico");
    ui.separator();

    // ─── Controles de conexión ─────────────────────────────────────────────
    ui.add_space(8.0);
    ui.label("Conexión");
    ui.separator();

    if !state.physical_robot.connected {
        // Botón Conectar
        if ui
            .button("🔌  Conectar Puerto Serie")
            .on_hover_text("Abrir conexión con el robot físico")
            .clicked()
        {
            // `robot_controller` puede ser MockRobotController (offline)
            // o SerialRobotController (hardware real). Ambos implementan
            // el trait RobotController — connect() abre el puerto serie real
            // o simula la conexión según la implementación activa.
            match state.robot_controller.connect() {
                Ok(()) => {
                    state.physical_robot.connected = true;
                    state.physical_robot.connection_error = None;
                }
                Err(e) => {
                    state.physical_robot.connection_error = Some(e);
                }
            }
        }

        // Mostrar error si existe
        if let Some(ref error) = state.physical_robot.connection_error {
            ui.add_space(4.0);
            ui.colored_label(egui::Color32::RED, format!("Error: {}", error));
        }
    } else {
        // Botón Desconectar
        if ui
            .button("🔌  Desconectar Puerto Serie")
            .on_hover_text("Cerrar conexión con el robot físico")
            .clicked()
        {
            // disconnect() cierra el puerto serie real (SerialRobotController)
            // o resetea el estado simulado (MockRobotController).
            let _ = state.robot_controller.disconnect();
            state.physical_robot.connected = false;
        }

        // Indicador de estado conectado
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Estado:");
            ui.colored_label(egui::Color32::GREEN, "● Conectado");
        });

        // TODO: Mostrar información del puerto (velocidad, puerto, etc.)
        // ui.label("Puerto: /dev/ttyUSB0");
        // ui.label("Baud rate: 115200");
    }

    // ─── Modelo cinemático ────────────────────────────────────────────────
    ui.add_space(16.0);
    ui.heading("Modelo Cinemático");
    ui.separator();

    if state.robots.is_empty() {
        ui.colored_label(
            egui::Color32::DARK_GRAY,
            "No hay robots definidos.",
        );
        if ui.button("+ Cargar FABRI Creator").clicked() {
            let idx = state.robots.len();
            state.robots.push(RobotDef::fabri_creator());
            state.physical_robot.model_index = Some(idx);
            state.physical_robot.angles.resize(5, 0.0);
        }
    } else {
        for (i, robot) in state.robots.iter().enumerate() {
            let is_selected = state.physical_robot.model_index == Some(i);
            let label = format!("{} — {} DOF", robot.name, robot.dof());
            if ui.selectable_label(is_selected, &label).clicked() {
                // Al seleccionar un modelo, redimensionar los ángulos a su DOF
                let dof = robot.segments.len();
                state.physical_robot.model_index = Some(i);
                state.physical_robot.angles.resize(dof, 0.0);
            }
        }
    }

    // ─── Telemetría ────────────────────────────────────────────────────────
    ui.add_space(16.0);
    ui.heading("Telemetría");
    ui.separator();

    // Sliders para cada articulación (número dinámico según el hardware)
    let num_joints = state.physical_robot.angles.len();
    for i in 0..num_joints {
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            // Etiqueta de la articulación: nombres conocidos para los primeros 4,
            // genéricos para el resto
            let label = match i {
                0 => "Base",
                1 => "Hombro",
                2 => "Codo",
                3 => "Muñeca",
                _ => "J",
            };
            ui.label(format!("{}:", label));

            // NOTA: Los sliders permiten ajustar ángulos objetivo manualmente.
            // Cuando el hardware real está conectado (SerialRobotController),
            // `send_angles()` transmite estos valores al firmware vía serial.
            ui.add(
                egui::Slider::new(&mut state.physical_robot.angles[i], -180.0..=180.0)
                    .suffix("°")
                    .text(format!("J{}", i + 1)),
            )
            .on_hover_text(format!(
                "Ángulo de la articulación {} ({})",
                i + 1,
                label
            ));
        });
    }

    ui.add_space(8.0);

    // Botones de acción (solo visibles cuando hay conexión)
    if state.physical_robot.connected {
        ui.horizontal(|ui| {
            if ui
                .button("📡  Leer telemetría")
                .on_hover_text("Solicitar ángulos actuales al robot")
                .clicked()
            {
                // read_angles() obtiene los últimos ángulos conocidos.
                // SerialRobotController devuelve el último estado enviado;
                // MockRobotController devuelve la simulación interna.
                //
                //   match state.robot_controller.read_angles() {
                //       Ok(angles) => { ... }
                //       Err(e) => { ... }
                //   }
                state.physical_robot.pending_read = true;
            }

            if ui
                .button("📤  Enviar ángulos")
                .on_hover_text("Enviar ángulos objetivo al robot")
                .clicked()
            {
                // send_angles() transmite los ángulos al hardware.
                // SerialRobotController usa ArduinoNano::send_and_verify()
                // para enviar y confirmar recepción OK/ERR.
                state.physical_robot.pending_send = true;
            }
        });

        // Procesar acciones pendientes (se ejecutan aquí para mantener
        // la propiedad prestada de state limpia)
        if state.physical_robot.pending_read {
            state.physical_robot.pending_read = false;
            match state.robot_controller.read_angles() {
                Ok(angles) => {
                    // Reemplazar todo el vector de ángulos con lo leído del hardware
                    state.physical_robot.angles = angles;
                    state.physical_robot.connection_error = None;
                }
                Err(e) => {
                    state.physical_robot.connection_error = Some(e);
                }
            }
        }

        if state.physical_robot.pending_send {
            state.physical_robot.pending_send = false;
            match state.robot_controller.send_angles(&state.physical_robot.angles) {
                Ok(()) => {
                    state.physical_robot.connection_error = None;
                }
                Err(e) => {
                    state.physical_robot.connection_error = Some(e);
                }
            }
        }
    } else {
        ui.add_space(8.0);
        ui.colored_label(
            egui::Color32::DARK_GRAY,
            "Conecte el robot para habilitar la telemetría.",
        );
    }

    // Mostrar errores de telemetría si los hay
    if let Some(ref error) = state.physical_robot.connection_error {
        ui.add_space(4.0);
        ui.colored_label(egui::Color32::RED, format!("Error: {}", error));
    }
}

// ---------------------------------------------------------------------------
// Cálculo de puntos 3D para el viewport
// ---------------------------------------------------------------------------

/// Calcula los puntos 3D del robot seleccionado en modo simulación.
///
/// Ejecuta cinemática directa sobre el robot definido por el usuario y
/// extrae las posiciones (translaciones) de cada frame, incluyendo la base.
///
/// Retorna un vector con: [base, joint_1, joint_2, ..., end_effector].
fn compute_simulation_points(state: &super::state::AppState) -> Vec<Point3D> {
    let Some(idx) = state.selected_robot else {
        return vec![Point3D::origin()];
    };

    let robot = &state.robots[idx];
    if robot.segments.is_empty() || state.sim_angles.is_empty() {
        return vec![Point3D::origin()];
    }

    // Ejecutar cinemática directa con los ángulos actuales de simulación
    let domain_robot = robot.to_robot_with_joints(&state.sim_angles);
    let (frames, _effector) = forward_kinematics(base_transform(), &domain_robot);

    // Construir lista de puntos: base + cada frame
    let mut points = vec![Point3D::origin()]; // ground en el origen
    for frame in &frames {
        let t = frame.translation.vector;
        points.push(Point3D::new(t.x as f32, t.y as f32, t.z as f32));
    }
    points
}

/// Calcula los puntos 3D del robot físico reutilizando el pipeline de FK de
/// `bombolab-core`, exactamente como hace `compute_simulation_points`.
///
/// Toma el modelo cinemático desde `state.physical_robot.model_index` (que
/// apunta a un `RobotDef` de la lista general) y le aplica los ángulos de
/// telemetría como valores articulares. Así se elimina la duplicación de
/// lógica DH que existía antes.
fn compute_physical_robot_points(state: &super::state::AppState) -> Vec<Point3D> {
    use bombolab_core::{base_transform, Robot, Segment};

    // Resolver qué modelo cinemático usar, con validación de índice
    // para evitar panic si el robot fue borrado desde la simulación.
    let model_idx = match state.physical_robot.model_index {
        Some(i) if i < state.robots.len() => i,
        _ => return vec![Point3D::origin()],
    };

    let robot_def = &state.robots[model_idx];
    if robot_def.segments.is_empty() || state.physical_robot.angles.is_empty() {
        return vec![Point3D::origin()];
    }

    // Validar que el DOF coincida con la cantidad de ángulos de telemetría
    if robot_def.segments.len() != state.physical_robot.angles.len() {
        return vec![Point3D::origin()];
    }

    // Construir segmentos de dominio aplicando los ángulos de telemetría
    // sobre el modelo DH del RobotDef seleccionado
    let segments: Vec<Segment> = robot_def
        .segments
        .iter()
        .zip(state.physical_robot.angles.iter())
        .map(|(seg_ui, angle_deg)| {
            // Los ángulos vienen en grados (desde los sliders de la UI)
            // y se convierten a radianes para el motor de FK
            let joint_value_rad = (*angle_deg as f64).to_radians();
            seg_ui.to_segment(joint_value_rad)
        })
        .collect();

    // Ejecutar cinemática directa con base transform (elevación real)
    let robot = Robot::new(segments);
    let (frames, _) = forward_kinematics(base_transform(), &robot);

    // Extraer puntos: base + cada frame transformado
    let mut points = vec![Point3D::origin()];
    for frame in &frames {
        let t = frame.translation.vector;
        points.push(Point3D::new(t.x as f32, t.y as f32, t.z as f32));
    }
    points
}

// ===========================================================================
// Funciones existentes (sin cambios en la lógica)
// ===========================================================================
//
// Las siguientes funciones son idénticas a la versión anterior del archivo.
// Se mantienen intactas para no romper la funcionalidad de simulación.
// ===========================================================================

// ── Main view ──

fn render_main(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    ui.add_space(8.0);

    // ─── Selección de robot ─────────────────────────────────────────────
    if ui.button("Select / Define Robot").clicked() {
        state.view = PanelView::RobotList;
    }

    // ─── Control de articulaciones ──────────────────────────────────────
    if let Some(idx) = state.selected_robot {
        let robot = &state.robots[idx];
        if !robot.segments.is_empty() {
            // Asegurar tamaño del vector de ángulos
            let dof = robot.segments.len();
            if state.sim_angles.len() != dof {
                state.sim_angles.resize(dof, 0.0);
            }

            ui.add_space(12.0);
            ui.separator();
            ui.label("Joint Control (q)");
            ui.separator();

            // Sliders para cada articulación
            for i in 0..dof {
                ui.add_space(2.0);
                let label = match i {
                    0 => "Base (Yaw)",
                    1 => "Shoulder",
                    2 => "Elbow",
                    3 => "Wrist Roll",
                    4 => "Wrist Pitch",
                    _ => "",
                };
                ui.add(
                    egui::Slider::new(&mut state.sim_angles[i], -90.0..=90.0)
                        .suffix("°")
                        .text(label),
                );
            }

            // ─── FK result ──────────────────────────────────────────────
            let domain_robot = robot.to_robot_with_joints(&state.sim_angles);
            let (_frames, effector) = forward_kinematics(base_transform(), &domain_robot);
            let tool_pose = effector * tool_transform();
            let pos = tool_pose.translation.vector;

            // Matriz de rotación del efector (top-left 3×3 de T_0_5)
            let m_ee = effector.to_matrix();
            let rot = || -> [[f64; 3]; 3] {
                [
                    [m_ee[(0, 0)], m_ee[(0, 1)], m_ee[(0, 2)]],
                    [m_ee[(1, 0)], m_ee[(1, 1)], m_ee[(1, 2)]],
                    [m_ee[(2, 0)], m_ee[(2, 1)], m_ee[(2, 2)]],
                ]
            };
            let r = rot();

            ui.add_space(12.0);
            ui.separator();
            ui.label("End-Effector (tool tip)");
            ui.separator();

            ui.indent("ee_pos", |ui| {
                ui.monospace(format!(
                    "Pos: {:>7.2}  {:>7.2}  {:>7.2}",
                    pos.x, pos.y, pos.z
                ));
                ui.monospace(format!(
                    "Rot: [{:>6.3} {:>6.3} {:>6.3}]",
                    r[0][0], r[0][1], r[0][2]
                ));
                ui.monospace(format!(
                    "     [{:>6.3} {:>6.3} {:>6.3}]",
                    r[1][0], r[1][1], r[1][2]
                ));
                ui.monospace(format!(
                    "     [{:>6.3} {:>6.3} {:>6.3}]",
                    r[2][0], r[2][1], r[2][2]
                ));
            });

            ui.add_space(8.0);
            if ui.button("View Details").clicked() {
                state.show_details = true;
            }
        } else {
            ui.add_space(16.0);
            ui.label("Robot sin segmentos definidos.");
        }
    } else {
        ui.add_space(16.0);
        ui.colored_label(
            egui::Color32::DARK_GRAY,
            "Seleccione o cree un robot en 'Select / Define Robot'",
        );
    }
}

// ── Robot list view ──

fn render_robot_list(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    ui.horizontal(|ui| {
        if ui.button("< Back").clicked() {
            state.view = PanelView::Main;
        }
        ui.heading("Robots");
    });
    ui.separator();

    if state.robots.is_empty() {
        ui.add_space(16.0);
        ui.colored_label(
            egui::Color32::DARK_GRAY,
            "No hay robots definidos.",
        );
        ui.add_space(8.0);
        if ui.button("+ Cargar FABRI Creator").clicked() {
            let idx = state.robots.len();
            state.robots.push(RobotDef::fabri_creator());
            state.selected_robot = Some(idx);
        }
    } else {
        for (i, robot) in state.robots.iter().enumerate() {
            ui.horizontal(|ui| {
                let label = format!("{} — {} DOF", robot.name, robot.dof());
                if ui
                    .selectable_label(state.selected_robot == Some(i), &label)
                    .clicked()
                {
                    state.selected_robot = Some(i);
                }
                if ui.small_button("Edit").clicked() {
                    state.view = PanelView::RobotEditor(i);
                }
            });
        }
    }

    ui.add_space(12.0);
    if ui.button("+ New Robot").clicked() {
        let idx = state.robots.len();
        state.robots.push(RobotDef::new("Robot"));
        state.selected_robot = Some(idx);
        state.view = PanelView::RobotEditor(idx);
    }
}

// ── Robot editor view ──

fn render_robot_editor(ui: &mut egui::Ui, state: &mut super::state::AppState, idx: usize) {
    ui.horizontal(|ui| {
        if ui.button("< Back").clicked() {
            state.view = PanelView::RobotList;
        }
        ui.heading("Edit Robot");
    });
    ui.separator();

    let robot = &mut state.robots[idx];

    // Robot name
    ui.horizontal(|ui| {
        ui.label("Name:");
        ui.text_edit_singleline(&mut robot.name);
    });

    // DOF selector
    ui.horizontal(|ui| {
        ui.label("DOF:");
        for n in 2..=6 {
            if ui
                .selectable_label(robot.segments.len() == n, n.to_string())
                .clicked()
            {
                while robot.segments.len() < n {
                    robot.segments.push(SegmentUi::new_revolute());
                }
                robot.segments.truncate(n);
            }
        }
    });

    ui.add_space(8.0);

    // Segments
    for (i, segment) in robot.segments.iter_mut().enumerate() {
        render_segment(ui, i, segment);
    }
}

// ── Movements view ──

fn render_movements(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    ui.horizontal(|ui| {
        if ui.button("< Back").clicked() {
            state.view = PanelView::Main;
        }
        ui.heading("Joint Control");
    });
    ui.separator();
    ui.add_space(8.0);
    ui.colored_label(
        egui::Color32::DARK_GRAY,
        "Usá los sliders en la vista principal para controlar las articulaciones en tiempo real.",
    );
}

// ── Segment form (shared) ──

fn render_segment(ui: &mut egui::Ui, index: usize, segment: &mut SegmentUi) {
    let header = format!("Segment {} [{}]", index + 1, segment.joint_type);

    egui::CollapsingHeader::new(header)
        .default_open(index == 0)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Type:");
                ui.selectable_value(&mut segment.joint_type, JointType::Revolute, "Revolute");
                ui.selectable_value(&mut segment.joint_type, JointType::Prismatic, "Prismatic");
            });

            ui.add_space(4.0);

            match segment.joint_type {
                JointType::Revolute => {
                    ui.add(
                        egui::Slider::new(&mut segment.theta, -360.0..=360.0)
                            .suffix("°")
                            .text("θ"),
                    );
                    ui.add(egui::Slider::new(&mut segment.d, -10.0..=10.0).text("d"));
                    ui.add(egui::Slider::new(&mut segment.a, 0.0..=10.0).text("a"));
                    ui.add(
                        egui::Slider::new(&mut segment.alpha, -360.0..=360.0)
                            .suffix("°")
                            .text("α"),
                    );
                }
                JointType::Prismatic => {
                    ui.add(egui::Slider::new(&mut segment.theta, -360.0..=360.0).text("θ"));
                    ui.add(
                        egui::Slider::new(&mut segment.d, -10.0..=10.0)
                            .suffix(" m")
                            .text("d"),
                    );
                    ui.add(egui::Slider::new(&mut segment.a, 0.0..=10.0).text("a"));
                    ui.add(
                        egui::Slider::new(&mut segment.alpha, -360.0..=360.0)
                            .suffix("°")
                            .text("α"),
                    );
                }
            }
        });
}

// ── Details popup ──

fn render_details(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    let Some(idx) = state.selected_robot else {
        ui.label("No robot selected.");
        return;
    };

    if state.robots[idx].segments.is_empty() {
        ui.label("No segments defined.");
        return;
    }

    ui.label(format!(
        "Robot: {} — {} DOF",
        state.robots[idx].name,
        state.robots[idx].dof()
    ));
    ui.separator();

    // ── Per-segment transformation matrices ──
    ui.heading("Segment Transformation Matrices (T_i)");
    ui.add_space(4.0);

    for i in 0..state.robots[idx].segments.len() {
        let label = format!(
            "Segment {} [{}]",
            i + 1,
            state.robots[idx].segments[i].joint_type
        );
        let default_open = i == 0;

        egui::CollapsingHeader::new(label)
            .default_open(default_open)
            .show(ui, |ui| {
                let seg = &mut state.robots[idx].segments[i];
                // DH parameters
                ui.horizontal(|ui| {
                    ui.label("θ:");
                    ui.add(egui::DragValue::new(&mut seg.theta).suffix("°").speed(0.5));
                    ui.label("d:");
                    ui.add(egui::DragValue::new(&mut seg.d).speed(0.05));
                });
                ui.horizontal(|ui| {
                    ui.label("a:");
                    ui.add(egui::DragValue::new(&mut seg.a).speed(0.05));
                    ui.label("α:");
                    ui.add(egui::DragValue::new(&mut seg.alpha).suffix("°").speed(0.5));
                });

                ui.add_space(4.0);

                // Matrix
                ui.label("T_i = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)");
                ui.monospace(format_matrix(i, seg.theta, seg.d, seg.a, seg.alpha));
            });
    }

    ui.add_space(8.0);
    ui.separator();

    // ── Cumulative transformation ──
    ui.heading("Accumulated Transformations");
    ui.add_space(4.0);

    let dof = state.robots[idx].dof();

    for i in 0..dof {
        if i == 0 {
            ui.label("T_0_1 = T_1");
        } else {
            ui.label(format!("T_0_{} = T_0_{} · T_{}", i + 1, i, i + 1));
        }
    }

    ui.add_space(8.0);
    ui.separator();
    ui.heading("End-Effector Pose (T_0_tool)");
    ui.add_space(4.0);

    // Compute FK con base + tool transform para el efector real
    let domain_robot = state.robots[idx].to_robot();
    let (_frames, effector) = forward_kinematics(base_transform(), &domain_robot);
    let tool_pose = effector * tool_transform();

    // Format the real transformation matrix
    let t = tool_pose;
    let m = t.to_matrix();
    ui.monospace(format!(
        "┌ {:7.3} {:7.3} {:7.3} {:7.3} ┐\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         └                    ┘",
        m[(0, 0)], m[(0, 1)], m[(0, 2)], m[(0, 3)],
        m[(1, 0)], m[(1, 1)], m[(1, 2)], m[(1, 3)],
        m[(2, 0)], m[(2, 1)], m[(2, 2)], m[(2, 3)],
        m[(3, 0)], m[(3, 1)], m[(3, 2)], m[(3, 3)]
    ));
}

fn format_matrix(_i: usize, theta: f64, d: f64, a: f64, alpha: f64) -> String {
    let t = theta.to_radians();
    let al = alpha.to_radians();
    let ct = t.cos();
    let st = t.sin();
    let ca = al.cos();
    let sa = al.sin();

    format!(
        "┌ {:7.3} {:7.3} {:7.3} {:7.3} ┐\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         └                    ┘",
        ct,
        -st * ca,
        st * sa,
        a * ct,
        st,
        ct * ca,
        -ct * sa,
        a * st,
        0.0,
        sa,
        ca,
        d,
        0.0,
        0.0,
        0.0,
        1.0
    )
}
