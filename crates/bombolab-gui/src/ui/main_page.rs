use bombolab_core::{Iso3, JointType, forward_kinematics};

use crate::ui::state::{PanelView, RobotDef, SegmentUi};

pub fn render(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    egui::Panel::top("top_bar").show_inside(ui, |ui| {
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.heading("Bombolab");
            ui.separator();
            ui.label("Forward Kinematics Visualizer");
        });
        ui.add_space(4.0);
    });

    egui::Panel::left("side_panel")
        .default_size(280.0)
        .show_inside(ui, |ui| match &state.view {
            PanelView::Main => render_main(ui, state),
            PanelView::RobotList => render_robot_list(ui, state),
            PanelView::RobotEditor(idx) => {
                let idx = *idx;
                render_robot_editor(ui, state, idx);
            }
            PanelView::Movements => render_movements(ui, state),
        });

    egui::CentralPanel::default().show_inside(ui, |ui| {
        let rect = ui.available_rect_before_wrap();
        let painter = ui.painter();
        painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(30, 30, 30));
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "3D Viewport",
            egui::FontId::proportional(16.0),
            egui::Color32::from_rgb(100, 100, 100),
        );
    });

    // Details popup
    if state.show_details {
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

// ── Main view ──

fn render_main(ui: &mut egui::Ui, state: &mut super::state::AppState) {
    ui.add_space(8.0);

    if ui.button("Select / Define Robot").clicked() {
        state.view = PanelView::RobotList;
    }

    ui.add_space(4.0);

    if ui.button("Define Movements").clicked() {
        state.view = PanelView::Movements;
    }

    // Show selected robot summary if any
    if let Some(idx) = state.selected_robot {
        ui.add_space(16.0);
        ui.separator();
        ui.label("Current Robot");
        ui.separator();
        let robot = &state.robots[idx];
        ui.label(format!("{} — {} DOF", robot.name, robot.dof()));
    }

    ui.add_space(16.0);
    ui.separator();
    ui.label("Results");
    ui.separator();

    // Compute FK if a robot is selected
    if let Some(idx) = state.selected_robot {
        let robot = &state.robots[idx];
        if !robot.segments.is_empty() {
            let domain_robot = robot.to_robot();
            let base = Iso3::identity();
            let (_frames, effector) = forward_kinematics(base, &domain_robot);

            let pos = effector.translation.vector;
            ui.label("End-Effector");
            ui.indent("ee_pos", |ui| {
                ui.label(format!("Pos: ({:.3}, {:.3}, {:.3})", pos.x, pos.y, pos.z));
                ui.label("Rot: (see details)");
            });

            ui.add_space(4.0);

            ui.label("Frames");
            ui.indent("frames", |ui| {
                ui.label(format!(
                    "Frame 0: ({:.3}, {:.3}, {:.3})",
                    pos.x, pos.y, pos.z
                ));
            });
        } else {
            ui.label("End-Effector");
            ui.indent("ee_pos", |ui| {
                ui.label("Pos: --");
                ui.label("Rot: --");
            });

            ui.add_space(4.0);

            ui.label("Frames");
            ui.indent("frames", |ui| {
                ui.label("Frame 0: --");
            });
        }
    } else {
        ui.label("End-Effector");
        ui.indent("ee_pos", |ui| {
            ui.label("Pos: --");
            ui.label("Rot: --");
        });

        ui.add_space(4.0);

        ui.label("Frames");
        ui.indent("frames", |ui| {
            ui.label("Frame 0: --");
        });
    }

    ui.add_space(8.0);
    if ui.button("View Details").clicked() {
        state.show_details = true;
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
        ui.label("No robots defined yet.");
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
        ui.heading("Movements");
    });
    ui.separator();
    ui.add_space(16.0);
    ui.label("Coming soon...");
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
    ui.heading("End-Effector Pose (T_0_n)");
    ui.add_space(4.0);

    // Compute real FK for end-effector
    let domain_robot = state.robots[idx].to_robot();
    let base = Iso3::identity();
    let (_frames, effector) = forward_kinematics(base, &domain_robot);

    // Format the real transformation matrix
    let t = effector;
    let m = t.to_matrix();
    ui.monospace(format!(
        "┌ {:7.3} {:7.3} {:7.3} {:7.3} ┐\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         │ {:7.3} {:7.3} {:7.3} {:7.3} │\n\
         └                    ┘",
        m[(0, 0)],
        m[(0, 1)],
        m[(0, 2)],
        m[(0, 3)],
        m[(1, 0)],
        m[(1, 1)],
        m[(1, 2)],
        m[(1, 3)],
        m[(2, 0)],
        m[(2, 1)],
        m[(2, 2)],
        m[(2, 3)],
        m[(3, 0)],
        m[(3, 1)],
        m[(3, 2)],
        m[(3, 3)]
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
