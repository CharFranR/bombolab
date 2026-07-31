use std::f64::consts::FRAC_PI_2;

use crate::math::Iso3;

use super::joint::{Joint, JointType};
use super::link::DHParams;
use super::segment::{Robot, Segment};

/// Create the FABRI Creator robot configuration.
///
/// Robot de 5 GDL con servos SG90 y Arduino Nano (control directo por pins,
/// sin placa PCA9685 — nota de variante de hardware si se agrega un driver).
/// Basado en la tabla de `docs/fabri-creator/table-definition.md`.
pub fn fabri_creator() -> Robot {
    // Límites del modelo = imagen inversa exacta del rango de servo
    // [5°, 175°] que aceptan firmware (main.cpp) y ServoCommand:
    //   q_eff = dir · (servo − offset)  →  servo(q_min) = 5, servo(q_max) = 175
    // Así el clamp del mapper NUNCA recorta q dentro del modelo (no hay
    // configuraciones prometidas que el hardware no pueda ejecutar).
    //
    // Nota histórica: el rango fue [10,170] hasta julio-2026 (commit
    // dc0bcc6 "change angles movement capacity from 10 to 170"); la
    // expansión a [5,175] restaura el recorrido completo del SG90 y
    // recupera los límites originales de J1/J2/J5.
    let q_j1_j2 = 85.0_f64.to_radians(); // servo = 90 − q → [5, 175] ⇒ q ∈ [−85, 85]
    let q_j3_max = 85.0_f64.to_radians(); // servo = 81 + q → 175 ⇒ q ≤ 94; tope físico 85 (sin recorte: servo 166)
    let q_j3_min = (-76.0_f64).to_radians(); // servo = 81 + q → 5 ⇒ q ≥ −76
    let q_j4_max = 85.0_f64.to_radians(); // servo = 95 − q → 5 ⇒ q ≤ 90; tope físico 85 (sin recorte: servo 10)
    let q_j4_min = (-80.0_f64).to_radians(); // servo = 95 − q → 175 ⇒ q ≥ −80

    let segments = vec![
        // Joint 1 — Base (Yaw)
        // θ=0,  d=85,  a=15,  α=-π/2  (d corregido: 7cm + 15mm = 85mm)
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_j1_j2, -q_j1_j2),
            DHParams::new(0.0, 85.0, 15.0, -FRAC_PI_2),
        ),
        // Joint 2 — Shoulder (eleva el brazo)
        // θ=-π/2,  d=0,  a=120,  α=0  (a corregido: 12cm)
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_j1_j2, -q_j1_j2),
            DHParams::new(-FRAC_PI_2, 0.0, 120.0, 0.0),
        ),
        // Joint 3 — Elbow (extiende el antebrazo)
        // θ=+π/2,  d=0,  a=90,  α=-π/2  (a corregido: 9cm)
        Segment::new(
            Joint::new(JointType::Revolute, 0.0, q_j3_max, q_j3_min),
            DHParams::new(FRAC_PI_2, 0.0, 90.0, -FRAC_PI_2),
        ),
        // Joint 4 — Wrist Roll (twist: rota sobre eje X)
        // θ=0,  d=15,  a=35,  α=+π/2
        Segment::new(
            Joint::new(JointType::Twist, 0.0, q_j4_max, q_j4_min),
            DHParams::new(0.0, 15.0, 35.0, FRAC_PI_2),
        ),
        // Joint 5 — Wrist Pitch
        // θ=0,  d=0,  a=0,  α=0
        // dir=-1, offset=60° → servo = 60° − q.
        // Con el rango [5,175]: servo 5°→ q=55°, servo 175°→ q=−115°  (q ∈ [−115, 55]).
        // Este era el rango que el comentario histórico describía — era
        // correcto para [5,175], pero el código usaba [10,170] (q=55 → servo 5
        // quedaba fuera). Al expandir el rango del servo, el comentario
        // original vuelve a ser matemáticamente exacto.
        Segment::new(
            Joint::new(
                JointType::Revolute,
                0.0,
                55.0_f64.to_radians(),
                (-115.0_f64).to_radians(),
            ),
            DHParams::new(0.0, 0.0, 0.0, 0.0),
        ),
    ];

    // Home pose: servo angles en q = [0; 5]
    // Ajustes para el robot físico:
    // J3 (Codo): dir=+1 → offset = 90° - 9° = 81°
    // J4 (Roll): dir=-1 → offset = 90° + 5° = 95°
    // J5 (Pitch): dir=-1 → offset = 90° + 8° = 98°
    let home_pose = vec![
        90.0_f64.to_radians(),
        90.0_f64.to_radians(),
        81.0_f64.to_radians(),
        95.0_f64.to_radians(),
        60.0_f64.to_radians(),
    ];

    // Offset = home_pose, así q_to_servo(&[0; 5]) = home_pose
    let servo_offsets = home_pose.clone();

    // Direcciones de giro por tabla:
    // J1: Anti Horario (-1), J2: Anti Horario (-1), J3: Horario (+1),
    // J4: Anti Horario (-1), J5: Anti Horario (-1)
    let servo_directions = vec![-1.0, -1.0, 1.0, -1.0, -1.0];

    Robot::with_directions(segments, home_pose, servo_offsets, servo_directions)
}

/// Base transform: vertical offset from ground to joint 1.
///
/// Translation: (0, 0, 57 mm), rotation: identity.
pub fn base_transform() -> Iso3 {
    Iso3::translation(0.0, 0.0, 57.0)
}

/// Tool transform: from J5 frame to marker tip.
///
/// The marker is mounted perpendicular to the last joint's rotation axis.
/// Translation: (75 mm, 0, 0), rotation: identity.
pub fn tool_transform() -> Iso3 {
    Iso3::translation(75.0, 0.0, 0.0)
}
