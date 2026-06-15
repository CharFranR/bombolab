use std::{collections::btree_map::Values, io::{self, Write}};

use nalgebra::{Isometry3, Vector3, base};

use super::hmatrix::{make_movement, Movement};

use crate::{domain::{DHParams, Joint, JointType, Result, Robot, Segment}, kinematics::dh_parameters::forward_kinematics};

fn leer_vec3(prompt: &str) -> Vector3<f64> {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    let parts: Vec<f64> = input
        .trim()
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    Vector3::new(parts[0], parts[1], parts[2])
}

fn leer_f64(prompt: &str) -> f64 {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().parse().unwrap()
}

fn leer_i32(prompt: &str) -> i32 {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().parse().unwrap()
}

fn leer_joint_type(prompt: &str) -> JointType {
    loop {
        print!("{} (0 = Revolute, 1 = Prismatic): ", prompt);
        io::stdout().flush().unwrap();
        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();
        match input.trim() {
            "0" => return JointType::Revolute,
            "1" => return JointType::Prismatic,
            _ => println!("Opción inválida, ingresá 0 o 1."),
        }
    }
}

fn leer_movimiento(n: usize) -> Movement {
    println!("\n--- Movimiento {} ---", n);
    let translation = leer_vec3("  Traslación (x y z): ");
    let axis = leer_vec3("  Eje de rotación (x y z): ");
    let grados = leer_f64("  Ángulo (grados): ");
    let isometry = leer_f64("  Orden (1 = rot+tras, 0 = tras+rot): ") != 0.0;

    Movement {
        translation,
        angles: grados.to_radians(),
        axis,
        isometry,
    }
}

pub fn run() {
    println!("=== Prueba de make_movement ===\n");

    let initial_pos = leer_vec3("Posición inicial del efector (x y z): ");
    let initial = Isometry3::new(initial_pos, nalgebra::zero());

    let mov1 = leer_movimiento(1);
    let mov2 = leer_movimiento(2);

    let (trayectoria, final_pos) = make_movement(initial, &[mov1, mov2]);

    println!("\n=== Resultados ===");
    for (i, pose) in trayectoria.iter().enumerate() {
        let p = pose.translation.vector;
        println!(
            "  Tras movimiento {}: ({:.4}, {:.4}, {:.4})",
            i + 1,
            p.x,
            p.y,
            p.z
        );
    }

    let fp = final_pos.translation.vector;
    println!(
        "\n  Posición final del efector: ({:.4}, {:.4}, {:.4})",
        fp.x, fp.y, fp.z
    );
}



pub fn make_robot() -> Robot {
    println!("=== Construyamos el robot ===\n");
    let num_segmentos = leer_i32("Número de segmentos: ") as usize;

    let mut segments: Vec<Segment> = Vec::new();

    for i in 0..num_segmentos {
        println!("\nSegmento {}", i + 1);
        let joint_type = leer_joint_type("Tipo de joint");
        let value = leer_f64("  Ángulo (grados): ").to_radians();
        let value_max = leer_f64("  Ángulo máximo (grados): ").to_radians();
        let value_min = leer_f64("  Ángulo mínimo (grados): ").to_radians();

        println!("Parámetros DH");
        let theta = leer_f64("  Theta (grados): ").to_radians();
        let alpha = leer_f64("  Alpha (grados): ").to_radians();
        let d = leer_f64("  d: ");
        let a = leer_f64("  a: ");

        let new_joint = Joint::new(joint_type, value, value_max, value_min);
        let new_link = DHParams::new(theta, d, a, alpha);

        segments.push(Segment::new(new_joint, new_link));
    }

    Robot::new(segments)
}

pub fn move_robot(robot: &mut Robot) -> Result<()> {
    println!("\nMovamos el robot \n");

    for i in 0..robot.dof() {
        println!("--- Joint {} ---", i + 1);
        let segment = robot.segment_mut(i)?;
        let old_val = segment.joint.value.to_degrees();
        let new_val = leer_f64(&format!("  Nuevo ángulo (grados) [actual: {:.2}]: ", old_val));
        segment.joint.value = new_val.to_radians();
    }

    println!("\n Movimiento completado");

    Ok(())
}

pub fn test_forward_kinematics() {
    let mut pajatron = make_robot();

    let base = Isometry3::<f64>::identity();

    let (frames, effector) = forward_kinematics(base, &pajatron);

    println!("\nPosiciones iniciales");
    for (i, f) in frames.iter().enumerate() {
        let p = f.translation.vector;
        println!("  Frame {}: ({:.4}, {:.4}, {:.4})", i + 1, p.x, p.y, p.z);
    }
    let ep = effector.translation.vector;
    println!("Efector final: ({:.4}, {:.4}, {:.4})", ep.x, ep.y, ep.z);

    if let Err(e) = move_robot(&mut pajatron) {
        println!("Error al mover robot: {}", e);
        return;
    }

    let (_frames2, effector2) = forward_kinematics(base, &pajatron);
    let ep2 = effector2.translation.vector;
    println!(
        "\nEfector después de mover: ({:.4}, {:.4}, {:.4})",
        ep2.x, ep2.y, ep2.z
    );
}