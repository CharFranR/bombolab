use std::io::{self, Write};

use nalgebra::{Isometry3, Vector3};

use super::hmatrix::{make_movement, Movement};

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
