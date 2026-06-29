use std::io::{self, Write};

use bombolab_core::math::quaternion::{Quaternion, solve_add, solve_divide, solve_multiply, solve_subtract};

fn read_input(prompt: &str) -> String {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().to_string()
}

fn read_quaternion(label: &str) -> Quaternion {
    println!("  --- {} ---", label);
    let a: f64 = read_input("  a: ").parse().expect("número válido");
    let b: f64 = read_input("  b: ").parse().expect("número válido");
    let c: f64 = read_input("  c: ").parse().expect("número válido");
    let d: f64 = read_input("  d: ").parse().expect("número válido");
    Quaternion::new(a, b, c, d)
}

fn main() {
    println!("=== Quaternion Solver ===\n");

    let op = loop {
        println!("Operación:");
        println!("  1 = Suma");
        println!("  2 = Resta");
        println!("  3 = Multiplicación (Hamilton)");
        println!("  4 = División");
        let choice = read_input("Opción: ");
        match choice.as_str() {
            "1" => break "add",
            "2" => break "subtract",
            "3" => break "multiply",
            "4" => break "divide",
            _ => println!("Opción inválida\n"),
        }
    };

    let n: usize = read_input("\nNúmero de cuaterniones: ").parse().expect("número válido");

    let mut quats: Vec<Quaternion> = Vec::new();
    for i in 0..n {
        quats.push(read_quaternion(&format!("Cuaternion {}", i + 1)));
    }

    let result = match op {
        "add" => solve_add(&quats),
        "subtract" => solve_subtract(&quats),
        "multiply" => solve_multiply(&quats),
        "divide" => solve_divide(&quats),
        _ => unreachable!(),
    };

    println!("\n  Resultado: {}", result);
}
