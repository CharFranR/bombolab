use std::io::{self, Write};

use bombolab_core::math::dh::{DHParameter, DHParameterSymbolic, DHValue, format_symbolic_matrix, solve};

fn read_input(prompt: &str) -> String {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().to_string()
}

fn parse_dh_value(input: &str) -> DHValue {
    match input.parse::<f64>() {
        Ok(v) => DHValue::Num(v),
        Err(_) => DHValue::Sym(input.to_string()),
    }
}

fn main() {
    println!("=== DH Solver ===\n");

    // Modo de ángulos
    let angle_mode = loop {
        println!("Unidad de ángulos:");
        println!("  1 = Grados");
        println!("  2 = Radianes");
        let choice = read_input("Opción: ");
        match choice.as_str() {
            "1" => break "grados",
            "2" => break "radianes",
            _ => println!("Opción inválida\n"),
        }
    };

    let n: usize = read_input("\nNúmero de eslabones: ").parse().expect("número válido");

    let mut table: Vec<DHParameterSymbolic> = Vec::new();

    for i in 0..n {
        println!("\n--- Eslabón {} ---", i + 1);
        let alpha = parse_dh_value(&read_input(&format!("  α ({}): ", angle_mode)));
        let a = parse_dh_value(&read_input("  a: "));
        let d = parse_dh_value(&read_input("  d: "));
        let theta = parse_dh_value(&read_input(&format!("  θ ({}): ", angle_mode)));
        table.push(DHParameterSymbolic::new(alpha, a, d, theta));
    }

    // Verificar si todos son numéricos
    let all_numeric = table.iter().all(|p| p.is_numeric());

    if all_numeric {
        // Modo numérico - convertir ángulos a radianes si es necesario
        let numeric_table: Vec<DHParameter> = table.iter().map(|p| {
            let alpha = p.alpha.as_num().unwrap();
            let a = p.a.as_num().unwrap();
            let d = p.d.as_num().unwrap();
            let theta = p.theta.as_num().unwrap();

            let (alpha_rad, theta_rad) = if angle_mode == "grados" {
                (alpha.to_radians(), theta.to_radians())
            } else {
                (alpha, theta)
            };

            DHParameter::new(alpha_rad, a, d, theta_rad)
        }).collect();
        let solution = solve(&numeric_table);
        println!("\n{}", solution);
    } else {
        // Modo simbólico
        let w = 10;
        let vals: Vec<(String,String,String,String)> = table.iter().map(|p| {
            (p.alpha.to_string(), p.a.to_string(), p.d.to_string(), p.theta.to_string())
        }).collect();

        println!("\n  TABLA DH (simbólica)");
        println!("    {:>2} | {:<w$} | {:<w$} | {:<w$} | {:<w$}", "i", "α", "a", "d", "θ", w=w);
        println!("  --+-{}-+-{}-+-{}-+-{}", "-".repeat(w), "-".repeat(w), "-".repeat(w), "-".repeat(w));
        for (i, (alpha, a, d, theta)) in vals.iter().enumerate() {
            println!("  {:>2} | {:<w$} | {:<w$} | {:<w$} | {:<w$}", i + 1, alpha, a, d, theta, w=w);
        }

        println!("\n  MATRICES A_i (simbólicas)");
        for (i, p) in table.iter().enumerate() {
            println!("\n  A{}:", i + 1);
            println!("{}", format_symbolic_matrix(p, angle_mode));
        }
    }
}
