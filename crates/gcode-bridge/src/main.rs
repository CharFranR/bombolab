//! Command-line interface for the G-code bridge.
//!
//! Reads a CIPRA G-code file, maps it, validates reachability with a dry-run
//! IK solve, and executes the drawing either in simulation (default, no
//! hardware) or against a real Arduino Nano over serial.
//!
//! # Default is safe
//!
//! Without `--port` the plan is only simulated; pass `--port` only when a
//! FABRI Creator is actually connected and ready to draw.

use std::path::PathBuf;

use bombolab_core::communication::{ArduinoNano, ANGLE_MAX, ANGLE_MIN};

use gcode_bridge::{GcodeBridge, MappingConfig, SerialSink, SimulationSink};

struct CliArgs {
    input: PathBuf,
    port: Option<String>,
    scale: Option<f64>,
    z_draw: Option<f64>,
    z_travel: Option<f64>,
    gripper: Option<u8>,
    export: Option<PathBuf>,
}

fn usage() -> String {
    format!(
        "gcode-bridge — dibuja G-code de CIPRA con el FABRI Creator

Usage:
  gcode-bridge <input.gcode> [options]

Options:
  --port <name>    Conectar al Arduino Nano por serial (por defecto: SIMULA).
  --scale <s>      Escala explícita (> 0). Omisión: auto-escala para caber.
  --z-draw <mm>    Altura de dibujo (por defecto: {}).
  --z-travel <mm>  Altura de viaje / pluma arriba (por defecto: {}).
  --gripper <5-175> Valor del gripper a enviar en grados (por defecto: 90).
  --export <file>  Volcar el plan a JSON de pasos 'steps' para la web y salir.
  -h, --help       Muestra esta ayuda.
",
        MappingConfig::default().z_draw,
        MappingConfig::default().z_travel
    )
}

/// Return the value consumed by a value-taking flag, or an error when the
/// value is missing or the next token is itself another flag.
fn take_value<'a>(args: &'a [String], i: usize, flag: &str) -> Result<&'a str, String> {
    match args.get(i) {
        Some(value) if !value.starts_with("--") => Ok(value.as_str()),
        _ => Err(format!("{flag} requiere un valor")),
    }
}

fn parse_args(args: &[String]) -> Result<CliArgs, String> {
    if args.is_empty() {
        return Err("falta el archivo .gcode".into());
    }
    let input = args[0].as_str();
    if input == "-h" || input == "--help" {
        return Err("help".into());
    }
    let mut port = None;
    let mut scale = None;
    let mut z_draw = None;
    let mut z_travel = None;
    let mut gripper = None;
    let mut export = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--port" => {
                i += 1;
                port = Some(take_value(args, i, "--port")?.to_string());
            }
            "--export" => {
                i += 1;
                export = Some(PathBuf::from(take_value(args, i, "--export")?));
            }
            "--scale" => {
                i += 1;
                let raw = take_value(args, i, "--scale")?;
                let value = raw
                    .parse::<f64>()
                    .map_err(|_| "--scale debe ser un número")?;
                if !value.is_finite() || value <= 0.0 {
                    return Err("--scale debe ser un número finito positivo".into());
                }
                scale = Some(value);
            }
            "--z-draw" => {
                i += 1;
                let raw = take_value(args, i, "--z-draw")?;
                let value = raw
                    .parse::<f64>()
                    .map_err(|_| "--z-draw debe ser un número")?;
                if !value.is_finite() {
                    return Err("--z-draw debe ser un número finito".into());
                }
                z_draw = Some(value);
            }
            "--z-travel" => {
                i += 1;
                let raw = take_value(args, i, "--z-travel")?;
                let value = raw
                    .parse::<f64>()
                    .map_err(|_| "--z-travel debe ser un número")?;
                if !value.is_finite() {
                    return Err("--z-travel debe ser un número finito".into());
                }
                z_travel = Some(value);
            }
            "--gripper" => {
                i += 1;
                let raw = take_value(args, i, "--gripper")?;
                let value = raw
                    .parse::<u8>()
                    .map_err(|_| "--gripper debe ser un número entero (5-175)")?;
                // Wire contract: the serial protocol only accepts servo angles
                // within [ANGLE_MIN, ANGLE_MAX] degrees (bombolab-core).
                if value < ANGLE_MIN as u8 || value > ANGLE_MAX as u8 {
                    return Err("--gripper debe estar entre 5 y 175 grados".into());
                }
                gripper = Some(value);
            }
            other => return Err(format!("argumento desconocido: {other}")),
        }
        i += 1;
    }

    Ok(CliArgs {
        input: PathBuf::from(input),
        port,
        scale,
        z_draw,
        z_travel,
        gripper,
        export,
    })
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cli = match parse_args(&args) {
        Ok(c) => c,
        Err(m) if m == "help" => {
            print!("{}", usage());
            return;
        }
        Err(m) => {
            eprintln!("error: {m}");
            eprintln!();
            eprint!("{}", usage());
            std::process::exit(1);
        }
    };

    // Build the mapping config with any user overrides.
    let def = MappingConfig::default();
    let config = MappingConfig {
        scale: cli.scale,
        z_draw: cli.z_draw.unwrap_or(def.z_draw),
        z_travel: cli.z_travel.unwrap_or(def.z_travel),
        ..def
    };

    let bridge = GcodeBridge::new(config);

    // Read the drawing.
    let gcode = match std::fs::read_to_string(&cli.input) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("error: no se pudo leer {}: {e}", cli.input.display());
            std::process::exit(1);
        }
    };

    // Plan (this dry-runs IK on every point, strict reachability).
    let plan = match bridge.plan(&gcode) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("error al planificar: {e}");
            std::process::exit(1);
        }
    };

    let strokes = plan.strokes.len();
    println!(
        "Plan: {strokes} trazos, {} comandos, escala efectiva {:.4}",
        plan.target_count(),
        plan.scale
    );

    // Export the plan to the web viewer's JSON document and exit.
    if let Some(path) = &cli.export {
        let gripper = cli.gripper.unwrap_or(90);
        let json = match plan.to_trajectory_json(Some(gripper)) {
            Ok(json) => json,
            Err(e) => {
                eprintln!("error: no se pudo serializar la trayectoria: {e}");
                std::process::exit(1);
            }
        };
        match std::fs::write(path, json) {
            Ok(()) => {
                println!("Trayectoria exportada a {}", path.display());
                return;
            }
            Err(e) => {
                eprintln!("error: no se pudo escribir {}: {e}", path.display());
                std::process::exit(1);
            }
        }
    }

    // Execute on hardware or simulate.
    let gripper = cli.gripper.unwrap_or(90);
    match cli.port {
        Some(port) => {
            print!("Conectando a {port} ... ");
            match ArduinoNano::connect(&port) {
                Ok(nano) => {
                    println!("ok");
                    let mut sink = SerialSink { arduino: nano };
                    match bridge.execute(&plan, &mut sink, gripper) {
                        Ok(n) => println!("Dibujado: {n} comandos enviados y verificados."),
                        Err(e) => {
                            eprintln!("\nerror durante la ejecución: {e}");
                            std::process::exit(1);
                        }
                    }
                }
                Err(e) => {
                    eprintln!("falló: {e}");
                    std::process::exit(1);
                }
            }
        }
        None => {
            println!("(simulación — conéctate con --port para dibujar en hardware)");
            let mut sink = SimulationSink::default();
            match bridge.execute(&plan, &mut sink, gripper) {
                Ok(n) => println!("Simulación completa: {n} comandos generados."),
                Err(e) => {
                    eprintln!("error durante la simulación: {e}");
                    std::process::exit(1);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn parses_valid_args() {
        let cli = parse_args(&args(&["draw.gcode", "--scale", "0.5", "--port", "COM3"]))
            .expect("valid args");
        assert_eq!(cli.input, PathBuf::from("draw.gcode"));
        assert_eq!(cli.scale, Some(0.5));
        assert_eq!(cli.port.as_deref(), Some("COM3"));
        assert_eq!(cli.export, None);
    }

    #[test]
    fn scale_rejects_zero_and_negative() {
        assert!(parse_args(&args(&["d.gcode", "--scale", "0"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--scale", "-1"])).is_err());
    }

    #[test]
    fn scale_rejects_non_numeric_and_non_finite() {
        assert!(parse_args(&args(&["d.gcode", "--scale", "abc"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--scale", "NaN"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--scale", "inf"])).is_err());
    }

    #[test]
    fn scale_accepts_positive_value() {
        let cli = parse_args(&args(&["d.gcode", "--scale", "0.25"])).expect("positive scale");
        assert_eq!(cli.scale, Some(0.25));
    }

    #[test]
    fn gripper_accepts_range_bounds() {
        let cli = parse_args(&args(&["d.gcode", "--gripper", "5"])).expect("min gripper");
        assert_eq!(cli.gripper, Some(5));
        let cli = parse_args(&args(&["d.gcode", "--gripper", "175"])).expect("max gripper");
        assert_eq!(cli.gripper, Some(175));
        let cli = parse_args(&args(&["d.gcode", "--gripper", "90"])).expect("default gripper");
        assert_eq!(cli.gripper, Some(90));
    }

    #[test]
    fn gripper_rejects_out_of_range() {
        assert!(parse_args(&args(&["d.gcode", "--gripper", "4"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--gripper", "176"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--gripper", "0"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--gripper", "255"])).is_err());
    }

    #[test]
    fn gripper_rejects_non_numeric() {
        assert!(parse_args(&args(&["d.gcode", "--gripper", "abc"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--gripper", "-5"])).is_err());
    }

    #[test]
    fn z_flags_reject_non_finite() {
        assert!(parse_args(&args(&["d.gcode", "--z-draw", "NaN"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--z-draw", "inf"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--z-draw", "-inf"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--z-travel", "NaN"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--z-travel", "Infinity"])).is_err());
    }

    #[test]
    fn z_flags_accept_finite_values() {
        let cli = parse_args(&args(&["d.gcode", "--z-draw", "-10.5", "--z-travel", "86"]))
            .expect("finite z values");
        assert_eq!(cli.z_draw, Some(-10.5));
        assert_eq!(cli.z_travel, Some(86.0));
    }

    #[test]
    fn value_flags_require_a_value() {
        assert!(parse_args(&args(&["d.gcode", "--port"])).is_err());
        assert!(parse_args(&args(&["d.gcode", "--export"])).is_err());
    }

    #[test]
    fn value_flag_consuming_next_flag_is_rejected() {
        let err = match parse_args(&args(&["d.gcode", "--port", "--export", "out.json"])) {
            Err(e) => e,
            Ok(_) => panic!("missing value must be reported"),
        };
        assert!(err.contains("--port"));
        assert!(parse_args(&args(&["d.gcode", "--scale", "--port", "COM3"])).is_err());
    }
}