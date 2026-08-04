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

use bombolab_core::communication::ArduinoNano;

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
  --scale <s>      Escala explícita (0..1). Omisión: auto-escala para caber.
  --z-draw <mm>    Altura de dibujo (por defecto: {}).
  --z-travel <mm>  Altura de viaje / pluma arriba (por defecto: {}).
  --gripper <0-255> Valor del gripper a enviar (por defecto: 90).
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
                z_draw = Some(
                    take_value(args, i, "--z-draw")?
                        .parse::<f64>()
                        .map_err(|_| "--z-draw debe ser un número")?,
                );
            }
            "--z-travel" => {
                i += 1;
                z_travel = Some(
                    take_value(args, i, "--z-travel")?
                        .parse::<f64>()
                        .map_err(|_| "--z-travel debe ser un número")?,
                );
            }
            "--gripper" => {
                i += 1;
                gripper = Some(
                    take_value(args, i, "--gripper")?
                        .parse::<u8>()
                        .map_err(|_| "--gripper debe ser un byte (0-255)")?,
                );
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
        let json = plan.to_trajectory_json(Some(gripper));
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