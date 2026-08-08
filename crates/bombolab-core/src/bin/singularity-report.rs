use std::env;
use std::fs;
use std::process::ExitCode;

use bombolab_core::kinematics::{
    GateReason, SingularityLevel, SingularityThresholds, analyze_path,
};
use bombolab_core::math::Iso3;
use bombolab_core::robot::{base_transform, fabri_creator};
use bombolab_core::trajectory::MotionCommand;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: singularity-report <waypoint-file>");
        return ExitCode::FAILURE;
    }
    let content = match fs::read_to_string(&args[1]) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cannot read {}: {e}", args[1]);
            return ExitCode::FAILURE;
        }
    };
    let mut commands: Vec<MotionCommand> = Vec::new();
    for (lineno, line) in content.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parts: Vec<f64> = line
            .split_whitespace()
            .filter_map(|p| p.parse().ok())
            .collect();
        if parts.len() < 3 {
            eprintln!("{}:{}: expected 'x y z'", args[1], lineno + 1);
            return ExitCode::FAILURE;
        }
        commands.push(MotionCommand::MoveLinear {
            target: [parts[0], parts[1], parts[2]],
            speed: 1.0,
        });
    }
    if commands.is_empty() {
        eprintln!("no waypoints in {}", args[1]);
        return ExitCode::FAILURE;
    }
    let robot = fabri_creator();
    let base: Iso3 = base_transform();
    let report = analyze_path(&robot, &base, &commands, &SingularityThresholds::default());
    println!(
        "singularity report: {} waypoints sampled, {} worst",
        report.sampled,
        report.worst.len()
    );
    for wp in &report.worst {
        let level = match wp.level {
            SingularityLevel::Ok => "ok",
            SingularityLevel::Warn => "warn",
            SingularityLevel::Block => "block",
        };
        let reason = match wp.reason {
            GateReason::Metrics => "",
            GateReason::IkNonConvergence => " (ik non-convergence)",
        };
        println!(
            "  #{} target ({:8.1}, {:8.1}, {:8.1}) sigma_min={:7.2} kappa={:9.2} yoshikawa={:9.3} level={}{}",
            wp.index,
            wp.target[0],
            wp.target[1],
            wp.target[2],
            wp.metrics.sigma_min,
            wp.metrics.kappa,
            wp.metrics.yoshikawa,
            level,
            reason
        );
    }
    ExitCode::SUCCESS
}
