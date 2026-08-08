use std::env;
use std::process::ExitCode;
use std::time::{SystemTime, UNIX_EPOCH};

use bombolab_core::kinematics::{
    SingularityLevel, SingularityThresholds, WaypointMetrics, WorkspaceMode, WorkspaceSampler,
    classify,
};

const WORST_N: usize = 10;
const STRIDE: usize = 5;

fn usage() {
    eprintln!("Usage: workspace-report <n> <mode> [seed]");
    eprintln!("       workspace-report --n <n> [--mode drawing-plane|full-5dof] [--seed <seed>]");
}

fn parse_mode(s: &str) -> Option<WorkspaceMode> {
    match s {
        "drawing-plane" => Some(WorkspaceMode::DrawingPlane),
        "full-5dof" => Some(WorkspaceMode::Full5Dof),
        _ => None,
    }
}

fn parse_args(args: &[String]) -> Option<(usize, WorkspaceMode, u64)> {
    let mut n: Option<usize> = None;
    let mut mode = WorkspaceMode::DrawingPlane;
    let mut seed: Option<u64> = None;
    let mut positional: Vec<&str> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--n" => {
                n = Some(args.get(i + 1)?.parse().ok()?);
                i += 2;
            }
            "--mode" => {
                mode = parse_mode(args.get(i + 1)?)?;
                i += 2;
            }
            "--seed" => {
                seed = Some(args.get(i + 1)?.parse().ok()?);
                i += 2;
            }
            flag if flag.starts_with("--") => return None,
            value => {
                positional.push(value);
                i += 1;
            }
        }
    }
    if let Some(value) = positional.first() {
        n = Some(value.parse().ok()?);
    }
    if let Some(value) = positional.get(1) {
        mode = parse_mode(value)?;
    }
    if let Some(value) = positional.get(2) {
        seed = Some(value.parse().ok()?);
    }
    let seed = seed.unwrap_or_else(|| {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0)
    });
    Some((n?, mode, seed))
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    let (n, mode, seed) = match parse_args(&args) {
        Some(parsed) => parsed,
        None => {
            usage();
            return ExitCode::FAILURE;
        }
    };
    let mut sampler = WorkspaceSampler::new(seed, mode);
    let batch = sampler.sample_batch(n);
    let stats = sampler.stats();
    let mode_str = match mode {
        WorkspaceMode::DrawingPlane => "drawing-plane",
        WorkspaceMode::Full5Dof => "full-5dof",
    };
    println!(
        "workspace report: mode={mode_str} n={n} n_valid={} n_rejected={}",
        stats.n_valid, stats.n_rejected
    );
    match stats.bounds_min {
        Some(min) => {
            let max = stats.bounds_max.unwrap();
            let centroid = stats.centroid.unwrap();
            println!(
                "  bounds: min ({:8.2}, {:8.2}, {:8.2}) max ({:8.2}, {:8.2}, {:8.2})",
                min[0], min[1], min[2], max[0], max[1], max[2]
            );
            println!(
                "  centroid: ({:8.2}, {:8.2}, {:8.2})",
                centroid[0], centroid[1], centroid[2]
            );
            println!("  reach: {:.2} mm", stats.reach.unwrap());
        }
        None => println!("  no accepted samples"),
    }
    let mut worst: Vec<[f64; 5]> = Vec::with_capacity(WORST_N);
    for chunk in batch.chunks_exact(STRIDE) {
        let candidate = [chunk[3], chunk[0], chunk[1], chunk[2], chunk[4]];
        if worst.len() < WORST_N {
            worst.push(candidate);
            worst.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());
        } else if candidate[0] < worst.last().unwrap()[0] {
            worst.pop();
            worst.push(candidate);
            worst.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());
        }
    }
    println!("  worst {} by sigma_min:", worst.len());
    for (i, w) in worst.iter().enumerate() {
        let metrics = WaypointMetrics {
            sigma_min: w[0],
            kappa: w[4],
            yoshikawa: 0.0,
            sv: [0.0; 3],
        };
        let level = classify(&metrics, &SingularityThresholds::default());
        let level_str = match level {
            SingularityLevel::Ok => "ok",
            SingularityLevel::Warn => "warn",
            SingularityLevel::Block => "block",
        };
        println!(
            "  #{} ({:8.2}, {:8.2}, {:8.2}) sigma_min={:7.2} kappa={:9.2} level={}",
            i + 1,
            w[1],
            w[2],
            w[3],
            w[0],
            w[4],
            level_str
        );
    }
    ExitCode::SUCCESS
}
