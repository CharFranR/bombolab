use nalgebra::SMatrix;

use crate::math::{Iso3, Vec3};
use crate::robot::Robot;
use crate::trajectory::MotionCommand;

use super::forward::forward_kinematics;
use super::ik::{IkSolver, build_robot, position_jacobian, solve_drawing_plane_ik};

const SAMPLE_STEP_MM: f64 = 4.0;

const MAX_SAMPLED: usize = 5000;

const WORST_N: usize = 10;

pub fn reduced_jacobian(
    robot: &Robot,
    q: &[f64; 5],
    base: &Iso3,
    tool: &Iso3,
) -> SMatrix<f64, 3, 3> {
    let robot_q = build_robot(robot, q);
    let (frames, _) = forward_kinematics(*base, &robot_q);
    let p_ee = (frames.last().unwrap() * tool).translation.vector;
    let j_full = position_jacobian(&robot_q, &frames, &p_ee, base, robot.dof().min(5));
    let mut jr = SMatrix::<f64, 3, 3>::zeros();
    for r in 0..3 {
        jr[(r, 0)] = j_full[(r, 0)];
        jr[(r, 1)] = j_full[(r, 1)] - j_full[(r, 4)];
        jr[(r, 2)] = j_full[(r, 2)] - j_full[(r, 4)];
    }
    jr
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaypointMetrics {
    pub sigma_min: f64,
    pub kappa: f64,
    pub yoshikawa: f64,
    pub sv: [f64; 3],
}

pub fn waypoint_metrics(jr: &SMatrix<f64, 3, 3>) -> WaypointMetrics {
    let svd = jr.svd(true, true);
    let mut sv = [
        svd.singular_values[0],
        svd.singular_values[1],
        svd.singular_values[2],
    ];
    sv.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let sigma_min = sv[0];
    let sigma_max = sv[2];
    let kappa = if sigma_min > 0.0 {
        sigma_max / sigma_min
    } else {
        f64::INFINITY
    };
    WaypointMetrics {
        sigma_min,
        kappa,
        yoshikawa: jr.determinant().abs(),
        sv,
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SingularityThresholds {
    pub warn_sigma_min: f64,
    pub warn_kappa: f64,
    pub block_sigma_min: f64,
    pub block_kappa: f64,
}

impl Default for SingularityThresholds {
    fn default() -> Self {
        Self {
            warn_sigma_min: 25.0,
            warn_kappa: 20.0,
            block_sigma_min: 5.0,
            block_kappa: 100.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SingularityLevel {
    Ok,
    Warn,
    Block,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateReason {
    Metrics,
    IkNonConvergence,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GateWaypoint {
    pub index: usize,
    pub target: [f64; 3],
    pub q: [f64; 5],
    pub metrics: WaypointMetrics,
    pub level: SingularityLevel,
    pub reason: GateReason,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GateReport {
    pub sampled: usize,
    pub worst: Vec<GateWaypoint>,
}

pub fn classify(metrics: &WaypointMetrics, thresholds: &SingularityThresholds) -> SingularityLevel {
    if metrics.sigma_min < thresholds.block_sigma_min || metrics.kappa > thresholds.block_kappa {
        SingularityLevel::Block
    } else if metrics.sigma_min < thresholds.warn_sigma_min || metrics.kappa > thresholds.warn_kappa
    {
        SingularityLevel::Warn
    } else {
        SingularityLevel::Ok
    }
}

fn plan_waypoints(commands: &[MotionCommand]) -> Vec<[f64; 3]> {
    let targets: Vec<[f64; 3]> = commands
        .iter()
        .filter_map(|cmd| match cmd {
            MotionCommand::MoveLinear { target, .. } => Some(*target),
            _ => None,
        })
        .collect();
    if targets.is_empty() {
        return Vec::new();
    }
    let mut counts: Vec<usize> = Vec::new();
    let mut prev = targets[0];
    for t in targets.iter().skip(1) {
        let len = (Vec3::new(t[0] - prev[0], t[1] - prev[1], t[2] - prev[2])).norm();
        counts.push((len / SAMPLE_STEP_MM).ceil() as usize);
        prev = *t;
    }
    let raw_total: usize = counts.iter().sum::<usize>() + 1;
    if raw_total > MAX_SAMPLED {
        let budget = MAX_SAMPLED - 1;
        let mut scaled: Vec<usize> = counts
            .iter()
            .map(|&n| ((n as f64 * budget as f64 / (raw_total - 1) as f64).floor()) as usize)
            .collect();
        let mut remaining = budget - scaled.iter().sum::<usize>();
        for c in scaled.iter_mut() {
            if remaining == 0 {
                break;
            }
            *c += 1;
            remaining -= 1;
        }
        counts = scaled;
    }
    let mut waypoints = vec![targets[0]];
    prev = targets[0];
    for (t, &n) in targets.iter().skip(1).zip(&counts) {
        for i in 1..=n {
            let f = i as f64 / n as f64;
            waypoints.push([
                prev[0] + f * (t[0] - prev[0]),
                prev[1] + f * (t[1] - prev[1]),
                prev[2] + f * (t[2] - prev[2]),
            ]);
        }
        prev = *t;
    }
    waypoints
}

fn solve_report_q(prev_q: &[f64]) -> [f64; 5] {
    prev_q[..5].try_into().unwrap()
}

pub fn analyze_path(
    robot: &Robot,
    base: &Iso3,
    commands: &[MotionCommand],
    thresholds: &SingularityThresholds,
) -> GateReport {
    let waypoints = plan_waypoints(commands);
    if waypoints.is_empty() {
        return GateReport {
            sampled: 0,
            worst: Vec::new(),
        };
    }
    let tool = *robot.tool().pose();
    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let mut prev_q = robot.kinematic_home();
    let mut worst: Vec<GateWaypoint> = Vec::with_capacity(WORST_N);
    for (index, target) in waypoints.iter().enumerate() {
        let wp = match solve_drawing_plane_ik(&solver, target, &prev_q, robot, base, &tool) {
            Ok(q) => {
                prev_q = q.to_vec();
                let metrics = waypoint_metrics(&reduced_jacobian(robot, &q, base, &tool));
                GateWaypoint {
                    index,
                    target: *target,
                    q,
                    level: classify(&metrics, thresholds),
                    reason: GateReason::Metrics,
                    metrics,
                }
            }
            Err(_) => {
                let q5 = solve_report_q(&prev_q);
                GateWaypoint {
                    index,
                    target: *target,
                    q: q5,
                    metrics: WaypointMetrics {
                        sigma_min: 0.0,
                        kappa: 0.0,
                        yoshikawa: 0.0,
                        sv: [0.0; 3],
                    },
                    level: SingularityLevel::Block,
                    reason: GateReason::IkNonConvergence,
                }
            }
        };
        if worst.len() < WORST_N {
            worst.push(wp);
            worst.sort_by(|a, b| {
                a.metrics
                    .sigma_min
                    .partial_cmp(&b.metrics.sigma_min)
                    .unwrap()
                    .then(a.index.cmp(&b.index))
            });
        } else if wp.metrics.sigma_min < worst.last().unwrap().metrics.sigma_min {
            worst.pop();
            worst.push(wp);
            worst.sort_by(|a, b| {
                a.metrics
                    .sigma_min
                    .partial_cmp(&b.metrics.sigma_min)
                    .unwrap()
                    .then(a.index.cmp(&b.index))
            });
        }
    }
    GateReport {
        sampled: waypoints.len(),
        worst,
    }
}

#[cfg(test)]
#[path = "singularity_tests.rs"]
mod singularity_tests;
