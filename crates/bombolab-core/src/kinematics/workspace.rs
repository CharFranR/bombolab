use crate::kinematics::singularity::{
    SingularityLevel, SingularityThresholds, WaypointMetrics, classify, reduced_jacobian,
    waypoint_metrics,
};
use crate::math::rng::Xoshiro256StarStar;
use crate::math::{Iso3, Vec3};
use crate::robot::{Robot, base_transform, fabri_creator};

use super::forward::forward_kinematics;
use super::ik::build_robot;

const STRIDE: usize = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WorkspaceMode {
    #[default]
    DrawingPlane,
    Full5Dof,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorkspaceStats {
    pub bounds_min: Option<[f64; 3]>,
    pub bounds_max: Option<[f64; 3]>,
    pub centroid: Option<[f64; 3]>,
    pub reach: Option<f64>,
    pub n_valid: usize,
    pub n_rejected: usize,
}

pub struct WorkspaceSampler {
    rng: Xoshiro256StarStar,
    mode: WorkspaceMode,
    robot: Robot,
    base: Iso3,
    n_valid: usize,
    n_rejected: usize,
    min: [f64; 3],
    max: [f64; 3],
    sum: [f64; 3],
    reach: f64,
}

impl WorkspaceSampler {
    pub fn new(seed: u64, mode: WorkspaceMode) -> Self {
        Self {
            rng: Xoshiro256StarStar::new(seed),
            mode,
            robot: fabri_creator(),
            base: base_transform(),
            n_valid: 0,
            n_rejected: 0,
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
            sum: [0.0; 3],
            reach: 0.0,
        }
    }

    fn joint_limits(&self, index: usize) -> (f64, f64) {
        let joint = &self.robot.segments[index].joint;
        (joint.value_min, joint.value_max)
    }

    fn uniform(&mut self, min: f64, max: f64) -> f64 {
        min + (max - min) * self.rng.next_f64_uniform()
    }

    fn next_q(&mut self) -> Option<[f64; 5]> {
        let mut q = [0.0; 5];
        for (i, item) in q.iter_mut().enumerate().take(3) {
            let (lo, hi) = self.joint_limits(i);
            *item = self.uniform(lo, hi);
        }
        match self.mode {
            WorkspaceMode::DrawingPlane => {
                q[3] = 0.0;
                q[4] = -(q[1] + q[2]);
                let (lo, hi) = self.joint_limits(4);
                if q[4] < lo || q[4] > hi {
                    self.n_rejected += 1;
                    return None;
                }
            }
            WorkspaceMode::Full5Dof => {
                for (i, item) in q.iter_mut().enumerate().skip(3) {
                    let (lo, hi) = self.joint_limits(i);
                    *item = self.uniform(lo, hi);
                }
            }
        }
        Some(q)
    }

    fn accept(&mut self, q: [f64; 5]) -> (Vec3, WaypointMetrics, SingularityLevel) {
        let tool = *self.robot.tool().pose();
        let robot_q = build_robot(&self.robot, &q);
        let (frames, _) = forward_kinematics(self.base, &robot_q);
        let p_ee = (frames.last().unwrap() * tool).translation.vector;
        let jr = reduced_jacobian(&self.robot, &q, &self.base, &tool);
        let metrics = waypoint_metrics(&jr);
        let level = classify(&metrics, &SingularityThresholds::default());
        self.n_valid += 1;
        for (v, (mn, (mx, sm))) in p_ee.iter().zip(
            self.min
                .iter_mut()
                .zip(self.max.iter_mut().zip(self.sum.iter_mut())),
        ) {
            *mn = (*mn).min(*v);
            *mx = (*mx).max(*v);
            *sm += *v;
        }
        let norm = p_ee.norm();
        if norm > self.reach {
            self.reach = norm;
        }
        (p_ee, metrics, level)
    }

    pub fn sample_batch(&mut self, k: usize) -> Vec<f64> {
        let mut out = Vec::with_capacity(k * STRIDE);
        for _ in 0..k {
            if let Some(q) = self.next_q() {
                let (p, metrics, _level) = self.accept(q);
                out.extend_from_slice(&[p[0], p[1], p[2], metrics.sigma_min, metrics.kappa]);
            }
        }
        out
    }

    pub fn stats(&self) -> WorkspaceStats {
        if self.n_valid == 0 {
            return WorkspaceStats {
                bounds_min: None,
                bounds_max: None,
                centroid: None,
                reach: None,
                n_valid: 0,
                n_rejected: self.n_rejected,
            };
        }
        WorkspaceStats {
            bounds_min: Some(self.min),
            bounds_max: Some(self.max),
            centroid: Some([
                self.sum[0] / self.n_valid as f64,
                self.sum[1] / self.n_valid as f64,
                self.sum[2] / self.n_valid as f64,
            ]),
            reach: Some(self.reach),
            n_valid: self.n_valid,
            n_rejected: self.n_rejected,
        }
    }
}

#[cfg(test)]
#[path = "workspace_tests.rs"]
mod workspace_tests;
