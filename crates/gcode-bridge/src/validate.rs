//! Dry-run reachability validation of every drawing target.
//!
//! Before any command reaches hardware, the whole drawing is validated by
//! solving inverse kinematics for every mapped robot target. This is the
//! authoritative workspace check: the target rectangle in [`crate::workspace`]
//! is only a fast pre-fit, but the real question — *can the arm reach this
//! point while holding the marker vertical?* — is answered here.
//!
//! The validator runs in **strict mode** by design: if any target is
//! unreachable, the caller must abort instead of silently skipping points.

use bombolab_core::math::Iso3;
use bombolab_core::robot::Robot;
use bombolab_core::{
    base_transform, fabri_creator, tool_transform, IkError, IkSolver,
};
use bombolab_core::kinematics::solve_drawing_plane_ik;

/// A single unreachable target and why it failed.
#[derive(Debug, Clone)]
pub struct ReachabilityFailure {
    /// Index of the failing target.
    pub index: usize,
    /// The robot target `(x, y, z)` in millimetres.
    pub target: (f64, f64, f64),
    /// IK error explaining why it is unreachable.
    pub error: IkError,
}

/// Result of dry-running IK over the drawing targets.
#[derive(Debug, Clone)]
pub struct Validation {
    pub total: usize,
    pub reachable: usize,
    pub failures: Vec<ReachabilityFailure>,
}

impl Validation {
    /// `true` when every target is reachable (strict mode passes).
    pub fn is_reachable(&self) -> bool {
        self.failures.is_empty()
    }

    /// Fraction of reachable targets (for reporting), in `[0, 1]`.
    pub fn reachable_ratio(&self) -> f64 {
        if self.total == 0 {
            1.0
        } else {
            self.reachable as f64 / self.total as f64
        }
    }
}

/// Reusable reachability checker that owns the solver, robot and transforms
/// so multiple targets are validated without rebuilding per point.
pub struct DrawingValidator {
    solver: IkSolver,
    robot: Robot,
    base: Iso3,
    tool: Iso3,
    q_init: [f64; 5],
}

impl Default for DrawingValidator {
    fn default() -> Self {
        Self::new()
    }
}

impl DrawingValidator {
    /// Build a validator for the FABRI Creator using sane solver defaults.
    pub fn new() -> Self {
        // Tuned for the drawing-plane solver (DLS): 200 iterations, 1.0 mm
        // tolerance, 0.05 damping, 0.5 rad step.
        let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
        Self {
            solver,
            robot: fabri_creator(),
            base: base_transform(),
            tool: tool_transform(),
            q_init: [0.0; 5],
        }
    }

    /// Check a single target for reachability under the drawing constraint.
    pub fn is_reachable(&self, target: (f64, f64, f64)) -> bool {
        match solve_drawing_plane_ik(
            &self.solver,
            &[target.0, target.1, target.2],
            &self.q_init,
            &self.robot,
            &self.base,
            &self.tool,
        ) {
            Ok(_) => true,
            Err(_) => false,
        }
    }

    /// Dry-run every target and collect the unreachable ones.
    pub fn validate(&self, targets: &[(f64, f64, f64)]) -> Validation {
        let mut failures = Vec::new();
        for (index, &(x, y, z)) in targets.iter().enumerate() {
            let result = solve_drawing_plane_ik(
                &self.solver,
                &[x, y, z],
                &self.q_init,
                &self.robot,
                &self.base,
                &self.tool,
            );
            if let Err(error) = result {
                failures.push(ReachabilityFailure {
                    index,
                    target: (x, y, z),
                    error,
                });
            }
        }
        Validation {
            total: targets.len(),
            reachable: targets.len() - failures.len(),
            failures,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn validator() -> DrawingValidator {
        DrawingValidator::new()
    }

    #[test]
    fn home_square_points_are_in_workspace() {
        // The browser demo square around (200, ±25, 80) is reachable.
        let v = validator();
        assert!(v.is_reachable((200.0, -25.0, 80.0)));
        assert!(v.is_reachable((225.0, 0.0, 80.0)));
        assert!(v.is_reachable((200.0, 25.0, 80.0)));
    }

    #[test]
    fn obviously_out_of_reach_target_is_rejected() {
        // 600 mm from base with tool offset is beyond the arm's reach.
        let v = validator();
        assert!(!v.is_reachable((600.0, 0.0, 80.0)));
    }

    #[test]
    fn validation_collects_failures() {
        let v = validator();
        let targets = vec![(200.0, 0.0, 80.0), (900.0, 0.0, 80.0), (210.0, 25.0, 80.0)];
        let res = v.validate(&targets);
        assert_eq!(res.total, 3);
        assert_eq!(res.reachable, 2);
        assert_eq!(res.failures.len(), 1);
        assert_eq!(res.failures[0].index, 1);
        assert!(!res.is_reachable());
    }

    #[test]
    fn empty_targets_are_reachable() {
        let v = validator();
        let res = v.validate(&[]);
        assert!(res.is_reachable());
        assert_eq!(res.total, 0);
        assert_eq!(res.reachable_ratio(), 1.0);
    }
}