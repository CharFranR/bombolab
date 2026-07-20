use std::fmt;

use nalgebra::SMatrix;

use crate::math::{Iso3, Vec3};
use crate::robot::{DHParams, Joint, JointType, Robot, Segment};

use super::forward::forward_kinematics;

// ─── Error ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub enum IkError {
    DegenerateChain,
    MaxIterationsReached { error: f64 },
}

impl fmt::Display for IkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IkError::DegenerateChain => write!(f, "robot chain is degenerate"),
            IkError::MaxIterationsReached { error } => {
                write!(f, "max iterations reached, error = {error:.6}")
            }
        }
    }
}

impl std::error::Error for IkError {}

// ─── Helpers ───────────────────────────────────────────────────────────────

/// Construye un Robot con los valores articulares q aplicados.
fn build_robot(robot: &Robot, q: &[f64]) -> Robot {
    let segments: Vec<Segment> = robot
        .segments
        .iter()
        .zip(q.iter())
        .map(|(seg, &val)| {
            let joint = Joint::new(
                seg.joint.joint_type,
                val,
                seg.joint.value_max,
                seg.joint.value_min,
            );
            let dh = DHParams::new(seg.dh.theta, seg.dh.d, seg.dh.a, seg.dh.alpha);
            Segment::new(joint, dh)
        })
        .collect();
    Robot::with_directions(
        segments,
        robot.home_pose.clone(),
        robot.servo_offsets.clone(),
        robot.servo_directions.clone(),
    )
}

/// Computa error de posición: ||target − p_ee|| con el robot en q.
fn position_error(robot: &Robot, target: &[f64; 3], base: &Iso3, tool: &Iso3) -> f64 {
    let (frames, _) = forward_kinematics(*base, robot);
    let tool_pose = frames.last().unwrap() * tool;
    let p_ee = tool_pose.translation.vector;
    let target_v = Vec3::new(target[0], target[1], target[2]);
    (target_v - p_ee).norm()
}

// ─── Solver ─────────────────────────────────────────────────────────────────

/// Inverse Kinematics solver con Damped Least Squares (Levenberg–Marquardt).
///
/// Resuelve posición [x, y, z] → q para un robot serial de 5 GDL máximo.
/// Usa la Jacobiana lineal 3×n (filas de velocidad lineal de la Jacobiana
/// geométrica). El damping λ es fijo (etapa 1).
///
/// `q_init` debe ser la última solución conocida (tracking entre frames).
pub struct IkSolver {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub damping: f64,
    pub step_size: f64,
}

impl IkSolver {
    pub fn new(max_iterations: usize, tolerance: f64, damping: f64, step_size: f64) -> Self {
        Self {
            max_iterations,
            tolerance,
            damping,
            step_size,
        }
    }

    /// Resuelve IK de posición.
    ///
    /// - `target`: [x, y, z] en mm (coordenadas del mundo/base)
    /// - `q_init`: aproximación inicial (rad), normalmente la q anterior
    /// - `robot`: robot canónico (se clonan segmentos internamente)
    /// - `base`: base_transform del robot
    /// - `tool`: tool_transform del robot
    pub fn solve_position(
        &self,
        target: &[f64; 3],
        q_init: &[f64],
        robot: &Robot,
        base: &Iso3,
        tool: &Iso3,
    ) -> Result<Vec<f64>, IkError> {
        if robot.dof() == 0 {
            return Err(IkError::DegenerateChain);
        }

        let n = robot.dof().min(5);
        let mut q = q_init.to_vec();
        let target_v = Vec3::new(target[0], target[1], target[2]);
            let damping_sq = self.damping * self.damping;

        for _iter in 0..self.max_iterations {
            let robot_q = build_robot(robot, &q);

            // 1. FK → tool tip
            let (frames, _) = forward_kinematics(*base, &robot_q);
            let tool_pose = frames.last().unwrap() * tool;
            let p_ee = tool_pose.translation.vector;

            // 2. Error
            let error = target_v - p_ee;
            let err_norm = error.norm();
            if err_norm < self.tolerance {
                return Ok(q);
            }

            // 3. Jacobiana lineal 3×n
            //    Joint i rota sobre el eje de su tipo:
            //    - Revolute / Prismatic: eje Z_{i-1}
            //    - Twist:               eje X_{i-1}
            //    J1 usa el eje base (Z₀ o X₀), J2 usa frames[0], etc.
            let mut j = SMatrix::<f64, 3, 5>::zeros();
            let base_z = Vec3::z();
            let base_x = Vec3::x();
            let base_p = base.translation.vector;
            for i in 0..n {
                let (axis_i, p_i) = if i == 0 {
                    let ax = match robot_q.segments[i].joint.joint_type {
                        JointType::Twist => base_x,
                        _ => base_z,
                    };
                    (ax, base_p)
                } else {
                    let prev = &frames[i - 1];
                    let ax = match robot_q.segments[i].joint.joint_type {
                        JointType::Twist => prev * Vec3::x(),
                        _ => prev * Vec3::z(),
                    };
                    (ax, prev.translation.vector)
                };
                j.column_mut(i).copy_from(&axis_i.cross(&(p_ee - p_i)));
            }

            // 4. DLS: Δq = J^T · (J·J^T + λ²·I)⁻¹ · error
            let jjt = &j * j.transpose();
            let reg = jjt + SMatrix::<f64, 3, 3>::identity() * damping_sq;

            if let Some(inv) = reg.try_inverse() {
                let delta_x = inv * error;
                let delta_q = j.transpose() * delta_x;

                let dq_norm = delta_q.norm();
                let scale = if dq_norm > self.step_size {
                    self.step_size / dq_norm
                } else {
                    1.0
                };
                for idx in 0..n {
                    q[idx] += delta_q[idx] * scale;
                    // Clamp a límites articulares
                    q[idx] = q[idx].clamp(
                        robot.segments[idx].joint.value_min,
                        robot.segments[idx].joint.value_max,
                    );
                }
            } else {
                // Jacobiana singular
                return Ok(q);
            }
        }

        // Último error para reporte
        let robot_q = build_robot(robot, &q);
        let final_err = position_error(&robot_q, target, base, tool);
        Err(IkError::MaxIterationsReached { error: final_err })
    }
}

// ─── Default ───────────────────────────────────────────────────────────────

impl Default for IkSolver {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1.0,   // 1 mm
            damping: 0.1,
            step_size: 0.5,   // ~28°/iteración máx
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::fabri_creator;

    fn make_test() -> (IkSolver, Robot, Iso3, Iso3) {
        let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
        let robot = fabri_creator();
        let base = crate::robot::base_transform();
        let tool = crate::robot::tool_transform();
        (solver, robot, base, tool)
    }

    #[test]
    fn solve_home_pose() {
        let (solver, robot, base, tool) = make_test();

        // En home (q=0), el tool tip está en (236, 0, 314) con base.
        // Preguntar por esa posición debería dar q≈0.
        let target = [236.0, 0.0, 314.0];
        let q_init = vec![0.0; 5];

        let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
        assert!(result.is_ok());

        let q = result.unwrap();
        for (i, &val) in q.iter().enumerate() {
            assert!(
                val.abs() < 0.05,
                "J{} debería estar cerca de 0, got {:.6}",
                i + 1,
                val
            );
        }
    }

    #[test]
    fn solve_reachable_pose() {
        let (solver, robot, base, tool) = make_test();

        // Punto alcanzable: el brazo puede llegar apretando el codo
        let target = [200.0, 0.0, 280.0];
        let q_init = vec![0.0; 5];

        let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
        assert!(result.is_ok(), "IK debería converger: {result:?}");

        let q = result.unwrap();
        let robot_q = build_robot(&robot, &q);
        let err = position_error(&robot_q, &target, &base, &tool);
        assert!(
            err < 10.0,
            "error debería ser <10mm, got {err:.3}"
        );
    }

    #[test]
    fn solve_upward() {
        let (solver, robot, base, tool) = make_test();

        // Apuntar hacia arriba: J2 negativo (hombro sube), J3 compensa
        let target = [150.0, 0.0, 400.0];
        let q_init = vec![0.0; 5];

        let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
        assert!(result.is_ok(), "IK debería converger: {result:?}");

        let q = result.unwrap();
        let robot_q = build_robot(&robot, &q);
        let err = position_error(&robot_q, &target, &base, &tool);
        assert!(
            err < 10.0,
            "error debería ser <10mm, got {err:.3}"
        );
    }

    #[test]
    fn solve_tracking() {
        let (solver, robot, base, tool) = make_test();

        let target_a = [200.0, 20.0, 280.0];
        let q_a = solver
            .solve_position(&target_a, &[0.0; 5], &robot, &base, &tool)
            .unwrap();

        let target_b = [210.0, 10.0, 270.0];
        let q_b = solver
            .solve_position(&target_b, &q_a, &robot, &base, &tool)
            .unwrap();

        let robot_q = build_robot(&robot, &q_b);
        let err = position_error(&robot_q, &target_b, &base, &tool);
        assert!(err < 10.0, "tracking error: {err:.3}");
    }

    #[test]
    fn solve_unreachable_returns_max_iterations() {
        let (solver, robot, base, tool) = make_test();

        // Punto muy lejano — no debería converger
        let target = [5000.0, 5000.0, 5000.0];
        let q_init = vec![0.0; 5];

        let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
        assert!(matches!(result, Err(IkError::MaxIterationsReached { .. })));
    }
}
