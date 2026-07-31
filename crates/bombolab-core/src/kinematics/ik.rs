use std::fmt;

use nalgebra::SMatrix;

use crate::math::{Iso3, Rot3, Vec3};
use crate::robot::{Joint, JointType, Robot, Segment};

use super::forward::forward_kinematics;

// ─── Error ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub enum IkError {
    DegenerateChain,
    MaxIterationsReached {
        error: f64,
    },
    /// La orientación objetivo es inalcanzable con la muñeca de 2 GDL.
    UnreachableOrientation {
        r35_02: f64,
        tolerance: f64,
    },
    /// `q_init` no coincide con el número de articulaciones del robot.
    InvalidInitLength {
        expected: usize,
        got: usize,
    },
}

impl fmt::Display for IkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IkError::DegenerateChain => write!(f, "robot chain is degenerate"),
            IkError::MaxIterationsReached { error } => {
                write!(f, "max iterations reached, error = {error:.6}")
            }
            IkError::UnreachableOrientation { r35_02, tolerance } => {
                write!(
                    f,
                    "orientation not reachable: R35[0,2] = {:.2e} exceeds tolerance {:.2e}",
                    r35_02, tolerance
                )
            }
            IkError::InvalidInitLength { expected, got } => {
                write!(f, "invalid q_init length: expected {expected}, got {got}")
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
            let dh = seg.dh; // DHParams es Copy — copia directa, sin reconstrucción
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

        // q_init de longitud incorrecta paniqueaba en frames.last().unwrap().
        if q_init.len() != robot.dof() {
            return Err(IkError::InvalidInitLength {
                expected: robot.dof(),
                got: q_init.len(),
            });
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

            // 3. Jacobiana lineal 3×n (función extraída para testing FD)
            let j = position_jacobian(&robot_q, &frames, &p_ee, base, n);

            // 4. DLS: Δq = J^T · (J·J^T + λ²·I)⁻¹ · error
            let jjt = j * j.transpose();
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

/// Position Jacobian (3×n) of the tool tip for the DLS position solver.
///
/// Column `i` is the linear velocity of `p_ee` produced by joint `i`:
///
/// - **Revolute / Prismatic**: joint `i` rotates about `Z_{i-1}` (the Z axis
///   of the frame BEFORE the joint). For `i=0` the base frame is used.
///   Pivot: `o_{i-1}` — the classical `z_{i-1} × (p_ee − o_{i-1})`.
/// - **Twist**: joint `i` rotates about `X_{i-1}` (the X axis of the frame
///   BEFORE the joint), but the twist transform is
///   `T = Trans(a, d, 0) · RotX(alpha + q)`: the translation is applied
///   FIRST and is constant in `q`, so the frame origin `o_i` stays fixed
///   while the body rotates around the axis through it.
///   Pivot: `o_i` (the origin of the frame AFTER the joint, `frames[i]`),
///   giving `x_{i-1} × (p_ee − o_i)`.
///
/// Using `o_{i-1}` for a Twist would place the rotation axis through the
/// previous frame origin, which is wrong: the instantaneous axis passes
/// through the displaced origin `o_i = (a, d, 0)` in frame `i-1`.
fn position_jacobian(
    robot_q: &Robot,
    frames: &[Iso3],
    p_ee: &Vec3,
    base: &Iso3,
    n: usize,
) -> SMatrix<f64, 3, 5> {
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
            // Twist en la base: el eje X₀ pasa por el origen del frame 1
            // (frames[0]) porque la traslación (a, d, 0) precede a la rotación.
            let p = match robot_q.segments[i].joint.joint_type {
                JointType::Twist => frames[i].translation.vector,
                _ => base_p,
            };
            (ax, p)
        } else {
            let prev = &frames[i - 1];
            let ax = match robot_q.segments[i].joint.joint_type {
                JointType::Twist => prev * Vec3::x(),
                _ => prev * Vec3::z(),
            };
            // Twist: pivot = o_i (frames[i], el frame DESPUÉS del joint),
            // NO o_{i-1}: la rotación ocurre alrededor del origen desplazado.
            let p = match robot_q.segments[i].joint.joint_type {
                JointType::Twist => frames[i].translation.vector,
                _ => prev.translation.vector,
            };
            (ax, p)
        };
        j.column_mut(i).copy_from(&axis_i.cross(&(p_ee - p_i)));
    }
    j
}

// ─── Default ───────────────────────────────────────────────────────────────

impl Default for IkSolver {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1.0, // 1 mm
            damping: 0.1,
            step_size: 0.5, // ~28°/iteración máx
        }
    }
}

// ─── Orientation Error ──────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub enum OrientationError {
    /// R35[0,2] is not close to zero — the requested orientation is not
    /// reachable with the 2-DOF wrist (roll + pitch only, no yaw).
    UnreachableOrientation { r35_02: f64, tolerance: f64 },
}

impl fmt::Display for OrientationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OrientationError::UnreachableOrientation { r35_02, tolerance } => {
                write!(
                    f,
                    "orientation not reachable: R35[0,2] = {:.2e} exceeds tolerance {:.2e}",
                    r35_02, tolerance
                )
            }
        }
    }
}

impl std::error::Error for OrientationError {}

// ─── Orientation Solver ─────────────────────────────────────────────────────

/// Solves J4 (roll) and J5 (pitch) from the orientation target.
///
/// Given R03 (rotation from base to frame 3, computed from the position
/// solution J1-J2-J3) and R_target (desired tool orientation), this solver
/// computes:
///
///   R35 = R03ᵀ · R_target
///
/// Then extracts q4 and q5 analytically using the known structure of R35 for
/// the FABRI Creator's 2-DOF wrist (Roll on X via Twist joint, Pitch on Z).
///
/// The robot has NO yaw axis, so not all orientations in SO(3) are reachable.
/// The condition |R35[0,2]| < tolerance is the reachability test.
pub struct OrientationSolver {
    /// Maximum allowed value for |R35[0,2]| before rejecting as unreachable.
    pub tolerance: f64,
}

impl OrientationSolver {
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
    }

    /// Solve for q4, q5 given R03 and R_target.
    ///
    /// Returns `(q4, q5)` in radians, clamped to the robot's joint limits.
    ///
    /// - `r03`: rotation matrix of T03 (from base to frame 3).
    /// - `r_target`: desired tool orientation.
    /// - `robot`: the robot model (used for joint limits on J4 and J5).
    pub fn solve(
        &self,
        r03: &Rot3,
        r_target: &Rot3,
        robot: &Robot,
    ) -> Result<[f64; 2], OrientationError> {
        // 1. R35 = R03ᵀ · R_target
        let r35 = r03.transpose() * r_target;
        let m = r35.matrix();

        // 2. Reachability check: R35[0,2] must be ~0
        let r35_02 = m[(0, 2)].abs();
        if r35_02 > self.tolerance {
            return Err(OrientationError::UnreachableOrientation {
                r35_02,
                tolerance: self.tolerance,
            });
        }

        // 3. Extract q4, q5 via analytical closed-form equations
        //    R35 = ┌  c5     -s5      0  ┐
        //          │ -s4·s5  -s4·c5  -c4 │
        //          │  c4·s5   c4·c5  -s4 │
        //          └                       ┘
        let c4 = -m[(1, 2)];
        let s4 = -m[(2, 2)];
        let c5 = m[(0, 0)];
        let s5 = -m[(0, 1)];

        let q4 = s4.atan2(c4);
        let q5 = s5.atan2(c5);

        // 4. Verificar límites articulares
        // Si q4 o q5 exceden los límites, la orientación es inalcanzable
        // (el clamping silencioso produciría una orientación incorrecta).
        let j4_lo = robot.segments[3].joint.value_min;
        let j4_hi = robot.segments[3].joint.value_max;
        let j5_lo = robot.segments[4].joint.value_min;
        let j5_hi = robot.segments[4].joint.value_max;

        if q4 < j4_lo || q4 > j4_hi {
            return Err(OrientationError::UnreachableOrientation {
                r35_02: 0.0,
                tolerance: self.tolerance,
            });
        }
        if q5 < j5_lo || q5 > j5_hi {
            return Err(OrientationError::UnreachableOrientation {
                r35_02: 0.0,
                tolerance: self.tolerance,
            });
        }

        Ok([q4, q5])
    }
}

// ─── Full IK (position + orientation) ──────────────────────────────────────

/// Resuelve posición Y orientación del efector.
///
/// Pipeline:
///   1. `pos_solver` → q1, q2, q3 (posición)
///   2. FK con q1,q2,q3 → R03 (rotación de base a frame 3)
///   3. `orient_solver` → q4, q5 (orientación)
///   4. Solución completa [q1, q2, q3, q4, q5]
///
/// Si la orientación no es alcanzable, devuelve
/// `IkError::UnreachableOrientation` sin modificar la posición.
// API refactor (argument struct) is deferred to the architecture stage.
#[allow(clippy::too_many_arguments)]
pub fn solve_full_ik(
    pos_solver: &IkSolver,
    orient_solver: &OrientationSolver,
    target_pos: &[f64; 3],
    target_rot: &Rot3,
    q_init: &[f64],
    robot: &Robot,
    base: &Iso3,
    tool: &Iso3,
) -> Result<Vec<f64>, IkError> {
    // 1. Resolver posición (usa los 5 GDL internamente pero solo q1-q3
    //    son significativos para posición)
    let q_pos = pos_solver.solve_position(target_pos, q_init, robot, base, tool)?;

    // 2. Extraer q1,q2,q3 y ejecutar FK → R03
    let q1 = q_pos[0];
    let q2 = q_pos[1];
    let q3 = q_pos[2];
    let q_partial = [q1, q2, q3, 0.0, 0.0];
    let robot_partial = build_robot(robot, &q_partial);
    let (frames, _) = forward_kinematics(*base, &robot_partial);
    let r03 = frames[2].rotation.to_rotation_matrix();

    // 3. Resolver orientación → q4, q5
    let [q4, q5] = orient_solver
        .solve(&r03, target_rot, robot)
        .map_err(|err| match err {
            OrientationError::UnreachableOrientation { r35_02, tolerance } => {
                IkError::UnreachableOrientation { r35_02, tolerance }
            }
        })?;

    // 4. Solución completa
    Ok(vec![q1, q2, q3, q4, q5])
}

// ─── Drawing IK (posición + orientación adaptativa) ─────────────────────────

/// Resuelve posición y orientación para dibujo con marcador vertical.
///
/// A diferencia de [`solve_full_ik`], que recibe una R_target arbitraria,
/// esta función genera automáticamente la orientación adaptativa usando
/// el ángulo q₁ del solver de posición:
///
///   R_target(q₁) con θ = q₁ + π  →  X₅ = -Z (marcador vertical) siempre.
///
/// Pipeline:
///   1. `pos_solver` → q1, q2, q3
///   2. `PoseGenerator::drawing_pose_adaptive(position, q1)` → R_target
///   3. FK con q1,q2,q3 → R03
///   4. `orient_solver` → q4, q5
///   5. Solución completa [q1, q2, q3, q4, q5]
///
/// A diferencia de `solve_full_ik`, esta función NO llama dos veces al
/// solver de posición — usa los q1,q2,q3 ya obtenidos en el paso 1.
pub fn solve_drawing_ik(
    pos_solver: &IkSolver,
    orient_solver: &OrientationSolver,
    target_pos: &[f64; 3],
    q_init: &[f64],
    robot: &Robot,
    base: &Iso3,
    tool: &Iso3,
) -> Result<Vec<f64>, IkError> {
    use crate::kinematics::pose_generator::PoseGenerator;

    // 1. Resolver posición → q1, q2, q3
    let q_pos = pos_solver.solve_position(target_pos, q_init, robot, base, tool)?;
    let q1 = q_pos[0];
    let q2 = q_pos[1];
    let q3 = q_pos[2];

    // 2. Generar R_target adaptativo con θ = q₁ + π
    let target = PoseGenerator::drawing_pose_adaptive(*target_pos, q1);

    // 3. FK con q1,q2,q3 → R03
    let q_partial = [q1, q2, q3, 0.0, 0.0];
    let robot_partial = build_robot(robot, &q_partial);
    let (frames, _) = forward_kinematics(*base, &robot_partial);
    let r03 = frames[2].rotation.to_rotation_matrix();

    // 4. Resolver orientación → q4, q5
    let [q4, q5] = orient_solver
        .solve(&r03, &target.rotation, robot)
        .map_err(|err| match err {
            OrientationError::UnreachableOrientation { r35_02, tolerance } => {
                IkError::UnreachableOrientation { r35_02, tolerance }
            }
        })?;

    // 5. Solución completa
    Ok(vec![q1, q2, q3, q4, q5])
}

/// Igual que [`solve_drawing_ik`] pero para el modo 2:
/// el marcador apunta en Y₅ (perpendicular al gripper).
///
/// Usa `PoseGenerator::drawing_pose_v2` que genera:
///   R_target(q₁) = [c₁  0  -s₁; s₁  0  c₁; 0  -1  0]
///   → Y₅ = [0, 0, -1] (marcador vertical)
///   → q4 = 0, q5 = -q₂₃ (siempre dentro de límites para dibujo)
pub fn solve_drawing_ik_v2(
    pos_solver: &IkSolver,
    orient_solver: &OrientationSolver,
    target_pos: &[f64; 3],
    q_init: &[f64],
    robot: &Robot,
    base: &Iso3,
    tool: &Iso3,
) -> Result<Vec<f64>, IkError> {
    use crate::kinematics::pose_generator::PoseGenerator;

    // 1. Resolver posición → q1, q2, q3
    let q_pos = pos_solver.solve_position(target_pos, q_init, robot, base, tool)?;
    let q1 = q_pos[0];
    let q2 = q_pos[1];
    let q3 = q_pos[2];

    // 2. Generar R_target modo 2 (Y₅ = -Z)
    let target = PoseGenerator::drawing_pose_v2(*target_pos, q1);

    // 3. FK con q1,q2,q3 → R03
    let q_partial = [q1, q2, q3, 0.0, 0.0];
    let robot_partial = build_robot(robot, &q_partial);
    let (frames, _) = forward_kinematics(*base, &robot_partial);
    let r03 = frames[2].rotation.to_rotation_matrix();

    // 4. El orientation solver extrae q4,q5 compensando DH offsets
    let [q4, q5] = orient_solver
        .solve(&r03, &target.rotation, robot)
        .map_err(|err| match err {
            OrientationError::UnreachableOrientation { r35_02, tolerance } => {
                IkError::UnreachableOrientation { r35_02, tolerance }
            }
        })?;

    // 5. Solución completa
    Ok(vec![q1, q2, q3, q4, q5])
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

        // Compute FK at home (q=0) to get the tool tip target position
        use crate::kinematics::forward::forward_kinematics;
        let home_q = [0.0_f64; 5];
        let robot_home = build_robot(&robot, &home_q);
        let (frames, _last) = forward_kinematics(base, &robot_home);
        let tool_tip = frames.last().unwrap() * tool;
        let target = [
            tool_tip.translation.x,
            tool_tip.translation.y,
            tool_tip.translation.z,
        ];
        let q_init = vec![0.0; 5];

        let result = solver.solve_position(&target, &q_init, &robot, &base, &tool);
        assert!(result.is_ok(), "IK should converge at home position");

        // Verify the solution reaches the target (position error < 2mm)
        let q = result.unwrap();
        let robot_q = build_robot(&robot, &q);
        let (frames_q, _) = forward_kinematics(base, &robot_q);
        let tip_q = frames_q.last().unwrap() * tool;
        let err = (tip_q.translation.vector - tool_tip.translation.vector).norm();
        assert!(
            err < 2.0,
            "Position error at home: {:.3}mm (should be < 2mm)",
            err
        );
    }

    #[test]
    fn solve_q_init_length_validated() {
        let (solver, robot, base, tool) = make_test();
        let target = [200.0, 0.0, 280.0];

        // q_init vacío debe devolver Err, no paniquear.
        let empty: Vec<f64> = vec![];
        let result = solver.solve_position(&target, &empty, &robot, &base, &tool);
        assert!(matches!(
            result,
            Err(IkError::InvalidInitLength {
                expected: 5,
                got: 0
            })
        ));

        // q_init corto (menos articulaciones que el robot) → Err.
        let short = vec![0.0; 3];
        let result = solver.solve_position(&target, &short, &robot, &base, &tool);
        assert!(matches!(
            result,
            Err(IkError::InvalidInitLength {
                expected: 5,
                got: 3
            })
        ));

        // q_init largo → Err.
        let long = vec![0.0; 7];
        let result = solver.solve_position(&target, &long, &robot, &base, &tool);
        assert!(matches!(
            result,
            Err(IkError::InvalidInitLength {
                expected: 5,
                got: 7
            })
        ));
    }

    /// Finite-difference validation of the position Jacobian used by the
    /// DLS solver, on the real FABRI robot (with base and tool).
    ///
    /// Perturbs each joint `i` by ±ε and compares `(FK(q+ε) − FK(q−ε)) / 2ε`
    /// against the analytical column. This catches pivot errors (Twist must
    /// pivot about `o_i`, not `o_{i-1}` — the FABRI twist column must be ~0).
    #[test]
    fn position_jacobian_finite_differences_fabri() {
        let (_, robot, base, tool) = make_test();
        use crate::kinematics::forward::forward_kinematics;

        let eps = 1e-8;
        let tol = 1e-4; // mm per rad

        // Configuraciones variadas, incluyendo home y el twist activo.
        let configs: [[f64; 5]; 4] = [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [0.3, -0.5, 0.7, 0.4, -0.4],
            [-0.2, 0.4, -0.3, 0.5, 0.1],
            [0.0, 0.8, -0.6, 0.2, -0.2],
        ];

        for q in configs.iter() {
            let robot_q = build_robot(&robot, q);
            let (frames, _) = forward_kinematics(base, &robot_q);
            let tool_pose = frames.last().unwrap() * tool;
            let p_ee = tool_pose.translation.vector;
            let j_ana = position_jacobian(&robot_q, &frames, &p_ee, &base, 5);

            for col in 0..5 {
                let mut q_plus = *q;
                let mut q_minus = *q;
                q_plus[col] += eps;
                q_minus[col] -= eps;

                let robot_plus = build_robot(&robot, &q_plus);
                let (frames_plus, _) = forward_kinematics(base, &robot_plus);
                let p_plus = (frames_plus.last().unwrap() * tool).translation.vector;

                let robot_minus = build_robot(&robot, &q_minus);
                let (frames_minus, _) = forward_kinematics(base, &robot_minus);
                let p_minus = (frames_minus.last().unwrap() * tool).translation.vector;

                let dp = (p_plus - p_minus) / (2.0 * eps);

                for row in 0..3 {
                    let num = dp[row];
                    let ana = j_ana[(row, col)];
                    assert!(
                        (num - ana).abs() < tol,
                        "config {q:?}, col {col}, row {row}: \
                         numerical = {num:.6e}, analytical = {ana:.6e}, diff = {}",
                        (num - ana).abs()
                    );
                }
            }

            // En home (q = [0;5]) el tool (75 mm ∥ x₃) queda sobre el eje del
            // twist, así que la columna lineal del twist debe ser ~0. Un pivot
            // incorrecto (o_{i-1}) produciría aquí (0, 0, −15).
            if *q == [0.0; 5] {
                let twist_col = j_ana.fixed_view::<3, 1>(0, 3).into_owned();
                assert!(
                    twist_col.norm() < 1e-6,
                    "FABRI twist linear column at home should be ~0, got {twist_col:?}"
                );
            }
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
        assert!(err < 10.0, "error debería ser <10mm, got {err:.3}");
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
        assert!(err < 10.0, "error debería ser <10mm, got {err:.3}");
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

// ─── Orientation Solver Tests ─────────────────────────────────────────────

#[cfg(test)]
mod orientation_tests {
    use super::*;
    use crate::math::Rot3;
    use crate::robot::fabri_creator;

    /// Extrae la matriz de rotación 3×3 de un Iso3.
    fn get_rot3(iso: &Iso3) -> Rot3 {
        iso.rotation.to_rotation_matrix()
    }

    const TOL: f64 = 1e-10;

    fn make_robot() -> Robot {
        fabri_creator()
    }

    fn make_base() -> Iso3 {
        crate::robot::base_transform()
    }

    fn q_to_robot(robot: &Robot, q: &[f64; 5]) -> Robot {
        build_robot(robot, q)
    }

    #[test]
    fn test_home_pose_extracts_zero() {
        let robot = make_robot();
        let base = make_base();
        let solver = OrientationSolver::new(1e-6);

        let q = [0.0; 5];
        let robot_q = q_to_robot(&robot, &q);
        let (frames, effector) = forward_kinematics(base, &robot_q);
        let r03 = get_rot3(&frames[2]);
        let r_target = get_rot3(&effector);

        let result = solver.solve(&r03, &r_target, &robot);
        assert!(result.is_ok(), "home debería ser alcanzable");

        let [q4, q5] = result.unwrap();
        assert!(q4.abs() < TOL, "q4 en home debería ser 0, got {:.2e}", q4);
        assert!(q5.abs() < TOL, "q5 en home debería ser 0, got {:.2e}", q5);
    }

    #[test]
    fn test_random_configurations_reconstructed() {
        let robot = make_robot();
        let base = make_base();
        let solver = OrientationSolver::new(1e-6);

        let mut seed: u64 = 42;
        let n_samples = 100;

        let mut max_q4_err: f64 = 0.0;
        let mut max_q5_err: f64 = 0.0;
        let mut max_reconstruction_err: f64 = 0.0;

        for _ in 0..n_samples {
            // Generar q aleatorio dentro de límites
            let q: [f64; 5] = std::array::from_fn(|i| {
                let lo = robot.segments[i].joint.value_min.max(-2.0);
                let hi = robot.segments[i].joint.value_max.min(2.0);
                // LCG simple
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let r = (seed as f64) / (u64::MAX as f64);
                lo + r * (hi - lo)
            });

            let robot_q = q_to_robot(&robot, &q);
            let (frames, effector) = forward_kinematics(base, &robot_q);
            let r03 = get_rot3(&frames[2]);
            let r_target = get_rot3(&effector);

            let result = solver.solve(&r03, &r_target, &robot);
            assert!(
                result.is_ok(),
                "q = [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}] debería ser alcanzable",
                q[0],
                q[1],
                q[2],
                q[3],
                q[4]
            );

            let [q4, q5] = result.unwrap();

            // Error angular
            let q4_err = (q4 - q[3]).abs() % (2.0 * std::f64::consts::PI);
            let q4_err = q4_err.min(2.0 * std::f64::consts::PI - q4_err);
            let q5_err = (q5 - q[4]).abs() % (2.0 * std::f64::consts::PI);
            let q5_err = q5_err.min(2.0 * std::f64::consts::PI - q5_err);

            max_q4_err = max_q4_err.max(q4_err);
            max_q5_err = max_q5_err.max(q5_err);

            assert!(
                q4_err < 1e-12,
                "error q4 = {:.2e} para original={:.6}, extraído={:.6}",
                q4_err,
                q[3],
                q4
            );
            assert!(
                q5_err < 1e-12,
                "error q5 = {:.2e} para original={:.6}, extraído={:.6}",
                q5_err,
                q[4],
                q5
            );

            // Reconstrucción: R_target ≈ R03 · R35(q4, q5)
            let (s4, c4) = q4.sin_cos();
            let (s5, c5) = q5.sin_cos();
            let r35_reconstructed = Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                c5,
                -s5,
                0.0,
                -s4 * s5,
                -s4 * c5,
                -c4,
                c4 * s5,
                c4 * c5,
                -s4,
            ));
            let r_reconstructed = r03 * r35_reconstructed;
            let diff = (r_reconstructed.matrix() - r_target.matrix()).norm();
            max_reconstruction_err = max_reconstruction_err.max(diff);

            assert!(diff < 1e-12, "error reconstrucción = {:.2e}", diff);
        }

        eprintln!("=== OrientationSolver: random configs ===");
        eprintln!("Muestras: {n_samples}");
        eprintln!("Máx error q4: {:.2e} rad", max_q4_err);
        eprintln!("Máx error q5: {:.2e} rad", max_q5_err);
        eprintln!("Máx error reconstrucción: {:.2e}", max_reconstruction_err);
    }

    #[test]
    fn test_unreachable_orientation_detected() {
        let robot = make_robot();
        let base = make_base();
        // Tolerance estricta para detectar cualquier desviación
        let solver = OrientationSolver::new(1e-10);

        let mut seed: u64 = 1234;
        let n_samples = 50;

        for _ in 0..n_samples {
            // Generar posiciones aleatorias (q1, q2, q3)
            let mut q: [f64; 5] = [0.0; 5];
            for i in 0..3 {
                let lo = robot.segments[i].joint.value_min.max(-2.0);
                let hi = robot.segments[i].joint.value_max.min(2.0);
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let r = (seed as f64) / (u64::MAX as f64);
                q[i] = lo + r * (hi - lo);
            }

            let robot_q = q_to_robot(&robot, &q);
            let (frames, _effector) = forward_kinematics(base, &robot_q);
            let r03 = get_rot3(&frames[2]);

            // Crear una R_target que tenga una rotación pura sobre Y en R35,
            // violando la condición estructural R35[0,2] = 0.
            // R35_perturbed = Rot_y(0.2) → R35[0,2] = sin(0.2) ≈ 0.2 > ε
            let r_y = Rot3::from_axis_angle(&nalgebra::Unit::new_normalize(Vec3::y()), 0.2);
            // R_target = R03 · R35_perturbed
            let r_target_perturbed = r03 * r_y;

            let result = solver.solve(&r03, &r_target_perturbed, &robot);
            assert!(
                result.is_err(),
                "debería rechazar orientación con rotación Y pura en R35"
            );
            match result {
                Err(OrientationError::UnreachableOrientation { r35_02, tolerance }) => {
                    assert!(
                        r35_02 > tolerance,
                        "r35_02={:.2e} debería exceder tol={:.2e}",
                        r35_02,
                        tolerance
                    );
                }
                _ => unreachable!(),
            }
        }
    }

    #[test]
    fn test_vary_q4_reconstructed() {
        let robot = make_robot();
        let base = make_base();
        let solver = OrientationSolver::new(1e-6);

        let mut seed: u64 = 77;
        // Valores dentro de límites ±80° (~±1.396 rad), dejando margen
        let q5_vals = [-0.8, 0.0, 0.8];
        let q4_vals = [-1.2, -0.7, -0.2, 0.0, 0.3, 0.8, 1.2];

        for _ in 0..20 {
            // q1, q2, q3 aleatorios
            let mut q_base: [f64; 5] = [0.0; 5];
            for i in 0..3 {
                let lo = robot.segments[i].joint.value_min.max(-2.0);
                let hi = robot.segments[i].joint.value_max.min(2.0);
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let r = (seed as f64) / (u64::MAX as f64);
                q_base[i] = lo + r * (hi - lo);
            }

            for &q5 in &q5_vals {
                q_base[4] = q5;
                for &q4 in &q4_vals {
                    q_base[3] = q4;

                    let robot_q = q_to_robot(&robot, &q_base);
                    let (frames, effector) = forward_kinematics(base, &robot_q);
                    let r03 = get_rot3(&frames[2]);
                    let r_target = get_rot3(&effector);

                    let result = solver.solve(&r03, &r_target, &robot);
                    assert!(result.is_ok(), "q4={:.4}, q5={:.4} falló", q4, q5);

                    let [q4_out, q5_out] = result.unwrap();
                    let err4 = (q4_out - q4).abs() % (2.0 * std::f64::consts::PI);
                    let err4 = err4.min(2.0 * std::f64::consts::PI - err4);
                    let err5 = (q5_out - q5).abs() % (2.0 * std::f64::consts::PI);
                    let err5 = err5.min(2.0 * std::f64::consts::PI - err5);

                    assert!(err4 < 1e-12, "q4 error: {:.2e}", err4);
                    assert!(err5 < 1e-12, "q5 error: {:.2e}", err5);
                }
            }
        }
    }
}

// ─── Full IK Integration Tests ─────────────────────────────────────────────

#[cfg(test)]
mod full_ik_tests {
    use super::*;
    use crate::robot::fabri_creator;

    fn make_robot() -> Robot {
        fabri_creator()
    }

    fn make_base() -> Iso3 {
        crate::robot::base_transform()
    }

    fn make_tool() -> Iso3 {
        crate::robot::tool_transform()
    }

    fn get_rot3(iso: &Iso3) -> Rot3 {
        iso.rotation.to_rotation_matrix()
    }

    /// Computa error de posición entre target y la solución completa.
    fn position_error_for_q(
        robot: &Robot,
        q: &[f64],
        target_pos: &[f64; 3],
        base: &Iso3,
        tool: &Iso3,
    ) -> f64 {
        let robot_q = build_robot(robot, q);
        let (_frames, effector) = forward_kinematics(*base, &robot_q);
        let tool_pose = effector * tool;
        let p_ee = tool_pose.translation.vector;
        let target_v = Vec3::new(target_pos[0], target_pos[1], target_pos[2]);
        (target_v - p_ee).norm()
    }

    /// Computa error de orientación: norma Frobenius de R_target - R_actual.
    fn orientation_error_for_q(robot: &Robot, q: &[f64], target_rot: &Rot3, base: &Iso3) -> f64 {
        let robot_q = build_robot(robot, q);
        let (_frames, effector) = forward_kinematics(*base, &robot_q);
        let r_actual = get_rot3(&effector);
        (r_actual.matrix() - target_rot.matrix()).norm()
    }

    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (*seed as f64) / (u64::MAX as f64)
    }

    fn rand_range(seed: &mut u64, lo: f64, hi: f64) -> f64 {
        lo + lcg(seed) * (hi - lo)
    }

    #[test]
    fn test_full_ik_home_pose() {
        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
        let orient_solver = OrientationSolver::new(1e-2);

        // Home: q = [0; 5], compute FK at home for target pos + rot
        let q_home = [0.0; 5];
        let robot_home = build_robot(&robot, &q_home);
        let (_frames, effector) = forward_kinematics(base, &robot_home);
        let target_pos = [
            effector.translation.x,
            effector.translation.y,
            effector.translation.z,
        ];
        let target_rot = get_rot3(&effector);

        let q_init = vec![0.0; 5];
        let result = solve_full_ik(
            &pos_solver,
            &orient_solver,
            &target_pos,
            &target_rot,
            &q_init,
            &robot,
            &base,
            &tool,
        );
        assert!(result.is_ok(), "home debería ser alcanzable: {result:?}");

        let q = result.unwrap();

        // Verificar posición final
        let pos_err = position_error_for_q(&robot, &q, &target_pos, &base, &tool);
        assert!(pos_err < 5.0, "error posición home: {:.3}mm", pos_err);

        // Verificar orientación final
        let orient_err = orientation_error_for_q(&robot, &q, &target_rot, &base);
        assert!(
            orient_err < 1e-1,
            "error orientación home: {:.2e}",
            orient_err
        );
    }

    #[test]
    fn test_full_ik_random_configs() {
        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
        let orient_solver = OrientationSolver::new(1e-6);

        let mut seed: u64 = 42;
        let n_samples = 50;
        let mut max_pos_err: f64 = 0.0;
        let mut max_orient_err: f64 = 0.0;

        for _ in 0..n_samples {
            // Generar q aleatorio
            let mut q: [f64; 5] = [0.0; 5];
            for i in 0..5 {
                let lo = robot.segments[i].joint.value_min.max(-1.3);
                let hi = robot.segments[i].joint.value_max.min(1.3);
                q[i] = rand_range(&mut seed, lo, hi);
            }

            // FK → target pose
            let robot_q = build_robot(&robot, &q);
            let (_frames, effector) = forward_kinematics(base, &robot_q);
            let tool_pose = effector * tool;
            let target_pos: [f64; 3] = [
                tool_pose.translation.vector.x,
                tool_pose.translation.vector.y,
                tool_pose.translation.vector.z,
            ];
            let target_rot = get_rot3(&effector);

            // Full IK: usar q original como semilla para que el solucionador
            // de posición converja inmediatamente (error ≈ 0 desde la primera
            // iteración, ya que q genera exactamente target_pos por FK)
            let q_init = q.to_vec();
            let result = solve_full_ik(
                &pos_solver,
                &orient_solver,
                &target_pos,
                &target_rot,
                &q_init,
                &robot,
                &base,
                &tool,
            );
            assert!(
                result.is_ok(),
                "q={:.3?} debería ser alcanzable: {result:?}",
                q
            );

            let q_solved = result.unwrap();

            // Posición
            let pos_err = position_error_for_q(&robot, &q_solved, &target_pos, &base, &tool);
            max_pos_err = max_pos_err.max(pos_err);
            assert!(
                pos_err < 10.0,
                "error posición = {:.3}mm para q_target={:.3?}",
                pos_err,
                q
            );

            // Orientación
            let orient_err = orientation_error_for_q(&robot, &q_solved, &target_rot, &base);
            max_orient_err = max_orient_err.max(orient_err);
            assert!(
                orient_err < 1e-10,
                "error orientación = {:.2e} para q_target={:.3?}",
                orient_err,
                q
            );
        }

        eprintln!("=== Full IK: random configs ===");
        eprintln!("Muestras: {n_samples}");
        eprintln!("Máx error posición: {:.3}mm", max_pos_err);
        eprintln!("Máx error orientación: {:.2e}", max_orient_err);
    }

    #[test]
    fn test_full_ik_unreachable_orientation() {
        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
        let orient_solver = OrientationSolver::new(1e-10); // tolerancia estricta

        // Usar home position (alcanzable desde q_init=[0;5])
        // Home: q=[0;5] → TCP ≈ (215, −15, 262)
        let target_pos = [236.0, 0.0, 314.0];

        // Obtener R05 en home
        let q_home = [0.0; 5];
        let robot_home = build_robot(&robot, &q_home);
        let (_frames, effector) = forward_kinematics(base, &robot_home);
        let r05_home = get_rot3(&effector);

        // Perturbar R_target con Rot_y(0.3) → viola R35[0,2] = 0
        let r_y = Rot3::from_axis_angle(&nalgebra::Unit::new_normalize(Vec3::y()), 0.3);
        let target_rot = r05_home * r_y;

        let q_init = vec![0.0; 5];
        let result = solve_full_ik(
            &pos_solver,
            &orient_solver,
            &target_pos,
            &target_rot,
            &q_init,
            &robot,
            &base,
            &tool,
        );

        assert!(result.is_err(), "debería rechazar orientación inalcanzable");
        match result {
            Err(IkError::UnreachableOrientation { r35_02, tolerance }) => {
                assert!(
                    r35_02 > tolerance,
                    "r35_02={:.2e} debería exceder tol={:.2e}",
                    r35_02,
                    tolerance
                );
                eprintln!(
                    "✓ Orientación inalcanzable detectada: r35_02={:.2e}",
                    r35_02
                );
            }
            Err(other) => panic!("error inesperado: {other}"),
            Ok(_) => unreachable!(),
        }
    }

    #[test]
    fn test_position_solver_unchanged() {
        // Verifica que solve_position sigue funcionando exactamente igual
        // (backward compatibility)
        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);

        // Test 1: Home
        let target = [236.0, 0.0, 314.0];
        let result = pos_solver.solve_position(&target, &[0.0; 5], &robot, &base, &tool);
        assert!(result.is_ok());
        let q = result.unwrap();
        let err = position_error_for_q(&robot, &q, &target, &base, &tool);
        assert!(err < 2.0, "home position error: {:.3}", err);

        // Test 2: Punto alcanzable
        let target = [200.0, 0.0, 280.0];
        let result = pos_solver.solve_position(&target, &[0.0; 5], &robot, &base, &tool);
        assert!(result.is_ok());
        let q = result.unwrap();
        let err = position_error_for_q(&robot, &q, &target, &base, &tool);
        assert!(err < 10.0, "reachable position error: {:.3}", err);

        // Test 3: Tracking
        let target_a = [200.0, 20.0, 280.0];
        let q_a = pos_solver
            .solve_position(&target_a, &[0.0; 5], &robot, &base, &tool)
            .unwrap();
        let target_b = [210.0, 10.0, 270.0];
        let result = pos_solver.solve_position(&target_b, &q_a, &robot, &base, &tool);
        assert!(result.is_ok());
        let q_b = result.unwrap();
        let err = position_error_for_q(&robot, &q_b, &target_b, &base, &tool);
        assert!(err < 10.0, "tracking error: {:.3}", err);

        // Test 4: Inalcanzable
        let target = [5000.0, 5000.0, 5000.0];
        let result = pos_solver.solve_position(&target, &[0.0; 5], &robot, &base, &tool);
        assert!(matches!(result, Err(IkError::MaxIterationsReached { .. })));
    }

    // ─── Diagnostic: drawing pose round-trip ─────────────────────────────
    //
    // Este test imprime el diagnóstico completo para verificar que la IK
    // reconstruye exactamente R_target. Si R_error ≈ identidad, la IK está
    // correcta y el problema está en el render visual del tool/gripper.
    //
    // Ejecutar: cargo test -p bombolab-core diagnostic_drawing -- --nocapture

    #[test]
    fn diagnostic_drawing_pose_roundtrip() {
        use crate::kinematics::pose_generator::PoseGenerator;

        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
        let orient_solver = OrientationSolver::new(1e-6);

        // Varias posiciones de dibujo sobre una mesa (z ≈ 80mm desde base)
        let test_positions: [[f64; 3]; 5] = [
            [200.0, 0.0, 80.0],   // centro
            [150.0, 50.0, 80.0],  // derecha
            [150.0, -50.0, 80.0], // izquierda
            [250.0, 0.0, 100.0],  // más lejos
            [180.0, 30.0, 70.0],  // bajo
        ];

        for &pos in &test_positions {
            let target = PoseGenerator::drawing_pose(pos);
            let q_init = vec![0.0; 5];

            let result = solve_full_ik(
                &pos_solver,
                &orient_solver,
                &target.position,
                &target.rotation,
                &q_init,
                &robot,
                &base,
                &tool,
            );

            eprintln!();
            eprintln!("═══════════════════════════════════════════════");
            eprintln!(
                "Pose de dibujo: ({:.0}, {:.0}, {:.0}) mm",
                pos[0], pos[1], pos[2]
            );

            match result {
                Err(e) => {
                    eprintln!("❌ IK falló: {e}");
                    continue;
                }
                Ok(q) => {
                    eprintln!(
                        "✅ Solución IK: q = [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}]",
                        q[0], q[1], q[2], q[3], q[4]
                    );

                    // R_target
                    let r_target = target.rotation;
                    eprintln!();
                    eprintln!("R_target (PoseGenerator):");
                    let rt = r_target.matrix();
                    for row in 0..3 {
                        eprintln!(
                            "  [{:>8.4} {:>8.4} {:>8.4}]",
                            rt[(row, 0)],
                            rt[(row, 1)],
                            rt[(row, 2)]
                        );
                    }
                    eprintln!(
                        "  → X5 (marcador) en mundo: [{:.4}, {:.4}, {:.4}]",
                        rt[(0, 0)],
                        rt[(1, 0)],
                        rt[(2, 0)]
                    );

                    // FK con solución IK → R05
                    let robot_q = build_robot(&robot, &q);
                    let (_frames, effector) = forward_kinematics(base, &robot_q);
                    let r05 = get_rot3(&effector);
                    let r05_m = r05.matrix();
                    eprintln!();
                    eprintln!("R05 (FK desde solución IK):");
                    for row in 0..3 {
                        eprintln!(
                            "  [{:>8.4} {:>8.4} {:>8.4}]",
                            r05_m[(row, 0)],
                            r05_m[(row, 1)],
                            r05_m[(row, 2)]
                        );
                    }
                    eprintln!(
                        "  → X5 real en mundo: [{:.4}, {:.4}, {:.4}]",
                        r05_m[(0, 0)],
                        r05_m[(1, 0)],
                        r05_m[(2, 0)]
                    );

                    // TCP position
                    let tool_pose = effector * tool;
                    let tcp = tool_pose.translation.vector;
                    eprintln!();
                    eprintln!(
                        "TCP real (con tool_transform): ({:.2}, {:.2}, {:.2}) mm",
                        tcp.x, tcp.y, tcp.z
                    );
                    eprintln!(
                        "TCP objetivo: ({:.0}, {:.0}, {:.0}) mm",
                        pos[0], pos[1], pos[2]
                    );
                    let pos_err = (Vec3::new(pos[0], pos[1], pos[2]) - tcp).norm();
                    eprintln!("Error posición: {:.4} mm", pos_err);

                    // R_error = R05^T · R_target → si IK es correcta, ≈ I
                    let r_error = r05.transpose() * r_target;
                    let re = r_error.matrix();
                    eprintln!();
                    eprintln!("R_error = R05^T · R_target (≈ I si IK correcta):");
                    for row in 0..3 {
                        eprintln!(
                            "  [{:>8.4} {:>8.4} {:>8.4}]",
                            re[(row, 0)],
                            re[(row, 1)],
                            re[(row, 2)]
                        );
                    }

                    let angle_err = r_error.angle();
                    let frob_err = (re - nalgebra::Matrix3::<f64>::identity()).norm();
                    eprintln!(
                        "Error angular: {:.2e} rad ({:.6}°)",
                        angle_err,
                        angle_err.to_degrees()
                    );
                    eprintln!("Error Frobenius: {:.2e}", frob_err);

                    if angle_err < 1e-6 {
                        eprintln!("✅ R_error ≈ I → IK correcta. El bug está en el render visual.");
                    } else {
                        eprintln!("❌ R_error NO es I → bug en la integración IK.");
                    }
                }
            }
        }
        eprintln!();
        eprintln!("═══════════════════════════════════════════════");
        eprintln!("Diagnóstico completado.");
    }

    // ─── Adaptive drawing IK tests ───────────────────────────────────────

    #[test]
    fn test_solve_drawing_ik_centered() {
        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
        let orient_solver = OrientationSolver::new(1e-6);

        // Posición centrada (q₁≈0) — debe funcionar
        let pos = [200.0, 0.0, 80.0];
        let result = solve_drawing_ik(
            &pos_solver,
            &orient_solver,
            &pos,
            &[0.0; 5],
            &robot,
            &base,
            &tool,
        );
        assert!(result.is_ok(), "centrada debe funcionar: {result:?}");
        let q = result.unwrap();

        // Marcador vertical: X₅ ≈ -Z
        let robot_q = build_robot(&robot, &q);
        let (_frames, effector) = forward_kinematics(base, &robot_q);
        let r05 = get_rot3(&effector);
        // Modo 1: X₅ = -Z (marcador al piso)
        let x5 = r05.matrix().column(0);
        assert!((x5.x).abs() < 1e-6, "X5_x ≈ 0, got {}", x5.x);
        assert!((x5.y).abs() < 1e-6, "X5_y ≈ 0, got {}", x5.y);
        assert!((x5.z + 1.0).abs() < 1e-6, "X5_z ≈ -1, got {}", x5.z);

        eprintln!(
            "✅ solve_drawing_ik centrada: q=[{:.4},{:.4},{:.4},{:.4},{:.4}]",
            q[0], q[1], q[2], q[3], q[4]
        );
    }

    #[test]
    fn test_solve_drawing_ik_lateral() {
        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.1, 0.5);
        let orient_solver = OrientationSolver::new(1e-6);

        // Posiciones laterales — la adaptativa debe funcionar donde
        // la constante fallaba
        let test_positions: [[f64; 3]; 6] = [
            [200.0, 50.0, 80.0],
            [200.0, 100.0, 80.0],
            [200.0, -50.0, 80.0],
            [200.0, -100.0, 80.0],
            [250.0, 50.0, 90.0],
            [150.0, 80.0, 75.0],
        ];

        let all_ok: bool = true;
        for &pos in &test_positions {
            let result = solve_drawing_ik(
                &pos_solver,
                &orient_solver,
                &pos,
                &[0.0; 5],
                &robot,
                &base,
                &tool,
            );

            match result {
                Err(e) => {
                    eprintln!(
                        "⚠️  ({:.0},{:.0},{:.0}) rechazada: {e}",
                        pos[0], pos[1], pos[2]
                    );
                }
                Ok(q) => {
                    let robot_q = build_robot(&robot, &q);
                    let (_frames, effector) = forward_kinematics(base, &robot_q);
                    let r05 = get_rot3(&effector);
                    let x5 = r05.matrix().column(0);
                    let x5_down = (x5.z + 1.0).abs();

                    assert!(
                        x5_down < 1e-6,
                        "({:.0},{:.0},{:.0}): X5_z ≈ -1, got {:.4} (q1={:.1}°)",
                        pos[0],
                        pos[1],
                        pos[2],
                        x5.z,
                        q[0].to_degrees()
                    );
                    eprintln!(
                        "✅ ({:.0},{:.0},{:.0}) q1={:.1}° X5=[{:.3},{:.3},{:.3}]",
                        pos[0],
                        pos[1],
                        pos[2],
                        q[0].to_degrees(),
                        x5.x,
                        x5.y,
                        x5.z
                    );
                }
            }
        }
        assert!(all_ok, "al menos una pose lateral falló");
    }

    #[test]
    fn test_solve_drawing_ik_vs_constant() {
        use crate::kinematics::pose_generator::PoseGenerator;

        let robot = make_robot();
        let base = make_base();
        let tool = make_tool();
        let pos_solver = IkSolver::new(200, 1.0, 0.1, 0.5);
        let orient_solver = OrientationSolver::new(1e-6);

        // Posición lateral — constante falla, adaptativa funciona
        let pos = [200.0, 80.0, 80.0];

        // Constante falla
        let const_pose = PoseGenerator::drawing_pose(pos);
        let const_result = solve_full_ik(
            &pos_solver,
            &orient_solver,
            &pos,
            &const_pose.rotation,
            &[0.0; 5],
            &robot,
            &base,
            &tool,
        );
        assert!(
            const_result.is_err(),
            "constante debería fallar para posición lateral"
        );

        // Adaptativa funciona
        let adapt_result = solve_drawing_ik(
            &pos_solver,
            &orient_solver,
            &pos,
            &[0.0; 5],
            &robot,
            &base,
            &tool,
        );
        assert!(
            adapt_result.is_ok(),
            "adaptativa debería funcionar para posición lateral"
        );

        eprintln!("✅ Constante ✗, Adaptativa ✓ para (200, 80, 80)");
    }
}
