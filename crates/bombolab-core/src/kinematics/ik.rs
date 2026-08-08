use std::fmt;

use nalgebra::SMatrix;

use crate::math::{Iso3, Rot3, Vec3};
use crate::robot::{Joint, JointType, Robot, Segment};

use super::forward::forward_kinematics;

#[derive(Debug, Clone)]
pub enum IkError {
    DegenerateChain,
    MaxIterationsReached { error: f64 },

    UnreachableOrientation { r35_02: f64, tolerance: f64 },

    DrawingConstraintViolated { q23: f64 },

    InvalidInitLength { expected: usize, got: usize },
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
            IkError::DrawingConstraintViolated { q23 } => {
                write!(
                    f,
                    "drawing constraint not reachable: q2+q3 = {q23:.3} rad (q5 = −q23) exceeds wrist pitch limits"
                )
            }
            IkError::InvalidInitLength { expected, got } => {
                write!(f, "invalid q_init length: expected {expected}, got {got}")
            }
        }
    }
}

impl std::error::Error for IkError {}

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
            let dh = seg.dh;
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

fn position_error(robot: &Robot, target: &[f64; 3], base: &Iso3, tool: &Iso3) -> f64 {
    let (frames, _) = forward_kinematics(*base, robot);
    let tool_pose = frames.last().unwrap() * tool;
    let p_ee = tool_pose.translation.vector;
    let target_v = Vec3::new(target[0], target[1], target[2]);
    (target_v - p_ee).norm()
}

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

            let (frames, _) = forward_kinematics(*base, &robot_q);
            let tool_pose = frames.last().unwrap() * tool;
            let p_ee = tool_pose.translation.vector;

            let error = target_v - p_ee;
            let err_norm = error.norm();
            if err_norm < self.tolerance {
                return Ok(q);
            }

            let j = position_jacobian(&robot_q, &frames, &p_ee, base, n);

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

                    q[idx] = q[idx].clamp(
                        robot.segments[idx].joint.value_min,
                        robot.segments[idx].joint.value_max,
                    );
                }
            } else {
                return Ok(q);
            }
        }

        let robot_q = build_robot(robot, &q);
        let final_err = position_error(&robot_q, target, base, tool);
        Err(IkError::MaxIterationsReached { error: final_err })
    }
}

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

impl Default for IkSolver {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1.0,
            damping: 0.1,
            step_size: 0.5,
        }
    }
}

#[derive(Debug, Clone)]
pub enum OrientationError {
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

pub struct OrientationSolver {
    pub tolerance: f64,
}

impl OrientationSolver {
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
    }

    pub fn solve(
        &self,
        r03: &Rot3,
        r_target: &Rot3,
        robot: &Robot,
    ) -> Result<[f64; 2], OrientationError> {
        let r35 = r03.transpose() * r_target;
        let m = r35.matrix();

        let r35_02 = m[(0, 2)].abs();
        if r35_02 > self.tolerance {
            return Err(OrientationError::UnreachableOrientation {
                r35_02,
                tolerance: self.tolerance,
            });
        }

        let c4 = -m[(1, 2)];
        let s4 = -m[(2, 2)];
        let c5 = m[(0, 0)];
        let s5 = -m[(0, 1)];

        let q4 = s4.atan2(c4);
        let q5 = s5.atan2(c5);

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
    let q_pos = pos_solver.solve_position(target_pos, q_init, robot, base, tool)?;

    let q1 = q_pos[0];
    let q2 = q_pos[1];
    let q3 = q_pos[2];
    let q_partial = [q1, q2, q3, 0.0, 0.0];
    let robot_partial = build_robot(robot, &q_partial);
    let (frames, _) = forward_kinematics(*base, &robot_partial);
    let r03 = frames[2].rotation.to_rotation_matrix();

    let [q4, q5] = orient_solver
        .solve(&r03, target_rot, robot)
        .map_err(|err| match err {
            OrientationError::UnreachableOrientation { r35_02, tolerance } => {
                IkError::UnreachableOrientation { r35_02, tolerance }
            }
        })?;

    Ok(vec![q1, q2, q3, q4, q5])
}

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

    let q_pos = pos_solver.solve_position(target_pos, q_init, robot, base, tool)?;
    let q1 = q_pos[0];
    let q2 = q_pos[1];
    let q3 = q_pos[2];

    let target = PoseGenerator::drawing_pose_adaptive(*target_pos, q1);

    let q_partial = [q1, q2, q3, 0.0, 0.0];
    let robot_partial = build_robot(robot, &q_partial);
    let (frames, _) = forward_kinematics(*base, &robot_partial);
    let r03 = frames[2].rotation.to_rotation_matrix();

    let [q4, q5] = orient_solver
        .solve(&r03, &target.rotation, robot)
        .map_err(|err| match err {
            OrientationError::UnreachableOrientation { r35_02, tolerance } => {
                IkError::UnreachableOrientation { r35_02, tolerance }
            }
        })?;

    Ok(vec![q1, q2, q3, q4, q5])
}

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

    let q_pos = pos_solver.solve_position(target_pos, q_init, robot, base, tool)?;
    let q1 = q_pos[0];
    let q2 = q_pos[1];
    let q3 = q_pos[2];

    let target = PoseGenerator::drawing_pose_v2(*target_pos, q1);

    let q_partial = [q1, q2, q3, 0.0, 0.0];
    let robot_partial = build_robot(robot, &q_partial);
    let (frames, _) = forward_kinematics(*base, &robot_partial);
    let r03 = frames[2].rotation.to_rotation_matrix();

    let [q4, q5] = orient_solver
        .solve(&r03, &target.rotation, robot)
        .map_err(|err| match err {
            OrientationError::UnreachableOrientation { r35_02, tolerance } => {
                IkError::UnreachableOrientation { r35_02, tolerance }
            }
        })?;

    Ok(vec![q1, q2, q3, q4, q5])
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DrawingConfiguration {
    pub q1: f64,
    pub q2: f64,
    pub q3: f64,
}

impl DrawingConfiguration {
    pub fn new(q1: f64, q2: f64, q3: f64) -> Self {
        Self { q1, q2, q3 }
    }

    pub fn from_q(q: &[f64]) -> Self {
        Self {
            q1: q[0],
            q2: q[1],
            q3: q[2],
        }
    }

    pub fn full_configuration(&self) -> [f64; 5] {
        [self.q1, self.q2, self.q3, 0.0, -(self.q2 + self.q3)]
    }

    pub fn q23(&self) -> f64 {
        self.q2 + self.q3
    }
}

pub fn solve_drawing_plane_ik(
    solver: &IkSolver,
    target: &[f64; 3],
    q_init: &[f64],
    robot: &Robot,
    base: &Iso3,
    tool: &Iso3,
) -> Result<[f64; 5], IkError> {
    if q_init.len() < 3 {
        return Err(IkError::InvalidInitLength {
            expected: 3,
            got: q_init.len(),
        });
    }
    if robot.dof() < 3 {
        return Err(IkError::DegenerateChain);
    }
    let n = robot.dof().min(3);
    let j5_lo = robot.segments[4].joint.value_min;
    let j5_hi = robot.segments[4].joint.value_max;

    let mut cfg = DrawingConfiguration::from_q(q_init);
    let target_v = Vec3::new(target[0], target[1], target[2]);
    let damping_sq = solver.damping * solver.damping;

    for _iter in 0..solver.max_iterations {
        let q_full = cfg.full_configuration();
        let robot_q = build_robot(robot, &q_full);
        let (frames, _) = forward_kinematics(*base, &robot_q);
        let p_ee = (frames.last().unwrap() * tool).translation.vector;

        let error = target_v - p_ee;
        let err_norm = error.norm();
        if err_norm < solver.tolerance {
            return Ok(q_full);
        }

        let j_full = position_jacobian(&robot_q, &frames, &p_ee, base, robot.dof().min(5));
        let mut jr = SMatrix::<f64, 3, 3>::zeros();
        for r in 0..3 {
            jr[(r, 0)] = j_full[(r, 0)];
            jr[(r, 1)] = j_full[(r, 1)] - j_full[(r, 4)];
            jr[(r, 2)] = j_full[(r, 2)] - j_full[(r, 4)];
        }

        let jjt = jr * jr.transpose();
        let reg = jjt + SMatrix::<f64, 3, 3>::identity() * damping_sq;
        if let Some(inv) = reg.try_inverse() {
            let delta_x = inv * error;
            let delta_q = jr.transpose() * delta_x;

            let dq_norm = delta_q.norm();
            let scale = if dq_norm > solver.step_size {
                solver.step_size / dq_norm
            } else {
                1.0
            };

            let mut vals = [
                cfg.q1 + delta_q[0] * scale,
                cfg.q2 + delta_q[1] * scale,
                cfg.q3 + delta_q[2] * scale,
            ];
            for (i, v) in vals.iter_mut().enumerate().take(n) {
                *v = v.clamp(
                    robot.segments[i].joint.value_min,
                    robot.segments[i].joint.value_max,
                );
            }
            cfg.q1 = vals[0];
            cfg.q2 = vals[1];
            cfg.q3 = vals[2];
        } else {
            return Ok(q_full);
        }

        let q23 = cfg.q23();
        let q5 = -q23;
        if q5 < j5_lo || q5 > j5_hi {
            return Err(IkError::DrawingConstraintViolated { q23 });
        }
    }

    let robot_q = build_robot(robot, &cfg.full_configuration());
    let (frames, _) = forward_kinematics(*base, &robot_q);
    let p_ee = (frames.last().unwrap() * tool).translation.vector;
    let final_err = (target_v - p_ee).norm();
    Err(IkError::MaxIterationsReached { error: final_err })
}

#[cfg(test)]
#[path = "ik_tests.rs"]
mod ik_tests;

#[cfg(test)]
#[path = "ik_orientation_tests.rs"]
mod ik_orientation_tests;

#[cfg(test)]
#[path = "ik_full_tests.rs"]
mod ik_full_tests;
