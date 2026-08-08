use nalgebra::DMatrix;

use crate::math::{Mat3, Mat4, Vec3};
use crate::robot::{JointType, Robot};

#[derive(Debug, Clone, PartialEq)]
pub struct LinkParams {
    pub mass: f64,

    pub inertia: Mat3,
}

impl LinkParams {
    pub fn new(mass: f64, inertia: Mat3) -> Self {
        Self { mass, inertia }
    }
}

fn joint_axes(frames: &[Mat4], joint_types: &[JointType]) -> Vec<Vec3> {
    let mut prevs = vec![Mat4::identity()];
    prevs.extend(frames.iter().take(frames.len().saturating_sub(1)).cloned());

    joint_types
        .iter()
        .enumerate()
        .map(|(j, t)| {
            let prev = &prevs[j];
            if *t == JointType::Twist {
                prev.fixed_view::<3, 1>(0, 0).into_owned()
            } else {
                prev.fixed_view::<3, 1>(0, 2).into_owned()
            }
        })
        .collect()
}

fn jacobian_com(frames: &[Mat4], types: &[JointType], axes: &[Vec3], i: usize) -> DMatrix<f64> {
    let n = frames.len();
    let p_i = frames[i].fixed_view::<3, 1>(0, 3).into_owned();
    let mut jc = DMatrix::zeros(3, n);
    for j in 0..=i {
        if types[j] == JointType::Twist {
            continue;
        }
        let p_prev = if j == 0 {
            Vec3::zeros()
        } else {
            frames[j - 1].fixed_view::<3, 1>(0, 3).into_owned()
        };
        jc.column_mut(j).copy_from(&axes[j].cross(&(p_i - p_prev)));
    }
    jc
}

fn jacobian_angular(axes: &[Vec3], i: usize) -> DMatrix<f64> {
    let n = axes.len();
    let mut jw = DMatrix::zeros(3, n);
    for (j, axis) in axes.iter().enumerate().take(i + 1) {
        jw.column_mut(j).copy_from(axis);
    }
    jw
}

pub fn inertia_matrix(robot: &Robot, frames: &[Mat4], links: &[LinkParams]) -> DMatrix<f64> {
    let n = robot.dof();
    let types: Vec<JointType> = robot.segments.iter().map(|s| s.joint.joint_type).collect();
    let axes = joint_axes(frames, &types);

    let mut m = DMatrix::zeros(n, n);
    for i in 0..n {
        let r_i: Mat3 = frames[i].fixed_view::<3, 3>(0, 0).into_owned();
        let jc = jacobian_com(frames, &types, &axes, i);
        let jw = jacobian_angular(&axes, i);
        let rotational = jw.transpose() * (r_i * links[i].inertia * r_i.transpose()) * &jw;
        m += links[i].mass * (jc.transpose() * &jc) + rotational;
    }
    m
}

pub fn gravity_vector(
    robot: &Robot,
    frames: &[Mat4],
    links: &[LinkParams],
    g: f64,
) -> DMatrix<f64> {
    let n = robot.dof();
    let types: Vec<JointType> = robot.segments.iter().map(|s| s.joint.joint_type).collect();
    let axes = joint_axes(frames, &types);

    let mut gvec = DMatrix::zeros(n, 1);
    for (j, link) in links.iter().enumerate() {
        let jc = jacobian_com(frames, &types, &axes, j);
        for i in 0..n {
            gvec[(i, 0)] += link.mass * jc[(2, i)];
        }
    }
    gvec * (g * 1e-3)
}

#[cfg(test)]
#[path = "dynamics_tests.rs"]
mod dynamics_tests;
