
use nalgebra::DMatrix;

use crate::math::{Mat3, Mat4, Vec3};
use crate::robot::{JointType, Robot};

/// Parámetros másicos de un eslabón.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkParams {
    /// Masa del eslabón (kg).
    pub mass: f64,
    /// Tensor de inercia respecto al COM en el sistema local (kg·mm²).
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

/// Jacobiano angular parcial del eslabón `i` (3×n): columna `j <= i` = ζ_j.
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
mod tests {
    use super::*;
    use crate::kinematics::forward::forward_kinematics;
    use crate::math::PI;
    use crate::robot::fabri_creator::fabri_creator;

    /// Parámetros másicos estimados del FABRI Creator (PETG 25% + servos
    /// MG996R en juntas 1-3 y MG90S en juntas 4-5); cilindro r = 20 mm.
    fn test_links() -> Vec<LinkParams> {
        let cyl = |mass: f64, len: f64| -> Mat3 {
            let r = 20.0;
            let i_xy = mass * (3.0 * r * r + len * len) / 12.0;
            let i_z = mass * r * r / 2.0;
            Mat3::from_diagonal(&Vec3::new(i_xy, i_xy, i_z))
        };
        vec![
            LinkParams::new(0.11, cyl(0.11, 50.0)),
            LinkParams::new(0.10, cyl(0.10, 120.0)),
            LinkParams::new(0.05, cyl(0.05, 90.0)),
            LinkParams::new(0.04, cyl(0.04, 50.0)),
            LinkParams::new(0.02, cyl(0.02, 40.0)),
        ]
    }

    fn eval(q: &[f64]) -> Vec<Mat4> {
        let mut robot = fabri_creator();
        for (i, v) in q.iter().enumerate() {
            robot.segments[i].joint.value = *v;
        }
        let (frames, _) = forward_kinematics(crate::math::Iso3::identity(), &robot);
        frames.iter().map(|iso| iso.to_matrix()).collect()
    }

    #[test]
    fn gravity_home_matches_document() {
        let frames = eval(&[0.0; 5]);
        let robot = fabri_creator();
        let g = gravity_vector(&robot, &frames, &test_links(), 9.81);
        let expected = [0.0, -0.1177, -0.1177, 0.0, 0.0];
        for (i, e) in expected.iter().enumerate() {
            assert!(
                (g[(i, 0)] - e).abs() < 1e-3,
                "g[{i}] = {}, esperado {e}",
                g[(i, 0)]
            );
        }
    }

    #[test]
    fn gravity_q_test_matches_document() {
        let q = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
        let frames = eval(&q);
        let robot = fabri_creator();
        let g = gravity_vector(&robot, &frames, &test_links(), 9.81);
        let expected = [0.0, -0.2925, -0.1177, 0.0, 0.0];
        for (i, e) in expected.iter().enumerate() {
            assert!(
                (g[(i, 0)] - e).abs() < 1e-3,
                "g[{i}] = {}, esperado {e}",
                g[(i, 0)]
            );
        }
    }

    #[test]
    fn inertia_home_matches_document() {
        let frames = eval(&[0.0; 5]);
        let robot = fabri_creator();
        let m = inertia_matrix(&robot, &frames, &test_links());
        let cases = [
            ((0, 0), 1978.9),
            ((1, 1), 4437.2),
            ((1, 2), 1393.2),
            ((3, 3), 17.0),
            ((4, 4), 4.0),
        ];
        for ((r, c), expected) in cases {
            assert!(
                (m[(r, c)] - expected).abs() < 0.2,
                "M[{}][{}] = {}, esperado {expected}",
                r,
                c,
                m[(r, c)]
            );
        }
    }

    #[test]
    fn inertia_q_test_matches_document() {
        let q = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
        let frames = eval(&q);
        let robot = fabri_creator();
        let m = inertia_matrix(&robot, &frames, &test_links());
        let cases = [
            ((0, 0), 6058.2),
            ((1, 1), 6477.5),
            ((1, 2), 2415.2),
            ((3, 3), 17.0),
            ((4, 4), 4.0),
        ];
        for ((r, c), expected) in cases {
            assert!(
                (m[(r, c)] - expected).abs() < 0.2,
                "M[{}][{}] = {}, esperado {expected}",
                r,
                c,
                m[(r, c)]
            );
        }
    }

    #[test]
    fn inertia_matrix_is_symmetric() {
        let q = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
        let frames = eval(&q);
        let robot = fabri_creator();
        let m = inertia_matrix(&robot, &frames, &test_links());
        for i in 0..5 {
            for j in 0..5 {
                assert!(
                    (m[(i, j)] - m[(j, i)]).abs() < 1e-6,
                    "M no simétrica en ({i},{j})"
                );
            }
        }
    }

    #[test]
    fn axes_match_document_home() {
        let frames = eval(&[0.0; 5]);
        let robot = fabri_creator();
        let types: Vec<JointType> = robot.segments.iter().map(|s| s.joint.joint_type).collect();
        let axes = joint_axes(&frames, &types);
        let expected = [
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        ];
        for (j, e) in expected.iter().enumerate() {
            assert!(
                (axes[j].x - e.0).abs() < 1e-9
                    && (axes[j].y - e.1).abs() < 1e-9
                    && (axes[j].z - e.2).abs() < 1e-9,
                "eje ζ{} = {:?}",
                j + 1,
                axes[j]
            );
        }
    }
}
