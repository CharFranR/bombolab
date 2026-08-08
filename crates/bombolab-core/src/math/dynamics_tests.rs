//! Unit tests for `dynamics_tests`.

use super::*;
use crate::kinematics::forward::forward_kinematics;
use crate::math::PI;
use crate::robot::fabri_creator::fabri_creator;

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

#[test]
fn dynamics_matches_independent_finite_differences() {
    let links = test_links();
    let g = 9.81;

    let q = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
    let n = 5;

    let frames_of = |qq: &[f64; 5]| eval(qq);

    let kinetic_energy = |qd: &[f64; 5]| -> f64 {
        let delta = 1e-7;
        let q_plus: [f64; 5] = std::array::from_fn(|i| q[i] + delta * qd[i]);
        let q_minus: [f64; 5] = std::array::from_fn(|i| q[i] - delta * qd[i]);
        let f_plus = frames_of(&q_plus);
        let f_minus = frames_of(&q_minus);

        let mut t = 0.0;
        for j in 0..n {
            let p_p = f_plus[j].fixed_view::<3, 1>(0, 3).into_owned();
            let p_m = f_minus[j].fixed_view::<3, 1>(0, 3).into_owned();
            let v = (p_p - p_m) / (2.0 * delta);
            t += 0.5 * links[j].mass * v.norm_squared();

            let r_p = f_plus[j].fixed_view::<3, 3>(0, 0).into_owned();
            let r_m = f_minus[j].fixed_view::<3, 3>(0, 0).into_owned();
            let r_rel = r_p * r_m.transpose();

            let wx = (r_rel[(2, 1)] - r_rel[(1, 2)]) / (4.0 * delta);
            let wy = (r_rel[(0, 2)] - r_rel[(2, 0)]) / (4.0 * delta);
            let wz = (r_rel[(1, 0)] - r_rel[(0, 1)]) / (4.0 * delta);
            let omega = Vec3::new(wx, wy, wz);
            let i_world = r_p * links[j].inertia * r_p.transpose();
            let rot_term = omega.transpose() * (i_world * omega);
            t += 0.5 * rot_term[(0, 0)];
        }
        t
    };

    let robot = fabri_creator();
    let frames = frames_of(&q);
    let m_code = inertia_matrix(&robot, &frames, &links);
    let h = 1e-3;
    let t0 = kinetic_energy(&[0.0; 5]);

    let mut t_axis = [0.0; 5];
    for i in 0..n {
        let mut e_i = [0.0; 5];
        e_i[i] = 1.0;
        t_axis[i] = kinetic_energy(&std::array::from_fn(|k| e_i[k] * h));
    }

    for i in 0..n {
        for jj in 0..n {
            let m_num = if i == jj {
                2.0 * (t_axis[i] - t0) / (h * h)
            } else {
                let mut e_i = [0.0; 5];
                e_i[i] = 1.0;
                let mut e_j = [0.0; 5];
                e_j[jj] = 1.0;
                let t_ij = kinetic_energy(&std::array::from_fn(|k| h * (e_i[k] + e_j[k])));
                (t_ij - t_axis[i] - t_axis[jj] + t0) / (h * h)
            };

            let tol = 0.05 * (m_num.abs().max(m_code[(i, jj)].abs()).max(1.0));
            assert!(
                (m_num - m_code[(i, jj)]).abs() < tol,
                "M[{}][{}]: FD = {:.4}, código = {:.4}, diff = {:.3e} (tol {:.3e})",
                i,
                jj,
                m_num,
                m_code[(i, jj)],
                (m_num - m_code[(i, jj)]).abs(),
                tol
            );
        }
    }

    let potential = |qq: &[f64; 5]| -> f64 {
        let ff = frames_of(qq);
        let mut v = 0.0;
        for j in 0..n {
            let z = ff[j].fixed_view::<3, 1>(0, 3)[(2, 0)];
            v += links[j].mass * g * z;
        }
        v * 1e-3
    };
    let g_code = gravity_vector(&robot, &frames, &links, g);
    let eps = 1e-7;
    for i in 0..n {
        let mut qp = q;
        let mut qm = q;
        qp[i] += eps;
        qm[i] -= eps;
        let g_num = (potential(&qp) - potential(&qm)) / (2.0 * eps);
        assert!(
            (g_num - g_code[(i, 0)]).abs() < 1e-4,
            "g[{}]: FD = {:.5e}, código = {:.5e}, diff = {:.3e}",
            i,
            g_num,
            g_code[(i, 0)],
            (g_num - g_code[(i, 0)]).abs()
        );
    }
}

#[test]
fn twist_com_column_is_zero_by_construction() {
    let q = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
    let frames = eval(&q);
    let robot = fabri_creator();
    let types: Vec<JointType> = robot.segments.iter().map(|s| s.joint.joint_type).collect();
    let axes = joint_axes(&frames, &types);

    let delta = 1e-7;
    let mut qp = q;
    let mut qm = q;
    qp[3] += delta;
    qm[3] -= delta;
    let fp = eval(&qp);
    let fm = eval(&qm);

    for i in 0..5 {
        let p_p = fp[i].fixed_view::<3, 1>(0, 3).into_owned();
        let p_m = fm[i].fixed_view::<3, 1>(0, 3).into_owned();
        let v = (p_p - p_m) / (2.0 * delta);
        let speed = v.norm();
        assert!(
            speed < 1e-6,
            "COM del eslabón {i} se mueve bajo q̇₄: ‖v‖ = {speed:.3e} — \
                 el continue del Twist en jacobian_com sería incorrecto"
        );

        let jc = jacobian_com(&frames, &types, &axes, i);
        assert!(
            jc[(0, 3)].abs() < 1e-12 && jc[(1, 3)].abs() < 1e-12 && jc[(2, 3)].abs() < 1e-12,
            "jacobian_com columna twist no es cero para eslabón {i}"
        );
    }
}
