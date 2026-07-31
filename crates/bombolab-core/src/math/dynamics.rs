use nalgebra::DMatrix;

use crate::math::{Mat3, Mat4, Vec3};
use crate::robot::{JointType, Robot};

/// Parámetros másicos de un eslabón.
///
/// # Simplificaciones deliberadas del modelo dinámico
///
/// Este módulo implementa un modelo **estático** (gravedad + inercia):
///
/// - **COM en el origen del frame**: el centro de masa de cada eslabón se
///   modela en el origen de su frame (no hay offset de Steiner). Esto hace
///   que la columna de un joint Twist en `jacobian_com` sea exactamente
///   cero (el Twist no traslada orígenes de frames), por lo que descartarla
///   con `continue` es correcto por construcción — no una aproximación.
///   Si en el futuro se agrega un `com_offset` real, el `continue` y el
///   pivot del Twist deberán revisarse (ver jacobian.rs, MATH-01/02).
/// - **Sin Coriolis/centrífugos**: no existe término C(q, q̇). La ecuación
///   implementada es τ = M(q)q̈ + g(q), válida solo en reposo o para
///   control cuasiestático. No usar para control dinámico de trayectorias.
/// - **Masas e inercias estimadas**: valores de prueba (PETG 25% + specs
///   de servos), no medidos. Ver `tests::test_links`.
///
/// # Unidades
///
/// Masa en kg, longitudes en mm, inercia en kg·mm²; las salidas de pares
/// están en N·m (factor 1e-3 aplicado en `gravity_vector`).
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

    /// Verificación INDEPENDIENTE de M(q) y g(q) por diferencias finitas.
    ///
    /// No usa las fórmulas del código (jacobian_com / jacobian_angular):
    /// construye la energía cinética directamente desde la FK:
    ///   T(q̇) = ½ Σ m_j·‖v_j‖² + ½ Σ ω_jᵀ·(R_j·I_j·R_jᵀ)·ω_j
    /// donde v_j y ω_j se obtienen numéricamente perturbando q en la
    /// dirección q̇ (central differences sobre la FK). Luego:
    ///   M_ij = ∂²T/∂q̇_i∂q̇_j   (Hessiano de T respecto a velocidades)
    ///   g_i  = ∂V/∂q_i,  V = Σ m_j·g·z_j(q)   (energía potencial)
    /// Compara contra `inertia_matrix` / `gravity_vector`.
    #[test]
    fn dynamics_matches_independent_finite_differences() {
        let links = test_links();
        let g = 9.81;

        // Configuración fuera de home: twist y pitch activos (q4, q5 ≠ 0).
        let q = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
        let n = 5;

        // ── Helper: frames para un q dado (reusa eval) ────────────────
        let frames_of = |qq: &[f64; 5]| eval(qq);

        // ── Energía cinética T(q̇) por FD de la FK ─────────────────────
        // Perturba q en la dirección q̇ con paso δ: v = (p(q+δq̇) − p(q−δq̇))/(2δ)
        // y ω extraída de R_rel = R(q+δq̇)·R(q−δq̇)ᵀ (skew simétrico).
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
                // ω ≈ vee((R_rel − R_relᵀ)/2) / (2δ)
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

        // ── M por identidad de forma cuadrática ─────────────────────────
        // T(q̇) = ½ q̇ᵀ M q̇ es exactamente cuadrática en q̇, así que:
        //   M_ii ≈ 2·T(h·e_i)/h²
        //   M_ij ≈ [T(h(e_i+e_j)) − T(h·e_i) − T(h·e_j)]/h²
        // (una sola capa de FD — el Hessiano numérico doble amplifica el
        // redondeo: v ~1e-6 → T ~1e-10 → M ~1e-2 sobre valores de ~6000)
        let robot = fabri_creator();
        let frames = frames_of(&q);
        let m_code = inertia_matrix(&robot, &frames, &links);
        let h = 1e-3;
        let t0 = kinetic_energy(&[0.0; 5]);

        // Precomputar T(h·e_i) para todos los ejes.
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
                // Tolerancia absoluta acorde a la magnitud (error numérico
                // ~1e-4 en valores de miles) — no es una discrepancia del
                // modelo, es precisión del método FD.
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

        // ── g_i = ∂V/∂q_i por FD central de primer orden ───────────────
        // V(q) = Σ m_j·g·z_j(q), con z_j la altura (componente Z del COM,
        // que por simplificación del modelo es el origen del frame j).
        let potential = |qq: &[f64; 5]| -> f64 {
            let ff = frames_of(qq);
            let mut v = 0.0;
            for j in 0..n {
                let z = ff[j].fixed_view::<3, 1>(0, 3)[(2, 0)];
                v += links[j].mass * g * z;
            }
            v * 1e-3 // mm → m
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

    /// Verifica que la columna del Twist en `jacobian_com` es CERO para el
    /// modelo COM-en-origen (no solo en home): con q₄ activo, ningún origen
    /// de frame se traslada con q₄ (la traslación del Twist es constante),
    /// por lo que descartar la columna (continue) es exacto — no una
    /// simplificación que rompa el modelo.
    #[test]
    fn twist_com_column_is_zero_by_construction() {
        let q = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
        let frames = eval(&q);
        let robot = fabri_creator();
        let types: Vec<JointType> = robot.segments.iter().map(|s| s.joint.joint_type).collect();
        let axes = joint_axes(&frames, &types);

        // Para CADA eslabón i, la velocidad del COM (origen del frame i)
        // bajo q̇ = e₃ (twist) debe ser 0: el twist no traslada orígenes.
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
            // La columna del código (que descarta twist) debe coincidir con
            // la velocidad FD del COM: columna i del Jc con j=3 → 0.
            let jc = jacobian_com(&frames, &types, &axes, i);
            assert!(
                jc[(0, 3)].abs() < 1e-12 && jc[(1, 3)].abs() < 1e-12 && jc[(2, 3)].abs() < 1e-12,
                "jacobian_com columna twist no es cero para eslabón {i}"
            );
        }
    }
}
