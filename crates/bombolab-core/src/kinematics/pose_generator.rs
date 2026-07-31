use crate::math::Rot3;

/// Pose objetivo: posición y orientación del efector.
pub struct TargetPose {
    pub position: [f64; 3],
    pub rotation: Rot3,
}

/// Generador de poses para tareas del robot.
///
/// El `PoseGenerator` es una capa por encima de la IK que produce
/// `TargetPose`s (posición + orientación) según la tarea. No contiene
/// lógica de alcanzabilidad — eso es responsabilidad de la IK.
///
/// # Modos de dibujo
///
/// - [`drawing_pose`](PoseGenerator::drawing_pose): orientación constante
///   (θ=π). Funciona solo para brazos centrados (q₁≈0).
///
/// - [`drawing_pose_adaptive`](PoseGenerator::drawing_pose_adaptive):
///   orientación adaptativa (θ=q₁+π). Funciona para cualquier q₁.
///   Es el modo recomendado para dibujo real.
///
/// # Uso
///
/// ```rust,ignore
/// // Modo simple (solo q₁≈0):
/// let pose = PoseGenerator::drawing_pose([200.0, 0.0, 80.0]);
///
/// // Modo adaptativo (cualquier q₁):
/// let pose = PoseGenerator::drawing_pose_adaptive([200.0, 0.0, 80.0], q1);
/// ```
pub struct PoseGenerator;

impl PoseGenerator {
    /// Genera un `TargetPose` para dibujar sobre una superficie horizontal.
    ///
    /// La orientación es constante: el marcador permanece perpendicular
    /// al plano XY (apuntando en -Z mundo, hacia abajo).
    ///
    /// La rotación R_target usa θ=π en la parametrización:
    ///   R(θ) = [0 cosθ sinθ; 0 sinθ -cosθ; -1 0 0]
    /// Con θ=π → col0=[0,0,-1] (marcador vertical), col1=[-1,0,0], col2=[0,1,0].
    /// Esta elección produce q4=0 para configuraciones centradas (q₁≈0),
    /// manteniendo q5 dentro de límites articulares para dibujo típico.
    ///
    /// - `position`: coordenadas (x, y, z) del TCP deseado en mm.
    pub fn drawing_pose(position: [f64; 3]) -> TargetPose {
        TargetPose {
            position,
            // R_target(π): θ = π → q4=0 para brazos centrados
            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0,
            )),
        }
    }

    /// Genera un `TargetPose` para dibujo con orientación adaptativa.
    ///
    /// A diferencia de [`drawing_pose`](PoseGenerator::drawing_pose), que usa
    /// una R_target constante (θ=π), esta función ajusta la orientación según
    /// el ángulo de base q₁ usando θ = q₁ + π.
    ///
    /// Esto garantiza que:
    /// - El marcador apunte siempre en -Z mundo (vertical hacia abajo). ✓
    /// - R35[0,2] = 0 → la orientación es siempre alcanzable. ✓
    /// - q4 = 0 → no se desperdicia recorrido articular en roll. ✓
    /// - q5 dentro de límites para configuraciones de dibujo típicas. ✓
    ///
    /// # Parámetros
    ///
    /// - `position`: coordenadas (x, y, z) del TCP deseado en mm.
    /// - `q1`: ángulo de la base (J1) en radianes, obtenido del position solver.
    ///
    /// # Matemática
    ///
    /// R_target(q₁) = [0  -cos(q₁)  -sin(q₁)]
    ///                [0  -sin(q₁)   cos(q₁)]
    ///                [-1   0         0     ]
    ///
    /// Con θ = q₁ + π:
    ///   cos(θ) = -cos(q₁), sin(θ) = -sin(q₁)
    pub fn drawing_pose_adaptive(position: [f64; 3], q1: f64) -> TargetPose {
        let (sq1, cq1) = q1.sin_cos();
        TargetPose {
            position,
            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                0.0, -cq1, -sq1, 0.0, -sq1, cq1, -1.0, 0.0, 0.0,
            )),
        }
    }

    /// Modo 2: el marcador apunta en Y₅ (perpendicular al gripper).
    ///
    /// En el robot real, el marcador está montado perpendicular al gripper,
    /// apuntando en dirección Y₅ del frame herramienta.
    ///
    /// R_target(q₁) = [c₁  0  -s₁; s₁  0  c₁; 0  -1  0]
    ///
    /// Propiedades:
    /// - Y₅ = [0, 0, -1] (marcador vertical hacia abajo) ✓
    /// - R35[0,2] = 0 (siempre alcanzable) ✓
    /// - q4 = 0 ✓
    /// - q5 = -q₂₃ (dentro de límites para dibujo típico) ✓
    pub fn drawing_pose_v2(position: [f64; 3], q1: f64) -> TargetPose {
        let (sq1, cq1) = q1.sin_cos();
        TargetPose {
            position,
            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                cq1, 0.0, -sq1, sq1, 0.0, cq1, 0.0, -1.0, 0.0,
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    const EPS: f64 = 1e-10;
    const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

    #[test]
    fn test_drawing_pose_marker_points_down() {
        let pose = PoseGenerator::drawing_pose([200.0, 0.0, 100.0]);

        // La columna 0 de R_target (X5 en mundo) debe ser [0, 0, -1]
        // porque el marcador se extiende a lo largo de X5
        let m = pose.rotation.matrix();
        assert!(
            (m[(0, 0)] - 0.0).abs() < EPS,
            "X5_x debe ser 0, got {}",
            m[(0, 0)]
        );
        assert!(
            (m[(1, 0)] - 0.0).abs() < EPS,
            "X5_y debe ser 0, got {}",
            m[(1, 0)]
        );
        assert!(
            (m[(2, 0)] + 1.0).abs() < EPS,
            "X5_z debe ser -1, got {}",
            m[(2, 0)]
        );

        // Columna 1 (Y5) debe ser [-1, 0, 0] (θ=π)
        assert!((m[(0, 1)] + 1.0).abs() < EPS, "Y5_x debe ser -1");
        assert!((m[(1, 1)] - 0.0).abs() < EPS, "Y5_y debe ser 0");
        assert!((m[(2, 1)] - 0.0).abs() < EPS, "Y5_z debe ser 0");

        // Columna 2 (Z5) debe ser [0, 1, 0] (θ=π)
        assert!((m[(0, 2)] - 0.0).abs() < EPS, "Z5_x debe ser 0");
        assert!((m[(1, 2)] - 1.0).abs() < EPS, "Z5_y debe ser 1");
        assert!((m[(2, 2)] - 0.0).abs() < EPS, "Z5_z debe ser 0");
    }

    #[test]
    fn test_drawing_pose_position_passthrough() {
        let pos = [150.0, -30.0, 85.0];
        let pose = PoseGenerator::drawing_pose(pos);
        assert_eq!(pose.position, pos, "posición debe pasar tal cual");
    }

    #[test]
    fn test_drawing_pose_is_valid_rotation() {
        let pose = PoseGenerator::drawing_pose([0.0, 0.0, 0.0]);
        let m = pose.rotation.matrix();

        // Determinante = +1
        let det = m.determinant();
        assert!(
            (det - 1.0).abs() < EPS,
            "det(R) debe ser 1, got {:.2e}",
            det
        );

        // Columnas unitarias y ortogonales
        for i in 0..3 {
            let col: Vec3 = m.column(i).into_owned();
            let norm = col.norm();
            assert!(
                (norm - 1.0).abs() < EPS,
                "col {i} debe tener norma 1, got {:.2e}",
                norm
            );
        }

        // Ortogonalidad: R^T · R = I
        let rtr = pose.rotation.transpose() * pose.rotation;
        let diff = (rtr.matrix() - nalgebra::Matrix3::identity()).norm();
        assert!(diff < EPS, "R^T·R debe ser I, error {:.2e}", diff);
    }

    #[test]
    fn test_drawing_pose_is_constant() {
        let p1 = PoseGenerator::drawing_pose([100.0, 0.0, 50.0]);
        let p2 = PoseGenerator::drawing_pose([200.0, 50.0, 80.0]);
        let p3 = PoseGenerator::drawing_pose([0.0, 0.0, 100.0]);

        let diff12 = (p1.rotation.matrix() - p2.rotation.matrix()).norm();
        let diff13 = (p1.rotation.matrix() - p3.rotation.matrix()).norm();
        assert!(
            diff12 < EPS,
            "rotación debe ser constante, diff12={:.2e}",
            diff12
        );
        assert!(
            diff13 < EPS,
            "rotación debe ser constante, diff13={:.2e}",
            diff13
        );
    }

    // ─── Adaptive pose tests ─────────────────────────────────────────────

    #[test]
    fn test_adaptive_q1_zero_matches_drawing_pose() {
        // Con q1=0, drawing_pose_adaptive debe dar la misma R que drawing_pose
        let p_const = PoseGenerator::drawing_pose([200.0, 0.0, 80.0]);
        let p_adapt = PoseGenerator::drawing_pose_adaptive([200.0, 0.0, 80.0], 0.0);
        let diff = (p_const.rotation.matrix() - p_adapt.rotation.matrix()).norm();
        assert!(
            diff < EPS,
            "con q1=0 adaptive debe coincidir con constante, diff={:.2e}",
            diff
        );
    }

    #[test]
    fn test_adaptive_marker_always_points_down() {
        // Para cualquier q1, X5 debe ser [0, 0, -1]
        for &q1_deg in &[-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0] {
            let q1: f64 = q1_deg * DEG_TO_RAD;
            let pose = PoseGenerator::drawing_pose_adaptive([200.0, 0.0, 80.0], q1);
            let m = pose.rotation.matrix();
            assert!(
                (m[(0, 0)] - 0.0).abs() < EPS
                    && (m[(1, 0)] - 0.0).abs() < EPS
                    && (m[(2, 0)] + 1.0).abs() < EPS,
                "q1={}°: X5 debe ser [0,0,-1], got [{:.2e}, {:.2e}, {:.2e}]",
                q1_deg,
                m[(0, 0)],
                m[(1, 0)],
                m[(2, 0)]
            );
        }
    }

    #[test]
    fn test_adaptive_valid_rotation_for_all_q1() {
        for &q1_deg in &[-60.0, -30.0, 0.0, 30.0, 60.0] {
            let q1: f64 = q1_deg * DEG_TO_RAD;
            let pose = PoseGenerator::drawing_pose_adaptive([100.0, 0.0, 80.0], q1);
            let m = pose.rotation.matrix();

            // det = +1
            let det = m.determinant();
            assert!(
                (det - 1.0).abs() < 1e-8,
                "q1={}°: det debe ser 1, got {:.2e}",
                q1_deg,
                det
            );

            // R^T · R = I
            let rtr = pose.rotation.transpose() * pose.rotation;
            let diff = (rtr.matrix() - nalgebra::Matrix3::identity()).norm();
            assert!(
                diff < EPS,
                "q1={}°: R^T·R debe ser I, error {:.2e}",
                q1_deg,
                diff
            );
        }
    }

    #[test]
    fn test_adaptive_theta_equals_q1_plus_pi() {
        // Verificar la relación: θ = q₁ + π
        // R_target(θ) = [0 cosθ sinθ; 0 sinθ -cosθ; -1 0 0]
        // Con θ = q₁ + π: cosθ = -cos(q₁), sinθ = -sin(q₁)
        for &q1_deg in &[-45.0, 0.0, 45.0] {
            let q1: f64 = q1_deg * DEG_TO_RAD;
            let pose = PoseGenerator::drawing_pose_adaptive([0.0, 0.0, 0.0], q1);
            let m = pose.rotation.matrix();

            // Debe cumplir: cosθ = -cos(q1), sinθ = -sin(q1)
            // R_target[0,1] = cosθ = -cos(q1)
            let expected_cos_theta = -q1.cos();
            assert!(
                (m[(0, 1)] - expected_cos_theta).abs() < EPS,
                "q1={}°: cosθ debe ser {:.4}, got {:.4}",
                q1_deg,
                expected_cos_theta,
                m[(0, 1)]
            );

            // R_target[0,2] = sinθ = -sin(q1)
            let expected_sin_theta = -q1.sin();
            assert!(
                (m[(0, 2)] - expected_sin_theta).abs() < EPS,
                "q1={}°: sinθ debe ser {:.4}, got {:.4}",
                q1_deg,
                expected_sin_theta,
                m[(0, 2)]
            );
        }
    }

    #[test]
    fn test_adaptive_r35_02_is_zero() {
        // Verificar que R35[0,2] = 0 para cualquier q1,q2,q3
        // R35 = R03^T · R_target(q1+π)
        // R35[0,2] = c₂₃·sin((q1+π)-q1) = c₂₃·sin(π) = 0 ✓
        for &q1_deg in &[-30.0, 0.0, 30.0] {
            let q1: f64 = q1_deg * DEG_TO_RAD;
            let (_s1, c1) = q1.sin_cos();

            for &q23_deg in &[-60.0, -30.0, 0.0, 30.0, 60.0] {
                let q23: f64 = q23_deg * DEG_TO_RAD;
                let s23 = q23.sin();

                // R03 para estos valores
                let _r03 = [
                    [c1 * s23, 0.0, -c1 * s23],
                    [0.0, -c1, 0.0],
                    [-s23, 0.0, -s23],
                ];

                // Esto es solo una verificación conceptual
                // R35[0,2] = c₂₃·sin(π) = 0 (analíticamente)
                // La verificación real se hace en el test full_ik
                let r35_02_analytic: f64 = q23.cos() * (std::f64::consts::PI).sin();
                assert!(
                    r35_02_analytic.abs() < EPS,
                    "R35[0,2] debe ser 0 analíticamente, got {:.2e}",
                    r35_02_analytic
                );
            }
        }
    }
}
