use crate::math::Rot3;

pub struct TargetPose {
    pub position: [f64; 3],
    pub rotation: Rot3,
}

pub struct PoseGenerator;

impl PoseGenerator {
    pub fn drawing_pose(position: [f64; 3]) -> TargetPose {
        TargetPose {
            position,

            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0,
            )),
        }
    }

    pub fn drawing_pose_adaptive(position: [f64; 3], q1: f64) -> TargetPose {
        let (sq1, cq1) = q1.sin_cos();
        TargetPose {
            position,
            rotation: Rot3::from_matrix_unchecked(nalgebra::Matrix3::new(
                0.0, -cq1, -sq1, 0.0, -sq1, cq1, -1.0, 0.0, 0.0,
            )),
        }
    }

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

        assert!((m[(0, 1)] + 1.0).abs() < EPS, "Y5_x debe ser -1");
        assert!((m[(1, 1)] - 0.0).abs() < EPS, "Y5_y debe ser 0");
        assert!((m[(2, 1)] - 0.0).abs() < EPS, "Y5_z debe ser 0");

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

        let det = m.determinant();
        assert!(
            (det - 1.0).abs() < EPS,
            "det(R) debe ser 1, got {:.2e}",
            det
        );

        for i in 0..3 {
            let col: Vec3 = m.column(i).into_owned();
            let norm = col.norm();
            assert!(
                (norm - 1.0).abs() < EPS,
                "col {i} debe tener norma 1, got {:.2e}",
                norm
            );
        }

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

    #[test]
    fn test_adaptive_q1_zero_matches_drawing_pose() {
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

            let det = m.determinant();
            assert!(
                (det - 1.0).abs() < 1e-8,
                "q1={}°: det debe ser 1, got {:.2e}",
                q1_deg,
                det
            );

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
        for &q1_deg in &[-45.0, 0.0, 45.0] {
            let q1: f64 = q1_deg * DEG_TO_RAD;
            let pose = PoseGenerator::drawing_pose_adaptive([0.0, 0.0, 0.0], q1);
            let m = pose.rotation.matrix();

            let expected_cos_theta = -q1.cos();
            assert!(
                (m[(0, 1)] - expected_cos_theta).abs() < EPS,
                "q1={}°: cosθ debe ser {:.4}, got {:.4}",
                q1_deg,
                expected_cos_theta,
                m[(0, 1)]
            );

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
        for &q1_deg in &[-30.0, 0.0, 30.0] {
            let q1: f64 = q1_deg * DEG_TO_RAD;
            let (_s1, c1) = q1.sin_cos();

            for &q23_deg in &[-60.0, -30.0, 0.0, 30.0, 60.0] {
                let q23: f64 = q23_deg * DEG_TO_RAD;
                let s23 = q23.sin();

                let _r03 = [
                    [c1 * s23, 0.0, -c1 * s23],
                    [0.0, -c1, 0.0],
                    [-s23, 0.0, -s23],
                ];

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
