use nalgebra::{Isometry3, Rotation3, Translation3, UnitQuaternion, Vector3};

use crate::robot::{Robot, Segment};

pub fn matrix_from_segment(segment: &Segment) -> Isometry3<f64> {
    let (theta, d, a, alpha) = segment.dh_params();

    let rot_z = Rotation3::from_axis_angle(&Vector3::z_axis(), theta);
    let rot_x = Rotation3::from_axis_angle(&Vector3::x_axis(), alpha);
    let rotation = UnitQuaternion::from_rotation_matrix(&(rot_z * rot_x));
    let translation = Translation3::new(a * theta.cos(), a * theta.sin(), d);

    Isometry3::from_parts(translation, rotation)
}

pub fn forward_kinematics(
    base: Isometry3<f64>,
    robot: &Robot,
) -> (Vec<Isometry3<f64>>, Isometry3<f64>) {
    let mut frames = Vec::new();
    let mut current = base;

    for segment in &robot.segments {
        current = current * matrix_from_segment(segment);
        frames.push(current);
    }

    (frames, current)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::{DHParams, Joint, JointType, Robot, Segment};

    fn make_segment(joint_type: JointType, value: f64, dh: DHParams) -> Segment {
        let joint = Joint::new(
            joint_type,
            value,
            std::f64::consts::PI,
            -std::f64::consts::PI,
        );
        Segment::new(joint, dh)
    }

    #[test]
    fn test_matrix_from_segment_identity() {
        let dh = DHParams::new(0.0, 0.0, 0.0, 0.0);
        let seg = make_segment(JointType::Revolute, 0.0, dh);
        let m = matrix_from_segment(&seg);

        // Should be identity transformation
        let t = m.translation.vector;
        assert!((t.x - 0.0).abs() < 1e-10);
        assert!((t.y - 0.0).abs() < 1e-10);
        assert!((t.z - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_matrix_from_segment_translation() {
        // theta=0, d=5, a=3, alpha=0
        // Translation should be (a*cos(0), a*sin(0), d) = (3, 0, 5)
        let dh = DHParams::new(0.0, 5.0, 3.0, 0.0);
        let seg = make_segment(JointType::Revolute, 0.0, dh);
        let m = matrix_from_segment(&seg);

        let t = m.translation.vector;
        assert!((t.x - 3.0).abs() < 1e-10);
        assert!((t.y - 0.0).abs() < 1e-10);
        assert!((t.z - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_matrix_from_segment_rotation() {
        // theta=PI/2, d=0, a=1, alpha=0
        // Translation should be (a*cos(PI/2), a*sin(PI/2), 0) = (0, 1, 0)
        let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
        let seg = make_segment(JointType::Revolute, std::f64::consts::FRAC_PI_2, dh);
        let m = matrix_from_segment(&seg);

        let t = m.translation.vector;
        assert!((t.x - 0.0).abs() < 1e-10);
        assert!((t.y - 1.0).abs() < 1e-10);
        assert!((t.z - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_forward_kinematics_single_segment() {
        let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
        let segments = vec![make_segment(JointType::Revolute, 0.0, dh)];
        let robot = Robot::new(segments);

        let base = Isometry3::identity();
        let (frames, effector) = forward_kinematics(base, &robot);

        assert_eq!(frames.len(), 1);

        let t = effector.translation.vector;
        assert!((t.x - 1.0).abs() < 1e-10);
        assert!((t.y - 0.0).abs() < 1e-10);
        assert!((t.z - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_forward_kinematics_two_segments() {
        // Two segments: both with a=1, theta=0
        // First segment moves to (1, 0, 0)
        // Second segment moves another (1, 0, 0) relative to first
        let dh1 = DHParams::new(0.0, 0.0, 1.0, 0.0);
        let dh2 = DHParams::new(0.0, 0.0, 1.0, 0.0);
        let segments = vec![
            make_segment(JointType::Revolute, 0.0, dh1),
            make_segment(JointType::Revolute, 0.0, dh2),
        ];
        let robot = Robot::new(segments);

        let base = Isometry3::identity();
        let (frames, effector) = forward_kinematics(base, &robot);

        assert_eq!(frames.len(), 2);

        // Final effector should be at (2, 0, 0)
        let t = effector.translation.vector;
        assert!((t.x - 2.0).abs() < 1e-10);
        assert!((t.y - 0.0).abs() < 1e-10);
        assert!((t.z - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_forward_kinematics_with_joint_angle() {
        // Single segment with theta=90 degrees (PI/2)
        let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
        let segments = vec![make_segment(
            JointType::Revolute,
            std::f64::consts::FRAC_PI_2,
            dh,
        )];
        let robot = Robot::new(segments);

        let base = Isometry3::identity();
        let (_frames, effector) = forward_kinematics(base, &robot);

        // Should be at (0, 1, 0)
        let t = effector.translation.vector;
        assert!((t.x - 0.0).abs() < 1e-10);
        assert!((t.y - 1.0).abs() < 1e-10);
        assert!((t.z - 0.0).abs() < 1e-10);
    }
}
