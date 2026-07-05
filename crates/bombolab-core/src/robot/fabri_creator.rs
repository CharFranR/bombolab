use crate::math::{Iso3, Tras};
use crate::robot::{DHParams, Joint, JointType, Robot, Segment};

/// Joint limits: 10° to 170° (mechanical safety).
const JOINT_MIN: f64 = std::f64::consts::PI / 18.0; // 10°
const JOINT_MAX: f64 = std::f64::consts::PI * 17.0 / 18.0; // 170°

/// Home pose: Arduino default servo positions (degrees → radians).
/// At kinematic zero (q=[0,0,0,0,0]), servos are physically at these angles.
/// Original Arduino defaults: [90°, 115°, 110°, 175°, 90°].
/// J4 clamped from 175° → 170° to stay within JOINT_MAX.
const HOME_POSE_DEG: [f64; 5] = [90.0, 115.0, 110.0, 170.0, 90.0];

/// Servo offsets: servo_angle = q_robot + offset (radians).
/// At kinematic zero (q=[0,0,0,0,0]), servos are at HOME_POSE_DEG.
const SERVO_OFFSETS_DEG: [f64; 5] = [90.0, 115.0, 110.0, 170.0, 90.0];

/// Creates a configured FABRI Creator 5-DOF robot.
/// DH convention: Craig, units: mm.
///
/// Base frame: X → right, Y → toward viewer, Z ↑ up (planar in ZX).
///
/// | i | α      | a    | d   | θ  |
/// |---|--------|------|-----|----|
/// | 1 | -90°   | 15   | 95  | θ₁ |
/// | 2 | 0°     | 0    | 162 | θ₂ |
/// | 3 | -90°   | 111  | 0   | θ₃ |
/// | 4 | 90°    | 35   | 0   | θ₄ |
/// | 5 | 0°     | 0    | 0   | θ₅ |
///
/// Joint values are kinematic coordinates (q). The `home_pose` and
/// `servo_offsets` fields encode the mapping between q and physical servo angles.
pub fn fabri_creator() -> Robot {
    let dh_table: Vec<(f64, f64, f64, f64)> = vec![
        (-std::f64::consts::FRAC_PI_2, 15.0, 95.0, 0.0), // α, a, d, θ(initial)
        (0.0, 0.0, 162.0, 0.0),
        (-std::f64::consts::FRAC_PI_2, 111.0, 0.0, 0.0),
        (std::f64::consts::FRAC_PI_2, 35.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
    ];

    let servo_offsets: Vec<f64> = SERVO_OFFSETS_DEG.iter().map(|d| d.to_radians()).collect();

    let segments: Vec<Segment> = dh_table
        .into_iter()
        .zip(servo_offsets.iter())
        .map(|((alpha, a, d, _), &offset)| {
            // Joint starts at kinematic zero (q=0), NOT at the home pose angle.
            // Joint limits are in kinematic space: q_min = servo_min - offset.
            let q_min = JOINT_MIN - offset;
            let q_max = JOINT_MAX - offset;
            let joint = Joint::new(JointType::Revolute, 0.0, q_max, q_min);
            // DHParams::new(theta, d, a, alpha) — theta=0 at kinematic zero.
            let dh = DHParams::new(0.0, d, a, alpha);
            Segment::new(joint, dh)
        })
        .collect();

    let home_pose: Vec<f64> = HOME_POSE_DEG.iter().map(|d| d.to_radians()).collect();

    Robot::with_offsets(segments, home_pose, servo_offsets)
}

/// Base transform: vertical offset of 57mm from ground to joint 1.
pub fn base_transform() -> Iso3 {
    let translation = Tras::new(0.0, 0.0, 57.0);
    Iso3::from_parts(translation, nalgebra::UnitQuaternion::identity())
}

/// Tool transform: translation from J5 frame to marker tip.
/// The marker is perpendicular to the end effector (extends along X of J5).
/// 75mm from J5 origin to marker tip.
///
/// Apply AFTER FK: `tool_transform() * forward_kinematics(base, &robot)`
pub fn tool_transform() -> Iso3 {
    let translation = Tras::new(75.0, 0.0, 0.0); // X, not Z — marker is perpendicular
    Iso3::from_parts(translation, nalgebra::UnitQuaternion::identity())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::forward_kinematics;
    use std::f64::consts::PI;

    #[test]
    fn test_fabri_creator_returns_5_revolute_segments() {
        let robot = fabri_creator();
        assert_eq!(robot.dof(), 5);
        for seg in &robot.segments {
            assert_eq!(seg.joint.joint_type, JointType::Revolute);
            // Joint values are kinematic coordinates — q=0 at home.
            assert!(
                (seg.joint.value - 0.0).abs() < 1e-10,
                "kinematic zero should be 0.0, got {}",
                seg.joint.value
            );
        }
    }

    #[test]
    fn test_dh_table_matches_spec() {
        let robot = fabri_creator();
        assert_eq!(robot.dof(), 5);

        // Craig convention: (theta, d, a, alpha)
        // Segment 1: α=-90°, a=15, d=95
        let (theta, d, a, alpha) = robot.segment(0).unwrap().dh_params();
        assert!(
            (alpha - (-PI / 2.0)).abs() < 1e-10,
            "alpha_1: expected -π/2, got {}",
            alpha
        );
        assert!((a - 15.0).abs() < 1e-10, "a_1: expected 15, got {}", a);
        assert!((d - 95.0).abs() < 1e-10, "d_1: expected 95, got {}", d);
        assert!(
            (theta - 0.0).abs() < 1e-10,
            "theta_1 should be 0.0 at kinematic zero, got {}",
            theta
        );

        // Segment 2: α=0°, a=0, d=162
        let (_theta, d, a, alpha) = robot.segment(1).unwrap().dh_params();
        assert!(
            (alpha - 0.0).abs() < 1e-10,
            "alpha_2: expected 0, got {}",
            alpha
        );
        assert!((a - 0.0).abs() < 1e-10, "a_2: expected 0, got {}", a);
        assert!((d - 162.0).abs() < 1e-10, "d_2: expected 162, got {}", d);

        // Segment 3: α=-90°, a=111, d=0
        let (_theta, d, a, alpha) = robot.segment(2).unwrap().dh_params();
        assert!(
            (alpha - (-PI / 2.0)).abs() < 1e-10,
            "alpha_3: expected -π/2, got {}",
            alpha
        );
        assert!((a - 111.0).abs() < 1e-10, "a_3: expected 111, got {}", a);
        assert!((d - 0.0).abs() < 1e-10, "d_3: expected 0, got {}", d);

        // Segment 4: α=90°, a=35, d=0
        let (_theta, d, a, alpha) = robot.segment(3).unwrap().dh_params();
        assert!(
            (alpha - (PI / 2.0)).abs() < 1e-10,
            "alpha_4: expected π/2, got {}",
            alpha
        );
        assert!((a - 35.0).abs() < 1e-10, "a_4: expected 35, got {}", a);
        assert!((d - 0.0).abs() < 1e-10, "d_4: expected 0, got {}", d);

        // Segment 5: α=0°, a=0, d=0
        let (_theta, d, a, alpha) = robot.segment(4).unwrap().dh_params();
        assert!(
            (alpha - 0.0).abs() < 1e-10,
            "alpha_5: expected 0, got {}",
            alpha
        );
        assert!((a - 0.0).abs() < 1e-10, "a_5: expected 0, got {}", a);
        assert!((d - 0.0).abs() < 1e-10, "d_5: expected 0, got {}", d);
    }

    #[test]
    fn test_joint_limits() {
        let robot = fabri_creator();
        let expected_min = PI / 18.0;
        let expected_max = PI * 17.0 / 18.0;

        assert_eq!(robot.dof(), 5, "robot must have 5 segments to check limits");

        // Joint limits are per-joint in kinematic space: q_min = servo_min - offset.
        // Verify that setting a servo value below JOINT_MIN or above JOINT_MAX is rejected.
        for (seg, &offset) in robot.segments.iter().zip(&robot.servo_offsets) {
            let q_min = expected_min - offset;
            let q_max = expected_max - offset;
            assert!(
                (seg.joint.value_min - q_min).abs() < 1e-10,
                "joint value_min: expected {}, got {}",
                q_min,
                seg.joint.value_min
            );
            assert!(
                (seg.joint.value_max - q_max).abs() < 1e-10,
                "joint value_max: expected {}, got {}",
                q_max,
                seg.joint.value_max
            );
        }

        // set_value(0.0) should succeed for all joints (kinematic zero)
        for seg in &robot.segments {
            let mut joint = Joint::new(
                JointType::Revolute,
                0.0,
                seg.joint.value_max,
                seg.joint.value_min,
            );
            assert!(
                joint.set_value(0.0).is_ok(),
                "set_value(0.0) should succeed at kinematic zero"
            );
        }
    }

    #[test]
    fn test_home_pose_within_limits() {
        let robot = fabri_creator();
        // Home pose is in physical servo space — must be within [JOINT_MIN, JOINT_MAX].
        let servo_min = PI / 18.0;
        let servo_max = PI * 17.0 / 18.0;
        for (i, &servo_angle) in robot.home_pose.iter().enumerate() {
            assert!(
                servo_angle >= servo_min && servo_angle <= servo_max,
                "home_pose[{}] = {} is outside physical limits [{}, {}]",
                i,
                servo_angle,
                servo_min,
                servo_max
            );
        }
    }

    #[test]
    fn test_base_transform() {
        let bt = base_transform();
        let t = bt.translation.vector;
        assert!(
            (t.x - 0.0).abs() < 1e-10,
            "base_transform x: expected 0, got {}",
            t.x
        );
        assert!(
            (t.y - 0.0).abs() < 1e-10,
            "base_transform y: expected 0, got {}",
            t.y
        );
        assert!(
            (t.z - 57.0).abs() < 1e-10,
            "base_transform z: expected 57.0, got {}",
            t.z
        );
    }

    #[test]
    fn test_tool_transform() {
        let tt = tool_transform();
        let t = tt.translation.vector;
        // Tool is 75mm along X from J5 (marker is perpendicular to end effector)
        assert!(
            (t.x - 75.0).abs() < 1e-10,
            "tool_transform x: expected 75.0, got {}",
            t.x
        );
        assert!(
            (t.y - 0.0).abs() < 1e-10,
            "tool_transform y: expected 0, got {}",
            t.y
        );
        assert!(
            (t.z - 0.0).abs() < 1e-10,
            "tool_transform z: expected 0, got {}",
            t.z
        );
    }

    #[test]
    fn test_tool_transform_rotation_is_identity() {
        let tt = tool_transform();
        // No rotation — marker extends along X with identity rotation
        let angle = tt.rotation.angle();
        assert!(
            angle.abs() < 1e-10,
            "tool transform rotation should be identity, got angle {}",
            angle
        );
    }

    #[test]
    fn test_q_to_servo_and_back() {
        let robot = fabri_creator();
        // At kinematic zero, servo angles should equal home pose
        let q_zero = vec![0.0; 5];
        let servo = robot.q_to_servo(&q_zero);
        for (i, (s, h)) in servo.iter().zip(&robot.home_pose).enumerate() {
            assert!(
                (*s - h).abs() < 1e-10,
                "q_to_servo(q=0)[{}]: expected {}, got {}",
                i,
                h,
                s
            );
        }
        // Round-trip: servo_to_q(q_to_servo(q)) == q
        let q_test = vec![0.1, -0.2, 0.3, -0.1, 0.15];
        let servo_test = robot.q_to_servo(&q_test);
        let q_roundtrip = robot.servo_to_q(&servo_test);
        for (i, (orig, rt)) in q_test.iter().zip(&q_roundtrip).enumerate() {
            assert!(
                (*orig - rt).abs() < 1e-10,
                "round-trip failed for q[{}]: started at {}, got back {}",
                i,
                orig,
                rt
            );
        }
    }

    #[test]
    fn test_kinematic_home_is_zero() {
        let robot = fabri_creator();
        let khome = robot.kinematic_home();
        for (i, &q) in khome.iter().enumerate() {
            assert!(
                q.abs() < 1e-10,
                "kinematic_home[{}] should be 0.0, got {}",
                i,
                q
            );
        }
    }

    #[test]
    fn test_fk_with_base_transform() {
        let robot = fabri_creator();
        let base = base_transform();
        let (frames, _effector) = forward_kinematics(base, &robot);

        // First frame z-translation should include 57mm base offset
        let first_frame_z = frames[0].translation.vector.z;
        assert!(
            first_frame_z > 57.0,
            "first frame z should include 57mm base offset, got {}",
            first_frame_z
        );
    }

    // --- Triangulation tests ---

    #[test]
    fn test_joint_limits_boundary_values() {
        let expected_min = PI / 18.0;
        let expected_max = PI * 17.0 / 18.0;

        // Exactly at minimum should be accepted
        let mut joint_min = Joint::new(JointType::Revolute, 0.0, expected_max, expected_min);
        assert!(
            joint_min.set_value(expected_min).is_ok(),
            "set_value at exact minimum {} should succeed",
            expected_min
        );

        // Exactly at maximum should be accepted
        let mut joint_max = Joint::new(JointType::Revolute, 0.0, expected_max, expected_min);
        assert!(
            joint_max.set_value(expected_max).is_ok(),
            "set_value at exact maximum {} should succeed",
            expected_max
        );

        // Just below minimum should fail
        let mut joint_below = Joint::new(JointType::Revolute, 0.0, expected_max, expected_min);
        assert!(
            joint_below.set_value(expected_min - 0.001).is_err(),
            "set_value just below minimum should fail"
        );

        // Just above maximum should fail
        let mut joint_above = Joint::new(JointType::Revolute, 0.0, expected_max, expected_min);
        assert!(
            joint_above.set_value(expected_max + 0.001).is_err(),
            "set_value just above maximum should fail"
        );
    }

    #[test]
    fn test_dh_params_theta_matches_joint_value() {
        let robot = fabri_creator();
        // Theta should always equal the joint value
        for seg in &robot.segments {
            let (theta, _d, _a, _alpha) = seg.dh_params();
            assert!(
                (theta - seg.joint.value).abs() < 1e-10,
                "theta {} should equal joint value {}",
                theta,
                seg.joint.value
            );
        }
    }

    #[test]
    fn test_base_transform_rotation_is_identity() {
        let bt = base_transform();
        let angle = bt.rotation.angle();
        assert!(
            angle.abs() < 1e-10,
            "base transform rotation should be identity, got angle {}",
            angle
        );
    }
}
