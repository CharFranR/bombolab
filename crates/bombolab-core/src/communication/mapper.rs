//! ServoMapper — centralized q→servo mapping with clamping.
//!
//! Converts kinematic coordinates (radians) to servo angles (degrees)
//! by delegating to `Robot::q_to_servo()`, then clamping to [10°, 170°]
//! and producing a `ServoCommand`.
//!
//! # Future direction inversion
//!
//! The design supports per-joint sign inversion (`servo = -q + offset`)
//! but no joint currently requires it. Add a `signs: [f64; 5]` field
//! to `ServoMapper` if needed.

use super::command::ServoCommand;
use crate::robot::Robot;

/// Maps kinematic joint angles (radians) to servo angles (degrees)
/// with clamping and gripper passthrough.
pub struct ServoMapper<'a> {
    robot: &'a Robot,
    angle_min: f64,
    angle_max: f64,
}

impl<'a> ServoMapper<'a> {
    /// Create a new mapper with default clamping [10°, 170°].
    pub fn new(robot: &'a Robot) -> Self {
        Self {
            robot,
            angle_min: super::ANGLE_MIN as f64,
            angle_max: super::ANGLE_MAX as f64,
        }
    }

    /// Map kinematic q (radians) to a `ServoCommand`.
    ///
    /// Delegates to `Robot::q_to_servo()` for rad→rad conversion,
    /// then converts to degrees and clamps to [`angle_min`, `angle_max`].
    pub fn map_q(&self, q: &[f64], gripper: u8) -> ServoCommand {
        let servo_rad = self.robot.q_to_servo(q);
        let mut joints = [0.0_f64; 5];
        for (i, &sr) in servo_rad.iter().enumerate().take(5) {
            let deg = sr.to_degrees();
            joints[i] = deg.clamp(self.angle_min, self.angle_max);
        }
        ServoCommand { joints, gripper }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::fabri_creator::fabri_creator;

    fn make_robot() -> Robot {
        fabri_creator()
    }

    #[test]
    fn test_home_pose_maps_correctly() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q = [0,0,0,0,0] → servo = offsets
        let cmd = mapper.map_q(&[0.0; 5], 90);
        // Offsets in degrees: [90, 115, 110, 170, 90]
        assert!((cmd.joints[0] - 90.0).abs() < 1e-6);
        assert!((cmd.joints[1] - 115.0).abs() < 1e-6);
        assert!((cmd.joints[2] - 110.0).abs() < 1e-6);
        assert!((cmd.joints[3] - 170.0).abs() < 1e-6);
        assert!((cmd.joints[4] - 90.0).abs() < 1e-6);
        assert_eq!(cmd.gripper, 90);
    }

    #[test]
    fn test_clamp_below_min() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q value that maps to a very small servo angle (< 10°)
        // Offset J0 = 90° → q = -1.4 rad → servo ≈ -80.2° → clamped to 10°
        let cmd = mapper.map_q(&[-1.4, 0.0, 0.0, 0.0, 0.0], 90);
        assert!((cmd.joints[0] - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_clamp_above_max() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q value that maps to a very large servo angle (> 170°)
        // Offset J0 = 90° → q = 1.5 rad → servo ≈ 175.9° → clamped to 170°
        let cmd = mapper.map_q(&[1.5, 0.0, 0.0, 0.0, 0.0], 90);
        assert!((cmd.joints[0] - 170.0).abs() < 1e-6);
    }

    #[test]
    fn test_gripper_passthrough() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        let cmd = mapper.map_q(&[0.0; 5], 127);
        assert_eq!(cmd.gripper, 127);
    }

    #[test]
    fn test_non_home_q_maps_with_offset() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q = [0.1, -0.2, 0.0, 0.15, -0.1] rad
        // servo_deg = (q + offset) * 180/π → clamped to [10, 170]
        let cmd = mapper.map_q(&[0.1, -0.2, 0.0, 0.15, -0.1], 45);
        // All offsets are in [10, 170], and q perturbations stay within range
        // J0: (0.1 + 90°=1.571) rad * 180/π ≈ 95.73°
        // J1: (-0.2 + 115°=2.007) rad * 180/π ≈ 103.5°
        // J2: (0.0 + 110°=1.919) rad * 180/π ≈ 110.0°
        // J3: (0.15 + 170°=2.967) rad * 180/π ≈ 178.0° → clamped to 170°
        // J4: (-0.1 + 90°=1.571) rad * 180/π ≈ 84.27°
        assert!((cmd.joints[0] - 95.73).abs() < 0.1);
        assert!((cmd.joints[1] - 103.53).abs() < 0.1);
        assert!((cmd.joints[2] - 110.0).abs() < 0.1);
        assert!((cmd.joints[3] - 170.0).abs() < 0.1); // clamped
        assert!((cmd.joints[4] - 84.27).abs() < 0.1);
    }

    #[test]
    fn test_output_is_servo_command() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        let cmd = mapper.map_q(&[0.1, -0.2, 0.3, -0.1, 0.15], 45);
        // to_wire should produce valid output
        let wire = cmd.to_wire();
        assert!(wire.ends_with('\n'));
        assert_eq!(wire.split(',').count(), 6);
    }
}
