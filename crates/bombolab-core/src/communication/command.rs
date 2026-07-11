//! Typed replacement for raw `[i32; 6]` arrays.
//!
//! Encapsulates 5 joint angles (degrees) and 1 gripper angle (0–180)
//! with range validation and wire serialization.

use super::{ANGLE_MAX, ANGLE_MIN};

/// Servo command — 5 joint angles + gripper.
///
/// Joints are in **degrees**, validated to [10°, 170°].
/// Gripper is in 0–180 range (standard servo range).
///
/// # Wire format
///
/// `to_wire()` produces `"a1,a2,a3,a4,a5,g\n"` — the same format
/// the Arduino firmware expects.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoCommand {
    /// 5 joint angles in degrees, indexed [J1..J5].
    pub joints: [f64; 5],
    /// Gripper angle (0–180, standard servo range).
    pub gripper: u8,
}

impl ServoCommand {
    /// Create a new `ServoCommand`, validating joint and gripper ranges.
    ///
    /// # Errors
    ///
    /// Returns an error string if any joint is outside [`ANGLE_MIN`, `ANGLE_MAX`]
    /// or if gripper > 180.
    pub fn new(joints: [f64; 5], gripper: u8) -> Result<Self, &'static str> {
        let min = ANGLE_MIN as f64;
        let max = ANGLE_MAX as f64;

        for &j in &joints {
            if j < min || j > max {
                return Err("joint angle out of range");
            }
        }

        if gripper > 180 {
            return Err("gripper angle out of range");
        }

        Ok(Self { joints, gripper })
    }

    /// Serialize to wire format: `"a1,a2,a3,a4,a5,g\n"`.
    ///
    /// Joint values are rounded to the nearest integer for compatibility
    /// with the existing firmware protocol.
    pub fn to_wire(&self) -> String {
        format!(
            "{},{},{},{},{},{}\n",
            self.joints[0].round() as i32,
            self.joints[1].round() as i32,
            self.joints[2].round() as i32,
            self.joints[3].round() as i32,
            self.joints[4].round() as i32,
            self.gripper,
        )
    }

    /// Convert to raw `[i32; 6]` array for use with `interpolate_all`.
    ///
    /// The first 5 elements are joint angles (rounded), the 6th is the gripper.
    pub fn to_raw_array(&self) -> [i32; 6] {
        [
            self.joints[0].round() as i32,
            self.joints[1].round() as i32,
            self.joints[2].round() as i32,
            self.joints[3].round() as i32,
            self.joints[4].round() as i32,
            self.gripper as i32,
        ]
    }

    /// Reconstruct from a raw `[i32; 6]` array.
    ///
    /// The first 5 elements become joints (as f64), the 6th becomes gripper.
    /// The resulting `ServoCommand` is *not* validated — this is a raw
    /// deserialization path for interpolation results that are already
    /// within bounds.
    pub fn from_raw_array(arr: &[i32; 6]) -> Self {
        Self {
            joints: [
                arr[0] as f64,
                arr[1] as f64,
                arr[2] as f64,
                arr[3] as f64,
                arr[4] as f64,
            ],
            gripper: arr[5] as u8,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─── Construction: valid ─────────────────────────────────────

    #[test]
    fn test_valid_construction_mid_range() {
        let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90).unwrap();
        assert_eq!(cmd.joints[0], 90.0);
        assert_eq!(cmd.joints[1], 115.0);
        assert_eq!(cmd.joints[2], 110.0);
        assert_eq!(cmd.joints[3], 170.0);
        assert_eq!(cmd.joints[4], 90.0);
        assert_eq!(cmd.gripper, 90);
    }

    #[test]
    fn test_valid_construction_at_boundaries() {
        let cmd = ServoCommand::new([10.0, 170.0, 10.0, 170.0, 90.0], 0).unwrap();
        assert_eq!(cmd.joints[0], 10.0);
        assert_eq!(cmd.joints[1], 170.0);
        assert_eq!(cmd.gripper, 0);
    }

    // ─── Construction: joint validation ──────────────────────────

    #[test]
    fn test_joint_below_min_rejected() {
        let result = ServoCommand::new([9.9, 115.0, 110.0, 170.0, 90.0], 90);
        assert!(result.is_err());
    }

    #[test]
    fn test_joint_above_max_rejected() {
        let result = ServoCommand::new([90.0, 115.0, 110.0, 170.1, 90.0], 90);
        assert!(result.is_err());
    }

    // ─── Construction: gripper validation ────────────────────────

    #[test]
    fn test_gripper_at_max_accepted() {
        let cmd = ServoCommand::new([90.0; 5], 180).unwrap();
        assert_eq!(cmd.gripper, 180);
    }

    #[test]
    fn test_gripper_overflow_rejected() {
        let result = ServoCommand::new([90.0; 5], 181);
        assert!(result.is_err());
    }

    // ─── Wire serialization ──────────────────────────────────────

    #[test]
    fn test_to_wire_matches_protocol() {
        let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90).unwrap();
        assert_eq!(cmd.to_wire(), "90,115,110,170,90,90\n");
    }

    #[test]
    fn test_to_wire_rounds_floats() {
        let cmd = ServoCommand::new([90.4, 115.6, 110.0, 170.0, 89.5], 45).unwrap();
        assert_eq!(cmd.to_wire(), "90,116,110,170,90,45\n");
    }

    // ─── Raw array conversion ────────────────────────────────────

    #[test]
    fn test_to_raw_array_format() {
        let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90).unwrap();
        let raw = cmd.to_raw_array();
        assert_eq!(raw, [90, 115, 110, 170, 90, 90]);
    }

    #[test]
    fn test_from_raw_array_roundtrip() {
        let raw = [90, 115, 110, 170, 90, 90];
        let cmd = ServoCommand::from_raw_array(&raw);
        assert_eq!(cmd.joints[0], 90.0);
        assert_eq!(cmd.joints[1], 115.0);
        assert_eq!(cmd.joints[2], 110.0);
        assert_eq!(cmd.joints[3], 170.0);
        assert_eq!(cmd.joints[4], 90.0);
        assert_eq!(cmd.gripper, 90);
    }

    #[test]
    fn test_raw_array_roundtrip_identity() {
        let cmd = ServoCommand::new([45.0, 67.0, 89.0, 120.0, 33.0], 127).unwrap();
        let raw = cmd.to_raw_array();
        let back = ServoCommand::from_raw_array(&raw);
        assert_eq!(cmd, back);
    }

    // ─── Edge cases ──────────────────────────────────────────────

    #[test]
    fn test_gripper_zero_accepted() {
        let cmd = ServoCommand::new([90.0; 5], 0).unwrap();
        assert_eq!(cmd.gripper, 0);
    }

    #[test]
    fn test_gripper_255_rejected() {
        // u8 can hold 255, but ServoCommand only accepts up to 180
        let result = ServoCommand::new([90.0; 5], 255);
        assert!(result.is_err());
    }
}
