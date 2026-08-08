use super::{ANGLE_MAX, ANGLE_MIN};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoCommand {
    pub joints: [f64; 5],

    pub gripper: u8,
}

impl ServoCommand {
    pub fn new(joints: [f64; 5], gripper: u8) -> Result<Self, &'static str> {
        let min = ANGLE_MIN as f64;
        let max = ANGLE_MAX as f64;

        for &j in &joints {
            if !j.is_finite() || j < min || j > max {
                return Err("joint angle out of range (non-finite)");
            }
        }

        if gripper < ANGLE_MIN as u8 || gripper > ANGLE_MAX as u8 {
            return Err("gripper angle out of range");
        }

        Ok(Self { joints, gripper })
    }

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
#[path = "command_tests.rs"]
mod command_tests;
