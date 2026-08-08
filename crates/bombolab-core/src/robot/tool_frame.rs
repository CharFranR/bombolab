use crate::math::Iso3;

pub const DEFAULT_MARKER_LENGTH: f64 = 75.0;

pub const PEN_LENGTH: f64 = 90.0;

pub const GRIPPER_LENGTH: f64 = 45.0;

pub struct ToolFrame {
    pose: Iso3,
    name: String,
}

impl ToolFrame {
    pub fn new(pose: Iso3, name: String) -> Self {
        Self { pose, name }
    }

    pub fn pose(&self) -> &Iso3 {
        &self.pose
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn marker_perpendicular() -> Self {
        Self::marker_perpendicular_len(DEFAULT_MARKER_LENGTH)
    }

    pub fn marker_perpendicular_len(len: f64) -> Self {
        Self::new(
            Iso3::translation(len, 0.0, 0.0),
            "marker_perpendicular".to_string(),
        )
    }

    pub fn pen() -> Self {
        Self::new(Iso3::translation(PEN_LENGTH, 0.0, 0.0), "pen".to_string())
    }

    pub fn gripper() -> Self {
        Self::new(
            Iso3::translation(GRIPPER_LENGTH, 0.0, 0.0),
            "gripper".to_string(),
        )
    }
}

#[cfg(test)]
#[path = "tool_frame_tests.rs"]
mod tool_frame_tests;
