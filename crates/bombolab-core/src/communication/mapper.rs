use super::command::ServoCommand;
use crate::robot::Robot;

pub struct ServoMapper<'a> {
    robot: &'a Robot,
    angle_min: f64,
    angle_max: f64,
}

impl<'a> ServoMapper<'a> {
    pub fn new(robot: &'a Robot) -> Self {
        Self {
            robot,
            angle_min: super::ANGLE_MIN as f64,
            angle_max: super::ANGLE_MAX as f64,
        }
    }

    pub fn map_q(&self, q: &[f64], gripper: u8) -> Result<ServoCommand, &'static str> {
        let servo_rad = self.robot.q_to_servo(q);
        let mut joints = [0.0_f64; 5];
        for (i, &sr) in servo_rad.iter().enumerate().take(5) {
            let deg = sr.to_degrees();
            if !deg.is_finite() {
                return Err("joint angle out of range (non-finite)");
            }
            joints[i] = deg.clamp(self.angle_min, self.angle_max);
        }
        ServoCommand::new(joints, gripper)
    }
}

#[cfg(test)]
#[path = "mapper_tests.rs"]
mod mapper_tests;
