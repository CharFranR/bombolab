use super::errors::{Error, Result};
use super::joint::{Joint, JointType};
use super::link::DHParams;

pub struct Segment {
    pub joint: Joint,
    pub dh: DHParams,
}

pub struct Robot {
    pub segments: Vec<Segment>,
    /// Kinematic home pose in radians — the "zero" for all FK/IK/Jacobians.
    pub home_pose: Vec<f64>,
    /// Servo offsets: servo_angle = q + offset (radians).
    pub servo_offsets: Vec<f64>,
}

impl Segment {
    pub fn new(joint: Joint, dh: DHParams) -> Self {
        Self { joint, dh }
    }

    pub fn dh_params(&self) -> (f64, f64, f64, f64) {
        match self.joint.joint_type {
            JointType::Revolute => (self.joint.value, self.dh.d, self.dh.a, self.dh.alpha),
            JointType::Prismatic => (self.dh.theta, self.joint.value, self.dh.a, self.dh.alpha),
        }
    }
}

impl Robot {
    /// Create a robot with default (zero) home pose and servo offsets.
    /// All joint values in segments are treated as kinematic coordinates (q).
    pub fn new(segments: Vec<Segment>) -> Self {
        let n = segments.len();
        Self {
            segments,
            home_pose: vec![0.0; n],
            servo_offsets: vec![0.0; n],
        }
    }

    /// Create a robot with explicit home pose and servo offsets.
    ///
    /// - `home_pose`: the servo angles (radians) at kinematic zero (q=[0,0,...,0]).
    /// - `servo_offsets`: servo_angle = q + offset (radians).
    pub fn with_offsets(
        segments: Vec<Segment>,
        home_pose: Vec<f64>,
        servo_offsets: Vec<f64>,
    ) -> Self {
        Self {
            segments,
            home_pose,
            servo_offsets,
        }
    }

    /// Convert kinematic coordinates (q) to servo angles.
    pub fn q_to_servo(&self, q: &[f64]) -> Vec<f64> {
        q.iter()
            .zip(&self.servo_offsets)
            .map(|(qi, off)| qi + off)
            .collect()
    }

    /// Convert servo angles to kinematic coordinates (q).
    pub fn servo_to_q(&self, servo: &[f64]) -> Vec<f64> {
        servo
            .iter()
            .zip(&self.servo_offsets)
            .map(|(s, off)| s - off)
            .collect()
    }

    /// Get the home pose expressed in kinematic coordinates (should be all zeros).
    pub fn kinematic_home(&self) -> Vec<f64> {
        self.servo_to_q(&self.home_pose)
    }

    pub fn dof(&self) -> usize {
        self.segments.len()
    }

    pub fn segment(&self, index: usize) -> Result<&Segment> {
        self.segments.get(index).ok_or(Error::IndexOutOfBounds {
            index,
            len: self.segments.len(),
        })
    }

    pub fn segment_mut(&mut self, index: usize) -> Result<&mut Segment> {
        let len = self.segments.len();
        self.segments
            .get_mut(index)
            .ok_or(Error::IndexOutOfBounds { index, len })
    }

    pub fn set_joint_values(&mut self, new_joints: Vec<Joint>) -> Result<()> {
        if new_joints.len() != self.segments.len() {
            return Err(Error::JointCountMismatch {
                expected: self.segments.len(),
                got: new_joints.len(),
            });
        }

        for joint in &new_joints {
            if joint.value > joint.value_max || joint.value < joint.value_min {
                return Err(Error::JointValueOutOfLimits {
                    value: joint.value,
                    min: joint.value_min,
                    max: joint.value_max,
                });
            }
        }

        for (segment, joint) in self.segments.iter_mut().zip(new_joints) {
            segment.joint = joint;
        }
        Ok(())
    }

    pub fn reset_to_zero(&mut self) {
        for segment in self.segments.iter_mut() {
            segment.joint.value = 0.0;
        }
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    pub fn add_segment(&mut self, segment: Segment) {
        self.segments.push(segment);
    }

    pub fn remove_segment(&mut self, index: usize) -> Result<Segment> {
        if index < self.segments.len() {
            Ok(self.segments.remove(index))
        } else {
            Err(Error::IndexOutOfBounds {
                index,
                len: self.segments.len(),
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_segment(joint_type: JointType, value: f64) -> Segment {
        let joint = Joint::new(joint_type, value, 1.0, -1.0);
        let dh = DHParams::new(0.0, 0.0, 1.0, 0.0);
        Segment::new(joint, dh)
    }

    #[test]
    fn test_segment_new() {
        let seg = make_test_segment(JointType::Revolute, 0.5);
        assert_eq!(seg.joint.joint_type, JointType::Revolute);
        assert_eq!(seg.joint.value, 0.5);
        assert_eq!(seg.dh.a, 1.0);
    }

    #[test]
    fn test_segment_dh_params_revolute() {
        let seg = make_test_segment(JointType::Revolute, 0.5);
        let (theta, d, a, alpha) = seg.dh_params();
        assert_eq!(theta, 0.5);
        assert_eq!(d, 0.0);
        assert_eq!(a, 1.0);
        assert_eq!(alpha, 0.0);
    }

    #[test]
    fn test_segment_dh_params_prismatic() {
        let seg = make_test_segment(JointType::Prismatic, 0.5);
        let (theta, d, a, alpha) = seg.dh_params();
        assert_eq!(theta, 0.0);
        assert_eq!(d, 0.5);
        assert_eq!(a, 1.0);
        assert_eq!(alpha, 0.0);
    }

    #[test]
    fn test_robot_new() {
        let segments = vec![make_test_segment(JointType::Revolute, 0.0)];
        let robot = Robot::new(segments);
        assert_eq!(robot.dof(), 1);
        assert!(!robot.is_empty());
    }

    #[test]
    fn test_robot_empty() {
        let robot = Robot::new(vec![]);
        assert_eq!(robot.dof(), 0);
        assert!(robot.is_empty());
    }

    #[test]
    fn test_robot_segment() {
        let segments = vec![
            make_test_segment(JointType::Revolute, 0.0),
            make_test_segment(JointType::Revolute, 0.5),
        ];
        let robot = Robot::new(segments);

        assert!(robot.segment(0).is_ok());
        assert!(robot.segment(1).is_ok());
        assert!(robot.segment(2).is_err());
    }

    #[test]
    fn test_robot_add_remove_segment() {
        let mut robot = Robot::new(vec![]);
        assert!(robot.is_empty());

        robot.add_segment(make_test_segment(JointType::Revolute, 0.0));
        assert_eq!(robot.dof(), 1);

        let removed = robot.remove_segment(0);
        assert!(removed.is_ok());
        assert!(robot.is_empty());
    }

    #[test]
    fn test_robot_remove_segment_out_of_bounds() {
        let mut robot = Robot::new(vec![]);
        assert!(robot.remove_segment(0).is_err());
    }

    #[test]
    fn test_robot_reset_to_zero() {
        let segments = vec![make_test_segment(JointType::Revolute, 0.5)];
        let mut robot = Robot::new(segments);
        robot.reset_to_zero();
        assert_eq!(robot.segments[0].joint.value, 0.0);
    }
}
