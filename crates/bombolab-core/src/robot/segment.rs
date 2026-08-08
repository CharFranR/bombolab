use super::errors::{Error, Result};
use super::joint::{Joint, JointType};
use super::link::DHParams;

pub struct Segment {
    pub joint: Joint,
    pub dh: DHParams,
}

pub struct Robot {
    pub segments: Vec<Segment>,

    pub home_pose: Vec<f64>,

    pub servo_offsets: Vec<f64>,

    pub servo_directions: Vec<f64>,
}

impl Segment {
    pub fn new(joint: Joint, dh: DHParams) -> Self {
        Self { joint, dh }
    }

    pub fn dh_params(&self) -> (f64, f64, f64, f64) {
        match self.joint.joint_type {
            JointType::Revolute => (
                self.joint.value + self.dh.theta,
                self.dh.d,
                self.dh.a,
                self.dh.alpha,
            ),
            JointType::Prismatic => (self.dh.theta, self.joint.value, self.dh.a, self.dh.alpha),

            JointType::Twist => (0.0, self.dh.d, self.dh.a, self.joint.value + self.dh.alpha),
        }
    }
}

impl Robot {
    pub fn new(segments: Vec<Segment>) -> Self {
        let n = segments.len();
        Self {
            segments,
            home_pose: vec![0.0; n],
            servo_offsets: vec![0.0; n],
            servo_directions: vec![1.0; n],
        }
    }

    pub fn with_offsets(
        segments: Vec<Segment>,
        home_pose: Vec<f64>,
        servo_offsets: Vec<f64>,
    ) -> Self {
        let n = segments.len();
        Self {
            segments,
            home_pose,
            servo_offsets,
            servo_directions: vec![1.0; n],
        }
    }

    pub fn with_directions(
        segments: Vec<Segment>,
        home_pose: Vec<f64>,
        servo_offsets: Vec<f64>,
        servo_directions: Vec<f64>,
    ) -> Self {
        Self {
            segments,
            home_pose,
            servo_offsets,
            servo_directions,
        }
    }

    pub fn q_to_servo(&self, q: &[f64]) -> Vec<f64> {
        q.iter()
            .zip(&self.servo_offsets)
            .zip(&self.servo_directions)
            .map(|((qi, off), dir)| dir * qi + off)
            .collect()
    }

    pub fn servo_to_q(&self, servo: &[f64]) -> Vec<f64> {
        servo
            .iter()
            .zip(&self.servo_offsets)
            .zip(&self.servo_directions)
            .map(|((s, off), dir)| (s - off) * dir)
            .collect()
    }

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
#[path = "segment_tests.rs"]
mod segment_tests;
