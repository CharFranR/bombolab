use crate::domain::base::Base;
use crate::domain::errors::{Error, Result};
use crate::domain::joint::Joint;
use crate::domain::link::DHParams;

pub struct Segment {
    pub joint: Joint,
    pub dh: DHParams,
}

pub struct Robot {
    pub base: Base,
    pub segments: Vec<Segment>,
}

impl Segment {
    pub fn new(joint: Joint, dh: DHParams) -> Self {
        Self { joint, dh }
    }
}

impl Robot {
    pub fn new(base: Base, segments: Vec<Segment>) -> Self {
        Self { base, segments }
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
        self.segments.get_mut(index).ok_or(Error::IndexOutOfBounds {
            index,
            len,
        })
    }

    pub fn set_joint_values(&mut self, new_joints: Vec<Joint>) -> Result<()> {
        if new_joints.len() != self.segments.len() {
            return Err(Error::JointCountMismatch {
                expected: self.segments.len(),
                got: new_joints.len(),
            });
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
