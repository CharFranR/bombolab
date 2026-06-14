use crate::domain::base::Base;
use crate::domain::joint::Joint;
use crate::domain::link::Link;

pub struct Segment {
    pub joint: Joint,
    pub link: Link
}
pub struct Robot {
    pub base: Base,
    pub segments: Vec<Segment>
}

impl Segment {
    pub fn new (joint: Joint, link: Link) -> Self {
        Self {
            joint, link
        }
    }
}

impl Robot {
    pub fn new (base: Base, segments: Vec<Segment>) -> Self {
        Self { base, segments }
    }

    pub fn dof (&self) -> usize  {
        self.segments.len()
    }

    pub fn segment (&self, index:usize) -> Option<&Segment> {
        self.segments.get(index) 
    }

    pub fn segment_mut (&mut self, index:usize) -> Option<&mut Segment> {
        self.segments.get_mut(index) 
    }

    pub fn set_joint_values (&mut self, new_joints: Vec<Joint>) {

        for (segment, joint) in self.segments.iter_mut().zip(new_joints) {
            segment.joint = joint;
        }
    }

    pub fn reset_to_zero (&mut self) {
        for segment in self.segments.iter_mut(){
            segment.joint.value= 0.0;
        }
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    pub fn add_segment(&mut self, segment: Segment) {
        self.segments.push(segment);
    }

    pub fn remove_segment(&mut self, index: usize) -> Option<Segment> {
        if index < self.segments.len() {
            Some(self.segments.remove(index))
        } else {
            None
        }
    }

}