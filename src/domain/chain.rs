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