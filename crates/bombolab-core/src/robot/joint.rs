use std::fmt;

use super::errors::{Error, Result};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,
    Prismatic,
    Twist,
}

impl fmt::Display for JointType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JointType::Revolute => write!(f, "R"),
            JointType::Prismatic => write!(f, "P"),
            JointType::Twist => write!(f, "T"),
        }
    }
}

pub struct Joint {
    pub joint_type: JointType,
    pub value: f64,
    pub value_max: f64,
    pub value_min: f64,
}

impl Joint {
    pub fn new(joint_type: JointType, value: f64, value_max: f64, value_min: f64) -> Self {
        Self {
            joint_type,
            value,
            value_max,
            value_min,
        }
    }

    pub fn range(&self) -> Vec<f64> {
        vec![self.value_min, self.value_max]
    }

    pub fn is_within_limits(&self) -> bool {
        self.value <= self.value_max && self.value >= self.value_min
    }

    pub fn clamp(&mut self) {
        if self.value > self.value_max {
            self.value = self.value_max
        }

        if self.value < self.value_min {
            self.value = self.value_min
        }
    }

    pub fn set_value(&mut self, value: f64) -> Result<()> {
        if value > self.value_max || value < self.value_min {
            return Err(Error::JointValueOutOfLimits {
                value,
                min: self.value_min,
                max: self.value_max,
            });
        }
        self.value = value;
        Ok(())
    }
}

#[cfg(test)]
#[path = "joint_tests.rs"]
mod joint_tests;
