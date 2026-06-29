use std::fmt;

use crate::domain::errors::{Error, Result};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,
    Prismatic,
}

impl fmt::Display for JointType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JointType::Revolute => write!(f, "R"),
            JointType::Prismatic => write!(f, "P"),
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
mod tests {
    use super::*;

    #[test]
    fn test_joint_new() {
        let joint = Joint::new(JointType::Revolute, 0.5, 1.0, -1.0);
        assert_eq!(joint.joint_type, JointType::Revolute);
        assert_eq!(joint.value, 0.5);
        assert_eq!(joint.value_max, 1.0);
        assert_eq!(joint.value_min, -1.0);
    }

    #[test]
    fn test_joint_is_within_limits() {
        let mut joint = Joint::new(JointType::Revolute, 0.5, 1.0, -1.0);
        assert!(joint.is_within_limits());

        joint.value = 1.5;
        assert!(!joint.is_within_limits());

        joint.value = -1.5;
        assert!(!joint.is_within_limits());
    }

    #[test]
    fn test_joint_clamp() {
        let mut joint = Joint::new(JointType::Revolute, 2.0, 1.0, -1.0);
        joint.clamp();
        assert_eq!(joint.value, 1.0);

        joint.value = -2.0;
        joint.clamp();
        assert_eq!(joint.value, -1.0);
    }

    #[test]
    fn test_joint_set_value() {
        let mut joint = Joint::new(JointType::Revolute, 0.0, 1.0, -1.0);
        assert!(joint.set_value(0.5).is_ok());
        assert_eq!(joint.value, 0.5);

        assert!(joint.set_value(1.5).is_err());
        assert!(joint.set_value(-1.5).is_err());
    }

    #[test]
    fn test_joint_type_display() {
        assert_eq!(JointType::Revolute.to_string(), "R");
        assert_eq!(JointType::Prismatic.to_string(), "P");
    }
}
