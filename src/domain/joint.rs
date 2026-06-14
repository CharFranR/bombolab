pub enum JointType {
    Revolute,
    Prismatic
}

pub struct Joint {
    pub joint_type: JointType,
    pub value: f64,
    pub value_max: f64,
    pub value_min: f64
}

impl Joint {
    pub fn new (joint_type: JointType, value: f64, value_max: f64, value_min: f64) -> Self{
        Self {
                joint_type, value, value_max, value_min
        }
    }

    pub fn range(&self) -> Vec<f64>{
        vec![self.value_min, self.value_max]
    }

    pub fn is_within_limits(&self) -> bool{
        self.value < self.value_max && self.value > self.value_min
    }

    pub fn clamp(&mut self) {
        if self.value > self.value_max{
            self.value = self.value_max
        } 

        if self.value < self.value_min {
            self.value = self.value_min
        }
    }

    pub fn set_value(&mut self, value: f64) {
        self.value = value
    }
}