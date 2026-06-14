pub struct Link {
    pub offset_to_next_joint: [f64; 3]
}

impl Link {
    pub fn new (offset_to_next_joint: [f64; 3]) -> Self {
        Self {
            offset_to_next_joint
        }
    }

    pub fn set_offset (&mut self, offset: [f64; 3]) {
        self.offset_to_next_joint = offset
    }

    pub fn length (&self) -> f64 {
        let [x, y, z] = self.offset_to_next_joint;
        let result: f64 = (x * x + y * y + z * z).sqrt();
        result
    }
}