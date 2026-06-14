pub struct Base {
    pub position: [f64; 3],
}

impl Base {
    pub fn new (position: [f64;3]) -> Self {
        Self {
            position
        }
    }

    pub fn set_position (&mut self, position: [f64;3]) {
        self.position = position
    }
}