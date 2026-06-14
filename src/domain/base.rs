pub struct Base {
    pub position: [f64; 3],
}

impl Base {
    pub fn new (position: [f64;3]) -> Self {
        Self {
            position
        }
    }
}