pub struct DHParams {
    pub theta: f64,
    pub d: f64,
    pub a: f64,
    pub alpha: f64,
}

impl DHParams {
    pub fn new(theta: f64, d: f64, a: f64, alpha: f64) -> Self {
        Self { theta, d, a, alpha }
    }
}
