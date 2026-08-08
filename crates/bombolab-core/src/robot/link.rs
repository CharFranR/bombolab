use crate::kinematics::DHParameter;

#[derive(Debug, Clone, Copy, PartialEq)]
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

impl From<DHParameter> for DHParams {
    fn from(p: DHParameter) -> Self {
        Self {
            theta: p.theta,
            d: p.d,
            a: p.a,
            alpha: p.alpha,
        }
    }
}

impl From<DHParams> for DHParameter {
    fn from(p: DHParams) -> Self {
        Self {
            alpha: p.alpha,
            a: p.a,
            d: p.d,
            theta: p.theta,
        }
    }
}

#[cfg(test)]
#[path = "link_tests.rs"]
mod link_tests;
