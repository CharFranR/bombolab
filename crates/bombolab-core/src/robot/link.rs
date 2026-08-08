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
mod tests {
    use super::*;

    #[test]
    fn from_dh_parameter_maps_fields_by_name() {
        let generic = DHParameter::new(1.0, 2.0, 3.0, 4.0);
        let concrete = DHParams::from(generic);
        assert_eq!(concrete.alpha, 1.0);
        assert_eq!(concrete.a, 2.0);
        assert_eq!(concrete.d, 3.0);
        assert_eq!(concrete.theta, 4.0);
    }

    #[test]
    fn from_dh_params_maps_fields_by_name() {
        let concrete = DHParams::new(1.0, 2.0, 3.0, 4.0);
        let generic = DHParameter::from(concrete);
        assert_eq!(generic.theta, 1.0);
        assert_eq!(generic.d, 2.0);
        assert_eq!(generic.a, 3.0);
        assert_eq!(generic.alpha, 4.0);
    }

    #[test]
    fn round_trip_is_identity() {
        let p = DHParams::new(0.5, 15.0, 120.0, -1.5707963);
        let back = DHParams::from(DHParameter::from(p));
        assert_eq!(back.theta, p.theta);
        assert_eq!(back.d, p.d);
        assert_eq!(back.a, p.a);
        assert_eq!(back.alpha, p.alpha);
    }

    #[test]
    fn dh_params_is_copy() {
        let a = DHParams::new(0.0, 1.0, 2.0, 3.0);
        let b = a;
        assert_eq!(a.theta, b.theta);
    }
}
