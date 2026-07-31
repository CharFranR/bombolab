use crate::kinematics::DHParameter;

/// Parámetros Denavit-Hartenberg de un eslabón (convención estándar).
///
/// `A_i = Rot_z(theta) · Trans_z(d) · Trans_x(a) · Rot_x(alpha)`
///
/// # Orden de campos
///
/// ⚠️ Este struct ordena los campos como `(theta, d, a, alpha)` — el orden
/// clásico de las tablas DH impresas. No confundir con [`DHParameter`]
/// (`kinematics`), que usa el orden inverso `(alpha, a, d, theta)` para
/// coincidir con la fórmula `compute_a_matrix`. Usa [`From`] para convertir
/// entre ambos y evitar mezclar θ↔α o a↔d silenciosamente.
///
/// # Unidades
///
/// Ángulos en radianes, distancias en milímetros.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DHParams {
    pub theta: f64,
    pub d: f64,
    pub a: f64,
    pub alpha: f64,
}

impl DHParams {
    /// Crea parámetros DH en orden de tabla clásico: `(theta, d, a, alpha)`.
    ///
    /// Ángulos en radianes, distancias en milímetros.
    pub fn new(theta: f64, d: f64, a: f64, alpha: f64) -> Self {
        Self { theta, d, a, alpha }
    }
}

/// Conversión campo-a-campo (por nombre) desde el tipo genérico de
/// `kinematics::dh`. El mapeo explícito hace imposible intercambiar θ↔α o
/// a↔d, que es el riesgo de mantener dos órdenes de campos en la API.
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

/// Conversión inversa: del modelo concreto del robot al tipo genérico
/// usado por `compute_a_matrix` / `solve` / `dh-solve`.
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

    /// El mapeo debe ser campo-a-campo por nombre: valores distintos en cada
    /// campo detectan cualquier intercambio accidental θ↔α o a↔d.
    #[test]
    fn from_dh_parameter_maps_fields_by_name() {
        let generic = DHParameter::new(1.0, 2.0, 3.0, 4.0); // α, a, d, θ
        let concrete = DHParams::from(generic);
        assert_eq!(concrete.alpha, 1.0);
        assert_eq!(concrete.a, 2.0);
        assert_eq!(concrete.d, 3.0);
        assert_eq!(concrete.theta, 4.0);
    }

    #[test]
    fn from_dh_params_maps_fields_by_name() {
        let concrete = DHParams::new(1.0, 2.0, 3.0, 4.0); // θ, d, a, α
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

    /// DHParams ahora es Copy — verificar que el derive no se rompa.
    #[test]
    fn dh_params_is_copy() {
        let a = DHParams::new(0.0, 1.0, 2.0, 3.0);
        let b = a; // copy, no move
        assert_eq!(a.theta, b.theta);
    }
}
