use std::fmt;

use crate::math::{Mat3, Vec3};

#[derive(Debug, Clone, PartialEq)]
pub struct Quaternion {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

impl Quaternion {
    pub fn new(a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { a, b, c, d }
    }

    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }

    pub fn norm_sq(&self) -> f64 {
        self.a * self.a + self.b * self.b + self.c * self.c + self.d * self.d
    }

    pub fn norm(&self) -> f64 {
        self.norm_sq().sqrt()
    }

    pub fn normalize(&self) -> Self {
        let n = self.norm();
        Self::new(self.a / n, self.b / n, self.c / n, self.d / n)
    }

    pub fn conjugate(&self) -> Self {
        Self::new(self.a, -self.b, -self.c, -self.d)
    }

    pub fn inverse(&self) -> Self {
        let n = self.norm_sq();
        Self::new(self.a / n, -self.b / n, -self.c / n, -self.d / n)
    }

    /// Producto de Hamilton entre dos cuaterniones (⊗).
    pub fn mul(&self, other: &Self) -> Self {
        Self::new(
            self.a * other.a - self.b * other.b - self.c * other.c - self.d * other.d,
            self.a * other.b + self.b * other.a + self.c * other.d - self.d * other.c,
            self.a * other.c - self.b * other.d + self.c * other.a + self.d * other.b,
            self.a * other.d + self.b * other.c - self.c * other.b + self.d * other.a,
        )
    }

    /// Escala todos los componentes por un factor.
    pub fn scale(&self, factor: f64) -> Self {
        Self::new(
            self.a * factor,
            self.b * factor,
            self.c * factor,
            self.d * factor,
        )
    }

    /// Convierte una matriz de rotación a cuaternión unitario
    /// mediante el método de la traza (Shepperd).
    pub fn from_rotation_matrix(r: &Mat3) -> Self {
        let tr = r.trace();
        let q = if tr > 0.0 {
            let s = (tr + 1.0).sqrt() * 2.0;
            Self::new(
                0.25 * s,
                (r[(2, 1)] - r[(1, 2)]) / s,
                (r[(0, 2)] - r[(2, 0)]) / s,
                (r[(1, 0)] - r[(0, 1)]) / s,
            )
        } else if r[(0, 0)] > r[(1, 1)] && r[(0, 0)] > r[(2, 2)] {
            let s = (1.0 + r[(0, 0)] - r[(1, 1)] - r[(2, 2)]).sqrt() * 2.0;
            Self::new(
                (r[(2, 1)] - r[(1, 2)]) / s,
                0.25 * s,
                (r[(0, 1)] + r[(1, 0)]) / s,
                (r[(0, 2)] + r[(2, 0)]) / s,
            )
        } else if r[(1, 1)] > r[(2, 2)] {
            let s = (1.0 + r[(1, 1)] - r[(0, 0)] - r[(2, 2)]).sqrt() * 2.0;
            Self::new(
                (r[(0, 2)] - r[(2, 0)]) / s,
                (r[(0, 1)] + r[(1, 0)]) / s,
                0.25 * s,
                (r[(1, 2)] + r[(2, 1)]) / s,
            )
        } else {
            let s = (1.0 + r[(2, 2)] - r[(0, 0)] - r[(1, 1)]).sqrt() * 2.0;
            Self::new(
                (r[(1, 0)] - r[(0, 1)]) / s,
                (r[(0, 2)] + r[(2, 0)]) / s,
                (r[(1, 2)] + r[(2, 1)]) / s,
                0.25 * s,
            )
        };
        q.normalize()
    }
}

impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({} + {}i + {}j + {}k)", self.a, self.b, self.c, self.d)
    }
}

/// Cuaternión dual unitario que representa una pose rígida (rotación + traslación).
///
/// `real` codifica la rotación y `dual` la traslación: $q_d = 1/2 t ⊗ q_r$,
/// con $t$ el vector de traslación como cuaternión puro.
#[derive(Debug, Clone, PartialEq)]
pub struct DualQuaternion {
    pub real: Quaternion,
    pub dual: Quaternion,
}

impl DualQuaternion {
    /// Construye el cuaternión dual desde la pose (matriz de rotación y traslación).
    pub fn from_pose(rotation: &Mat3, translation: &Vec3) -> Self {
        let real = Quaternion::from_rotation_matrix(rotation);
        let t = Quaternion::new(0.0, translation.x, translation.y, translation.z);
        let dual = t.mul(&real).scale(0.5);
        Self { real, dual }
    }

    /// Reconstruye la traslación: $t = 2 q_d ⊗ q_r^star$ (parte vectorial).
    pub fn translation(&self) -> Vec3 {
        let v = self.dual.mul(&self.real.conjugate()).scale(2.0);
        Vec3::new(v.b, v.c, v.d)
    }
}

pub fn solve_add(quaternions: &[Quaternion]) -> Quaternion {
    let mut result = Quaternion::zero();
    for q in quaternions {
        result.a += q.a;
        result.b += q.b;
        result.c += q.c;
        result.d += q.d;
    }
    result
}

pub fn solve_subtract(quaternions: &[Quaternion]) -> Quaternion {
    let mut result = Quaternion::zero();
    for q in quaternions {
        result.a -= q.a;
        result.b -= q.b;
        result.c -= q.c;
        result.d -= q.d;
    }
    result
}

pub fn solve_multiply(quaternions: &[Quaternion]) -> Quaternion {
    let mut result = Quaternion::identity();
    for q in quaternions {
        let a = result.a * q.a - result.b * q.b - result.c * q.c - result.d * q.d;
        let b = result.a * q.b + result.b * q.a + result.c * q.d - result.d * q.c;
        let c = result.a * q.c - result.b * q.d + result.c * q.a + result.d * q.b;
        let d = result.a * q.d + result.b * q.c - result.c * q.b + result.d * q.a;
        result = Quaternion::new(a, b, c, d);
    }
    result
}

pub fn solve_divide(quaternions: &[Quaternion]) -> Quaternion {
    let mut result = Quaternion::identity();
    for q in quaternions {
        let n = q.norm_sq();
        let a = (result.a * q.a + result.b * q.b + result.c * q.c + result.d * q.d) / n;
        let b = (result.b * q.a - result.a * q.b - result.c * q.d + result.d * q.c) / n;
        let c = (result.c * q.a + result.a * q.c - result.b * q.d - result.d * q.b) / n;
        let d = (result.d * q.a - result.a * q.d + result.b * q.c - result.c * q.b) / n;
        result = Quaternion::new(a, b, c, d);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-10
    }

    fn quat_approx_eq(q1: &Quaternion, q2: &Quaternion) -> bool {
        approx_eq(q1.a, q2.a)
            && approx_eq(q1.b, q2.b)
            && approx_eq(q1.c, q2.c)
            && approx_eq(q1.d, q2.d)
    }

    #[test]
    fn test_new() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.a, 1.0);
        assert_eq!(q.b, 2.0);
        assert_eq!(q.c, 3.0);
        assert_eq!(q.d, 4.0);
    }

    #[test]
    fn test_identity() {
        let q = Quaternion::identity();
        assert!(quat_approx_eq(&q, &Quaternion::new(1.0, 0.0, 0.0, 0.0)));
    }

    #[test]
    fn test_zero() {
        let q = Quaternion::zero();
        assert!(quat_approx_eq(&q, &Quaternion::new(0.0, 0.0, 0.0, 0.0)));
    }

    #[test]
    fn test_display() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.to_string(), "(1 + 2i + 3j + 4k)");
    }

    #[test]
    fn test_norm_sq() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        assert!(approx_eq(q.norm_sq(), 30.0));
    }

    #[test]
    fn test_norm() {
        let q = Quaternion::new(0.0, 3.0, 4.0, 0.0);
        assert!(approx_eq(q.norm(), 5.0));
    }

    #[test]
    fn test_normalize() {
        let q = Quaternion::new(0.0, 3.0, 4.0, 0.0);
        let n = q.normalize();
        assert!(approx_eq(n.norm(), 1.0));
        assert!(approx_eq(n.b, 0.6));
        assert!(approx_eq(n.c, 0.8));
    }

    #[test]
    fn test_normalize_identity() {
        let q = Quaternion::identity();
        assert!(quat_approx_eq(&q.normalize(), &q));
    }

    #[test]
    fn test_conjugate() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let c = q.conjugate();
        assert!(quat_approx_eq(&c, &Quaternion::new(1.0, -2.0, -3.0, -4.0)));
    }

    #[test]
    fn test_inverse() {
        let q = Quaternion::new(1.0, 2.0, 0.0, 0.0);
        let inv = q.inverse();
        let product = solve_multiply(&[q.clone(), inv]);
        assert!(quat_approx_eq(&product, &Quaternion::identity()));
    }

    #[test]
    fn test_add_two() {
        let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let q2 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        let result = solve_add(&[q1, q2]);
        assert!(quat_approx_eq(
            &result,
            &Quaternion::new(6.0, 8.0, 10.0, 12.0)
        ));
    }

    #[test]
    fn test_add_empty() {
        let result = solve_add(&[]);
        assert!(quat_approx_eq(&result, &Quaternion::zero()));
    }

    #[test]
    fn test_subtract_two() {
        let q1 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        let q2 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        // solve_subtract empieza en 0 y resta cada uno: 0 - q1 - q2
        let result = solve_subtract(&[q1, q2]);
        assert!(quat_approx_eq(
            &result,
            &Quaternion::new(-6.0, -8.0, -10.0, -12.0)
        ));
    }

    #[test]
    fn test_multiply_identity() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let result = solve_multiply(&[q.clone(), Quaternion::identity()]);
        assert!(quat_approx_eq(&result, &q));
    }

    #[test]
    fn test_multiply_two() {
        let q1 = Quaternion::new(1.0, 1.0, 0.0, 0.0); // 1 + i
        let q2 = Quaternion::new(1.0, 0.0, 0.0, 1.0); // 1 + k
        let result = solve_multiply(&[q1, q2]);
        // (1+i)(1+k) = 1 + k + i + ik = 1 + i - j + k
        assert!(approx_eq(result.a, 1.0));
        assert!(approx_eq(result.b, 1.0));
        assert!(approx_eq(result.c, -1.0));
        assert!(approx_eq(result.d, 1.0));
    }

    #[test]
    fn test_divide_single() {
        let q = Quaternion::new(1.0, 2.0, 0.0, 0.0);
        // solve_divide(&[q]) = id * q^{-1} = q^{-1}
        let result = solve_divide(&[q.clone()]);
        assert!(quat_approx_eq(&result, &q.inverse()));
    }

    #[test]
    fn test_divide_inverse() {
        let q = Quaternion::new(1.0, 2.0, 0.0, 0.0);
        let result = solve_divide(&[Quaternion::identity(), q.clone()]);
        let expected = q.inverse();
        assert!(quat_approx_eq(&result, &expected));
    }

    #[test]
    fn test_add_multiple() {
        let q = Quaternion::new(1.0, 1.0, 1.0, 1.0);
        let result = solve_add(&[q.clone(), q.clone(), q.clone()]);
        assert!(quat_approx_eq(
            &result,
            &Quaternion::new(3.0, 3.0, 3.0, 3.0)
        ));
    }

    #[test]
    fn test_multiply_associativity() {
        let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let q2 = Quaternion::new(5.0, 6.0, 7.0, 8.0);
        let q3 = Quaternion::new(9.0, 10.0, 11.0, 12.0);
        let r1 = solve_multiply(&[solve_multiply(&[q1.clone(), q2.clone()]), q3.clone()]);
        let r2 = solve_multiply(&[q1, q2, q3]);
        assert!(quat_approx_eq(&r1, &r2));
    }

    #[test]
    fn test_from_rotation_matrix_identity() {
        let r = Mat3::identity();
        let q = Quaternion::from_rotation_matrix(&r);
        assert!(quat_approx_eq(&q, &Quaternion::new(1.0, 0.0, 0.0, 0.0)));
    }

    #[test]
    fn test_from_rotation_matrix_rotx_90() {
        // Matriz del efector en home (documento): R_0,5 = [[1,0,0],[0,0,1],[0,-1,0]].
        let r = Mat3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
        let q = Quaternion::from_rotation_matrix(&r);
        // Método de la traza → (√2/2, −√2/2, 0, 0).
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!((q.a - h).abs() < 1e-9);
        assert!((q.b + h).abs() < 1e-9);
        assert!(q.c.abs() < 1e-9);
        assert!(q.d.abs() < 1e-9);
        assert!((q.norm() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_dual_quaternion_pose_reconstruction() {
        // Pose home del FABRI: R = [[1,0,0],[0,0,1],[0,-1,0]], t = (140, -15, 205) mm.
        let r = Mat3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
        let t = Vec3::new(140.0, -15.0, 205.0);
        let dq = DualQuaternion::from_pose(&r, &t);
        let t_rec = dq.translation();
        assert!((t_rec.x - 140.0).abs() < 1e-9);
        assert!((t_rec.y + 15.0).abs() < 1e-9);
        assert!((t_rec.z - 205.0).abs() < 1e-9);
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!((dq.real.a - h).abs() < 1e-9);
        assert!((dq.real.b + h).abs() < 1e-9);
    }
}
