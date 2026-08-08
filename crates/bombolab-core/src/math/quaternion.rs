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

    pub fn mul(&self, other: &Self) -> Self {
        Self::new(
            self.a * other.a - self.b * other.b - self.c * other.c - self.d * other.d,
            self.a * other.b + self.b * other.a + self.c * other.d - self.d * other.c,
            self.a * other.c - self.b * other.d + self.c * other.a + self.d * other.b,
            self.a * other.d + self.b * other.c - self.c * other.b + self.d * other.a,
        )
    }

    pub fn scale(&self, factor: f64) -> Self {
        Self::new(
            self.a * factor,
            self.b * factor,
            self.c * factor,
            self.d * factor,
        )
    }

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

#[derive(Debug, Clone, PartialEq)]
pub struct DualQuaternion {
    pub real: Quaternion,
    pub dual: Quaternion,
}

impl DualQuaternion {
    pub fn from_pose(rotation: &Mat3, translation: &Vec3) -> Self {
        let real = Quaternion::from_rotation_matrix(rotation);
        let t = Quaternion::new(0.0, translation.x, translation.y, translation.z);
        let dual = t.mul(&real).scale(0.5);
        Self { real, dual }
    }

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
#[path = "quaternion_tests.rs"]
mod quaternion_tests;
