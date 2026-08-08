use std::fmt;

use crate::math::{Mat3, Mat4, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DHParameter {
    pub alpha: f64,
    pub a: f64,
    pub d: f64,
    pub theta: f64,
}

impl DHParameter {
    pub fn new(alpha: f64, a: f64, d: f64, theta: f64) -> Self {
        Self { alpha, a, d, theta }
    }
}

#[derive(Debug, Clone)]
pub enum DHValue {
    Num(f64),
    Sym(String),
}

impl DHValue {
    pub fn is_numeric(&self) -> bool {
        matches!(self, DHValue::Num(_))
    }

    pub fn as_num(&self) -> Option<f64> {
        match self {
            DHValue::Num(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            DHValue::Sym(s) => s,
            _ => "",
        }
    }
}

impl fmt::Display for DHValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DHValue::Num(v) => write!(f, "{}", v),
            DHValue::Sym(s) => write!(f, "{}", s),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DHParameterSymbolic {
    pub alpha: DHValue,
    pub a: DHValue,
    pub d: DHValue,
    pub theta: DHValue,
}

impl DHParameterSymbolic {
    pub fn new(alpha: DHValue, a: DHValue, d: DHValue, theta: DHValue) -> Self {
        Self { alpha, a, d, theta }
    }

    pub fn is_numeric(&self) -> bool {
        self.alpha.is_numeric()
            && self.a.is_numeric()
            && self.d.is_numeric()
            && self.theta.is_numeric()
    }

    pub fn to_numeric(&self) -> Option<DHParameter> {
        Some(DHParameter::new(
            self.alpha.as_num()?,
            self.a.as_num()?,
            self.d.as_num()?,
            self.theta.as_num()?,
        ))
    }
}

pub fn compute_a_matrix(p: DHParameter) -> Mat4 {
    let (st, ct) = p.theta.sin_cos();
    let (sa, ca) = p.alpha.sin_cos();

    Mat4::new(
        ct,
        -st * ca,
        st * sa,
        p.a * ct,
        st,
        ct * ca,
        -ct * sa,
        p.a * st,
        0.0,
        sa,
        ca,
        p.d,
        0.0,
        0.0,
        0.0,
        1.0,
    )
}

pub struct DHSolution {
    pub table: Vec<DHParameter>,
    pub a_matrices: Vec<Mat4>,
    pub intermediates: Vec<Mat4>,
    pub final_transform: Mat4,
}

impl DHSolution {
    pub fn rotation(&self) -> Mat3 {
        self.final_transform.fixed_view::<3, 3>(0, 0).into_owned()
    }

    pub fn translation(&self) -> Vec3 {
        self.final_transform.fixed_view::<3, 1>(0, 3).into_owned()
    }
}

pub fn solve(table: &[DHParameter]) -> DHSolution {
    let a_matrices: Vec<_> = table.iter().map(|p| compute_a_matrix(*p)).collect();

    let mut intermediates = Vec::with_capacity(a_matrices.len());
    let mut acc = Mat4::identity();
    for a in &a_matrices {
        acc *= a;
        intermediates.push(acc);
    }

    let final_transform = intermediates.last().copied().unwrap_or_else(Mat4::identity);

    DHSolution {
        table: table.to_vec(),
        a_matrices,
        intermediates,
        final_transform,
    }
}

impl fmt::Display for DHSolution {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  TABLA DH")?;
        writeln!(
            f,
            "  {:>3} │ {:>10} │ {:>10} │ {:>10} │ {:>10}",
            "i", "α", "a", "d", "θ"
        )?;
        writeln!(
            f,
            "  {:>3}─┼─{:>10}─┼─{:>10}─┼─{:>10}─┼─{:>10}",
            "─", "─", "─", "─", "─"
        )?;
        for (i, p) in self.table.iter().enumerate() {
            writeln!(
                f,
                "  {:>3} │ {:>10.4} │ {:>10.4} │ {:>10.4} │ {:>10.4}",
                i + 1,
                p.alpha,
                p.a,
                p.d,
                p.theta
            )?;
        }

        writeln!(f, "\n  MATRICES A_i")?;
        for (i, a) in self.a_matrices.iter().enumerate() {
            writeln!(f, "\n  A{}:", i + 1)?;
            write_matrix(f, a)?;
        }

        writeln!(f, "\n  FRAMES (posiciones de cada joint)")?;
        for (i, t) in self.intermediates.iter().enumerate() {
            let pos = t.fixed_view::<3, 1>(0, 3);
            writeln!(
                f,
                "  Frame {}: ({:>7.4}, {:>7.4}, {:>7.4})",
                i + 1,
                pos[(0, 0)],
                pos[(1, 0)],
                pos[(2, 0)]
            )?;
        }

        writeln!(f, "\n  POSE FINAL (efector)")?;
        let pos = self.translation();
        writeln!(
            f,
            "  Posición: ({:>7.4}, {:>7.4}, {:>7.4})",
            pos.x, pos.y, pos.z
        )?;
        let r = self.rotation();
        writeln!(f, "  Rotación:")?;
        for row in 0..3 {
            writeln!(
                f,
                "    [{:>7.4} {:>7.4} {:>7.4}]",
                r[(row, 0)],
                r[(row, 1)],
                r[(row, 2)]
            )?;
        }

        Ok(())
    }
}

fn write_matrix(f: &mut fmt::Formatter<'_>, m: &Mat4) -> fmt::Result {
    for row in 0..4 {
        writeln!(
            f,
            "    [{:>8.4} {:>8.4} {:>8.4} {:>8.4}]",
            m[(row, 0)],
            m[(row, 1)],
            m[(row, 2)],
            m[(row, 3)]
        )?;
    }
    Ok(())
}

pub fn format_symbolic_matrix(p: &DHParameterSymbolic, angle_unit: &str) -> String {
    let theta = &p.theta;
    let alpha = &p.alpha;
    let a = &p.a;
    let d = &p.d;

    let (alpha_is_zero, alpha_is_90, alpha_is_neg90) = match alpha {
        DHValue::Num(v) => {
            let deg = if angle_unit == "grados" {
                *v
            } else {
                v.to_degrees()
            };
            (
                deg % 360.0 == 0.0,
                deg % 360.0 == 90.0,
                deg % 360.0 == 270.0,
            )
        }
        _ => (false, false, false),
    };

    let sin_alpha = if alpha_is_zero {
        "0".to_string()
    } else if alpha_is_90 {
        "1".to_string()
    } else if alpha_is_neg90 {
        "-1".to_string()
    } else {
        format!("sin({})", fmt_angle(alpha, angle_unit))
    };
    let cos_alpha = if alpha_is_zero {
        "1".to_string()
    } else if alpha_is_90 || alpha_is_neg90 {
        "0".to_string()
    } else {
        format!("cos({})", fmt_angle(alpha, angle_unit))
    };

    let c = format!("cos({})", fmt_angle(theta, angle_unit));
    let s = format!("sin({})", fmt_angle(theta, angle_unit));
    let a_str = fmt_val(a, angle_unit);
    let d_str = fmt_val(d, angle_unit);

    let r00 = c.clone();
    let r01 = match cos_alpha.as_str() {
        "0" => "0".to_string(),
        "1" => format!("-{}", s),
        _ => format!("-{}·{}", s, cos_alpha),
    };
    let r02 = match sin_alpha.as_str() {
        "0" => "0".to_string(),
        "1" => s.clone(),
        _ => format!("{}·{}", s, sin_alpha),
    };
    let r03 = format!("{}·{}", a_str, c);

    let r10 = s.clone();
    let r11 = match cos_alpha.as_str() {
        "0" => "0".to_string(),
        "1" => c.clone(),
        _ => format!("{}·{}", c, cos_alpha),
    };
    let r12 = match sin_alpha.as_str() {
        "0" => "0".to_string(),
        "1" => format!("-{}", c),
        _ => format!("-{}·{}", c, sin_alpha),
    };
    let r13 = format!("{}·{}", a_str, s);

    let r21 = sin_alpha.clone();
    let r22 = cos_alpha.clone();

    let row0 = format!("  {:<12} {:<12} {:<12} {:<12}", r00, r01, r02, r03);
    let row1 = format!("  {:<12} {:<12} {:<12} {:<12}", r10, r11, r12, r13);
    let row2 = format!("  {:<12} {:<12} {:<12} {:<12}", "0", r21, r22, d_str);
    let row3 = format!("  {:<12} {:<12} {:<12} {:<12}", "0", "0", "0", "1");

    format!("{}\n{}\n{}\n{}", row0, row1, row2, row3)
}

fn fmt_val(v: &DHValue, _angle_unit: &str) -> String {
    match v {
        DHValue::Num(n) => format!("{}", n),
        DHValue::Sym(s) => s.clone(),
    }
}

fn fmt_angle(v: &DHValue, angle_unit: &str) -> String {
    match v {
        DHValue::Num(n) => {
            if angle_unit == "grados" {
                format!("{}°", n)
            } else {
                format!("{}", n)
            }
        }
        DHValue::Sym(s) => s.clone(),
    }
}

#[cfg(test)]
#[path = "dh_tests.rs"]
mod dh_tests;
