//! Loading and applying the post-IK spatial calibration.

use serde::Deserialize;
use std::cmp::Ordering;
use std::env;
use std::fmt;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

const MAX_INTERPOLATION_POINTS: usize = 4;
const IDW_POWER: f64 = 2.0;

pub type Point2 = [f64; 2];

#[derive(Debug, Clone, Deserialize)]
pub struct StlCalibrationEntry {
    pub filename: String,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

/// The post-IK calibration document.
///
/// Optional STL fields are retained for API compatibility. Unknown fields,
/// including future STL fields, are ignored by serde.
#[derive(Debug, Clone, Deserialize)]
pub struct CalibrationConfig {
    #[serde(default)]
    pub version: Option<u32>,
    #[serde(rename = "stlScale", default)]
    pub stl_scale: Option<f64>,
    #[serde(default)]
    pub entries: Vec<StlCalibrationEntry>,
    #[serde(default)]
    pub backlash_offset_x_mm: Option<f64>,
    #[serde(default)]
    pub k_velocity_factor: Option<f64>,
    #[serde(default)]
    pub calibration_points: Option<Vec<CalibrationPoint>>,
}

#[derive(Debug, Clone, Deserialize, PartialEq)]
pub struct CalibrationPoint {
    pub center_x: f64,
    pub center_y: f64,
    pub a11: f64,
    pub a12: f64,
    pub a21: f64,
    pub a22: f64,
    #[serde(rename = "b_x")]
    pub b_x: f64,
    #[serde(rename = "b_y")]
    pub b_y: f64,
}

impl CalibrationPoint {
    fn validate(&self, index: usize) -> Result<(), CalibrationError> {
        let values = [
            self.center_x,
            self.center_y,
            self.a11,
            self.a12,
            self.a21,
            self.a22,
            self.b_x,
            self.b_y,
        ];
        if values.iter().any(|value| !value.is_finite()) {
            return Err(CalibrationError::InvalidCalibrationPoint {
                index,
                reason: "contains a non-finite value",
            });
        }

        let determinant = self.determinant();
        if !determinant.is_finite() || determinant <= f64::EPSILON {
            return Err(CalibrationError::InvalidDeterminant { index, determinant });
        }
        Ok(())
    }

    fn determinant(&self) -> f64 {
        self.a11 * self.a22 - self.a12 * self.a21
    }
}

#[derive(Debug)]
pub enum CalibrationError {
    Io {
        path: PathBuf,
        source: io::Error,
    },
    Json {
        path: PathBuf,
        source: serde_json::Error,
    },
    MissingCalibrationFile {
        searched: Vec<PathBuf>,
    },
    MissingPostIkField(&'static str),
    EmptyCalibrationPoints,
    InvalidCalibrationPoint {
        index: usize,
        reason: &'static str,
    },
    InvalidDeterminant {
        index: usize,
        determinant: f64,
    },
    NonFiniteInput,
    NonFiniteInterpolation,
}

impl fmt::Display for CalibrationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(formatter, "unable to read {}: {source}", path.display())
            }
            Self::Json { path, source } => write!(
                formatter,
                "invalid calibration JSON in {}: {source}",
                path.display()
            ),
            Self::MissingCalibrationFile { searched } => write!(
                formatter,
                "post-IK calibration file not found; checked {}",
                searched
                    .iter()
                    .map(|path| path.display().to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
            Self::MissingPostIkField(field) => {
                write!(formatter, "missing post-IK calibration field: {field}")
            }
            Self::EmptyCalibrationPoints => {
                write!(formatter, "post-IK calibration_points is empty")
            }
            Self::InvalidCalibrationPoint { index, reason } => {
                write!(formatter, "invalid calibration point {index}: {reason}")
            }
            Self::InvalidDeterminant { index, determinant } => {
                write!(
                    formatter,
                    "calibration point {index} has invalid determinant {determinant}"
                )
            }
            Self::NonFiniteInput => {
                write!(formatter, "calibration input contains a non-finite value")
            }
            Self::NonFiniteInterpolation => write!(
                formatter,
                "calibration interpolation produced a non-finite value"
            ),
        }
    }
}

impl std::error::Error for CalibrationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            Self::Json { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// Applies an interpolated spatial affine transform in the inverse direction.
#[derive(Debug, Clone)]
pub struct SpatialAffineCompensator {
    points: Vec<CalibrationPoint>,
    backlash_offset_x_mm: f64,
    k_velocity_factor: f64,
}

impl SpatialAffineCompensator {
    pub fn new(points: Vec<CalibrationPoint>) -> Result<Self, CalibrationError> {
        Self::with_motion_parameters(points, 0.0, 0.0)
    }

    pub fn with_motion_parameters(
        points: Vec<CalibrationPoint>,
        backlash_offset_x_mm: f64,
        k_velocity_factor: f64,
    ) -> Result<Self, CalibrationError> {
        if points.is_empty() {
            return Err(CalibrationError::EmptyCalibrationPoints);
        }
        if !backlash_offset_x_mm.is_finite() || !k_velocity_factor.is_finite() {
            return Err(CalibrationError::NonFiniteInput);
        }
        for (index, point) in points.iter().enumerate() {
            point.validate(index)?;
        }
        Ok(Self {
            points,
            backlash_offset_x_mm,
            k_velocity_factor,
        })
    }

    pub fn points(&self) -> &[CalibrationPoint] {
        &self.points
    }

    pub fn backlash_offset_x_mm(&self) -> f64 {
        self.backlash_offset_x_mm
    }

    pub fn k_velocity_factor(&self) -> f64 {
        self.k_velocity_factor
    }

    pub fn velocity_factor(&self, feed_rate: f64) -> Result<f64, CalibrationError> {
        if !feed_rate.is_finite() {
            return Err(CalibrationError::NonFiniteInput);
        }
        let factor = 1.0 + self.k_velocity_factor * (feed_rate - 1200.0);
        if !factor.is_finite() {
            return Err(CalibrationError::NonFiniteInterpolation);
        }
        Ok(factor)
    }

    /// Inverse-compensate a target using the IDW-interpolated affine transform.
    /// The four nearest calibration points are weighted with inverse squared
    /// distance. An exact center uses that point directly.
    pub fn inverse_compensate(&self, target: Point2) -> Result<Point2, CalibrationError> {
        if target.iter().any(|value| !value.is_finite()) {
            return Err(CalibrationError::NonFiniteInput);
        }

        let mut distances: Vec<(usize, f64)> = self
            .points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                let dx = target[0] - point.center_x;
                let dy = target[1] - point.center_y;
                (index, dx * dx + dy * dy)
            })
            .collect();
        if distances.iter().any(|(_, distance)| !distance.is_finite()) {
            return Err(CalibrationError::NonFiniteInterpolation);
        }
        distances.sort_by(|left, right| left.1.partial_cmp(&right.1).unwrap_or(Ordering::Equal));

        let selected = &distances[..distances.len().min(MAX_INTERPOLATION_POINTS)];

        // Coincident calibration centers are valid when they represent
        // different experiments at the same workspace location. Average all
        // coincident transforms instead of selecting one by insertion order.
        let exact: Vec<_> = distances
            .iter()
            .filter(|(_, distance_squared)| *distance_squared <= f64::EPSILON)
            .collect();
        if !exact.is_empty() {
            let mut coefficients = [0.0; 6];
            for (index, _) in &exact {
                let point = &self.points[*index];
                coefficients[0] += point.a11;
                coefficients[1] += point.a12;
                coefficients[2] += point.a21;
                coefficients[3] += point.a22;
                coefficients[4] += point.b_x;
                coefficients[5] += point.b_y;
            }
            let count = exact.len() as f64;
            for coefficient in &mut coefficients {
                *coefficient /= count;
            }
            let determinant = coefficients[0] * coefficients[3] - coefficients[1] * coefficients[2];
            if !determinant.is_finite() || determinant <= f64::EPSILON {
                return Err(CalibrationError::InvalidDeterminant {
                    index: usize::MAX,
                    determinant,
                });
            }
            return Self::inverse_affine(
                target,
                coefficients[0],
                coefficients[1],
                coefficients[2],
                coefficients[3],
                coefficients[4],
                coefficients[5],
                determinant,
                usize::MAX,
            );
        }

        let mut weight_sum = 0.0;
        let mut coefficients = [0.0; 6];
        for (index, distance_squared) in selected {
            let weight = 1.0 / distance_squared.powf(IDW_POWER / 2.0);
            let point = &self.points[*index];
            weight_sum += weight;
            coefficients[0] += weight * point.a11;
            coefficients[1] += weight * point.a12;
            coefficients[2] += weight * point.a21;
            coefficients[3] += weight * point.a22;
            coefficients[4] += weight * point.b_x;
            coefficients[5] += weight * point.b_y;
        }
        if !weight_sum.is_finite() || weight_sum == 0.0 {
            return Err(CalibrationError::NonFiniteInterpolation);
        }
        for coefficient in &mut coefficients {
            *coefficient /= weight_sum;
        }

        let determinant = coefficients[0] * coefficients[3] - coefficients[1] * coefficients[2];
        if !determinant.is_finite() || determinant <= f64::EPSILON {
            return Err(CalibrationError::InvalidDeterminant {
                index: usize::MAX,
                determinant,
            });
        }
        Self::inverse_affine(
            target,
            coefficients[0],
            coefficients[1],
            coefficients[2],
            coefficients[3],
            coefficients[4],
            coefficients[5],
            determinant,
            usize::MAX,
        )
    }

    pub fn compensate(&self, target: Point2) -> Result<Point2, CalibrationError> {
        self.inverse_compensate(target)
    }

    fn inverse_affine(
        target: Point2,
        a11: f64,
        a12: f64,
        a21: f64,
        a22: f64,
        b_x: f64,
        b_y: f64,
        determinant: f64,
        index: usize,
    ) -> Result<Point2, CalibrationError> {
        let translated = [target[0] - b_x, target[1] - b_y];
        let result = [
            (a22 * translated[0] - a12 * translated[1]) / determinant,
            (-a21 * translated[0] + a11 * translated[1]) / determinant,
        ];
        if result.iter().any(|value| !value.is_finite()) {
            return Err(CalibrationError::InvalidCalibrationPoint {
                index,
                reason: "inverse compensation is non-finite",
            });
        }
        Ok(result)
    }
}

#[derive(Debug, Clone)]
pub struct CalibrationLoader {
    config: CalibrationConfig,
    compensator: SpatialAffineCompensator,
    backlash_offset_x_mm: f64,
    k_velocity_factor: f64,
}

impl CalibrationLoader {
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self, CalibrationError> {
        let path = path.as_ref().to_path_buf();
        let contents = fs::read_to_string(&path).map_err(|source| CalibrationError::Io {
            path: path.clone(),
            source,
        })?;
        let config: CalibrationConfig =
            serde_json::from_str(&contents).map_err(|source| CalibrationError::Json {
                path: path.clone(),
                source,
            })?;
        Self::from_config(config)
    }

    pub fn load_default() -> Result<Self, CalibrationError> {
        let mut candidates = Vec::new();
        if let Some(path) = env::var_os("BOMBOLAB_POST_IK_CALIBRATION") {
            candidates.push(PathBuf::from(path));
        }
        candidates.extend([
            PathBuf::from("web/public/post-ik-calibration.json"),
            PathBuf::from("post-ik-calibration.json"),
        ]);

        // Keep the previous locations as compatibility fallbacks, but only
        // accept them when they actually contain post-IK fields. In
        // particular, the STL-only calibration must never be treated as an
        // identity post-IK calibration.
        if let Some(path) = env::var_os("BOMBOLAB_CALIBRATION") {
            candidates.push(PathBuf::from(path));
        }
        candidates.extend([
            PathBuf::from("web/public/calibration.json"),
            PathBuf::from("calibration.json"),
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../web/public/calibration.json"),
        ]);

        let mut missing_post_ik = None;
        for path in &candidates {
            if !path.is_file() {
                continue;
            }
            match Self::from_path(path) {
                Ok(loader) => return Ok(loader),
                Err(error @ CalibrationError::MissingPostIkField(_)) => {
                    missing_post_ik.get_or_insert(error);
                }
                Err(error) => return Err(error),
            }
        }

        if let Some(error) = missing_post_ik {
            return Err(error);
        }
        Err(CalibrationError::MissingCalibrationFile {
            searched: candidates,
        })
    }

    pub fn from_config(config: CalibrationConfig) -> Result<Self, CalibrationError> {
        let backlash_offset_x_mm = config
            .backlash_offset_x_mm
            .ok_or(CalibrationError::MissingPostIkField("backlash_offset_x_mm"))?;
        let k_velocity_factor = config
            .k_velocity_factor
            .ok_or(CalibrationError::MissingPostIkField("k_velocity_factor"))?;
        if !backlash_offset_x_mm.is_finite() {
            return Err(CalibrationError::NonFiniteInput);
        }
        if !k_velocity_factor.is_finite() {
            return Err(CalibrationError::NonFiniteInput);
        }
        let points = config
            .calibration_points
            .as_ref()
            .ok_or(CalibrationError::MissingPostIkField("calibration_points"))?
            .clone();
        let compensator = SpatialAffineCompensator::with_motion_parameters(
            points,
            backlash_offset_x_mm,
            k_velocity_factor,
        )?;
        Ok(Self {
            config,
            compensator,
            backlash_offset_x_mm,
            k_velocity_factor,
        })
    }

    pub fn config(&self) -> &CalibrationConfig {
        &self.config
    }

    pub fn compensator(&self) -> &SpatialAffineCompensator {
        &self.compensator
    }

    pub fn backlash_offset_x_mm(&self) -> f64 {
        self.backlash_offset_x_mm
    }

    pub fn k_velocity_factor(&self) -> f64 {
        self.k_velocity_factor
    }

    pub fn inverse_compensate(&self, target: Point2) -> Result<Point2, CalibrationError> {
        self.compensator.inverse_compensate(target)
    }
}

pub fn load_from_path(path: impl AsRef<Path>) -> Result<CalibrationLoader, CalibrationError> {
    CalibrationLoader::from_path(path)
}

pub fn load_default() -> Result<CalibrationLoader, CalibrationError> {
    CalibrationLoader::load_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_separate_post_ik_fields() {
        let json = r#"
        {
          "version": 1,
          "backlash_offset_x_mm": 12.333333333333334,
          "k_velocity_factor": 0.000002450980392156863,
          "calibration_points": [{"center_x": 1, "center_y": 2, "a11": 1, "a12": 0, "a21": 0, "a22": 1, "b_x": 3, "b_y": 4}]
        }
        "#;
        let config: CalibrationConfig =
            serde_json::from_str(json).expect("valid post-IK calibration JSON");
        assert_eq!(config.version, Some(1));
        assert!(config.entries.is_empty());
        let loader = CalibrationLoader::from_config(config).expect("complete post-IK calibration");
        assert_eq!(loader.backlash_offset_x_mm(), 12.333333333333334);
        assert_eq!(loader.compensator().points().len(), 1);
    }

    #[test]
    fn stl_calibration_without_post_ik_fields_is_rejected() {
        let json = r#"
        {
          "version": 1,
          "stlScale": 1,
          "entries": [{"filename": "Base.stl", "translation": [0, 0, 0], "rotation": [0, 0, 0, 1]}]
        }
        "#;
        let config: CalibrationConfig =
            serde_json::from_str(json).expect("valid STL calibration JSON");
        assert!(matches!(
            CalibrationLoader::from_config(config),
            Err(CalibrationError::MissingPostIkField("backlash_offset_x_mm"))
        ));
    }

    #[test]
    fn inverse_compensation_uses_positive_determinant() {
        let point = CalibrationPoint {
            center_x: 0.0,
            center_y: 0.0,
            a11: 2.0,
            a12: 0.0,
            a21: 0.0,
            a22: 4.0,
            b_x: 1.0,
            b_y: -2.0,
        };
        let compensator = SpatialAffineCompensator::new(vec![point]).expect("positive determinant");
        let corrected = compensator
            .inverse_compensate([5.0, 6.0])
            .expect("finite inverse");
        assert!((corrected[0] - 2.0).abs() < 1e-12);
        assert!((corrected[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn idw_returns_exact_center_transform() {
        let point = CalibrationPoint {
            center_x: 10.0,
            center_y: 20.0,
            a11: 1.0,
            a12: 0.0,
            a21: 0.0,
            a22: 1.0,
            b_x: 2.0,
            b_y: 3.0,
        };
        let compensator = SpatialAffineCompensator::new(vec![point]).expect("valid point");
        let corrected = compensator
            .inverse_compensate([12.0, 23.0])
            .expect("exact center inverse");
        assert_eq!(corrected, [10.0, 20.0]);
    }

    #[test]
    fn averages_coincident_calibration_centers() {
        let first = CalibrationPoint {
            center_x: 0.0,
            center_y: 0.0,
            a11: 1.0,
            a12: 0.0,
            a21: 0.0,
            a22: 1.0,
            b_x: 0.0,
            b_y: 0.0,
        };
        let second = CalibrationPoint {
            center_x: 0.0,
            center_y: 0.0,
            a11: 2.0,
            a12: 0.0,
            a21: 0.0,
            a22: 2.0,
            b_x: 0.0,
            b_y: 0.0,
        };
        let compensator = SpatialAffineCompensator::new(vec![first, second])
            .expect("valid coincident calibration points");
        let corrected = compensator
            .inverse_compensate([3.0, 3.0])
            .expect("finite averaged inverse");
        assert!((corrected[0] - 2.0).abs() < 1e-12);
        assert!((corrected[1] - 2.0).abs() < 1e-12);
    }
}
