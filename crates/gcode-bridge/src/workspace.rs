//! Target drawing region and auto-scaling for the FABRI Creator.
//!
//! CIPRA emits drawings on an A4 portrait plane (210×297 mm). The FABRI
//! Creator can only draw on a much more limited horizontal band in front of
//! its base. Rather than over-engineer an approximate workspace estimate, this
//! module treats the reachable region as a **calibratable target rectangle**
//! (`origin` + size) that the user or the mapper configures, and provides the
//! auto-scaling that fits an A4-sized drawing inside it while preserving the
//! aspect ratio.
//!
//! The definitive reachability check lives in [`crate::validate`] via a dry-run
//! IK solve; these bounds are the fast pre-fit and the visualisation guide.

/// Horizontal bounds of the target drawing rectangle, in robot millimetres.
///
/// `x` is the distance outward from the base axis and `y` the lateral sweep,
/// matching the drawing plane (a horizontal plane at the chosen drawing
/// height `z_draw`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DrawingBounds {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

impl DrawingBounds {
    pub fn width(&self) -> f64 {
        self.x_max - self.x_min
    }
    pub fn height(&self) -> f64 {
        self.y_max - self.y_min
    }
    pub fn contains(&self, x: f64, y: f64) -> bool {
        x >= self.x_min && x <= self.x_max && y >= self.y_min && y <= self.y_max
    }
    /// Scale factor that fits a `drawing_w`×`drawing_h` box inside this
    /// rectangle, preserving aspect ratio and never enlarging (≤ 1.0).
    pub fn fit_scale(&self, drawing_w: f64, drawing_h: f64) -> f64 {
        if drawing_w <= 0.0 || drawing_h <= 0.0 || drawing_w.is_nan() || drawing_h.is_nan() {
            return 1.0;
        }
        let sx = self.width() / drawing_w;
        let sy = self.height() / drawing_h;
        sx.min(sy).min(1.0)
    }

    /// A sensible default reachable area in front of the base, matching the
    /// region the browser demo draws within. This is a **starting point** to be
    /// calibrated against the physical robot; the mapper uses it as the target.
    pub fn default_for_fabri() -> Self {
        // Centred ~200 mm off the base axis, ~±50 mm laterally — the demo
        // square sits around these values.
        Self {
            x_min: 150.0,
            x_max: 250.0,
            y_min: -50.0,
            y_max: 50.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contains_checks_full_square() {
        let b = DrawingBounds {
            x_min: 0.0,
            x_max: 100.0,
            y_min: -50.0,
            y_max: 50.0,
        };
        assert!(b.contains(50.0, 0.0));
        assert!(b.contains(0.0, -50.0));
        assert!(!b.contains(101.0, 0.0));
        assert!(!b.contains(50.0, 51.0));
    }

    #[test]
    fn fit_scale_preserves_aspect_and_never_enlarges() {
        let b = DrawingBounds {
            x_min: 0.0,
            x_max: 200.0,
            y_min: -50.0,
            y_max: 50.0, // 200 wide × 100 tall
        };
        // Squares: limited by height (100) → scale = 1.0, 0.5, 1.0.
        assert!((b.fit_scale(100.0, 100.0) - 1.0).abs() < 1e-9);
        assert!((b.fit_scale(200.0, 200.0) - 0.5).abs() < 1e-9);
        // Non-square: width constraint dominates.
        assert!((b.fit_scale(400.0, 100.0) - 0.5).abs() < 1e-9);
        // Smaller than workspace stays 1.0 (no upscaling).
        assert!((b.fit_scale(10.0, 10.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn fit_scale_handles_zero_or_nan_inputs() {
        let b = DrawingBounds::default_for_fabri();
        assert_eq!(b.fit_scale(0.0, 100.0), 1.0);
        assert_eq!(b.fit_scale(f64::NAN, 10.0), 1.0);
        assert_eq!(b.fit_scale(10.0, -5.0), 1.0);
    }

    #[test]
    fn default_for_fabri_is_within_arm_reach() {
        let b = DrawingBounds::default_for_fabri();
        assert!(b.width() > 0.0 && b.height() > 0.0);
        // Entirely in front of the base (positive x).
        assert!(b.x_min > 0.0);
    }
}