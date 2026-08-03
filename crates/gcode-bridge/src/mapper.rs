//! Maps CIPRA A4-drawing coordinates onto the robot drawing plane.
//!
//! CIPRA emits coordinates on an A4 portrait plane (210 mm wide × 297 mm tall,
//! origin bottom-left). The FABRI Creator draws on a horizontal band in front
//! of its base (see [`crate::workspace::DrawingBounds`]). This module converts
//! each A4 point into a robot `(x, y, z)` target:
//!
//! - scales the whole drawing (preserving aspect ratio) so it fits inside the
//!   target rectangle, unless the user overrides the scale,
//! - translates the drawing so its bounding box is centred in the rectangle,
//! - keeps the tool at height `z` for the whole drawing plane.
//!
//! `y` (lateral) in robot space maps directly from `y` in A4 space; `x`
//! (distance from base) maps from `x` in A4 space. The mapping is an affine
//! transform with a single scale along both axes.

use crate::workspace::DrawingBounds;

/// Mapping parameters for converting the A4 plane into the robot plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MappingConfig {
    /// Target rectangle in the robot frame (where the drawing must land).
    pub target: DrawingBounds,
    /// Drawing height in mm (a horizontal plane the tool tip stays on).
    pub z_draw: f64,
    /// Lift height in mm for travel moves (pen up between strokes).
    pub z_travel: f64,
    /// Optional explicit scale. `None` → auto-scale to fit [`target`](DrawingBounds).
    pub scale: Option<f64>,
}

impl Default for MappingConfig {
    fn default() -> Self {
        Self {
            target: DrawingBounds::default_for_fabri(),
            z_draw: 80.0,
            z_travel: 86.0,
            scale: None,
        }
    }
}

/// The z-height of a given move: drawing points sit at `z_draw`, travel moves
/// (pen up) at `z_travel`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MoveZ {
    Draw,
    Travel,
}

impl MoveZ {
    pub fn height(self, config: &MappingConfig) -> f64 {
        match self {
            MoveZ::Draw => config.z_draw,
            MoveZ::Travel => config.z_travel,
        }
    }
}

/// Result of mapping a whole drawing: ready robot targets and the effective
/// scale that was applied (to report to the user).
#[derive(Debug, Clone, PartialEq)]
pub struct MappingResult {
    /// Robot-space `(x, y, z)` target for every original A4 point.
    pub targets: Vec<(f64, f64, f64)>,
    /// Effective scale applied (1.0 if the drawing already fit or scale forced), ≤ 1.0.
    pub scale: f64,
}

/// Computer the effective scale: user override wins, else auto-fit.
fn effective_scale(drawing_w: f64, drawing_h: f64, config: &MappingConfig) -> f64 {
    match config.scale {
        Some(s) => s.max(0.0),
        None => config.target.fit_scale(drawing_w, drawing_h),
    }
}

/// Map a single A4 point to a robot-space target.
///
/// A4 `x` travels across the drawing width, A4 `y` travels across the height.
/// The scaled drawing is centred into `target`.
pub fn map_point(
    x: f64,
    y: f64,
    drawing_w: f64,
    drawing_h: f64,
    config: &MappingConfig,
    z: MoveZ,
) -> (f64, f64, f64) {
    let scale = effective_scale(drawing_w, drawing_h, config);
    let t = &config.target;

    // Centre the scaled A4 box inside the target rectangle.
    let offset_x = t.x_min + (t.width() - drawing_w * scale) / 2.0;
    let offset_y = t.y_min + (t.height() - drawing_h * scale) / 2.0;

    let rx = offset_x + x * scale;
    let ry = offset_y + y * scale;

    (rx, ry, z.height(config))
}

/// Map an entire drawing (a list of A4 points) onto robot targets.
pub fn map_drawing(
    points: &[(f64, f64)],
    drawing_w: f64,
    drawing_h: f64,
    config: &MappingConfig,
) -> MappingResult {
    let scale = effective_scale(drawing_w, drawing_h, config);
    let targets = points
        .iter()
        .map(|&(x, y)| map_point(x, y, drawing_w, drawing_h, config, MoveZ::Draw))
        .collect();
    MappingResult { targets, scale }
}

/// Infer the A4 drawing bounding box from a list of strokes (A4 units).
pub fn drawing_bounding_box(strokes: &[crate::parser::Stroke]) -> Option<(f64, f64, f64, f64)> {
    let mut b: Option<(f64, f64, f64, f64)> = None;
    for stroke in strokes {
        if let Some((mnx, mny, mxx, mxy)) = stroke.bounds() {
            b = Some(match b {
                None => (mnx, mny, mxx, mxy),
                Some((lx, ly, hx, hy)) => (lx.min(mnx), ly.min(mny), hx.max(mxx), hy.max(mxy)),
            });
        }
    }
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> MappingConfig {
        MappingConfig {
            target: DrawingBounds {
                x_min: 150.0,
                x_max: 250.0,
                y_min: -50.0,
                y_max: 50.0,
            },
            z_draw: 80.0,
            z_travel: 86.0,
            scale: None,
        }
    }

    #[test]
    fn auto_scale_fits_a4_into_region() {
        // A4 portrait 210×297 into a 100×100 target → scale limited by width.
        let c = config();
        let s = c.target.fit_scale(210.0, 297.0);
        assert!(s < 1.0 && s > 0.0);
        // Mapped extremes must land inside the rectangle.
        let tl = map_point(0.0, 0.0, 210.0, 297.0, &c, MoveZ::Draw);
        let br = map_point(210.0, 297.0, 210.0, 297.0, &c, MoveZ::Draw);
        assert!(c.target.contains(tl.0, tl.1));
        assert!(c.target.contains(br.0, br.1));
        assert_eq!(tl.2, 80.0);
        assert_eq!(br.2, 80.0);
    }

    #[test]
    fn explicit_scale_overrides_fit() {
        let c = MappingConfig {
            scale: Some(0.5),
            ..config()
        };
        let (rx, ry, z) = map_point(10.0, 20.0, 210.0, 297.0, &c, MoveZ::Draw);
        // offset_x = 150 + (100 − 105)/2 = 147.5 → 147.5 + 10*0.5 = 152.5
        assert!((rx - 152.5).abs() < 1e-9);
        // offset_y = −50 + (100 − 148.5)/2 = −74.25 → −74.25 + 20*0.5 = −64.25
        assert!((ry - -64.25).abs() < 1e-9);
        assert_eq!(z, 80.0);
    }

    #[test]
    fn travel_move_uses_lift_height() {
        let c = config();
        let (_, _, z) = map_point(5.0, 5.0, 210.0, 297.0, &c, MoveZ::Travel);
        assert_eq!(z, 86.0);
    }

    #[test]
    fn auto_fit_keeps_mapped_points_contained() {
        let c = config();
        let corners = [(0.0, 0.0), (210.0, 0.0), (0.0, 297.0), (210.0, 297.0)];
        for (x, y) in corners {
            let (rx, ry, _) = map_point(x, y, 210.0, 297.0, &c, MoveZ::Draw);
            assert!(c.target.contains(rx, ry), "({x},{y}) → ({rx:.2},{ry:.2})");
        }
    }

    #[test]
    fn map_drawing_preserves_count_and_z() {
        let pts = vec![(0.0, 0.0), (210.0, 0.0), (0.0, 297.0)];
        let r = map_drawing(&pts, 210.0, 297.0, &config());
        assert_eq!(r.targets.len(), 3);
        assert!(r.targets.iter().all(|p| p.2 == 80.0));
        assert!(r.targets.iter().all(|p| config().target.contains(p.0, p.1)));
    }

    #[test]
    fn bounding_box_from_strokes() {
        use crate::parser::parse_gcode;
        let strokes = parse_gcode("G0 X0 Y0\nM3\nG1 X100 Y0\nG1 X100 Y200\nM5\n").unwrap();
        // Expect one stroke (the M3/M5 pair) with 3 points.
        assert_eq!(strokes.len(), 1);
        let bb = drawing_bounding_box(&strokes).unwrap();
        assert_eq!(bb, (0.0, 0.0, 100.0, 200.0));
    }

    #[test]
    fn empty_drawing_maps_to_empty() {
        let r = map_drawing(&[], 210.0, 297.0, &config());
        assert!(r.targets.is_empty());
    }
}