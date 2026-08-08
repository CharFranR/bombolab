//! First-class tool pose: an immutable `ToolFrame` with named preset factories.
//!
//! The tool pose is the transform from the last joint frame to the tool tip.
//! Solver signatures keep taking `&Iso3`; callers pass `tool_frame.pose()`.
//!
//! # Roll-only rotation invariant
//!
//! The constrained drawing IK of [ADR-0001] is derived for a tool mounted as a
//! pure translation along the last joint X-axis. Only **roll about the tool
//! X-axis** preserves the drawing variety `M = { q4 = 0, q5 = −(q2+q3) }` and
//! the reduced Jacobian `Jᵣ = [ J₁ , J₂ − J₅ , J₃ − J₅ ]` of ADR-0001; any
//! rotation about Y or Z invalidates the derivation (see
//! [ADR-0004](../../../book/src/adr/0004-tool-frame-roll-only-invariant.md)).
//! Every preset therefore uses identity rotation with a translation-only
//! offset along X.
//!
//! [ADR-0001]: ../../../book/src/adr/0001-constrained-drawing-ik.md

use crate::math::Iso3;

/// Default marker holder length in mm. Matches the legacy `tool_transform()`;
/// the marker remains the default tool.
pub const DEFAULT_MARKER_LENGTH: f64 = 75.0;

/// Pen tip length in mm. Provisional — pending physical validation.
pub const PEN_LENGTH: f64 = 90.0;

/// Gripper jaw center length in mm. Provisional — pending physical validation.
pub const GRIPPER_LENGTH: f64 = 45.0;

/// Immutable tool pose attached to the last joint frame.
///
/// Built once through [`ToolFrame::new`] or a preset factory; fields are
/// private and there are no mutating accessors, so a frame cannot change after
/// construction. Read it through [`ToolFrame::pose`] and [`ToolFrame::name`].
pub struct ToolFrame {
    pose: Iso3,
    name: String,
}

impl ToolFrame {
    /// Builds a frame from an explicit pose and name.
    pub fn new(pose: Iso3, name: String) -> Self {
        Self { pose, name }
    }

    /// The tool pose (rotation + translation) relative to the last joint frame.
    pub fn pose(&self) -> &Iso3 {
        &self.pose
    }

    /// The tool name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Marker mounted perpendicular to the last joint axis, at the default
    /// length [`DEFAULT_MARKER_LENGTH`].
    pub fn marker_perpendicular() -> Self {
        Self::marker_perpendicular_len(DEFAULT_MARKER_LENGTH)
    }

    /// Marker mounted perpendicular to the last joint axis, at an explicit
    /// length in mm.
    pub fn marker_perpendicular_len(len: f64) -> Self {
        Self::new(
            Iso3::translation(len, 0.0, 0.0),
            "marker_perpendicular".to_string(),
        )
    }

    /// Pen tip at [`PEN_LENGTH`] mm along the tool X-axis.
    pub fn pen() -> Self {
        Self::new(Iso3::translation(PEN_LENGTH, 0.0, 0.0), "pen".to_string())
    }

    /// Gripper jaw center at [`GRIPPER_LENGTH`] mm along the tool X-axis.
    pub fn gripper() -> Self {
        Self::new(
            Iso3::translation(GRIPPER_LENGTH, 0.0, 0.0),
            "gripper".to_string(),
        )
    }
}

#[cfg(test)]
#[path = "tool_frame_tests.rs"]
mod tool_frame_tests;
