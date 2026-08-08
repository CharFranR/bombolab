//! Unit tests for `tool_frame`.

use super::*;
use crate::math::Iso3;

fn iso3_bits(iso: &Iso3) -> Vec<u64> {
    let m = iso.to_matrix();
    (0..3)
        .flat_map(|r| (0..4).map(move |c| m[(r, c)].to_bits()))
        .collect()
}

#[test]
fn test_default_marker_byte_identical_to_legacy_pose() {
    let frame = ToolFrame::marker_perpendicular();
    let expected = Iso3::translation(75.0, 0.0, 0.0);
    assert_eq!(iso3_bits(frame.pose()), iso3_bits(&expected));
}

#[test]
fn test_default_marker_name() {
    assert_eq!(
        ToolFrame::marker_perpendicular().name(),
        "marker_perpendicular"
    );
}

#[test]
fn test_marker_custom_length_pose() {
    let frame = ToolFrame::marker_perpendicular_len(100.0);
    let m = frame.pose().to_matrix();
    assert_eq!(m[(0, 3)], 100.0);
    assert_eq!(m[(1, 3)], 0.0);
    assert_eq!(m[(2, 3)], 0.0);
    assert_eq!(m[(0, 0)], 1.0);
    assert_eq!(m[(1, 1)], 1.0);
    assert_eq!(m[(2, 2)], 1.0);
    assert_eq!(m[(0, 1)], 0.0);
    assert_eq!(m[(0, 2)], 0.0);
    assert_eq!(m[(1, 0)], 0.0);
    assert_eq!(m[(1, 2)], 0.0);
    assert_eq!(m[(2, 0)], 0.0);
    assert_eq!(m[(2, 1)], 0.0);
}

#[test]
fn test_pen_preset() {
    let frame = ToolFrame::pen();
    assert_eq!(frame.name(), "pen");
    let m = frame.pose().to_matrix();
    assert_eq!(m[(0, 3)], 90.0);
    assert_eq!(m[(1, 3)], 0.0);
    assert_eq!(m[(2, 3)], 0.0);
}

#[test]
fn test_gripper_preset() {
    let frame = ToolFrame::gripper();
    assert_eq!(frame.name(), "gripper");
    let m = frame.pose().to_matrix();
    assert_eq!(m[(0, 3)], 45.0);
    assert_eq!(m[(1, 3)], 0.0);
    assert_eq!(m[(2, 3)], 0.0);
}

#[test]
fn test_pose_and_name_stable_across_reads() {
    let frame = ToolFrame::marker_perpendicular_len(123.5);
    assert_eq!(iso3_bits(frame.pose()), iso3_bits(frame.pose()));
    assert_eq!(frame.name(), frame.name());
}

#[test]
fn test_new_accessors_match_construction() {
    let pose = Iso3::translation(10.0, 20.0, 30.0);
    let frame = ToolFrame::new(pose, "custom".to_string());
    assert_eq!(frame.name(), "custom");
    assert_eq!(iso3_bits(frame.pose()), iso3_bits(&pose));
}

#[test]
#[allow(deprecated)]
fn test_deprecated_alias_byte_identical_to_factory() {
    let legacy = crate::robot::tool_transform();
    let factory = *ToolFrame::marker_perpendicular().pose();
    assert_eq!(iso3_bits(&legacy), iso3_bits(&factory));
}
