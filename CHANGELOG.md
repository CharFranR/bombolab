# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Visual fidelity modes**: Low/High fidelity toggle with STL 3D model rendering
- `RobotRenderer` abstraction with `SimpleRobotScene` (primitives) and `StlRobotScene` (STL meshes)
- Real-time calibration panel with step buttons, numeric inputs, save/upload/reload
- TransformControls gizmo for interactive STL positioning (translate/rotate)
- Debug visualization: Joint Frames, STL Origins, Calibration Axes toggles
- Global `stlScale` slider for uniform STL model scaling
- Workspace point cloud rendering in high fidelity mode
- IK target ball in high fidelity mode
- Error boundary for graceful STL load failure
- Diagnostic logs for STL loading and transform application

### Changed

- **DH parameters corrected** to match physical robot measurements:
  - Base → shoulder: d₁ = 85mm (was 95)
  - Shoulder → elbow: a₂ = 120mm (was 162)
  - Elbow → wrist roll: a₃ = 90mm (was 111)
  - Wrist roll offset: d₄ = 15mm (was 0)
- **Twist FK formula** now includes `d` parameter (was hardcoded to 0), applied on DH Y axis
- Gripper jaw animation order: `world × jawM × scale × cal` (FK local space, matches SimpleRobotScene)
- Removed low-fi skeleton overlay from high fidelity mode

### Fixed

- STL-to-joint mapping corrected per user-identified physical parts
- Gripper jaw directions corrected for center-closing behavior
- `dh_params()` for Twist joints now returns actual `d` value instead of hardcoded 0

## [0.1.0] - 2026-01-01

### Added

- Initial project setup
- Domain model: Joint, JointType, DHParams, Segment, Robot
- Forward kinematics computation with DH parameters
- Transformation matrix construction from DH parameters
- Custom error handling with Error enum
- CLI-based interactive robot builder and tester
- GUI with egui/eframe for desktop application
- DH parameter editor with sliders
- Transformation matrix display in details popup
- Robot creation with 2-6 DOF
- Navigation between views (Main, RobotList, RobotEditor, Movements)

### Known Issues

- 3D Viewport is a placeholder (not implemented)
- UI defines separate data types from domain module
- Forward kinematics not connected to UI
- No unit tests
- Documentation in Spanish (being translated)
