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

### Known Technical Debt

- **DH → Three.js representation is a reflection (C-3)** — `framePose` in
  `web/src/renderers/types.ts` maps `(x, y, z)_DH → (x, z, y)_three`, a
  transformation with `det = −1` (improper). The correct Z-up→Y-up rotation
  is `(x, −z, y)` (`det = +1`). The rendered robot is the MIRROR IMAGE of
  the physical robot: non-vertical joint rotations (J2–J5) appear inverted
  and asymmetric parts appear on the wrong side. Internally consistent (all
  layers use the same mapping and calibration was tuned on the mirrored
  render), and the FABRI's near-symmetry hides it in practice — but a
  camera/computer-vision integration or any asymmetric part would expose it
  directly. Fix is one sign flip (`te[6] = -f[6]` or `te[14] = -f[7]`) but
  requires full visual re-calibration of the 11 STL entries. **Deferred by
  maintainer decision**: correct when validated against the physical robot
  or when vision integration requires it.
- **Physical pin table unverified** — servo pin mapping (J1→A1, J2→A0,
  J3→A2, J4→A4, J5→13, Gripper→A5) was unified to the firmware as the
  source of truth; still needs validation against the physical wiring
  (marked `VERIFICAR contra el cableado físico real` in
  `communication/mod.rs` and firmware `main.cpp`).
- **Dynamics simplifications** — `dynamics.rs` models COM at frame origins,
  masses/inertias are estimates (PETG 25% + MG996R/MG90S specs), and no
  Coriolis/centrifugal term exists. Acceptable for educational use;
  revisit if used for control.

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
