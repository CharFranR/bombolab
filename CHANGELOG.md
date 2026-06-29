# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- README.md with project documentation
- CONTRIBUTING.md with contribution guidelines
- CHANGELOG.md for tracking changes
- .editorconfig for consistent formatting

### Changed

- Translated all code, comments, and documentation to English
- Restructured project into Cargo workspace (bombolab-core, bombolab-gui)
- Connected UI layer with domain and kinematics modules
- Unified data types between UI and domain

### Fixed

- Eliminated duplicate JointType definitions between UI and domain
- Connected forward kinematics computation to UI display

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
