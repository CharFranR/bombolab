//! G-Code bridge: translates CIPRA-generated drawing G-code into
//! FABRI Creator robot movements.
//!
//! CIPRA emits a minimal geometric dialect (`G21 G90`, `G0 X Y`, `G1 X Y`,
//! pen-up `/ pen-down` `M3`/`M5`) over an A4 work area in millimetres. This
//! crate parses that dialect, maps the 2-D plane into the FABRI Creator's
//! reachable drawing workspace, validates every point with a dry-run IK solve,
//! and then executes the drawing on real hardware (or simulates).
//!
//! Responsibility separation:
//! - CIPRA              → image → trajectory (G-code)
//! - **this crate**     → trajectory → robot movements (IK orchestration)
//! - `bombolab-core`    → kinematics + serial hardware control

/// Parser for the CIPRA G-code dialect.
pub mod parser;

/// Reachable drawing workspace of the FABRI Creator.
pub mod workspace;

/// Mapping of A4 drawing coordinates onto the robot drawing plane.
pub mod mapper;

/// Dry-run IK reachability validation (strict mode).
pub mod validate;

/// Orchestrator: pipeline and hardware/simulation execution.
pub mod orchestrator;

pub use mapper::{MappingConfig, MappingResult, map_drawing, map_point};
pub use orchestrator::{
    BridgeError, DrawingPlan, GcodeBridge, MotionSink, ResolvedTarget, SerialSink, SimulationSink,
    TrajectoryFile, TrajectoryStep,
};
pub use validate::{DrawingValidator, ReachabilityFailure, Validation};
pub use workspace::DrawingBounds;