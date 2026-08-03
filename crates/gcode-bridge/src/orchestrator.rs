//! Orchestrator: turns a G-code document into robot movements and executes it.
//!
//! This wires together the pipeline:
//!
//! ```text
//! gcode ─ parser → strokes ─ mapper → robot targets ─ IK resolve → sink.send
//! ```
//!
//! The executor is decoupled from hardware through the [`MotionSink`] trait:
//! the CLI supplies either a real serial sink (Arduino Nano) or a simulation
//! sink that just logs the servo commands, so the whole drawing can be verified
//! without a robot attached.

use bombolab_core::communication::ArduinoNano;
use bombolab_core::kinematics::solve_drawing_plane_ik;
use bombolab_core::math::Iso3;
use bombolab_core::robot::Robot;
use bombolab_core::{base_transform, fabri_creator, tool_transform, IkSolver, ServoCommand};

use crate::mapper::{MappingConfig, MoveZ, map_point};
use crate::parser::parse_gcode;

/// A sink that consumes resolved servo commands (degrees + gripper).
///
/// [`ServoCommand::to_wire`] produces the `"a1,a2,a3,a4,a5,g\n"` wire format
/// that the Arduino firmware expects.
pub trait MotionSink {
    /// Send one command to the robot. Implementations may block for verification.
    fn send(&mut self, cmd: &ServoCommand) -> Result<(), String>;
}

/// Sink that sends commands to a real Arduino Nano over serial.
pub struct SerialSink {
    pub arduino: ArduinoNano,
}

impl MotionSink for SerialSink {
    fn send(&mut self, cmd: &ServoCommand) -> Result<(), String> {
        self.arduino.send_and_verify(cmd).map_err(|e| e.to_string())
    }
}

/// Sink that logs resolved commands and never touches hardware (for dry runs).
#[derive(Default)]
pub struct SimulationSink {
    pub commands: Vec<ServoCommand>,
}

impl MotionSink for SimulationSink {
    fn send(&mut self, cmd: &ServoCommand) -> Result<(), String> {
        self.commands.push(*cmd);
        Ok(())
    }
}

/// One resolved robot target: the `(x, y, z)` point and the IK angles for it.
#[derive(Debug, Clone)]
pub struct ResolvedTarget {
    pub target: (f64, f64, f64),
    pub q: [f64; 5],
}

/// The result of turning a G-code document into an executable plan.
///
/// Each `strokes[i]` is a connected drawing stroke: travel + draw points, in
/// draw order, with their resolved joint angles.
#[derive(Debug, Clone, Default)]
pub struct DrawingPlan {
    /// Effective mapping scale applied (auto-scaling), for reporting.
    pub scale: f64,
    /// Strokes, each a list of resolved targets (travel first, then draw).
    pub strokes: Vec<Vec<ResolvedTarget>>,
}

/// One step of a trajectory consumed by the web viewer.
///
/// `q` are joint angles in **radians**, already resolved by IK. The web app
/// resolves each step's tool-tip position with forward kinematics and renders
/// the polyline, so no Cartesian data needs to be embedded here.
#[derive(Debug, Clone, serde::Serialize)]
pub struct TrajectoryStep {
    pub q: [f64; 5],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gripper: Option<u8>,
}

/// The full serializable trajectory document: `{ "steps": [ ... ] }`.
#[derive(Debug, Clone, serde::Serialize)]
pub struct TrajectoryFile {
    pub steps: Vec<TrajectoryStep>,
}

impl DrawingPlan {
    /// Total number of servo commands the plan would emit.
    pub fn target_count(&self) -> usize {
        self.strokes.iter().map(|s| s.len()).sum()
    }

    /// Flatten every stroke into an ordered list of trajectory steps.
    ///
    /// `gripper` is threaded into every step (when given) so the web viewer can
    /// reflect pen-engaging values; it is omitted when `None`.
    pub fn to_trajectory(&self, gripper: Option<u8>) -> TrajectoryFile {
        let steps = self
            .strokes
            .iter()
            .flat_map(|s| s.iter())
            .map(|r| TrajectoryStep {
                q: r.q,
                gripper,
            })
            .collect();
        TrajectoryFile { steps }
    }

    /// Serialize the plan to the web-viewer JSON document.
    pub fn to_trajectory_json(&self, gripper: Option<u8>) -> String {
        serde_json::to_string_pretty(&self.to_trajectory(gripper))
            .expect("trajectory JSON serialization cannot fail")
    }
}

/// Errors raised while building or executing a drawing plan.
#[derive(Debug)]
pub enum BridgeError {
    Parse(crate::parser::ParseError),
    Geometry(String),
    Unreachable {
        index: usize,
        target: (f64, f64, f64),
        reason: String,
    },
    ServoOutOfRange {
        index: usize,
        joint: usize,
        servo_deg: f64,
    },
    Execution(String),
}

impl std::fmt::Display for BridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BridgeError::Parse(e) => write!(f, "parse error: {e}"),
            BridgeError::Geometry(m) => write!(f, "mapping error: {m}"),
            BridgeError::Unreachable {
                target, reason, ..
            } => write!(f, "unreachable target {target:?}: {reason}"),
            BridgeError::ServoOutOfRange {
                index,
                joint,
                servo_deg,
            } => write!(
                f,
                "target #{index}: joint J{} resolved to {servo_deg:.2}° servo angle outside [5,175]°",
                joint + 1
            ),
            BridgeError::Execution(m) => write!(f, "execution error: {m}"),
        }
    }
}

impl std::error::Error for BridgeError {}

/// The bridge itself: parses, maps and resolves a drawing to servo commands.
pub struct GcodeBridge {
    config: MappingConfig,
    solver: IkSolver,
    robot: Robot,
    base: Iso3,
    tool: Iso3,
}

impl GcodeBridge {
    pub fn new(config: MappingConfig) -> Self {
        Self {
            config,
            solver: IkSolver::new(200, 1.0, 0.05, 0.5),
            robot: fabri_creator(),
            base: base_transform(),
            tool: tool_transform(),
        }
    }

    /// Parse, map and resolve every stroke into an executable plan.
    ///
    /// Applies **strict** reachability validation: any target the arm cannot
    /// reach returns [`BridgeError::Unreachable`] instead of being skipped.
    pub fn plan(&self, gcode: &str) -> Result<DrawingPlan, BridgeError> {
        let strokes = parse_gcode(gcode).map_err(BridgeError::Parse)?;
        let bb = crate::mapper::drawing_bounding_box(&strokes)
            .ok_or_else(|| BridgeError::Geometry("no stroke points found".into()))?;
        let (w, h) = (bb.2 - bb.0, bb.3 - bb.1);
        let scale = self.config.target.fit_scale(w, h);

        let mut plan = DrawingPlan {
            scale,
            ..Default::default()
        };

        for stroke in &strokes {
            let first = &stroke.points[0];
            let travel = map_point(first.0, first.1, w, h, &self.config, MoveZ::Travel);

            let mut resolved: Vec<ResolvedTarget> = Vec::new();
            let travel_target = travel;
            let r = self
                .resolve(travel_target)
                .map_err(|e| extract(e, 0))?;
            self.servo_check(&r.q)
                .map_err(|e| with_index(e, 0))?;
            resolved.push(r);

            for (i, &(x, y)) in stroke.points.iter().enumerate() {
                let t = map_point(x, y, w, h, &self.config, MoveZ::Draw);
                let r = self.resolve(t).map_err(|e| extract(e, i + 1))?;
                self.servo_check(&r.q).map_err(|e| with_index(e, i + 1))?;
                resolved.push(r);
            }
            plan.strokes.push(resolved);
        }

        Ok(plan)
    }

    /// Resolve a single robot target with IK (strict: error if unreachable).
    fn resolve(&self, target: (f64, f64, f64)) -> Result<ResolvedTarget, BridgeError> {
        let q_init = [0.0_f64; 5];
        match solve_drawing_plane_ik(
            &self.solver, &[target.0, target.1, target.2], &q_init,
            &self.robot, &self.base, &self.tool,
        ) {
            Ok(q) => Ok(ResolvedTarget { target, q }),
            Err(e) => Err(BridgeError::Unreachable {
                index: 0,
                target,
                reason: e.to_string(),
            }),
        }
    }

    /// Validate that the resolved joints map to servo angles within the
    /// physical range. Returns the index of the offending joint (1-based J).
    fn servo_check(&self, q: &[f64; 5]) -> Result<(), BridgeError> {
        let servo = self.robot.q_to_servo(q);
        for (i, v) in servo.iter().enumerate().take(5) {
            let deg = v.to_degrees();
            if !deg.is_finite() || !(5.0..=175.0).contains(&deg) {
                return Err(BridgeError::ServoOutOfRange {
                    index: 0,
                    joint: i,
                    servo_deg: deg,
                });
            }
        }
        Ok(())
    }

    /// Execute a plan through a sink, returning the number of commands sent.
    pub fn execute<S: MotionSink>(
        &self,
        plan: &DrawingPlan,
        sink: &mut S,
        gripper: u8,
    ) -> Result<usize, BridgeError> {
        let mut count = 0;
        for stroke in &plan.strokes {
            for resolved in stroke {
                self.servo_check(&resolved.q)?;
                let servo = self.robot.q_to_servo(&resolved.q);
                let mut joints = [0.0_f64; 5];
                for (i, v) in servo.iter().enumerate().take(5) {
                    joints[i] = v.to_degrees();
                }
                let cmd = ServoCommand::new(joints, gripper)
                    .map_err(|e| BridgeError::Execution(e.to_string()))?;
                sink.send(&cmd).map_err(BridgeError::Execution)?;
                count += 1;
            }
        }
        Ok(count)
    }
}

/// Collapse a resolve error preserving the failing target index.
fn extract(e: BridgeError, index: usize) -> BridgeError {
    match e {
        BridgeError::Unreachable { target, reason, .. } => BridgeError::Unreachable {
            index,
            target,
            reason,
        },
        other => other,
    }
}

/// Stamp the failing target index onto a servo-range error.
fn with_index(e: BridgeError, index: usize) -> BridgeError {
    match e {
        BridgeError::ServoOutOfRange { joint, servo_deg, .. } => BridgeError::ServoOutOfRange {
            index,
            joint,
            servo_deg,
        },
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bridge() -> GcodeBridge {
        GcodeBridge::new(MappingConfig::default())
    }

    const SQUARE: &str = "G21 G90\nG0 X10.00 Y10.00\nM3\nG1 X50.00 Y50.00\nM5\n";

    #[test]
    fn plans_simple_drawing() {
        let b = bridge();
        let plan = b.plan(SQUARE).expect("plan");
        // 1 travel + 2 drawing points.
        assert_eq!(plan.target_count(), 3);
        assert_eq!(plan.strokes.len(), 1);
    }

    #[test]
    fn execution_against_simulation_sink() {
        let b = bridge();
        let plan = b.plan(SQUARE).unwrap();
        let mut sink = SimulationSink::default();
        let n = b.execute(&plan, &mut sink, 90).unwrap();
        assert_eq!(n, 3);
        assert!(!sink.commands.is_empty());
    }

    #[test]
    fn execution_emits_valid_wire() {
        let b = bridge();
        let plan = b.plan(SQUARE).unwrap();
        let mut sink = SimulationSink::default();
        b.execute(&plan, &mut sink, 90).unwrap();
        for cmd in &sink.commands {
            assert!(cmd.joints.iter().all(|&j| (5.0..=175.0).contains(&j)));
        }
    }

    #[test]
    fn plan_rejects_out_of_workspace() {
        let b = bridge();
        let bad = "G0 X10000 Y10000\nM3\nG1 X10500 Y10500\nM5\n";
        assert!(b.plan(bad).is_err());
    }

    #[test]
    fn servo_check_accepts_in_range_pose() {
        let b = bridge();
        // Home pose (all q = 0) maps to the physical home servo angles (in range).
        let home = [0.0_f64; 5];
        assert!(b.servo_check(&home).is_ok());
    }

    #[test]
    fn servo_command_rejects_out_of_range_pose() {
        let b = bridge();
        // A pose that forces a servo angle far out of range is rejected by
        // ServoCommand (strict), not clamped into range.
        let out_of_range = b.robot.q_to_servo(&[0.0, 0.0, 3.0, 0.0, 0.0]);
        let joints: [f64; 5] = std::array::from_fn(|i| out_of_range[i].to_degrees());
        assert!(bombolab_core::ServoCommand::new(joints, 90).is_err());
    }

    #[test]
    fn to_trajectory_flattens_strokes_with_gripper() {
        let b = bridge();
        let plan = b.plan(SQUARE).unwrap();
        let traj = plan.to_trajectory(Some(90));
        assert_eq!(traj.steps.len(), plan.target_count());
        assert!(traj.steps.iter().all(|s| s.gripper == Some(90)));
    }

    #[test]
    fn to_trajectory_omits_gripper_when_none() {
        let b = bridge();
        let plan = b.plan(SQUARE).unwrap();
        let traj = plan.to_trajectory(None);
        assert!(traj.steps.iter().all(|s| s.gripper.is_none()));
    }

    #[test]
    fn to_trajectory_json_is_valid_and_matches_shape() {
        let b = bridge();
        let plan = b.plan(SQUARE).unwrap();
        let json = plan.to_trajectory_json(Some(90));
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert!(parsed.get("steps").unwrap().is_array());
        assert_eq!(parsed["steps"].as_array().unwrap().len(), plan.target_count());
    }
}