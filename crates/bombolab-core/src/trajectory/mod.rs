//! Trajectory execution: motion commands, the playback state machine and the
//! planner seam. Pure logic — no I/O and no timing source: the caller drives
//! `MotionPlayer::update(dt)` with a frame delta, so the same player serves
//! the simulator, WASM and real hardware unchanged.
//!
//! Two-level design (intention vs. movement):
//!
//! - Providers (app layer) emit [`MotionCommand`]s — geometric intent such as
//!   linear moves, pen lift and dwells. Future providers (G-code, SVG) may add
//!   higher-level variants (arcs) without breaking the player.
//! - The [`TrajectoryPlanner`] adapts commands to this robot before playback:
//!   speed limits, workspace fit, PenUp/PenDown materialization (today a z
//!   offset, tomorrow a gripper or solenoid), discretization of arcs. Slice 1
//!   is a pass-through that validates the seam.
//! - [`MotionPlayer`] executes the planned commands in wall-clock time via
//!   `update(dt)`.
//!
//! The player only produces cartesian targets. Converting them to joint
//! configurations (DrawingIK) and pacing the hardware (RobotController with
//! its servo interpolation) are separate layers.

/// Maximum dt accepted by the player (seconds). Guards against frame spikes
/// after a tab is backgrounded: a 5 s pause must not teleport the arm.
pub const MAX_DT: f64 = 0.1;

/// Position epsilon (mm): below this a move is considered arrived.
const EPS: f64 = 1e-6;

/// A motion command produced by a trajectory provider.
///
/// This is the *intent* level: `PenUp`/`PenDown` mean "stop/start drawing",
/// not "move the TCP by X" — the planner decides how to realize them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionCommand {
    /// Linear move to a cartesian target at the given speed (mm/s).
    MoveLinear {
        target: [f64; 3],
        speed: f64,
    },
    /// Tool-lift request; materialized by the planner.
    PenUp,
    /// Tool-down request; materialized by the planner.
    PenDown,
    /// Hold the current position for the given duration (seconds).
    Wait { duration: f64 },
}

/// Playback state of the [`MotionPlayer`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlayerState {
    /// No commands loaded or not started.
    Idle,
    /// Advancing through the commands.
    Running,
    /// Suspended; `resume()` continues from the current position.
    Paused,
    /// All commands consumed.
    Completed,
    /// Explicitly cancelled (user or error). Distinct from `Completed` so
    /// the UI can offer Replay vs. Resume semantics.
    Stopped,
}

/// Executes a planned command sequence in wall-clock time.
///
/// The player is a pure state machine: `update(dt)` advances the TCP target
/// according to each command's speed/duration. It does not know about IK,
/// serial or servos.
#[derive(Debug, Clone)]
pub struct MotionPlayer {
    state: PlayerState,
    commands: Vec<MotionCommand>,
    index: usize,
    current: [f64; 3],
    hold_remaining: f64,
}

impl MotionPlayer {
    /// Create a player for `commands`, starting from cartesian `start` (mm).
    pub fn new(commands: Vec<MotionCommand>, start: [f64; 3]) -> Self {
        Self {
            state: PlayerState::Idle,
            commands,
            index: 0,
            current: start,
            hold_remaining: 0.0,
        }
    }

    pub fn state(&self) -> PlayerState {
        self.state
    }

    /// Current commanded TCP position (mm) — the target the IK should solve.
    pub fn current_target(&self) -> [f64; 3] {
        self.current
    }

    /// Fraction of commands consumed, 0..=1 (for progress UI).
    pub fn progress(&self) -> f64 {
        if self.commands.is_empty() {
            return 1.0;
        }
        (self.index as f64 / self.commands.len() as f64).min(1.0)
    }

    /// Start (or restart) playback from the beginning.
    pub fn play(&mut self) {
        self.index = 0;
        self.hold_remaining = 0.0;
        self.state = PlayerState::Running;
    }

    /// Suspend playback; position is frozen at the last commanded target.
    pub fn pause(&mut self) {
        if self.state == PlayerState::Running {
            self.state = PlayerState::Paused;
        }
    }

    /// Resume a paused playback.
    pub fn resume(&mut self) {
        if self.state == PlayerState::Paused {
            self.state = PlayerState::Running;
        }
    }

    /// Cancel playback. The TCP target stays where it was; the trajectory is
    /// not resumed by `resume()` (use `play()` to restart).
    pub fn stop(&mut self) {
        self.state = PlayerState::Stopped;
    }

    /// Advance the trajectory by `dt` seconds. No-op unless `Running`.
    ///
    /// A single call can consume several short commands; time is split
    /// proportionally so the TCP never overshoots a segment.
    pub fn update(&mut self, dt: f64) {
        if self.state != PlayerState::Running || dt <= 0.0 {
            return;
        }
        let mut remaining = dt.min(MAX_DT);

        while remaining > 0.0 {
            let Some(cmd) = self.commands.get(self.index) else {
                self.state = PlayerState::Completed;
                return;
            };

            match cmd {
                MotionCommand::MoveLinear { target, speed } => {
                    let dx = target[0] - self.current[0];
                    let dy = target[1] - self.current[1];
                    let dz = target[2] - self.current[2];
                    let dist = (dx * dx + dy * dy + dz * dz).sqrt();

                    // Already there (or an instant move): consume no time.
                    if dist <= EPS || *speed <= 0.0 {
                        self.current = *target;
                        self.index += 1;
                        continue;
                    }

                    let step = speed * remaining;
                    if step >= dist {
                        // Arrive: spend exactly the time this segment needs.
                        self.current = *target;
                        self.index += 1;
                        remaining = (remaining - dist / speed).max(0.0);
                    } else {
                        let t = step / dist;
                        self.current = [
                            self.current[0] + dx * t,
                            self.current[1] + dy * t,
                            self.current[2] + dz * t,
                        ];
                        remaining = 0.0;
                    }
                }
                // Pass-through in slice 1: the planner materializes these into
                // concrete motion before playback, so they consume no time.
                MotionCommand::PenUp | MotionCommand::PenDown => {
                    self.index += 1;
                }
                MotionCommand::Wait { duration } => {
                    if self.hold_remaining <= 0.0 {
                        self.hold_remaining = *duration;
                    }
                    if remaining >= self.hold_remaining {
                        remaining -= self.hold_remaining;
                        self.hold_remaining = 0.0;
                        self.index += 1;
                    } else {
                        self.hold_remaining -= remaining;
                        remaining = 0.0;
                    }
                }
            }
        }

        if self.index >= self.commands.len() {
            self.state = PlayerState::Completed;
        }
    }
}

/// Adapts a trajectory to this robot before playback.
///
/// Slice 1 is a pass-through that establishes the seam. Future transforms:
/// speed/acceleration limits, workspace fit (scale/translate/rotate),
/// PenUp/PenDown materialization (z offset vs. gripper vs. solenoid),
/// arc discretization, marker-thickness compensation.
#[derive(Debug, Clone, Copy, Default)]
pub struct TrajectoryPlanner;

impl TrajectoryPlanner {
    pub fn new() -> Self {
        Self
    }

    /// Plan `commands` into the concrete sequence the player will execute.
    pub fn plan(&self, commands: Vec<MotionCommand>) -> Vec<MotionCommand> {
        commands
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const Z: f64 = 80.0;

    fn square(speed: f64) -> Vec<MotionCommand> {
        vec![
            MotionCommand::MoveLinear { target: [175.0, -25.0, Z], speed },
            MotionCommand::MoveLinear { target: [225.0, -25.0, Z], speed },
            MotionCommand::MoveLinear { target: [225.0, 25.0, Z], speed },
            MotionCommand::MoveLinear { target: [175.0, 25.0, Z], speed },
        ]
    }

    #[test]
    fn moves_along_the_first_segment_at_speed() {
        let mut p = MotionPlayer::new(square(100.0), [175.0, -25.0, Z]);
        p.play();
        // 0.25 s at 100 mm/s = 25 mm along a 50 mm side (frame dt below MAX_DT)
        for _ in 0..5 {
            p.update(0.05);
        }
        let t = p.current_target();
        assert!((t[0] - 200.0).abs() < 1e-6, "x should be 200, got {}", t[0]);
        assert!((t[1] + 25.0).abs() < 1e-6, "y should stay -25, got {}", t[1]);
        assert_eq!(p.state(), PlayerState::Running);
    }

    #[test]
    fn completes_after_all_segments() {
        let mut p = MotionPlayer::new(square(50.0), [175.0, -25.0, Z]);
        p.play();
        // 4 sides of 50 mm at 50 mm/s = 4 s total
        for _ in 0..80 {
            p.update(0.05);
        }
        assert_eq!(p.state(), PlayerState::Completed);
        assert!((p.progress() - 1.0).abs() < 1e-9);
        let t = p.current_target();
        assert!((t[0] - 175.0).abs() < 1e-6);
        assert!((t[1] - 25.0).abs() < 1e-6);
    }

    #[test]
    fn single_update_can_consume_short_segments() {
        let mut p = MotionPlayer::new(square(1000.0), [175.0, -25.0, Z]);
        p.play();
        // One frame of 0.1 s at 1000 mm/s covers 100 mm -> two full 50 mm sides
        p.update(0.1);
        assert_eq!(p.state(), PlayerState::Running);
        let t = p.current_target();
        assert!((t[0] - 225.0).abs() < 1e-6, "x should be 225, got {}", t[0]);
        assert!((t[1] - 25.0).abs() < 1e-6, "y should be 25, got {}", t[1]);
    }

    #[test]
    fn pause_freezes_and_resume_continues() {
        let mut p = MotionPlayer::new(square(100.0), [175.0, -25.0, Z]);
        p.play();
        p.update(0.1);
        let frozen = p.current_target();
        p.pause();
        p.update(1.0);
        assert_eq!(p.current_target(), frozen);
        p.resume();
        p.update(0.1);
        let t = p.current_target();
        assert!(
            t[0] > frozen[0] + 1.0,
            "should have advanced, frozen={frozen:?} now={t:?}"
        );
    }

    #[test]
    fn stop_is_distinct_from_completed() {
        let mut p = MotionPlayer::new(square(100.0), [175.0, -25.0, Z]);
        p.play();
        p.update(0.1);
        p.stop();
        assert_eq!(p.state(), PlayerState::Stopped);
        p.update(1.0);
        assert_eq!(p.state(), PlayerState::Stopped);
        p.resume();
        assert_eq!(p.state(), PlayerState::Stopped); // stop is not resumable
        p.play();
        assert_eq!(p.state(), PlayerState::Running);
    }

    #[test]
    fn wait_holds_position_then_advances() {
        let mut p = MotionPlayer::new(
            vec![
                MotionCommand::MoveLinear { target: [200.0, 0.0, Z], speed: 100.0 },
                MotionCommand::Wait { duration: 1.0 },
                MotionCommand::MoveLinear { target: [220.0, 0.0, Z], speed: 100.0 },
            ],
            [150.0, 0.0, Z],
        );
        p.play();
        // First move: 50 mm at 100 mm/s = 0.5 s
        for _ in 0..10 {
            p.update(0.05);
        }
        let at_wait = p.current_target();
        assert!((at_wait[0] - 200.0).abs() < 1e-6, "x should be 200, got {}", at_wait[0]);
        // Inside the wait: frozen
        for _ in 0..8 {
            p.update(0.05); // 0.4 s of the 1.0 s wait
        }
        assert_eq!(p.current_target(), at_wait);
        // Finish the wait (0.6 s more) and the last move (20 mm at 100 = 0.2 s)
        for _ in 0..16 {
            p.update(0.05);
        }
        assert_eq!(p.state(), PlayerState::Completed);
        let t = p.current_target();
        assert!((t[0] - 220.0).abs() < 1e-6);
    }

    #[test]
    fn pen_commands_pass_through_without_time() {
        let mut p = MotionPlayer::new(
            vec![
                MotionCommand::PenDown,
                MotionCommand::MoveLinear { target: [200.0, 0.0, Z], speed: 50.0 },
                MotionCommand::PenUp,
            ],
            [150.0, 0.0, Z],
        );
        p.play();
        // Pen-down is instant; move is 50 mm at 50 mm/s = 1.0 s
        for _ in 0..20 {
            p.update(0.05);
        }
        let t = p.current_target();
        assert!((t[0] - 200.0).abs() < 1e-6, "x should be 200, got {}", t[0]);
        // One more frame: pen-up is instant -> Completed
        p.update(0.05);
        assert_eq!(p.state(), PlayerState::Completed);
    }

    #[test]
    fn large_dt_is_clamped() {
        let mut p = MotionPlayer::new(square(100.0), [175.0, -25.0, Z]);
        p.play();
        p.update(5.0); // a 5 s frame spike must not teleport: max 0.1 s advance
        let t = p.current_target();
        assert!((t[0] - 185.0).abs() < 1e-6, "x should be 185 (clamped), got {}", t[0]);
    }

    #[test]
    fn planner_is_a_pass_through() {
        let planner = TrajectoryPlanner::new();
        let cmds = square(40.0);
        let planned = planner.plan(cmds.clone());
        assert_eq!(planned, cmds);
    }
}
