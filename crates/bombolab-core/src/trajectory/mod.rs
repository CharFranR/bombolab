pub const MAX_DT: f64 = 0.1;

const EPS: f64 = 1e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionCommand {
    MoveLinear { target: [f64; 3], speed: f64 },

    PenUp,

    PenDown,

    Wait { duration: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlayerState {
    Idle,

    Running,

    Paused,

    Completed,

    Stopped,
}

#[derive(Debug, Clone)]
pub struct MotionPlayer {
    state: PlayerState,
    commands: Vec<MotionCommand>,
    index: usize,
    current: [f64; 3],
    hold_remaining: f64,
}

impl MotionPlayer {
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

    pub fn current_target(&self) -> [f64; 3] {
        self.current
    }

    pub fn progress(&self) -> f64 {
        if self.commands.is_empty() {
            return 1.0;
        }
        (self.index as f64 / self.commands.len() as f64).min(1.0)
    }

    pub fn play(&mut self) {
        self.index = 0;
        self.hold_remaining = 0.0;
        self.state = PlayerState::Running;
    }

    pub fn pause(&mut self) {
        if self.state == PlayerState::Running {
            self.state = PlayerState::Paused;
        }
    }

    pub fn resume(&mut self) {
        if self.state == PlayerState::Paused {
            self.state = PlayerState::Running;
        }
    }

    pub fn stop(&mut self) {
        self.state = PlayerState::Stopped;
    }

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

                    if dist <= EPS || *speed <= 0.0 {
                        self.current = *target;
                        self.index += 1;
                        continue;
                    }

                    let step = speed * remaining;
                    if step >= dist {
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

#[derive(Debug, Clone, Copy, Default)]
pub struct TrajectoryPlanner;

impl TrajectoryPlanner {
    pub fn new() -> Self {
        Self
    }

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
            MotionCommand::MoveLinear {
                target: [175.0, -25.0, Z],
                speed,
            },
            MotionCommand::MoveLinear {
                target: [225.0, -25.0, Z],
                speed,
            },
            MotionCommand::MoveLinear {
                target: [225.0, 25.0, Z],
                speed,
            },
            MotionCommand::MoveLinear {
                target: [175.0, 25.0, Z],
                speed,
            },
        ]
    }

    #[test]
    fn moves_along_the_first_segment_at_speed() {
        let mut p = MotionPlayer::new(square(100.0), [175.0, -25.0, Z]);
        p.play();

        for _ in 0..5 {
            p.update(0.05);
        }
        let t = p.current_target();
        assert!((t[0] - 200.0).abs() < 1e-6, "x should be 200, got {}", t[0]);
        assert!(
            (t[1] + 25.0).abs() < 1e-6,
            "y should stay -25, got {}",
            t[1]
        );
        assert_eq!(p.state(), PlayerState::Running);
    }

    #[test]
    fn completes_after_all_segments() {
        let mut p = MotionPlayer::new(square(50.0), [175.0, -25.0, Z]);
        p.play();

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
        assert_eq!(p.state(), PlayerState::Stopped);
        p.play();
        assert_eq!(p.state(), PlayerState::Running);
    }

    #[test]
    fn wait_holds_position_then_advances() {
        let mut p = MotionPlayer::new(
            vec![
                MotionCommand::MoveLinear {
                    target: [200.0, 0.0, Z],
                    speed: 100.0,
                },
                MotionCommand::Wait { duration: 1.0 },
                MotionCommand::MoveLinear {
                    target: [220.0, 0.0, Z],
                    speed: 100.0,
                },
            ],
            [150.0, 0.0, Z],
        );
        p.play();

        for _ in 0..10 {
            p.update(0.05);
        }
        let at_wait = p.current_target();
        assert!(
            (at_wait[0] - 200.0).abs() < 1e-6,
            "x should be 200, got {}",
            at_wait[0]
        );

        for _ in 0..8 {
            p.update(0.05);
        }
        assert_eq!(p.current_target(), at_wait);

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
                MotionCommand::MoveLinear {
                    target: [200.0, 0.0, Z],
                    speed: 50.0,
                },
                MotionCommand::PenUp,
            ],
            [150.0, 0.0, Z],
        );
        p.play();

        for _ in 0..20 {
            p.update(0.05);
        }
        let t = p.current_target();
        assert!((t[0] - 200.0).abs() < 1e-6, "x should be 200, got {}", t[0]);

        p.update(0.05);
        assert_eq!(p.state(), PlayerState::Completed);
    }

    #[test]
    fn large_dt_is_clamped() {
        let mut p = MotionPlayer::new(square(100.0), [175.0, -25.0, Z]);
        p.play();
        p.update(5.0);
        let t = p.current_target();
        assert!(
            (t[0] - 185.0).abs() < 1e-6,
            "x should be 185 (clamped), got {}",
            t[0]
        );
    }

    #[test]
    fn planner_is_a_pass_through() {
        let planner = TrajectoryPlanner::new();
        let cmds = square(40.0);
        let planned = planner.plan(cmds.clone());
        assert_eq!(planned, cmds);
    }
}
