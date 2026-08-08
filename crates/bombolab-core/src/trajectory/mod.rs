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
mod tests;
