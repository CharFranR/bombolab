use super::ServoCommand;

pub struct InterpolationConfig {
    pub step_size: i32,

    pub delay_ms: u64,
}

impl Default for InterpolationConfig {
    fn default() -> Self {
        Self {
            step_size: 5,
            delay_ms: 100,
        }
    }
}

pub fn interpolate_joint(current: i32, target: i32, step_size: i32) -> Vec<i32> {
    assert!(step_size > 0, "step_size must be > 0");

    if current == target {
        return Vec::new();
    }

    let distance = (target as i64 - current as i64).abs();
    if step_size as i64 >= distance {
        return vec![target];
    }

    let direction: i64 = if target > current { 1 } else { -1 };
    let step = step_size as i64;
    let target_i = target as i64;
    let mut steps = Vec::new();
    let mut pos: i64 = current as i64;

    loop {
        pos += direction * step;
        steps.push(pos as i32);
        if pos == target_i {
            break;
        }

        let next = pos + direction * step;
        if direction * (next - target_i) >= 0 {
            steps.push(target);
            break;
        }
    }

    steps
}

pub fn interpolate_all(
    current: &[i32; 6],
    target: &[i32; 6],
    config: &InterpolationConfig,
) -> Vec<[i32; 6]> {
    assert!(config.step_size > 0, "step_size must be > 0");

    let joint_steps: Vec<Vec<i32>> = current
        .iter()
        .zip(target.iter())
        .map(|(c, t)| interpolate_joint(*c, *t, config.step_size))
        .collect();

    let max_len = joint_steps.iter().map(|s| s.len()).max().unwrap_or(0);
    if max_len == 0 {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(max_len);
    for step_idx in 0..max_len {
        let mut row = [0i32; 6];
        for joint_idx in 0..6 {
            let steps = &joint_steps[joint_idx];
            if step_idx < steps.len() {
                row[joint_idx] = steps[step_idx];
            } else {
                row[joint_idx] = *steps.last().unwrap_or(&current[joint_idx]);
            }
        }
        result.push(row);
    }

    result
}

pub fn interpolate_all_command(
    current: &ServoCommand,
    target: &ServoCommand,
    config: &InterpolationConfig,
) -> Vec<ServoCommand> {
    let current_raw = current.to_raw_array();
    let target_raw = target.to_raw_array();
    let steps = interpolate_all(&current_raw, &target_raw, config);
    steps.iter().map(ServoCommand::from_raw_array).collect()
}

#[cfg(test)]
#[path = "interpolation_tests.rs"]
mod interpolation_tests;
