//! Linear interpolation for smooth servo movement.
//!
//! Moves each joint independently in steps of `step_size` degrees,
//! padding shorter movements with their final value so all joints
//! arrive at the target simultaneously.

use super::ServoCommand;

/// Configuration for interpolation step size and timing.
pub struct InterpolationConfig {
    /// Angle increment per step in degrees.
    pub step_size: i32,
    /// Delay between steps in milliseconds.
    pub delay_ms: u64,
}

impl Default for InterpolationConfig {
    fn default() -> Self {
        Self {
            step_size: 5,
            delay_ms: 40,
        }
    }
}

/// Linear interpolation from current to target angle, stepping by `step_size`.
/// Returns intermediate angles (exclusive start, inclusive end).
pub fn interpolate_joint(current: i32, target: i32, step_size: i32) -> Vec<i32> {
    if current == target {
        return Vec::new();
    }

    let distance = (target - current).abs();
    if step_size >= distance {
        return vec![target];
    }

    let direction = if target > current { 1 } else { -1 };
    let mut steps = Vec::new();
    let mut pos = current;

    loop {
        pos += direction * step_size;
        steps.push(pos);
        if pos == target {
            break;
        }
        // If next step would overshoot the target, append target and finish
        let next = pos + direction * step_size;
        if direction * (next - target) >= 0 {
            steps.push(target);
            break;
        }
    }

    steps
}

/// Interpolate all 6 joints independently, then pad shorter ones
/// with their final value so all rows align (max steps across joints).
pub fn interpolate_all(
    current: &[i32; 6],
    target: &[i32; 6],
    config: &InterpolationConfig,
) -> Vec<[i32; 6]> {
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
                // Pad with final value (last element)
                row[joint_idx] = *steps.last().unwrap_or(&current[joint_idx]);
            }
        }
        result.push(row);
    }

    result
}

/// Interpolate between two `ServoCommand` values, returning intermediate steps.
///
/// Converts to raw arrays, delegates to `interpolate_all()`, and converts back.
/// This is a zero-change wrapper — the interpolation logic is unchanged.
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
mod tests {
    use super::*;

    // ─── interpolate_joint ──────────────────────────────────────────

    #[test]
    fn interpolate_joint_exact_multiple() {
        // 90 → 100, step 5 → steps: [95, 100]
        let steps = interpolate_joint(90, 100, 5);
        assert_eq!(steps, vec![95, 100]);
    }

    #[test]
    fn interpolate_joint_non_exact_final_adjusts() {
        // 90 → 102, step 5 → [95, 100, 102]
        let steps = interpolate_joint(90, 102, 5);
        assert_eq!(steps, vec![95, 100, 102]);
    }

    #[test]
    fn interpolate_joint_no_movement() {
        let steps = interpolate_joint(90, 90, 5);
        assert!(steps.is_empty());
    }

    #[test]
    fn interpolate_joint_step_exceeds_distance() {
        // 90 → 92, step 5 → [92] (single step to target)
        let steps = interpolate_joint(90, 92, 5);
        assert_eq!(steps, vec![92]);
    }

    #[test]
    fn interpolate_joint_descending() {
        // 100 → 90, step 5 → [95, 90]
        let steps = interpolate_joint(100, 90, 5);
        assert_eq!(steps, vec![95, 90]);
    }

    #[test]
    fn interpolate_joint_descending_non_exact() {
        // 100 → 88, step 5 → [95, 90, 88]
        let steps = interpolate_joint(100, 88, 5);
        assert_eq!(steps, vec![95, 90, 88]);
    }

    // ─── interpolate_all ────────────────────────────────────────────

    #[test]
    fn interpolate_all_same_length_joints() {
        let current = [90, 90, 90, 90, 90, 90];
        let target = [95, 100, 85, 115, 90, 90];
        let config = InterpolationConfig {
            step_size: 5,
            delay_ms: 0,
        };
        let steps = interpolate_all(&current, &target, &config);

        // Joint 0: 90→95 (1 step), Joint 1: 90→100 (2 steps)
        // Joint 2: 90→85 (1 step), Joint 3: 90→115 (5 steps)
        // Max is 5 steps. Shorter joints pad with final value.
        assert_eq!(steps.len(), 5);
        assert_eq!(steps[0], [95, 95, 85, 95, 90, 90]);
        assert_eq!(steps[1], [95, 100, 85, 100, 90, 90]);
        assert_eq!(steps[2], [95, 100, 85, 105, 90, 90]);
        assert_eq!(steps[3], [95, 100, 85, 110, 90, 90]);
        assert_eq!(steps[4], [95, 100, 85, 115, 90, 90]);
    }

    #[test]
    fn interpolate_all_shorter_joint_pads() {
        let current = [90, 90, 90, 90, 90, 90];
        // Joint 0 needs 2 steps (90→100), Joint 1 needs 4 steps (90→110)
        let target = [100, 110, 90, 90, 90, 90];
        let config = InterpolationConfig {
            step_size: 5,
            delay_ms: 0,
        };
        let steps = interpolate_all(&current, &target, &config);

        // Joint 1 dictates 4 steps (90→95→100→105→110)
        assert_eq!(steps.len(), 4);
        // Joint 0 finishes at step 2 (100), pads 100 for steps 3-4
        assert_eq!(steps[0], [95, 95, 90, 90, 90, 90]);
        assert_eq!(steps[1], [100, 100, 90, 90, 90, 90]);
        assert_eq!(steps[2], [100, 105, 90, 90, 90, 90]);
        assert_eq!(steps[3], [100, 110, 90, 90, 90, 90]);
    }

    #[test]
    fn interpolate_all_no_movement() {
        let current = [90; 6];
        let target = [90; 6];
        let config = InterpolationConfig::default();
        let steps = interpolate_all(&current, &target, &config);
        assert!(steps.is_empty());
    }

    #[test]
    fn interpolate_all_single_step() {
        let current = [90; 6];
        let target = [92; 6];
        let config = InterpolationConfig {
            step_size: 5,
            delay_ms: 0,
        };
        let steps = interpolate_all(&current, &target, &config);
        assert_eq!(steps.len(), 1);
        assert_eq!(steps[0], [92; 6]);
    }
}
