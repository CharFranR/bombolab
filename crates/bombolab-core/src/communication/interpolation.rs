//! Linear interpolation for smooth servo movement.
//!
//! Moves each joint independently in steps of `step_size` degrees,
//! padding shorter movements with their final value so all joints
//! arrive at the target simultaneously.

use super::ServoCommand;

/// Configuration for interpolation step size and timing.
pub struct InterpolationConfig {
    /// Angle increment per step in degrees.
    ///
    /// Must be `> 0` — asserted in `interpolate_all` and `interpolate_joint`.
    pub step_size: i32,
    /// Delay between steps in milliseconds.
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

/// Linear interpolation from current to target angle, stepping by `step_size`.
/// Returns intermediate angles (exclusive start, inclusive end).
///
/// Precondition: `step_size > 0` (asserted).
pub fn interpolate_joint(current: i32, target: i32, step_size: i32) -> Vec<i32> {
    assert!(step_size > 0, "step_size must be > 0");

    if current == target {
        return Vec::new();
    }

    // i64 internals: `target - current` can overflow i32 (e.g. -170 → i32::MAX).
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
        // If next step would overshoot the target, append target and finish
        let next = pos + direction * step;
        if direction * (next - target_i) >= 0 {
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

    // ─── Preconditions and overflow ──────────────────────────────────────

    #[test]
    #[should_panic(expected = "step_size must be > 0")]
    fn interpolate_joint_zero_step_panics() {
        interpolate_joint(90, 100, 0);
    }

    #[test]
    #[should_panic(expected = "step_size must be > 0")]
    fn interpolate_joint_negative_step_panics() {
        interpolate_joint(90, 100, -5);
    }

    #[test]
    #[should_panic(expected = "step_size must be > 0")]
    fn interpolate_all_zero_step_panics() {
        let config = InterpolationConfig {
            step_size: 0,
            delay_ms: 0,
        };
        interpolate_all(&[90; 6], &[100; 6], &config);
    }

    #[test]
    fn interpolate_joint_extreme_values_no_overflow() {
        // La distancia (-170 → i32::MAX) desborda i32: el cálculo interno
        // debe usar i64. Con step grande el resultado es corto.
        let steps = interpolate_joint(-170, i32::MAX, 1_000_000_000);
        assert!(!steps.is_empty());
        assert_eq!(*steps.last().unwrap(), i32::MAX);
        assert_eq!(steps.len(), 3);
        assert_eq!(steps[0], 999_999_830);
        assert_eq!(steps[1], 1_999_999_830);

        // Descendente con distancia máxima (i32::MAX → i32::MIN).
        let steps = interpolate_joint(i32::MAX, i32::MIN, i32::MAX);
        assert_eq!(*steps.last().unwrap(), i32::MIN);
        assert_eq!(steps.len(), 3);
    }
}
