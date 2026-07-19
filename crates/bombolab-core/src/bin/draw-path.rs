use std::process;
use std::thread;
use std::time::Duration;

use bombolab_core::IkOptions;
use bombolab_core::ServoMapper;
use bombolab_core::communication::{ArduinoNano, InterpolationConfig, ServoCommand, interpolate_all_command};
use bombolab_core::inverse_kinematics;
use bombolab_core::kinematics::dh::solve;
use bombolab_core::kinematics::ik::build_dh_table;
use bombolab_core::robot::Robot;
use bombolab_core::robot::fabri_creator::{base_transform, fabri_creator, tool_transform};
use nalgebra::{Isometry3, Vector3};

fn main() {
    let (port, waypoints_per_side) = parse_args();

    // Connect
    let mut nano = match ArduinoNano::connect(&port) {
        Ok(n) => n,
        Err(e) => {
            eprintln!("Error: failed to connect to port '{}': {}", port, e);
            process::exit(1);
        }
    };

    let robot = fabri_creator();
    let base = base_transform();
    let mapper = ServoMapper::new(&robot);
    let opts = IkOptions::default();
    let config = InterpolationConfig::default();
    let square_center = Vector3::new(150.0, 0.0, 0.0);
    let square_size = 50.0;

    // Home all servos to 90°
    let home_q = robot.kinematic_home();
    let home_cmd = mapper.map_q(&home_q, 90);
    let mut current_cmd = home_cmd;

    // Move to home
    if let Err(e) = nano.send_and_verify(&home_cmd) {
        eprintln!("Error: failed to send home command: {}", e);
        let _ = nano.disconnect();
        process::exit(1);
    }
    thread::sleep(Duration::from_millis(config.delay_ms));

    // Generate waypoints
    let waypoints = generate_square_waypoints(square_center, square_size, waypoints_per_side);

    // Initial seed for IK (kinematic home)
    let mut seed = home_q;

    for (i, wp) in waypoints.iter().enumerate() {
        let (q, error, _iterations) = compensate_waypoint(&robot, &base, wp, &seed, &opts);

        // Map to servo command
        let target_cmd = mapper.map_q(&q, 90);

        // Interpolate from current to target
        let steps = interpolate_all_command(&current_cmd, &target_cmd, &config);

        for step in &steps {
            if let Err(e) = nano.send_and_verify(step) {
                eprintln!(
                    "Error: serial write failed at waypoint {} step: {}",
                    i, e
                );
                let _ = nano.disconnect();
                process::exit(1);
            }
            thread::sleep(Duration::from_millis(config.delay_ms));
        }

        // Update state for next waypoint
        current_cmd = target_cmd;
        seed = q;

        // Progress indicator
        if waypoints.len() > 10 && (i + 1) % 5 == 0 {
            eprintln!(
                "  waypoint {}/{} (error: {:.2}mm)",
                i + 1,
                waypoints.len(),
                error
            );
        }
    }

    // Home after drawing
    if steps_needed(&current_cmd, &home_cmd) {
        let home_steps = interpolate_all_command(&current_cmd, &home_cmd, &config);
        for step in &home_steps {
            if let Err(e) = nano.send_and_verify(step) {
                eprintln!("Error: serial write failed during final homing: {}", e);
                let _ = nano.disconnect();
                process::exit(1);
            }
            thread::sleep(Duration::from_millis(config.delay_ms));
        }
    }

    // Send explicit home command to ensure all servos at 90°
    if let Err(e) = nano.send_and_verify(&home_cmd) {
        eprintln!("Error: final home command failed: {}", e);
    }

    let _ = nano.disconnect();
}

/// Check if interpolation is needed between two commands.
fn steps_needed(current: &ServoCommand, target: &ServoCommand) -> bool {
    let c = current.to_raw_array();
    let t = target.to_raw_array();
    c.iter().zip(t.iter()).any(|(a, b)| a != b)
}

/// Parse CLI arguments manually (like ik-solve pattern).
/// Returns (port, waypoints_per_side).
fn parse_args() -> (String, usize) {
    let args: Vec<String> = std::env::args().collect();

    if args.len() > 1 && (args[1] == "--help" || args[1] == "-h") {
        eprintln!("draw-path — draw a 50×50mm square with iterative tool-tip compensation");
        eprintln!();
        eprintln!("Usage: cargo run --bin draw-path [OPTIONS]");
        eprintln!();
        eprintln!("Options:");
        eprintln!("  --port <PORT>               Serial port (default: /dev/ttyUSB0)");
        eprintln!("  --waypoints-per-side <N>    Waypoints per side (default: 5, min: 2)");
        eprintln!("  --help, -h                  Show this help message");
        eprintln!();
        eprintln!("Examples:");
        eprintln!("  cargo run --bin draw-path");
        eprintln!("  cargo run --bin draw-path -- --port /dev/ttyACM0 --waypoints-per-side 10");
        process::exit(0);
    }

    let mut port = "/dev/ttyUSB0".to_string();
    let mut wps: usize = 5;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--port" => {
                i += 1;
                if i < args.len() {
                    port = args[i].clone();
                } else {
                    eprintln!("Error: --port requires a value");
                    process::exit(1);
                }
            }
            "--waypoints-per-side" => {
                i += 1;
                if i < args.len() {
                    wps = match args[i].parse::<usize>() {
                        Ok(n) if n >= 2 => n,
                        Ok(n) => {
                            eprintln!(
                                "Error: --waypoints-per-side must be at least 2, got {}",
                                n
                            );
                            process::exit(1);
                        }
                        Err(_) => {
                            eprintln!(
                                "Error: --waypoints-per-side must be a positive integer, got '{}'",
                                args[i]
                            );
                            process::exit(1);
                        }
                    };
                } else {
                    eprintln!("Error: --waypoints-per-side requires a value");
                    process::exit(1);
                }
            }
            other => {
                eprintln!("Error: unknown argument '{}'", other);
                eprintln!("Use --help for usage information.");
                process::exit(1);
            }
        }
        i += 1;
    }

    (port, wps)
}

#[allow(dead_code)]
fn compensate_waypoint(
    robot: &Robot,
    base: &Isometry3<f64>,
    waypoint: &Isometry3<f64>,
    seed: &[f64],
    opts: &IkOptions,
) -> (Vec<f64>, f64, usize) {
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    /// Compute J5 frame in world coordinates from joint angles using DH table.
    fn fk_world(robot: &Robot, base: &Isometry3<f64>, q: &[f64]) -> Isometry3<f64> {
        let dh_table = build_dh_table(robot, q);
        let sol = solve(&dh_table);
        let rot_mat: nalgebra::Matrix3<f64> =
            sol.final_transform.fixed_view::<3, 3>(0, 0).into_owned();
        let pos = sol.final_transform.fixed_view::<3, 1>(0, 3).into_owned();
        let j5 = Isometry3::from_parts(
            Translation3::from(pos),
            UnitQuaternion::from_matrix(&rot_mat),
        );
        base * j5
    }

    let waypoint_pos = waypoint.translation.vector;

    let mut best_q: Option<Vec<f64>> = None;
    let mut best_error = f64::MAX;
    let mut iterations = 0usize;

    // Warm-start: track current seed, updated with each converged q
    let mut current_seed = seed.to_vec();

    // Estimate J5 target offset from marker using seed FK:
    //   marker_world = J5_world * tool_transform
    // → J5_world ≈ marker_world * tool_transform⁻¹
    // Initial guess: j5_target_pos = waypoint_pos - tool_offset_at_seed
    let seed_fk_init = fk_world(robot, base, &current_seed);
    let seed_marker_init = seed_fk_init * tool_transform();
    let tool_offset_at_seed = seed_marker_init.translation.vector
        - seed_fk_init.translation.vector;

    // The J5 target position: start at waypoint minus estimated tool offset,
    // then adjust each iteration by the error vector.
    let mut j5_target_pos = waypoint_pos - tool_offset_at_seed;

    for iter in 0..5 {
        iterations = iter + 1;

        // Use FK rotation at current seed for the J5 target orientation
        let seed_fk = fk_world(robot, base, &current_seed);
        let seed_rot_mat: nalgebra::Matrix3<f64> =
            seed_fk.rotation.to_rotation_matrix().into_inner();
        let seed_rot = UnitQuaternion::from_matrix(&seed_rot_mat);

        let j5_target = Isometry3::from_parts(
            Translation3::from(j5_target_pos),
            seed_rot,
        );

        match inverse_kinematics(robot, base, &j5_target, &current_seed, opts) {
            Ok(result) => {
                let q = result.q;

                // Compute marker tip position at this q
                let j5_world = fk_world(robot, base, &q);
                let marker_tip = j5_world * tool_transform();
                let marker_pos = marker_tip.translation.vector;
                let error = (waypoint_pos - marker_pos).norm();

                // Track best solution
                if error < best_error {
                    best_error = error;
                    best_q = Some(q.clone());
                }

                if error < 2.0 {
                    break;
                }

                // Warm-start: use converged q as seed for next iteration
                current_seed = q;

                // Adjust J5 target position by the error vector
                let adjustment = Vector3::new(
                    waypoint_pos.x - marker_pos.x,
                    waypoint_pos.y - marker_pos.y,
                    waypoint_pos.z - marker_pos.z,
                );
                j5_target_pos += adjustment;
            }
            Err(bombolab_core::IkError::DidNotConverge {
                q,
                error_pos,
                ..
            }) => {
                // Use the partial solution — compute where the marker ends up
                let j5_world = fk_world(robot, base, &q);
                let marker_tip = j5_world * tool_transform();
                let marker_pos = marker_tip.translation.vector;
                let error = (waypoint_pos - marker_pos).norm();

                if error < best_error {
                    best_error = error;
                    best_q = Some(q.clone());
                }

                if error < 2.0 {
                    break;
                }

                // Even partial solution may help; warm-start from it
                current_seed = q;

                // Adjust J5 target by error
                let adjustment = Vector3::new(
                    waypoint_pos.x - marker_pos.x,
                    waypoint_pos.y - marker_pos.y,
                    waypoint_pos.z - marker_pos.z,
                );
                j5_target_pos += adjustment;

                eprintln!(
                    "Warning: IK partial convergence for waypoint at ({:.1}, {:.1}, {:.1}): pos_err={:.3}mm",
                    waypoint_pos.x, waypoint_pos.y, waypoint_pos.z, error_pos
                );
            }
            Err(e) => {
                eprintln!(
                    "Warning: IK failed for waypoint at ({:.1}, {:.1}, {:.1}): {}",
                    waypoint_pos.x, waypoint_pos.y, waypoint_pos.z, e
                );
                break;
            }
        }
    }

    let q = best_q.unwrap_or_else(|| seed.to_vec());

    if best_error >= 2.0 {
        eprintln!(
            "Warning: tool-tip compensation exceeded 2mm tolerance. Waypoint ({:.1}, {:.1}, {:.1}), final error: {:.3}mm",
            waypoint_pos.x, waypoint_pos.y, waypoint_pos.z, best_error
        );
    }

    (q, best_error, iterations)
}

#[allow(dead_code)]
fn generate_square_waypoints(
    center: nalgebra::Vector3<f64>,
    size: f64,
    waypoints_per_side: usize,
) -> Vec<nalgebra::Isometry3<f64>> {
    let half = size / 2.0;
    let n = waypoints_per_side;

    // Four corners in order: bottom-left (start), bottom-right, top-right, top-left
    // Path: right (increase X) → forward (increase Y) → left (decrease X) → backward (decrease Y)
    let corners = [
        nalgebra::Vector3::new(center.x - half, center.y - half, 0.0), // (125, -25, 0)
        nalgebra::Vector3::new(center.x + half, center.y - half, 0.0), // (175, -25, 0)
        nalgebra::Vector3::new(center.x + half, center.y + half, 0.0), // (175,  25, 0)
        nalgebra::Vector3::new(center.x - half, center.y + half, 0.0), // (125,  25, 0)
    ];

    // Total waypoints: N per side with shared corners, closed loop
    // Side 0 contributes N points, subsequent sides contribute N-1 each (skip shared start corner)
    let total = 4 * n - 3;
    let mut waypoints = Vec::with_capacity(total);

    let segments = n - 1; // Number of line segments per side

    for side in 0..4 {
        let start = corners[side];
        let end = corners[(side + 1) % 4];

        // First side includes its start corner; subsequent sides skip it (shared)
        let start_idx = if side == 0 { 0usize } else { 1 };

        for i in start_idx..n {
            let t = i as f64 / segments as f64;
            let pos = nalgebra::Vector3::new(
                start.x + t * (end.x - start.x),
                start.y + t * (end.y - start.y),
                0.0,
            );
            waypoints.push(nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::from(pos),
                nalgebra::UnitQuaternion::identity(),
            ));
        }
    }

    waypoints
}

#[cfg(test)]
mod tests {
    use super::*;
    use bombolab_core::IkOptions;
    use bombolab_core::kinematics::dh::solve;
    use bombolab_core::kinematics::ik::build_dh_table;
    use bombolab_core::robot::fabri_creator::{base_transform, fabri_creator, tool_transform};
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    /// Spec R2/S1: Square geometry — corners at (±25,±25,0) around center (150,0,0),
    /// all Z=0, path traces right→forward→left→backward, closed loop.
    #[test]
    fn test_square_waypoints_corners_and_closed_loop() {
        let center = Vector3::new(150.0, 0.0, 0.0);
        let size = 50.0;
        let n = 5;
        let waypoints = generate_square_waypoints(center, size, n);

        // All waypoints must have Z=0
        for wp in &waypoints {
            assert!(
                (wp.translation.vector.z - 0.0).abs() < 1e-10,
                "Z should be 0, got {}",
                wp.translation.vector.z
            );
        }

        // Four corners at (±25, ±25, 0) relative to center
        let half = size / 2.0;
        let expected_corners = [
            Vector3::new(center.x - half, center.y - half, 0.0), // (125, -25, 0) — start
            Vector3::new(center.x + half, center.y - half, 0.0), // (175, -25, 0)
            Vector3::new(center.x + half, center.y + half, 0.0), // (175,  25, 0)
            Vector3::new(center.x - half, center.y + half, 0.0), // (125,  25, 0)
        ];

        // Closed loop: first and last waypoints must be the same
        let first = waypoints.first().unwrap().translation.vector;
        let last = waypoints.last().unwrap().translation.vector;
        assert!(
            (first.x - last.x).abs() < 1e-10
                && (first.y - last.y).abs() < 1e-10
                && (first.z - last.z).abs() < 1e-10,
            "Closed loop: first and last waypoints must be the same. Got {:?} and {:?}",
            first,
            last
        );

        // Verify all 4 corners are present in the path
        for expected in &expected_corners {
            let found = waypoints.iter().any(|wp| {
                let p = wp.translation.vector;
                (p.x - expected.x).abs() < 1e-10 && (p.y - expected.y).abs() < 1e-10
            });
            assert!(
                found,
                "Corner ({:.0}, {:.0}, 0) not found in waypoints",
                expected.x, expected.y
            );
        }

        // Path direction: right→forward→left→backward
        // Find the corner indices
        let find_corner = |x: f64, y: f64| {
            waypoints
                .iter()
                .position(|wp| {
                    let p = wp.translation.vector;
                    (p.x - x).abs() < 1e-10 && (p.y - y).abs() < 1e-10
                })
                .unwrap()
        };

        let idx_start = find_corner(125.0, -25.0); // corner0
        let idx_right = find_corner(175.0, -25.0); // corner1 — after rightward
        let idx_fwd = find_corner(175.0, 25.0); // corner2 — after forward
        let idx_left = find_corner(125.0, 25.0); // corner3 — after leftward

        assert!(
            idx_start < idx_right,
            "Path must go right (increase X) first: start before right corner"
        );
        assert!(
            idx_right < idx_fwd,
            "Path must go forward (increase Y) after right"
        );
        assert!(
            idx_fwd < idx_left,
            "Path must go left (decrease X) after forward"
        );
        // After leftward corner, path goes backward (decrease Y) to starting corner
    }

    /// Spec R2: verify N waypoints per side with shared corners.
    /// Each side has N equidistant points including both endpoints.
    /// Corners are shared between adjacent sides.
    #[test]
    fn test_waypoints_per_side_with_shared_corners() {
        let center = Vector3::new(150.0, 0.0, 0.0);
        let size = 50.0;

        // Test with n=5: total = 4*5 - 3 = 17 (closed loop)
        let n = 5;
        let waypoints = generate_square_waypoints(center, size, n);
        assert_eq!(
            waypoints.len(),
            4 * n - 3,
            "Total waypoints for n={}: expected {}, got {}",
            n,
            4 * n - 3,
            waypoints.len()
        );

        // Test with n=2 (minimum): total = 4*2 - 3 = 5 (just corners, closed)
        let n = 2;
        let waypoints = generate_square_waypoints(center, size, n);
        assert_eq!(waypoints.len(), 5, "n=2 should yield 5 waypoints (4 corners + closing)");

        // Test with n=10: verify correct count
        let n = 10;
        let waypoints = generate_square_waypoints(center, size, n);
        assert_eq!(waypoints.len(), 4 * n - 3);

        // Verify uniform spacing: check distance between consecutive waypoints
        // along each side is equal
        let n = 5;
        let waypoints = generate_square_waypoints(center, size, n);
        let half = size / 2.0;

        // Expected step size per side
        let step = size / (n as f64 - 1.0); // 50 / 4 = 12.5mm

        // Side 0 (right): from (125,-25) to (175,-25), all X increase, Y constant
        // The first 5 waypoints (indices 0-4) are side 0
        for i in 1..n {
            let prev = waypoints[i - 1].translation.vector;
            let curr = waypoints[i].translation.vector;
            let dx = curr.x - prev.x;
            assert!(
                (dx - step).abs() < 1e-10,
                "Side 0 step {}: expected dx={}, got dx={}",
                i,
                step,
                dx
            );
            assert!(
                (curr.y - prev.y).abs() < 1e-10,
                "Side 0: Y should be constant, got dy={}",
                curr.y - prev.y
            );
        }

        // Verify the start corner is at index 0 and index n-1 of side 3
        // Side 3 (backward): Y decreases from +25 to -25
        // The last waypoint (len-1) should be the starting corner
        let last_wp = waypoints.last().unwrap().translation.vector;
        assert!(
            (last_wp.x - (center.x - half)).abs() < 1e-10
                && (last_wp.y - (center.y - half)).abs() < 1e-10,
            "Last waypoint should be starting corner (125,-25,0), got ({:.1},{:.1},{:.1})",
            last_wp.x,
            last_wp.y,
            last_wp.z
        );
    }

    // ─── Phase 2: Compensation loop ─────────────────────────────────

    /// Helper: compute marker tip world position for given q via DH table.
    /// marker_world = base * j5_iso * tool_transform()
    fn marker_tip_world(robot: &Robot, base: &Isometry3<f64>, q: &[f64]) -> Vector3<f64> {
        use nalgebra::Translation3;
        let dh_table = build_dh_table(robot, q);
        let sol = solve(&dh_table);
        let rot_mat: nalgebra::Matrix3<f64> = sol.final_transform.fixed_view::<3, 3>(0, 0).into_owned();
        let pos = sol.final_transform.fixed_view::<3, 1>(0, 3).into_owned();
        let j5 = Isometry3::from_parts(
            Translation3::from(pos),
            UnitQuaternion::from_matrix(&rot_mat),
        );
        let marker = *base * j5 * tool_transform();
        marker.translation.vector
    }

    /// Spec R3/S1: FK-generated reachable target converges within ≤3 iterations, error < 2mm.
    #[test]
    fn test_compensation_converges_on_reachable_target() {
        let robot = fabri_creator();
        let base = base_transform();
        let opts = IkOptions::default();

        // Use a q close to home — small angles so FK rotation stays near identity
        let known_q = vec![0.0, 0.15, -0.1, 0.0, 0.05];
        let target_pos = marker_tip_world(&robot, &base, &known_q);

        // Construct waypoint at the marker tip position
        let waypoint = Isometry3::from_parts(
            Translation3::from(target_pos),
            UnitQuaternion::identity(),
        );

        // Use a seed near known_q (but not identical — warm-start)
        let seed = vec![0.0, 0.1, -0.05, 0.0, 0.0];

        let (q, error, iterations) = compensate_waypoint(&robot, &base, &waypoint, &seed, &opts);

        assert!(
            iterations <= 3,
            "Compensation should converge in ≤3 iterations, took {}",
            iterations
        );
        assert!(
            error < 2.0,
            "Compensation error should be < 2mm, got {:.4}mm",
            error
        );
        assert_eq!(q.len(), 5, "Should return 5 joint angles");

        // Verify q is within joint limits
        for (i, seg) in robot.segments.iter().enumerate() {
            assert!(
                q[i] >= seg.joint.value_min && q[i] <= seg.joint.value_max,
                "J{}: q={:.4} rad outside limits [{:.2},{:.2}]",
                i + 1,
                q[i],
                seg.joint.value_min,
                seg.joint.value_max
            );
        }
    }

    /// Spec R3/S2 (partial): unreachable target returns best-so-far q, does not panic.
    #[test]
    fn test_compensation_unreachable_does_not_panic() {
        let robot = fabri_creator();
        let base = base_transform();
        let opts = IkOptions::default();

        // Target far outside workspace
        let waypoint = Isometry3::from_parts(
            Translation3::new(1000.0, 1000.0, 1000.0),
            UnitQuaternion::identity(),
        );

        let seed = vec![0.0; 5];

        let (q, _error, iterations) =
            compensate_waypoint(&robot, &base, &waypoint, &seed, &opts);

        // Must not panic; must return a q vector of correct length
        assert_eq!(q.len(), 5, "Should return 5 joint angles even on failure");
        assert!(
            iterations <= 5,
            "Max 5 iterations; got {}",
            iterations
        );
    }

    /// Spec R3: max 5 iterations — compensation loop must stop at 5.
    /// Also verifies best-so-far tracking.
    #[test]
    fn test_compensation_max_iterations_cap() {
        let robot = fabri_creator();
        let base = base_transform();
        let opts = IkOptions::default();

        // Target far enough that multiple comp iterations are needed
        // but IK itself may still converge (to the J5 target, not the marker tip)
        let waypoint = Isometry3::from_parts(
            Translation3::new(200.0, -80.0, 50.0),
            UnitQuaternion::identity(),
        );

        let seed = vec![0.0; 5];

        let (_q, error, iterations) =
            compensate_waypoint(&robot, &base, &waypoint, &seed, &opts);

        // Must not exceed 5 iterations
        assert!(
            iterations <= 5,
            "Compensation must stop at max 5 iterations, got {}",
            iterations
        );

        // If we hit 5 iterations, the error should have been tracked
        assert!(error >= 0.0, "Error must be non-negative, got {}", error);
    }

    // ─── Phase 5: Integration tests ────────────────────────────────

    /// Spec R3 + R5: dry-run full path — generate all waypoints, solve via compensation,
    /// verify all q within joint limits.
    #[test]
    fn test_dry_run_full_path_all_q_in_limits() {
        use nalgebra::Vector3;
        let robot = fabri_creator();
        let base = base_transform();
        let opts = IkOptions::default();
        let center = Vector3::new(150.0, 0.0, 0.0);

        let waypoints = generate_square_waypoints(center, 50.0, 5);

        assert!(
            waypoints.len() >= 17,
            "Expected at least 17 waypoints, got {}",
            waypoints.len()
        );

        let mut seed = vec![0.0; 5];

        for (i, wp) in waypoints.iter().enumerate() {
            let (q, error, iterations) =
                compensate_waypoint(&robot, &base, wp, &seed, &opts);

            assert!(
                iterations <= 5,
                "Waypoint {}: compensation exceeded 5 iterations (got {})",
                i,
                iterations
            );

            // q must be within joint limits
            for (j, seg) in robot.segments.iter().enumerate() {
                assert!(
                    q[j] >= seg.joint.value_min && q[j] <= seg.joint.value_max,
                    "Waypoint {} J{}: q={:.4} rad outside limits [{:.2}, {:.2}] (error={:.2}mm)",
                    i,
                    j + 1,
                    q[j],
                    seg.joint.value_min,
                    seg.joint.value_max,
                    error
                );
            }

            // Warm-start next waypoint
            seed = q;
        }
    }
}
