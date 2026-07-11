use std::process::Command;

/// Integration test for the `ik-solve` binary.
///
/// Runs the binary with position (161, 162, 152) — the home pose position
/// which is known to converge — and checks that servo angle output is
/// within the valid [10°, 170°] range.
#[test]
fn test_ik_solve_servo_angles_in_range() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args(["run", "--bin", "ik-solve", "--", "161", "162", "152"])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run ik-solve");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() {
        eprintln!("stdout:\n{}", stdout);
        eprintln!("stderr:\n{}", stderr);
        panic!("ik-solve exited with {}\nstderr: {}", output.status, stderr);
    }

    // Parse servo lines from output
    let mut servo_count = 0;
    for line in stdout.lines() {
        let line = line.trim();
        if let Some(suffix) = line.strip_prefix("S") {
            let parts: Vec<&str> = suffix.split(": ").collect();
            if parts.len() == 2 {
                let angle_str = parts[1].trim_end_matches('°');
                if let Ok(angle) = angle_str.parse::<f64>() {
                    assert!(
                        (10.0..=170.0).contains(&angle),
                        "Servo angle {}° is outside valid range [10°, 170°]",
                        angle
                    );
                    servo_count += 1;
                }
            }
        }
    }

    assert!(
        stdout.contains("Gripper:"),
        "Expected 'Gripper:' in output, got:\n{}",
        stdout
    );

    assert!(
        servo_count >= 5,
        "Expected at least 5 servo angle lines, found {}.\nOutput:\n{}",
        servo_count,
        stdout
    );
}
