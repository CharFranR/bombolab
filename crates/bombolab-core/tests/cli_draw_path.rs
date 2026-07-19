use std::process::Command;

/// Integration test: CLI arg defaults and --help output.
#[test]
fn test_draw_path_help_output() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args(["run", "--bin", "draw-path", "--", "--help"])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run draw-path --help");

    assert!(
        output.status.success(),
        "draw-path --help should exit 0, got {}",
        output.status
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    // Usage info should be in stderr (like ik-solve)
    let output_text = if stderr.is_empty() { stdout } else { stderr };
    assert!(
        output_text.contains("draw-path"),
        "Help should mention 'draw-path', got:\n{}",
        output_text
    );
    assert!(
        output_text.contains("--port"),
        "Help should mention --port, got:\n{}",
        output_text
    );
    assert!(
        output_text.contains("--waypoints-per-side"),
        "Help should mention --waypoints-per-side, got:\n{}",
        output_text
    );
}

/// Verify that --waypoints-per-side rejects values below 2.
#[test]
fn test_draw_path_waypoints_minimum_enforced() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args([
            "run",
            "--bin",
            "draw-path",
            "--",
            "--waypoints-per-side",
            "1",
        ])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run draw-path");

    assert!(
        !output.status.success(),
        "draw-path with waypoints-per-side=1 should exit non-zero"
    );

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("at least 2") || stderr.contains("must be"),
        "Error should mention minimum value, got:\n{}",
        stderr
    );
}

/// Verify that --waypoints-per-side with invalid input fails.
#[test]
fn test_draw_path_invalid_waypoints_arg() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args([
            "run",
            "--bin",
            "draw-path",
            "--",
            "--waypoints-per-side",
            "abc",
        ])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run draw-path");

    assert!(
        !output.status.success(),
        "draw-path with non-numeric waypoints-per-side should exit non-zero"
    );

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("integer") || stderr.contains("must be"),
        "Error should indicate invalid input, got:\n{}",
        stderr
    );
}

/// Verify that unknown args fail.
#[test]
fn test_draw_path_unknown_arg() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args(["run", "--bin", "draw-path", "--", "--bogus"])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run draw-path");

    assert!(
        !output.status.success(),
        "draw-path with unknown arg should exit non-zero"
    );

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("unknown") || stderr.contains("--help"),
        "Error should mention unknown argument, got:\n{}",
        stderr
    );
}
