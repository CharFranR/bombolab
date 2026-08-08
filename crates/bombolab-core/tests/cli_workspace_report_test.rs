use std::process::Command;

#[test]
fn test_workspace_report_invalid_mode_usage() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args([
            "run",
            "--bin",
            "workspace-report",
            "--",
            "--mode",
            "bogus",
            "--n",
            "10",
        ])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run workspace-report");

    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        !output.status.success(),
        "invalid mode should exit with non-zero status"
    );

    assert!(
        stderr.contains("Usage:"),
        "Expected 'Usage:' in stderr output:\n{}",
        stderr
    );

    assert!(
        stderr.contains("workspace-report"),
        "Expected 'workspace-report' in stderr output:\n{}",
        stderr
    );
}

#[test]
fn test_workspace_report_valid_run() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args([
            "run",
            "--bin",
            "workspace-report",
            "--",
            "--seed",
            "7",
            "--n",
            "100",
        ])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run workspace-report");

    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(
        output.status.success(),
        "valid run should exit 0, stderr:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );

    assert!(
        stdout.contains("workspace report:"),
        "Expected report on stdout:\n{}",
        stdout
    );
}
