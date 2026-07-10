use std::process::Command;

/// Integration test for the `serial-test` binary.
///
/// Runs the binary with `--help` and asserts usage info is printed to stderr.
#[test]
fn test_serial_test_help_output() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));

    let output = Command::new("cargo")
        .args(["run", "--bin", "serial-test", "--", "--help"])
        .current_dir(crate_dir)
        .output()
        .expect("failed to run serial-test");

    let stderr = String::from_utf8_lossy(&output.stderr);

    // --help should exit with non-zero status
    assert!(
        !output.status.success(),
        "serial-test --help should exit with non-zero status"
    );

    assert!(
        stderr.contains("Usage:"),
        "Expected 'Usage:' in stderr output:\n{}",
        stderr
    );

    assert!(
        stderr.contains("serial-test"),
        "Expected 'serial-test' in stderr output:\n{}",
        stderr
    );
}
