# Contributing Guidelines

Standards and conventions for contributing to Bombolab.

## Code Style

### Rust Edition

The project uses **edition 2024**. All new code must be compatible with it.

### Formatting

Run `cargo fmt` before committing. The project uses default rustfmt settings.

### Linting

Run `cargo clippy` and fix all warnings. Common things clippy catches:

- Unused imports
- Unnecessary clones
- Missing documentation on public items
- Non-idiomatic Rust patterns

## Naming Conventions

Follow standard Rust naming:

| Item | Convention | Example |
|------|-----------|---------|
| Types | PascalCase | `DHParameter`, `JointType` |
| Functions | snake_case | `compute_a_matrix`, `forward_kinematics` |
| Constants | SCREAMING_SNAKE | `PI`, `DEG_TO_RAD` |
| Modules | snake_case | `robot::joint`, `math::dh` |
| Files | snake_case | `dh.rs`, `joint.rs` |

## Documentation

### Public Items

All public types, functions, and modules should have doc comments:

```rust
/// Compute the A_i transformation matrix from DH parameters.
///
/// Uses the Craig convention: RotZ(θ) · TransZ(d) · TransX(a) · RotX(α).
///
/// # Arguments
/// * `p` - DH parameters for one link
///
/// # Returns
/// A 4x4 transformation matrix.
pub fn compute_a_matrix(p: DHParameter) -> Matrix4<f64> {
    // ...
}
```

### Internal Code

Use comments to explain *why*, not *what*:

```rust
// Good: explains the reasoning
// Isometry3 stores rotation as UnitQuaternion, so we must convert
let rotation = UnitQuaternion::from_rotation_matrix(&(rot_z * rot_x));

// Bad: restates the code
// Convert rotation matrix to quaternion
let rotation = UnitQuaternion::from_rotation_matrix(&(rot_z * rot_x));
```

## Testing

### Unit Tests

Every module should have tests. Place them in a `#[cfg(test)] mod tests` block at the end of the file.

Test naming convention:

```rust
#[test]
fn test_DHParameter_new() { ... }

#[test]
fn test_solve_empty_table() { ... }

#[test]
fn test_robot_set_joint_values_out_of_limits() { ... }
```

Use the `approx` pattern for floating-point comparisons:

```rust
fn approx(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-10
}
```

### Test Coverage

Key areas that must have tests:

- DH parameter computation (`compute_a_matrix`)
- DH table solving (`solve`)
- Forward kinematics (`forward_kinematics`, `matrix_from_segment`)
- Joint limits and error handling
- Robot model operations (add/remove segments, set joint values)

## Git Conventions

### Commit Messages

Use conventional commits:

```
feat: add inverse kinematics solver
fix: correct DH matrix for alpha=pi/2
docs: update API reference for DHParameter
test: add edge cases for solve_empty_table
refactor: extract matrix formatting into separate function
```

### Branch Naming

```
feature/description
fix/description
docs/description
```

## Architecture Rules

1. **Core has no GUI dependency**. `bombolab-core` must never import `egui` or `eframe`.
2. **GUI depends on core**, never the reverse.
3. **New math goes in `math::`**. New robot model types go in `robot::`. New kinematics algorithms go in `kinematics::`.
4. **Error types are explicit**. Use the `Error` enum, don't panic in library code.
5. **DH parameters use Craig convention**. Document this clearly when adding new DH-related code.

## Adding a New Feature

1. Check if it belongs in `bombolab-core` or `bombolab-gui`
2. Add types to the appropriate module
3. Write tests first (or alongside implementation)
4. Update the API reference in `docs/src/api/core.md`
5. Update this guide if you establish new patterns

## Adding a New CLI Tool

1. Create `crates/bombolab-core/src/bin/your-tool.rs`
2. Add `[[bin]]` entry to `crates/bombolab-core/Cargo.toml`
3. Add a chapter in `docs/src/cli/`
4. Update `SUMMARY.md`

## Questions?

Open an issue on GitHub or start a discussion.
