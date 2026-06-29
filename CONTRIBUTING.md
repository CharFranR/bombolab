# Contributing to Bombolab

Thank you for your interest in contributing to Bombolab! This document provides guidelines and instructions for contributing.

## Getting Started

1. Fork the repository
2. Clone your fork
3. Create a feature branch: `git checkout -b feature/my-feature`
4. Make your changes
5. Run tests: `cargo test`
6. Run linter: `cargo clippy`
7. Format code: `cargo fmt`
8. Commit your changes
9. Push to your fork and submit a Pull Request

## Development Setup

### Prerequisites

- Rust 1.85+ (edition 2024)
- Cargo

### Building

```bash
# Debug build
cargo build

# Release build
cargo build --release
```

### Running

```bash
cargo run
```

### Testing

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture
```

## Code Style

### Language

- **All code, comments, and documentation must be in English**
- Variable names, function names, and types use English
- Error messages use English

### Naming Conventions

Follow standard Rust naming conventions:

| Item | Convention | Example |
|------|-----------|---------|
| Functions | `snake_case` | `forward_kinematics()` |
| Variables | `snake_case` | `joint_value` |
| Structs | `PascalCase` | `DHParams` |
| Enums | `PascalCase` | `JointType` |
| Traits | `PascalCase` | `Display` |
| Constants | `SCREAMING_SNAKE_CASE` | `MAX_JOINTS` |
| Modules | `snake_case` | `dh_parameters` |

### Formatting

Use `cargo fmt` to format all code. The project uses default rustfmt settings.

```bash
# Check formatting
cargo fmt -- --check

# Apply formatting
cargo fmt
```

### Linting

Use `cargo clippy` to catch common mistakes and improve code quality.

```bash
cargo clippy
```

Fix all warnings before submitting a PR.

## Project Structure

```
bombolab/
├── src/
│   ├── main.rs           # Entry point
│   ├── domain/           # Core data model
│   ├── kinematics/       # Math/computation
│   └── ui/               # GUI layer
```

### Module Guidelines

- **`domain/`**: Core types and business logic. No UI or external dependencies.
- **`kinematics/`**: Mathematical computations. Depends on `domain` and `nalgebra`.
- **`ui/`**: GUI rendering. Depends on `domain`, `kinematics`, `egui`, `eframe`.

## Commit Messages

Use [Conventional Commits](https://www.conventionalcommits.org/) format:

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

### Types

| Type | Description |
|------|-------------|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation changes |
| `style` | Code style changes (formatting, no logic change) |
| `refactor` | Code refactoring (no feature change) |
| `test` | Adding or updating tests |
| `chore` | Maintenance tasks |

### Examples

```
feat(kinematics): add Jacobian computation
fix(ui): correct matrix display formatting
docs: update README with installation instructions
refactor(domain): simplify Joint validation logic
```

## Pull Request Guidelines

### Before Submitting

- [ ] Code compiles without errors
- [ ] All tests pass (`cargo test`)
- [ ] No clippy warnings (`cargo clippy`)
- [ ] Code is formatted (`cargo fmt`)
- [ ] Documentation is updated if needed
- [ ] Commit messages follow conventions

### PR Description

Include:

1. **What** -- Brief description of changes
2. **Why** -- Motivation for the changes
3. **How** -- Implementation approach (if non-obvious)
4. **Testing** -- How to test the changes

### Review Process

1. PR is reviewed by maintainer
2. Feedback is addressed
3. PR is merged when approved

## Reporting Issues

Use GitHub Issues to report bugs or request features.

### Bug Reports

Include:

- Steps to reproduce
- Expected behavior
- Actual behavior
- Environment (OS, Rust version)

### Feature Requests

Include:

- Description of the feature
- Use case
- Proposed implementation (if any)

## Code of Conduct

Be respectful and constructive. We are all here to learn and build something useful.

## Questions?

Open an issue or reach out to the maintainer.
