# Development Setup

How to set up a development environment for contributing to Bombolab.

## Prerequisites

- **Rust** (edition 2024, stable)
- **Git**
- An editor with rust-analyzer support (VS Code, Lapce, Neovim, etc.)

## Clone the Repository

```bash
git clone https://github.com/charfranr/bombolab.git
cd bombolab
```

## Build

```bash
# Build everything
cargo build

# Build with warnings
cargo build 2>&1

# Release build
cargo build --release
```

## Run

```bash
# Run the GUI
cargo run -p bombolab-gui

# Run the DH solver CLI
cargo run --bin dh-solve

# Run the quaternion solver CLI
cargo run --bin quaternion-solve
```

## Test

```bash
# Run all tests
cargo test

# Run tests for a specific crate
cargo test -p bombolab-core

# Run a specific test
cargo test test_solve_single_joint

# Show test output
cargo test -- --nocapture
```

## Project Layout

See [Project Structure](../architecture/project-structure.md) for the full directory layout.

## Useful Commands

```bash
# Check for compilation errors without building
cargo check

# Format code
cargo fmt

# Lint with clippy
cargo clippy

# Generate and open documentation
cargo doc --open
```

## IDE Setup

### VS Code

Install the `rust-analyzer` extension. The workspace is automatically detected.

Recommended settings:

```json
{
  "rust-analyzer.check.command": "clippy",
  "rust-analyzer.inlayHints.chainingHints.enable": true
}
```

### Other Editors

Any editor with rust-analyzer support will work. The workspace `Cargo.toml` at the project root provides all the context rust-analyzer needs.

## Making Changes

1. Create a branch: `git checkout -b feature/my-change`
2. Make your changes
3. Run tests: `cargo test`
4. Run clippy: `cargo clippy`
5. Format: `cargo fmt`
6. Commit with a conventional message
7. Push and create a PR

## Guidelines

See [Guidelines](./guidelines.md) for coding standards and contribution rules.
