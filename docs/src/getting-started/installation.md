# Installation

## Prerequisites

- **Rust** (edition 2024) -- install via [rustup](https://rustup.rs/)
- **Git**

Verify your installation:

```bash
rustc --version   # should show 1.85+ (edition 2024)
cargo --version
```

## Clone and Build

```bash
git clone https://github.com/charfranr/bombolab.git
cd bombolab
cargo build
```

The workspace builds two crates:

| Crate | Output |
|-------|--------|
| `bombolab-core` | Library + CLI binaries (`dh-solve`, `quaternion-solve`) |
| `bombolab-gui` | Desktop application |

## Build the GUI Only

```bash
cargo build -p bombolab-gui
```

## Run the CLI Tools

```bash
cargo run --bin dh-solve
cargo run --bin quaternion-solve
```

## Run the GUI

```bash
cargo run -p bombolab-gui
```

## Run Tests

```bash
cargo test
```

## Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| `nalgebra` | 0.35.0 | Linear algebra, matrices, isometries |
| `egui` | 0.34.3 | Immediate-mode GUI framework |
| `eframe` | 0.34.3 | egui application wrapper |

## Troubleshooting

**Build fails with edition errors**: Ensure your Rust toolchain supports edition 2024. Run `rustup update stable`.

**GUI doesn't launch**: eframe requires a display server. On headless systems, use the CLI tools instead.
