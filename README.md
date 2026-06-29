# Bombolab

Forward Kinematics Visualizer for robotic arms, built in Rust.

Bombolab models robots as serial chains of revolute/prismatic joints using Denavit-Hartenberg (DH) parameters, computes forward kinematics, and provides a desktop GUI for interactive visualization and parameter editing.

## Features

- **DH Parameter Editor** -- Define robot segments with theta, d, a, alpha parameters
- **Joint Types** -- Support for both revolute and prismatic joints
- **Forward Kinematics** -- Real-time computation of transformation matrices and end-effector pose
- **Interactive GUI** -- Desktop application with egui for editing robot configurations
- **3D Viewport** -- Visual representation of the robot (planned)

## Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/tools/install) 1.85+ (edition 2024)
- Cargo (included with Rust)

### Installation

```bash
git clone https://github.com/charfranr/bombolab.git
cd bombolab
cargo run
```

### Build

```bash
# Release build
cargo build --release

# Debug build
cargo build
```

## Project Structure

```
bombolab/
├── Cargo.toml                # Package manifest
├── LICENSE                   # MIT License
├── docs/
│   └── forward_kinematics.md # DH parameters and FK documentation
└── src/
    ├── main.rs               # Application entry point (egui/eframe)
    ├── domain/               # Core data model
    │   ├── mod.rs            # Module declarations and re-exports
    │   ├── joint.rs          # JointType, Joint structs
    │   ├── link.rs           # DHParams struct
    │   ├── chain.rs          # Segment, Robot structs
    │   └── errors.rs         # Custom error types
    ├── kinematics/           # Math/computation layer
    │   ├── mod.rs            # Module declarations and re-exports
    │   ├── dh_parameters.rs  # Forward kinematics, matrix_from_segment
    │   ├── hmatrix.rs        # Movement, rotation helpers
    │   └── init.rs           # CLI-based interactive tester
    └── ui/                   # GUI layer
        ├── mod.rs            # Module declarations
        ├── state.rs          # AppState, RobotDef, SegmentUi
        └── main_page.rs      # egui rendering logic
```

## Architecture

The project is organized into three layers:

| Layer | Module | Responsibility |
|-------|--------|----------------|
| **Domain** | `domain/` | Core data model: joints, links, segments, robot chain |
| **Kinematics** | `kinematics/` | Forward kinematics computation, transformation matrices |
| **UI** | `ui/` | Desktop GUI with egui, parameter editing, visualization |

### Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| [nalgebra](https://crates.io/crates/nalgebra) | 0.35.0 | Linear algebra: Isometry3, Rotation3, Vector3 |
| [egui](https://crates.io/crates/egui) | 0.34.3 | Immediate-mode GUI framework |
| [eframe](https://crates.io/crates/eframe) | 0.34.3 | Native application wrapper for egui |

## How It Works

### Robot Model

A robot is a chain of **Segments**. Each segment combines a **Joint** (motor) and **DH Parameters** (geometry):

```
Segment {
    joint: Joint      // Revolute or Prismatic, with current value and limits
    dh: DHParams      // theta, d, a, alpha (fixed geometry)
}
```

### Forward Kinematics

The forward kinematics algorithm composes transformation matrices through the chain:

```
T_0_n = T_1 * T_2 * ... * T_n
```

Each transformation matrix is built from DH parameters:

```
T_i = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
```

For more details, see [docs/forward_kinematics.md](docs/forward_kinematics.md).

## Current Status

- [x] Domain model (joints, links, segments, robot)
- [x] Forward kinematics computation
- [x] GUI with egui/eframe
- [x] DH parameter editor
- [x] Transformation matrix display
- [ ] 3D viewport rendering
- [ ] Robot model catalog
- [ ] Jacobian computation
- [ ] Inverse kinematics

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

MIT License. See [LICENSE](LICENSE) for details.
