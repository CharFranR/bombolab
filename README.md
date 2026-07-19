# Bombolab

Forward Kinematics Visualizer for robotic arms, built in Rust.

Bombolab models robots as serial chains of revolute/prismatic joints using Denavit-Hartenberg (DH) parameters, computes forward kinematics, and provides a desktop GUI for interactive visualization and parameter editing.

## Features

- **DH Parameter Editor** -- Define robot segments with theta, d, a, alpha parameters
- **Joint Types** -- Support for both revolute and prismatic joints
- **Forward Kinematics** -- Real-time computation of transformation matrices and end-effector pose
- **3D Wireframe Viewport** -- Isometric skeleton rendering with ground grid
- **Physical Robot Tab** -- Serial port connection, telemetry, joint angle control
- **Hardware Abstraction** -- Trait-based controller with mock for offline development
- **Interactive GUI** -- Desktop application with egui for editing robot configurations
- **CLI Tools** -- `dh-solve` (numeric/symbolic) and `quaternion-solve`

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
├── Cargo.toml                   # Root app manifest (eframe entrypoint)
├── crates/
│   ├── bombolab-core/           # Domain model, kinematics, math (lib + 2 bins)
│   │   └── src/
│   │       ├── lib.rs           # Re-exports: robot, math, kinematics
│   │       ├── robot/           # Joint, DHParams, Segment, Robot, errors
│   │       ├── kinematics/      # Forward kinematics, DH solve
│   │       ├── math/            # Isometries, quaternions, constants
│   │       └── bin/             # CLI binaries: dh-solve, quaternion-solve
│   └── bombolab-gui/            # egui/eframe GUI layer
│       └── src/
│           ├── lib.rs           # Re-exports: render, AppState, hardware
│           ├── ui/
│           │   ├── main_page.rs # Tabbed UI (Simulation / Physical Robot)
│           │   ├── state.rs     # AppState, RobotDef, modes
│           │   └── viewport.rs  # Isometric 3D wireframe renderer
│           └── hardware.rs      # RobotController trait + MockRobotController
├── arduino/Arduino Nano/        # PlatformIO firmware (Arduino Nano, Servo lib)
├── docs/                        # mdBook documentation (docs/src/ → docs/book/)
└── AGENTS.md                    # OpenCode / AI assistant instructions
```

## Architecture

The project is organized as a Cargo workspace with 3 Rust crates:

| Crate | Type | Responsibility |
|-------|------|----------------|
| **bombolab** (root) | binary | eframe entrypoint, wires GUI crate |
| **bombolab-core** | lib + 2 bins | Domain model, kinematics, math, CLI tools |
| **bombolab-gui** | lib | egui rendering, UI state, hardware abstraction |

Additional non-Rust components:

| Directory | Platform | Firmware |
|-----------|----------|----------|
| `arduino/` | PlatformIO (C++) | Arduino Nano servo control via `Servo` library |

### Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| [nalgebra](https://crates.io/crates/nalgebra) | 0.35.0 | Linear algebra: Iso3, Rot3, Vec3 |
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

## Status

### Implemented

- Domain model (joints, links, segments, robot)
- Forward kinematics computation
- GUI with egui/eframe — tabbed interface (Simulation / Physical Robot)
- DH parameter editor
- Transformation matrix display
- 3D wireframe viewport — isometric skeleton with ground grid
- Physical robot tab — connect/disconnect, telemetry sliders, send/read angles
- Hardware abstraction — `RobotController` trait + mock implementation for offline dev
- CLI tools: `dh-solve` (numeric/symbolic) and `quaternion-solve`
- Arduino Nano firmware (PlatformIO C++)

### Planned

- Robot model catalog
- Jacobian computation
- Inverse kinematics
- Hardware serial implementation (`serialport` crate)

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

MIT License. See [LICENSE](LICENSE) for details.
