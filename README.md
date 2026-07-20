# Bombolab

Forward Kinematics Visualizer for robotic arms.

Bombolab models robots as serial chains of revolute/prismatic/twist joints using Denavit-Hartenberg (DH) parameters, computes forward and inverse kinematics, and provides a **web 3D visualizer** (React + Three.js) powered by a **Rust WASM** core.

## Features

- **Forward Kinematics** — Standard DH + Twist (wrist roll), validada por diferencias finitas
- **Inverse Kinematics** — Damped Least Squares (Levenberg–Marquardt) position-only IK, 5 GDL máx
- **Jacobian** — Geométrica 3×n (lineal), validada numéricamente
- **Web 3D Visualizer** — React + Three.js con cámara orbital, IK target arrastrable
- **WASM Core** — Rust compilado a WASM, FK e IK unificados
- **Serial Control** — Arduino Nano via USB (115200 baud)
- **Gripper** — Pinza paralela 75mm con control de apertura
- **CLI Tools** — `dh-solve`, `quaternion-solve`, `serial-test`

## Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/tools/install) 1.85+ (edition 2024)
- [Node.js](https://nodejs.org/) 20+ (para web visualizer)
- Cargo + npm
- [wasm-pack](https://rustwasm.github.io/wasm-pack/) (para WASM builds)

### Web Visualizer

```bash
# Terminal 1 — compilar WASM
cd web && npm run wasm

# Terminal 2 — dev server
cd web && npm install && npm run dev
# → http://localhost:5173
```

### CLI Tools

```bash
cargo run --bin dh-solve -p bombolab-core
cargo run --bin quaternion-solve -p bombolab-core
cargo run --bin serial-test -p bombolab-core
```

### WASM Build (when Rust changes)

```bash
npm run wasm   # desde web/
```

## Project Structure

```
bombolab/
├── Cargo.toml                       # Workspace root
├── book.toml                        # mdBook configuration
├── crates/
│   ├── bombolab-core/               # Domain model, kinematics, math (lib + bins)
│   │   └── src/
│   │       ├── lib.rs               # Re-exports: robot, math, kinematics, communication
│   │       ├── robot/               # Joint, DHParams, Segment, Robot, errors, fabri_creator
│   │       ├── kinematics/          # Forward kinematics, IK solver, DH solve
│   │       ├── math/                # Isometries (nalgebra wrappers), quaternions, constants
│   │       ├── communication/       # Serial protocol, ServoMapper, interpolation
│   │       └── bin/                 # CLI binaries: dh-solve, quaternion-solve, serial-test
│   ├── bombolab-wasm/               # WASM bridge (wasm-bindgen): FK, IK, fabri_creator
│       └── src/lib.rs
├── web/                             # Visualizador 3D web
│   ├── src/
│   │   ├── components/              # React: RobotViewer, JointControls, InfoPanel, IkTarget
│   │   ├── kinematics/types.ts      # TypeScript interfaces (Mat4, Segment, RobotDef)
│   │   ├── wasm.ts                  # WASM bridge TS (FK, IK delegates to Rust)
│   │   └── serial.ts                # WebSerial
│   ├── package.json
│   └── vite.config.ts
├── arduino/                         # Firmware (PlatformIO, C++ — Arduino Nano)
├── book/                            # mdBook documentation (book/src/ → book/book/)
├── docs/fabri-creator/              # Hardware docs, DH table definition
└── AGENTS.md                        # OpenCode / AI assistant instructions
```

## Architecture

The project is organized as a Cargo workspace with 3 Rust crates:

| Crate | Type | Responsibility |
|-------|------|----------------|
| **bombolab** (root) | placeholder | — |
| **bombolab-core** | lib + 3 bins | Domain model, kinematics, math, serial, CLI tools |
| **bombolab-wasm** | lib (wasm) | wasm-bindgen bridge: FK, IK, robot factory |

Additional non-Rust components:

| Directory | Platform | Description |
|-----------|----------|-------------|
| `web/` | TypeScript + React | 3D web visualizer (Three.js) |
| `arduino/` | PlatformIO (C++) | Arduino Nano servo control via `Servo` library |

### Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| [nalgebra](https://crates.io/crates/nalgebra) | 0.35.0 | Linear algebra: Iso3, Rot3, Vec3 |
| [wasm-bindgen](https://crates.io/crates/wasm-bindgen) | 0.2 | WASM interop |
| [serde](https://crates.io/crates/serde) | 1.0 | Serialization for WASM bridge |

## How It Works

### Robot Model

A robot is a chain of **Segments**. Each segment combines a **Joint** (motor) and **DH Parameters** (geometry):

```
Segment {
    joint: Joint      // Revolute, Prismatic, or Twist
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
Revolute/Prismatic:  T_i = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
Twist (wrist roll):  T_i = RotX(alpha + q) * TransX(a)
```

For more details, see [book/src/core-concepts/forward-kinematics.md](book/src/core-concepts/forward-kinematics.md) (build the mdBook or read it directly).

### Inverse Kinematics

Position-only Damped Least Squares solver. Takes a 3D target and initial joint guess, returns joint angles. Runs entirely in Rust — accessible both natively and via WASM.

## Status

### Implemented

- Domain model (joints, links, segments, robot) — Revolute, Prismatic, **Twist**
- Forward kinematics computation — Standard DH + Twist wrist roll
- Inverse kinematics — DLS position-only (IkSolver), 5 GDL max
- Geometric Jacobian — 3×n linear velocity, validated with finite differences
- WASM bridge — FK, IK, and robot factory exported to TypeScript
- Web visualizer — React + Three.js, orbit controls, IK drag target
- CLI tools: `dh-solve` (numeric/symbolic), `quaternion-solve`, `serial-test`
- Serial communication — Arduino Nano protocol, ServoMapper, interpolation
- Arduino Nano firmware (PlatformIO C++)

### Planned

- Robot model catalog
- Orientation IK (6-DOF)
- WebSerial direct control (currently CLI-only)

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

MIT License. See [LICENSE](LICENSE) for details.
