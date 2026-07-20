# Project Structure

Bombolab is a Rust workspace with WASM interop and a web frontend.

## Workspace Layout

```
bombolab/
├── Cargo.toml                    # Workspace root
├── book.toml                     # mdBook configuration
├── crates/
│   ├── bombolab-core/            # Core library + CLI binaries
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs            # Public API re-exports
│   │       ├── math/
│   │       │   ├── mod.rs
│   │       │   ├── dh.rs         # DH parameter types and solver
│   │       │   ├── hmatrix.rs    # Iso3 helpers
│   │       │   ├── quaternion.rs # Quaternion type and operations
│   │       │   └── constants.rs  # PI, DEG_TO_RAD, RAD_TO_DEG
│   │       ├── robot/
│   │       │   ├── mod.rs
│   │       │   ├── joint.rs      # JointType (Revolute, Prismatic, Twist), Joint
│   │       │   ├── link.rs       # DHParams
│   │       │   ├── segment.rs    # Segment, Robot (with servo offsets/directions)
│   │       │   ├── errors.rs     # Error, Result types
│   │       │   └── fabri_creator.rs  # FABRI Creator robot config
│   │       ├── kinematics/
│   │       │   ├── mod.rs
│   │       │   ├── forward.rs    # forward_kinematics(), matrix_from_segment()
│   │       │   ├── ik.rs         # IkSolver (DLS IK), IkError
│   │       │   └── init.rs       # Interactive CLI tester
│   │       ├── communication/    # Serial communication with hardware
│   │       │   ├── mod.rs        # ConnectionError, protocol constants
│   │       │   ├── arduino_nano.rs   # ArduinoNano serial wrapper
│   │       │   ├── mapper.rs     # ServoMapper (q → servo degrees)
│   │       │   └── interpolation.rs  # Smooth servo movement
│   │       └── bin/
│   │           ├── dh-solve.rs           # DH table solver CLI
│   │           ├── quaternion-solve.rs   # Quaternion operations CLI
│   │           └── serial-test.rs        # Interactive serial tester
│   └── bombolab-wasm/             # WASM bridge
│       ├── Cargo.toml
│       └── src/lib.rs             # FK, IK, fabri_creator exports via wasm-bindgen
├── web/                           # Web visualizer
│   ├── package.json
│   ├── vite.config.ts
│   └── src/
│       ├── components/            # React components
│       │   ├── RobotViewer.tsx    # 3D scene (Three.js), FK rendering, gripper
│       │   ├── JointControls.tsx  # Joint slider UI
│       │   ├── InfoPanel.tsx      # End-effector position/rotation display
│       │   └── IkTarget.tsx       # Draggable IK target
│       ├── kinematics/types.ts    # TypeScript interfaces (Mat4, Segment, RobotDef)
│       ├── wasm.ts                # WASM bridge: FK, IK, robot factory
│       ├── serial.ts              # WebSerial
│       └── pkg/                   # Generated WASM binary (gitignored)
├── arduino/                       # PlatformIO firmware (C++)
├── book/                          # mdBook source
│   ├── book.toml
│   └── src/
│       ├── SUMMARY.md
│       └── ...
└── docs/fabri-creator/            # Hardware docs, DH table definition
```

## Crate Responsibilities

### bombolab-core

The core library contains all math, data models, and kinematics computation. It has **no GUI dependency** and can be compiled to WASM.

| Module | What It Contains |
|--------|-----------------|
| `math::dh` | `DHParameter`, `DHParameterSymbolic`, `DHValue`, `compute_a_matrix()`, `solve()` |
| `math::hmatrix` | `Movement`, `rotation_and_translation()`, `translation_and_rotation()`, `make_movement()` |
| `math::quaternion` | `Quaternion`, `solve_add()`, `solve_subtract()`, `solve_multiply()`, `solve_divide()` |
| `math::constants` | `PI`, `DEG_TO_RAD`, `RAD_TO_DEG`, `FRAC_PI_2`, `FRAC_PI_4`, `EPS`, `TAU` |
| `robot::joint` | `JointType` (Revolute/Prismatic/**Twist**), `Joint` |
| `robot::link` | `DHParams` |
| `robot::segment` | `Segment`, `Robot` (with `home_pose`, `servo_offsets`, `servo_directions`, q/servo conversion) |
| `robot::errors` | `Error` enum, `Result<T>` type alias |
| `robot::fabri_creator` | FABRI Creator 5-DOF robot configuration, base/tool transforms |
| `kinematics::forward` | `forward_kinematics()`, `matrix_from_segment()` |
| `kinematics::ik` | `IkSolver`, `IkError` |
| `kinematics::init` | Interactive CLI for building robots and testing FK |
| `communication` | Serial protocol constants, `ConnectionError` enum |
| `communication::arduino_nano` | `ArduinoNano` serial connection wrapper |
| `communication::mapper` | `ServoMapper` — maps kinematic q to servo angles |
| `communication::interpolation` | `InterpolationConfig`, `interpolate_joint()`, `interpolate_all()` |

### bombolab-wasm

The WASM bridge exposes core functionality to the web frontend:

| Export | Description |
|--------|-------------|
| `fabri_creator()` | Returns FABRI Creator robot definition |
| `forward_kinematics(robot)` | Compute FK frames + end-effector |
| `solve_ik(robot, target, q_init)` | IK solver returning q + convergence |
| `base_transform()` / `tool_transform()` | Robot transforms |

All matrix data is serialized as row-major 3×4 arrays, matching the original TypeScript FK format.

## Data Flow

```
┌─────────────────┐
│  Sliders / IK   │  ← user input (React UI)
│  Target Drag     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  wasm.ts         │  ← TypeScript bridge → Rust WASM
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Rust core       │  ← forward_kinematics() / IkSolver
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  frames (Iso3)   │  ← poses in DH coordinates
└────────┬────────┘
         │
         ▼
┌──────────────────────────┐
│  framePose() → Three.js  │  ← coordinate swap (Z-up → Y-up)
└──────────────────────────┘
```

## Dependency Graph

```
bombolab (workspace root)
├── bombolab-core    (nalgebra, serialport, ctrlc)
└── bombolab-wasm    (bombolab-core + wasm-bindgen + serde)
```

The core crate depends on `nalgebra` for linear algebra, `serialport` for hardware communication (behind feature gate `serial` for WASM compat), and `ctrlc` for clean shutdown. The WASM crate adds `wasm-bindgen` and `serde` for JavaScript interop.

## References

- [Design Decisions](./design-decisions.md) -- why the code is structured this way
- [bombolab-core API](../api/core.md) -- full API reference
