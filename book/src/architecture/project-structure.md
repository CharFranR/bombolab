# Project Structure

Bombolab is a Rust workspace with a clean separation between the math/model layer and the GUI layer.

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
│   │       │   ├── joint.rs      # JointType, Joint
│   │       │   ├── link.rs       # DHParams
│   │       │   ├── segment.rs    # Segment, Robot (with servo offsets)
│   │       │   ├── errors.rs     # Error, Result types
│   │       │   └── fabri_creator.rs  # FABRI Creator robot config
│   │       ├── kinematics/
│   │       │   ├── mod.rs
│   │       │   ├── forward.rs    # forward_kinematics(), matrix_from_segment()
│   │       │   └── init.rs       # Interactive CLI tester
│   │       ├── communication/    # Serial communication with hardware
│   │       │   ├── mod.rs        # ConnectionError, protocol constants
│   │       │   ├── arduino_nano.rs   # ArduinoNano serial wrapper
│   │       │   └── interpolation.rs  # Smooth servo movement
│   │       └── bin/
│   │           ├── dh-solve.rs           # DH table solver CLI
│   │           ├── quaternion-solve.rs   # Quaternion operations CLI
│   │           └── serial-test.rs        # Interactive serial tester
│   └── bombolab-gui/             # Desktop GUI
│       ├── Cargo.toml
│       └── src/
│           ├── lib.rs
│           └── ui/
│               ├── main_page.rs  # egui rendering and layout
│               └── state.rs      # AppState, RobotDef, SegmentUi
└── docs/
    ├── forward_kinematics.md     # Original FK documentation
    └── src/                      # mdBook source
        ├── SUMMARY.md
        └── ...
```

## Crate Responsibilities

### bombolab-core

The core library contains all math, data models, and kinematics computation. It has **no GUI dependency**.

| Module | What It Contains |
|--------|-----------------|
| `math::dh` | `DHParameter`, `DHParameterSymbolic`, `DHValue`, `compute_a_matrix()`, `solve()` |
| `math::hmatrix` | `Movement`, `rotation_and_translation()`, `translation_and_rotation()`, `make_movement()` |
| `math::quaternion` | `Quaternion`, `solve_add()`, `solve_subtract()`, `solve_multiply()`, `solve_divide()` |
| `math::constants` | `PI`, `DEG_TO_RAD`, `RAD_TO_DEG`, `FRAC_PI_2`, `FRAC_PI_4`, `EPS`, `TAU` |
| `robot::joint` | `JointType` (Revolute/Prismatic), `Joint` |
| `robot::link` | `DHParams` |
| `robot::segment` | `Segment`, `Robot` (with `home_pose`, `servo_offsets`, q/servo conversion) |
| `robot::errors` | `Error` enum, `Result<T>` type alias |
| `robot::fabri_creator` | FABRI Creator 5-DOF robot configuration, base/tool transforms |
| `kinematics::forward` | `forward_kinematics()`, `matrix_from_segment()` |
| `kinematics::init` | Interactive CLI for building robots and testing FK |
| `communication` | Serial protocol constants, `ConnectionError` enum |
| `communication::arduino_nano` | `ArduinoNano` serial connection wrapper |
| `communication::interpolation` | `InterpolationConfig`, `interpolate_joint()`, `interpolate_all()` |

### bombolab-gui

The GUI layer depends on `bombolab-core` and adds:

| Module | What It Contains |
|--------|-----------------|
| `ui::state` | `AppState`, `RobotDef`, `SegmentUi`, `PanelView` |
| `ui::main_page` | egui panel layout, robot editor, FK visualization |

## Data Flow

```
User Input (GUI or CLI)
        │
        ▼
┌─────────────────┐
│  Robot / Table   │  ← robot model or raw DH parameters
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Forward Kin.    │  ← forward_kinematics() or solve()
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Iso3 /     │  ← frames, end-effector pose
│  Matrix4         │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Display         │  ← GUI rendering or CLI output
└─────────────────┘
```

## Dependency Graph

```
bombolab (workspace root)
├── bombolab-core    (nalgebra, serialport, ctrlc)
└── bombolab-gui     (bombolab-core + egui + eframe)
```

The core crate depends on `nalgebra` for linear algebra, `serialport` for hardware communication, and `ctrlc` for clean shutdown handling. The GUI crate adds `egui` and `eframe` for the desktop interface.

## References

- [Design Decisions](./design-decisions.md) -- why the code is structured this way
- [bombolab-core API](../api/core.md) -- full API reference
