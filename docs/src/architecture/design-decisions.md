# Design Decisions

Key architectural choices in Bombolab and the reasoning behind them.

## 1. Two-Crate Split: Core vs GUI

**Decision**: Separate `bombolab-core` (math + model) from `bombolab-gui` (egui rendering).

**Why**:
- The core library is testable without a display server
- CLI tools (`dh-solve`, `quaternion-solve`) can use core without pulling in egui
- The GUI can be swapped or extended without touching the math
- Core can be used as a library in other Rust projects

**Tradeoff**: Slightly more boilerplate for cross-crate imports, but the isolation is worth it.

## 2. Craig Convention (Modified DH)

**Decision**: Use the Craig convention for DH parameters, not the standard (Spong) convention.

**Why**: In the Craig convention:
- `a` is measured along `X_i` (the current frame's X axis)
- `alpha` rotates around `X_i`
- The transformation order is: RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)

This matches the convention used in "Introduction to Robotics" by Craig, which is the most common textbook for robotics courses.

**Tradeoff**: Users trained on the standard convention need to remap their parameters.

## 3. Isometry3 for Transformations

**Decision**: Use `nalgebra::Isometry3<f64>` instead of raw `Matrix4<f64>` for forward kinematics.

**Why**:
- `Isometry3` semantically represents a rigid body transformation (rotation + translation)
- It stores rotation as a `UnitQuaternion`, which is more numerically stable than a 3x3 rotation matrix
- nalgebra provides optimized multiplication for isometries
- The `forward_kinematics()` function returns `Vec<Isometry3>`, giving both position and rotation in a type-safe way

The numeric solver (`solve()`) uses `Matrix4<f64>` directly because it works with the raw DH math and needs to display the full 4x4 matrix.

## 4. Joint Value as the Only Dynamic State

**Decision**: Only `Joint.value` changes at runtime. Everything else (DH parameters, limits) is fixed.

**Why**:
- Matches the physical reality: robot geometry doesn't change, only joint angles do
- Simplifies the kinematics computation: no need to rebuild DH tables
- Makes the model easy to serialize/deserialize for saving robot configurations

## 5. Segment Combines Joint + Geometry

**Decision**: `Segment` wraps both `Joint` (actuator) and `DHParams` (geometry) together.

**Why**:
- Each link in a serial chain has exactly one joint and one set of DH parameters
- The `dh_params()` method resolves which parameter is the joint variable (theta for revolute, d for prismatic)
- This keeps the API simple: you work with segments, not separate joint and geometry lists

## 6. Symbolic DH Support

**Decision**: Support symbolic values (`DHValue::Sym`) in the DH solver, not just numeric.

**Why**:
- Robotics students need to derive symbolic expressions for homework/research
- The `dh-solve` CLI can output matrices with variable names (e.g., `cos(theta1)`)
- Useful for verifying closed-form solutions before implementing them numerically

## 7. Immediate-Mode GUI (egui)

**Decision**: Use egui (immediate-mode) instead of a retained-mode framework.

**Why**:
- egui is lightweight and easy to integrate with Rust
- Immediate-mode simplifies state management: no event handlers, no widget tree
- The GUI is relatively simple (parameter input + 3D viewport placeholder)
- eframe handles window creation and rendering backend

**Tradeoff**: egui's 3D capabilities are limited. The current "3D Viewport" is a placeholder. For full 3D rendering, a future version might integrate `wgpu` or `three-d`.

## 8. Custom Error Type

**Decision**: Define a project-specific `Error` enum instead of using `anyhow` or `thiserror`.

**Why**:
- The error cases are well-defined and few (joint count mismatch, out of bounds, out of limits)
- A custom type gives precise error messages without pulling in error-handling frameworks
- The `Result<T>` alias keeps function signatures clean

## 9. Left-to-Right Matrix Multiplication

**Decision**: Forward kinematics multiplies `current = current * T_i` (post-multiplication).

**Why**: In a serial chain, each transformation is expressed in the previous frame's coordinate system. Post-multiplication correctly composes relative transformations. Pre-multiplication (`T_i * current`) would apply movements in the world frame, which is wrong for articulated robots.

## References

- [Project Structure](./project-structure.md) -- how these decisions map to the code layout
- [DH Parameters](../core-concepts/dh-parameters.md) -- the Craig convention explained
- [Forward Kinematics](../core-concepts/forward-kinematics.md) -- how multiplication order works
