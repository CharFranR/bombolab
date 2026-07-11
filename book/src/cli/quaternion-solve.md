# quaternion-solve

**Interactive CLI tool for quaternion arithmetic.** Perform addition, subtraction, multiplication (Hamilton product), and division on quaternions.

## Usage

```bash
cargo run --bin quaternion-solve
```

## Interactive Flow

### 1. Choose Operation

```
Operación:
  1 = Suma
  2 = Resta
  3 = Multiplicación (Hamilton)
  4 = División
Opción:
```

### 2. Enter Number of Quaternions

```
Número de cuaterniones:
```

### 3. Enter Each Quaternion

For each quaternion, enter the four components:

```
  --- Cuaternion 1 ---
  a: 1.0
  b: 2.0
  c: 3.0
  d: 4.0
```

The quaternion is stored as `a + bi + cj + dk`.

### 4. See the Result

```
  Resultado: (6 + 8i + 10j + 12k)
```

## Operations

### Addition (Suma)

Component-wise addition:

```
q₁ + q₂ = (a₁+a₂) + (b₁+b₂)i + (c₁+c₂)j + (d₁+d₂)k
```

With multiple quaternions, adds all sequentially from zero.

### Subtraction (Resta)

Component-wise subtraction from zero:

```
result = 0 - q₁ - q₂ - ...
```

Note: subtract starts at zero and subtracts each quaternion, so `solve_subtract(&[q1, q2])` returns `-q1 - q2`.

### Multiplication (Hamilton Product)

Sequential Hamilton product:

```
result = q₁ ⊗ q₂ ⊗ q₃ ⊗ ...
```

Starting from the identity quaternion `(1, 0, 0, 0)`. Quaternion multiplication is **not commutative** -- order matters.

The Hamilton product of two quaternions:

```
(a₁ + b₁i + c₁j + d₁k) ⊗ (a₂ + b₂i + c₂j + d₂k) =
  (a₁a₂ - b₁b₂ - c₁c₂ - d₁d₂) +
  (a₁b₂ + b₁a₂ + c₁d₂ - d₁c₂)i +
  (a₁c₂ - b₁d₂ + c₁a₂ + d₁b₂)j +
  (a₁d₂ + b₁c₂ - c₁b₂ + d₁a₂)k
```

### Division (División)

Sequential right-division:

```
result = q₁ / q₂ / q₃ / ...
```

Starting from the identity quaternion. Division is implemented as multiplication by the inverse.

## Quaternion Type

```rust
pub struct Quaternion {
    pub a: f64,  // real part
    pub b: f64,  // i component
    pub c: f64,  // j component
    pub d: f64,  // k component
}
```

Display format: `(a + bi + cj + dk)`

## Useful Properties

| Method | Description |
|--------|-------------|
| `q.norm()` | Magnitude: `√(a² + b² + c² + d²)` |
| `q.normalize()` | Unit quaternion (norm = 1) |
| `q.conjugate()` | `(a, -b, -c, -d)` |
| `q.inverse()` | Conjugate / norm² |
| `Quaternion::identity()` | `(1, 0, 0, 0)` -- no rotation |
| `Quaternion::zero()` | `(0, 0, 0, 0)` |

## Examples

### Rotate a Vector

Quaternions represent rotations. To rotate a vector:

1. Convert vector to pure quaternion: `q_v = (0, x, y, z)`
2. Apply rotation: `q_result = q_rotation ⊗ q_v ⊗ q_rotation*`

### Compose Rotations

Multiply quaternions to compose rotations:

```rust
use bombolab_core::math::quaternion::{Quaternion, solve_multiply};

let rotate_x = Quaternion::new(0.7071, 0.7071, 0.0, 0.0); // 90° around X
let rotate_y = Quaternion::new(0.7071, 0.0, 0.7071, 0.0); // 90° around Y

let combined = solve_multiply(&[rotate_x, rotate_y]);
// Apply rotate_x first, then rotate_y
```

## Implementation

Source: `crates/bombolab-core/src/bin/quaternion-solve.rs`

The tool uses `bombolab_core::math::quaternion` functions:
- `solve_add()` -- sum of quaternions
- `solve_subtract()` -- sequential subtraction from zero
- `solve_multiply()` -- sequential Hamilton product
- `solve_divide()` -- sequential right-division via inverse

## References

- [bombolab-core API](../api/core.md) -- Quaternion type and operations reference
