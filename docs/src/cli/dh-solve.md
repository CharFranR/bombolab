# dh-solve

**Interactive CLI tool for solving DH parameter tables.** Enter your robot's Denavit-Hartenberg parameters and get the full solution: A matrices, intermediate frames, and end-effector pose.

## Usage

```bash
cargo run --bin dh-solve
```

## Interactive Flow

### 1. Choose Angle Unit

```
Unidad de ángulos:
  1 = Grados
  2 = Radianes
Opción:
```

Select **Grados** (degrees) or **Radianes** (radians) for input. The tool converts to radians internally for computation.

### 2. Enter Number of Links

```
Número de eslabones:
```

Enter the number of links (joints) in your robot.

### 3. Enter DH Parameters

For each link, you're prompted:

```
--- Eslabón 1 ---
  α (grados): 0
  a: 1.0
  d: 0.0
  θ (grados): 45
```

### 4. See the Solution

**Numeric mode** (all values are numbers):

```
  TABLA DH
    i │          α │          a │          d │          θ
  ────┼────────────┼────────────┼────────────┼────────────
    1 │     0.0000 │     1.0000 │     0.0000 │     0.7854

  MATRICES A_i

  A1:
    [  0.7071   -0.7071    0.0000    0.7071]
    [  0.7071    0.7071    0.0000    0.7071]
    [  0.0000    0.0000    1.0000    0.0000]
    [  0.0000    0.0000    0.0000    1.0000]

  FRAMES (posiciones de cada joint)
  Frame 1: (  0.7071,   0.7071,   0.0000)

  POSE FINAL (efector)
  Posición: (  0.7071,   0.7071,   0.0000)
  Rotación:
    [  0.7071  -0.7071   0.0000]
    [  0.7071   0.7071   0.0000]
    [  0.0000   0.0000   1.0000]
```

**Symbolic mode** (any value is a variable name):

```
  TABLA DH (simbólica)
   i | α          | a          | d          | θ
  --+------------+------------+------------+------------
   1 | 0          | L1         | 0          | theta1

  MATRICES A_i (simbólicas)

  A1:
  cos(theta1)   -sin(theta1)  0             L1·cos(theta1)
  sin(theta1)    cos(theta1)  0             L1·sin(theta1)
  0              0             1             0
  0              0             0             1
```

## Symbolic Mode

If any input value is not a valid number, it's treated as a symbolic variable. Examples:

| Input | Parsed As |
|-------|-----------|
| `0` | `DHValue::Num(0.0)` |
| `3.14` | `DHValue::Num(3.14)` |
| `L1` | `DHValue::Sym("L1")` |
| `theta1` | `DHValue::Sym("theta1")` |
| `pi/2` | `DHValue::Sym("pi/2")` |

Symbolic matrices show the trigonometric expressions, simplifying known values (e.g., `sin(90°)` becomes `1`).

## Input Format

| Parameter | What to Enter | Example |
|-----------|--------------|---------|
| α (alpha) | Twist angle | `0`, `90`, `pi/2` |
| a | Link length | `1.0`, `5.0` |
| d | Offset along Z | `0.0`, `3.5` |
| θ (theta) | Joint angle | `45`, `0.785` |

## Examples

### 2-Link Planar Arm

```
Grados
2 links
Link 1: α=0, a=1, d=0, θ=0
Link 2: α=0, a=1, d=0, θ=0
```

Result: end-effector at (2.0, 0.0, 0.0).

### SCARA Robot

```
Grados
3 links
Link 1: α=0, a=0, d=5, θ=0
Link 2: α=0, a=3, d=0, θ=45
Link 3: α=0, a=2, d=0, θ=30
```

## Implementation

The tool reads from stdin, parses inputs into `DHParameterSymbolic` values, checks if all are numeric, and either:

- **Numeric**: converts to `DHParameter`, calls `solve()`, prints the `DHSolution`
- **Symbolic**: calls `format_symbolic_matrix()` for each link

Source: `crates/bombolab-core/src/bin/dh-solve.rs`

## References

- [DH Parameters](../core-concepts/dh-parameters.md) -- theory behind the parameters
- [Forward Kinematics](../core-concepts/forward-kinematics.md) -- how the solution is computed
