# Forward Kinematics con DH

## De qué va esto

Este documento explica cómo funciona la cinemática directa en bombolab:
cómo describís un robot, cómo se arma cada matriz de transformación, y cómo se
componen para obtener la posición de cada eslabón y del efector final.

---

## 1. Cómo describís un robot

Un robot es una cadena de **Segmentos**. Cada segmento tiene dos partes:

```
Segment {
    joint: Joint,    // el motor: qué tipo, cuánto gira, límites
    dh: DHParams,    // la geometría fija entre este joint y el siguiente
}
```

### Joint

```rust
pub struct Joint {
    pub joint_type: JointType,  // Revolute | Prismatic
    pub value: f64,             // valor ACTUAL del joint (radianes)
    pub value_max: f64,
    pub value_min: f64,
}
```

- **Revolute**: el joint **rota** sobre su eje Z. `value` es un ángulo (θ).
- **Prismatic**: el joint **desliza** sobre su eje Z. `value` es una distancia (d).

Siempre se mueve sobre Z. Si necesitás movimiento en otra dirección, se
reorienta el Z con `alpha` (explicado más abajo).

### DHParams

```rust
pub struct DHParams {
    pub theta: f64,  // ángulo sobre Z (radianes)
    pub d: f64,      // traslación sobre Z
    pub a: f64,      // traslación sobre X — "largo del link"
    pub alpha: f64,  // twist sobre X — ángulo entre Zs (radianes)
}
```

Son los 4 parámetros de Denavit–Hartenberg. Definen la **geometría fija** del
link. El único valor que cambia dinámicamente es el del joint:

| JointType  | theta viene de | d viene de |
|------------|---------------|------------|
| Revolute   | joint.value   | dh.d (fijo) |
| Prismatic  | dh.theta (fijo) | joint.value |

Esto lo resuelve automáticamente `segment.dh_params()`.

---

## 2. Parámetro por parámetro

### theta — rotación sobre Z (radianes)

Es el ángulo del joint si es Revolute. **Siempre sobre Z del frame local.**

theta = 0 → el link apunta en X pura.
theta = 90° → el link apunta en Y pura.

### d — traslación sobre Z

Separa el frame actual del anterior sobre el eje Z. Representa:

- Altura de una base (d = 5 → la base mide 5 hacia arriba).
- Offset vertical entre joints.
- Desplazamiento prismático si el joint es Prismatic.

**d no genera movimiento en X ni en Y.** Es puro Z.

### a — traslación sobre X ("largo del link")

La distancia desde el eje Z actual hasta el próximo eje Z, medida sobre X.
Es el largo del eslabón. Cuando theta cambia, este largo se proyecta en X e Y:

```
a·cos(θ)  → componente en X
a·sin(θ)  → componente en Y
```

Con theta=0, `a` va todo en X. Con theta=90°, `a` va todo en Y.

### alpha — twist entre Zs (radianes)

**La clave de DH.** Es el ángulo de rotación alrededor de X que tuerce el Z
actual para alinearlo con el Z del próximo joint.

- alpha = 0 → Zs paralelos (el brazo sigue en el mismo plano).
- alpha = 90° (π/2) → el próximo Z es perpendicular al actual (codo que
  cambia de plano).

**Sin alpha, todos los joints de tu robot giran en el mismo plano 2D.**
Con alpha, podés modelar robots 3D reales.

---

## 3. El truco con Isometry3 en `matrix_from_segment`

La función que construye la matriz de cada segmento:

```rust
pub fn matrix_from_segment(segment: &Segment) -> Isometry3<f64> {
    let (theta, d, a, alpha) = segment.dh_params();

    // Rotación final: RotZ(theta) * RotX(alpha)
    let rot_z = Rotation3::from_axis_angle(&Vector3::z_axis(), theta);
    let rot_x = Rotation3::from_axis_angle(&Vector3::x_axis(), alpha);
    let rotation = UnitQuaternion::from_rotation_matrix(&(rot_z * rot_x));

    // Traslación: a·cos(θ) en X, a·sin(θ) en Y, d en Z
    let translation = Translation3::new(a * theta.cos(), a * theta.sin(), d);

    Isometry3::from_parts(translation, rotation)
}
```

### ¿Por qué UnitQuaternion?

`Isometry3<f64>` guarda la rotación internamente como `UnitQuaternion<f64>`,
no como `Rotation3<f64>`. Por eso necesitamos convertir con
`UnitQuaternion::from_rotation_matrix()`.

También podés escribir `rot.into()` si tenés un `Rotation3` — la conversión
es automática:

```rust
let r: Rotation3<f64> = rot_z * rot_x;
Isometry3::from_parts(translation, r.into())
```

### ¿Por qué rot_z * rot_x?

El orden de la fórmula DH canónica es:

```
T = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)
```

1. RotZ(θ) — gira el frame
2. TransZ(d) — sube/baja sobre Z
3. TransX(a) — avanza sobre X (largo del link)
4. RotX(α) — tuerce el Z para el próximo frame

RotZ y RotX se componen como `rot_z * rot_x` porque en la fórmula parámetros
DH, RotX(α) se post-multiplica: primero se aplica RotZ(θ), después RotX(α)
al frame de salida.

**Si alpha = 0**, RotX(α) es identidad y toda la rotación se reduce a RotZ(θ).

### La traslación no es un vector libre

Fijate que `a·cos(θ)` y `a·sin(θ)` aparecen en X e Y — eso es el **largo del
link proyectado después de rotar**. No es "movimiento en X", es el resultado
de rotar un link de largo `a`.

d va directo a Z porque RotZ(θ) no afecta Z. RotX(α) sí podría afectarlo,
pero en la fórmula DH la traslación se aplica ANTES de RotX(α).

---

## 4. Forward kinematics completa

```rust
pub fn forward_kinematics(
    base: Isometry3<f64>,   // dónde está el mundo respecto al robot
    robot: &Robot,
) -> (Vec<Isometry3<f64>>, Isometry3<f64>) {
    let mut frames = Vec::new();
    let mut current = base;

    for segment in &robot.segments {
        current = current * matrix_from_segment(segment);
        frames.push(current);
    }

    (frames, current)
}
```

- **frames**: `Vec<Isometry3<f64>>` — la pose de CADA eslabón. `frames[i]` es
  la posición del frame i+1 después de aplicar los joints 0..=i.
- **current** (efector final): la pose del último frame, después de todos los
  joints.

### ¿Y la base?

`base` es un `Isometry3<f64>` externo — no está atado al robot. Podés pasar
`Isometry3::identity()` si el robot está en el origen, o una transformación
cualquiera si está desplazado/rotado en el mundo. El mismo robot puede usarse
en distintas posiciones sin modificarlo.

---

## 5. Cómo describirle tu robot al sistema

### Robot 2D (brazo plano, SCARA)

```
Segmento 1 (base):  Revolute | articular + d (altura)
Segmento 2 (brazo): Revolute | a (largo)
Segmento 3 (brazo): Revolute | a (largo)
alpha = 0 siempre   → el brazo se mueve en XY

Ejemplo:
  S1: d=5, a=0          → base de 5 de altura
  S2: d=0, a=3          → antebrazo de 3
  S3: d=0, a=3          → brazo de 3
```

### Robot 3D (con cambio de plano)

```
Segmento 1 (base):    Revolute | d + alpha=0
Segmento 2 (hombro):  Revolute | a + alpha=π/2 (torce el plano)
Segmento 3 (codo):    Revolute | a + alpha=0   (sigue el nuevo plano)
```

alpha = π/2 en el hombro hace que el brazo se mueva en vertical en vez de
horizontal.

### Tabla rápida

| Parámetro | Representa | Unidad |
|-----------|-----------|--------|
| theta | rotación del joint sobre Z | radianes |
| d | traslación sobre Z | mm / m |
| a | largo del link (sobre X) | mm / m |
| alpha | twist entre Zs | radianes |

### Lo que NO es cada parámetro

- theta **no es** el largo del link.
- d **no es** movimiento horizontal.
- a **no es** la altura de un segmento.
- alpha **no es** el ángulo del joint.

---

## 6. Orden de multiplicación: local vs global

La cinemática directa compone así:

```
current = current * T_i
```

Cada `T_i` es la transformación **del frame i-1 al frame i**, expresada en
el frame i-1. Post-multiplicar significa que cada paso es relativo al frame
anterior — que es exactamente cómo funciona una cadena cinemática.

Si cambiás a `current = T_i * current`, estarías aplicando cada movimiento
en el marco global del mundo. Eso sirve para mover un cuerpo rígido libre
por el espacio, **no** para un robot articulado.
