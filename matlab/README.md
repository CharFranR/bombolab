# Paquete MATLAB — Cinemática del FABRI Creator

Paquete de códigos MATLAB (compatibles con GNU Octave, sin toolboxes) con la
resolución analítica de la **cinemática directa por parámetros Denavit-Hartenberg
(DH)** y de la **cinemática diferencial por cuaterniones duales** del manipulador
FABRI Creator, requerido en la entrega del proyecto de Robótica Básica (ULSA,
Robotec-2026).

El modelo replica **exactamente** la implementación del gemelo digital de
bombolab-core (Rust/web): misma tabla DH, misma base, mismo útil y misma
convención para la articulación 4 (tipo *twist*). Los valores de referencia que
imprime el script principal provienen del binario `test-case-report` del
proyecto.

## Estructura

| Archivo | Contenido |
| --- | --- |
| `interactivo.m` | **Ventana interactiva**: robot en 3D, sliders y IK. |
| `dibujar_robot.m` | Dibuja el robot en 3D (eslabones, articulaciones, marcador). |
| `cinematica_inversa.m` | IK de posición (Levenberg-Marquardt, igual que bombolab-core). |
| `main_entrega.m` | Script principal: tabla DH, validaciones y resultados. |
| `fabri_creator.m` | Definición del robot (tabla DH, base, útil, servos). |
| `dh_matrix.m` | Matriz de transformación homogénea A_i de un eslabón. |
| `cd_dh.m` | Cinemática directa por parámetros DH. |
| `cd_cuaterniones_duales.m` | Cinemática directa por cuaterniones duales. |
| `cinematica_diferencial.m` | Jacobiano por cuaterniones duales y jacobiano geométrico clásico. |
| `quat_mul.m`, `quat_conj.m`, `quat_from_rot.m`, `quat_to_rot.m` | Álgebra de cuaterniones. |
| `dq_from_pose.m`, `dq_mul.m`, `dq_conj.m`, `dq_to_pose.m`, `dq_power.m` | Álgebra de cuaterniones duales. |

## Cómo ejecutar

### Ventana interactiva (recomendado para probar el robot)

En **MATLAB** o **GNU Octave** con interfaz gráfica, desde la carpeta `matlab/`:

```matlab
interactivo
```

En GNU Octave desde la terminal:

```bash
cd matlab
octave --gui
interactivo
```

La ventana muestra el robot en 3D con sliders para cada articulación
(cinemática directa en vivo, la pose del marcador se actualiza al moverlos),
un botón **Home (reposo)** y un panel **Resolver IK** donde se escribe un
punto objetivo X, Y, Z en milímetros y el robot se coloca en la
configuración que alcanza ese punto (si existe). El solver de IK es el mismo
algoritmo (Levenberg-Marquardt con damping) y los mismos parámetros que usa
el gemelo digital.

> Nota: en Octave se usa el toolkit gráfico `qt` (el toolkit `gnuplot` no
> soporta los controles de la ventana). En sistemas sin Octave-Qt, usar
> MATLAB o instalar el paquete Qt de Octave.

### Script de validación (para la memoria de cálculo)

## Qué valida el script

1. **Tabla DH** del FABRI Creator (geometría MG996R) y home de servos.
2. **Cinemática directa por DH**: matrices A_i, marcos F0..F5 y pose del
   efector; comparación con el gemelo digital (error < 1e-4 mm, limitado por
   el redondeo de los valores impresos de referencia).
3. **Cinemática directa por cuaterniones duales**: cuaternión real y dual de la
   pose, comparación con el gemelo digital y con la solución DH (error ~1e-13).
4. **Pose del efector** (marcador perpendicular de 117 mm) en reposo y en
   configuración de prueba, con base (57 mm) y útil.
5. **Cinemática diferencial**: jacobiano geométrico clásico [v; ω], jacobiano
   por cuaterniones duales [ω; v], validación por diferencias finitas y contra
   el gemelo digital.
6. **Velocidad del efector** con ambos jacobianos.
7. **Interpolación de trayectoria ScLERP** (movimiento de tornillo) con
   cuaterniones duales entre dos poses.

## Convenciones del modelo

- **Unidades**: milímetros (longitudes) y radianes (ángulos).
- **Eslabón revolute**: `theta = q + theta0`, rotación `RotZ(theta)*RotX(alpha)`,
  traslación `(a*cos(theta), a*sin(theta), d)`.
- **Eslabón twist** (articulación 4): rotación `RotX(alpha + q)` y traslación
  constante `(a, d, 0)` — la articulación gira alrededor del eje X del marco
  anterior.
- **Base**: traslación `(0, 0, 57) mm`. **Útil**: marcador perpendicular de
  `117 mm` a lo largo del eje X local.
- **Home**: servos en `[90°, 90°, 81°, 95°, 60°]` corresponden al home
  cinemático `q = [0, 0, 0, 0, 0]` (robot en reposo, vertical).
- Cuaterniones en la forma `[a; b; c; d]` (a = parte escalar); cuaterniones
  duales como estructura con campos `real` y `dual`.

## Cuaterniones duales

La cinemática directa representa cada transformación de eslabón como un
cuaternión dual unitario `qd = (r, 0.5·t·r)` y compone la pose como producto:

```
QD_ee = QD_base · QD_1(q1) · ... · QD_5(q5) · QD_tool
```

La cinemática diferencial deriva el producto por la regla del producto
`d/dq_i (QD_ee) = prefix_i · QD_i' · suffix_{i+1}` y obtiene el twist del
efector `2·QD_ee'·conj(QD_ee)`, que se convierte a la velocidad del punto del
efector. Esta formulación no presenta singularidades de representación
(gimbal lock), a diferencia de los ángulos de Euler.