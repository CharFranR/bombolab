# Calibración post-IK — protocolo de medición robusto

Este protocolo reemplaza al método anterior (cadenas de distancias + ángulos),
que producía matrices `A` inestables y no reproducibles. Acá se miden
**coordenadas absolutas** de una retícula de landmarks, con 3 pasadas para
medir la repetibilidad en el mismo archivo.

## Materiales

- `diag-08-landmarks.gcode` (cuadrícula 3×3 = 9 landmarks, 3 pasadas).
- Papel nuevo por corrida, bien plano y fijo con cinta.
- Regla rígida (mm). Escuadra si tenés (facilita medir perpendicular, pero no es indispensable).

## Ejecución

1. F5 en la web (recargar) para el plano de viaje nuevo (z=120).
2. Activar **"Modo calibración: dibujar SIN compensación post-IK"** (checkbox en
   la web, junto al estado de la calibración). Sin esto, las marcas se
   compensarían con la calibración vieja y la medición quedaría contaminada.
3. Cargar `diag-08-landmarks.gcode` con **autofit OFF** (escala 1:1).
4. Dejar que termine el dibujo completo.

El dibujo contiene:
- Una **L de referencia** (esquina superior-izquierda del área).
- **27 marcas**: 9 landmarks × 3 pasadas, cada uno un **punto de tinta**
  (el robot baja el marcador y espera 2 s, sin moverse).

### Sobre las 3 pasadas (simple)

Cada landmark se dibuja 3 veces; las 3 marcas de un mismo punto caen **juntas**
(son la misma posición repetida). **No tenés que distinguirlas**: medís las 3
marcas de cada punto en cualquier orden y el script usa su promedio y su
dispersión (independiente del orden). El barrido de cada pasada es el mismo:
L01..L09 en filas desde arriba (y=−25, −45, −65).

## Qué se mide y respecto a qué (la parte importante)

Cada landmark es un **punto de tinta** (mancha dejada por el dwell de 2 s).
Medís el **centro de la mancha**. Y el "respecto a qué" NO es una constelación
de distancias sueltas: es la posición **(x, y) de cada mancha respecto al
VÉRTICE de la L de referencia** que dibuja el robot.

La L materializa en el papel el **origen y los ejes del robot**:

```
                     origen de la L (vértice)
        L ───────────────────────────────────→  (eje X del robot)
        │
        │  (eje Y del robot)
        │
        ↓
        ·  ●        ●        ●     ← puntos de la cuadrícula (3×3)
        ·  ●        ●        ●
        ·  ●        ●        ●
```

- La **línea horizontal de la L** es el eje X: medís la coordenada x de cada
  mancha a lo largo de ella, apoyando la regla sobre esa línea.
- La **línea vertical de la L** (o la escuadra) es el eje Y: medís la
  coordenada y perpendicular a la regla.
- El vértice de la L es el origen. El robot dice que ese vértice está en
  (212, −12) en sus propias coordenadas — por eso tus mediciones quedan
  directamente en el sistema del robot.

### ¿Por qué alinear la regla a la L y no al borde del papel?

Porque el papel puede estar apoyado rotado respecto al robot. Si tu sistema de
medición estuviera rotado aunque sea 2°, esa rotación se metería DENTRO de la
matriz A y la calibración saldría deformada. Al alinear la regla a la L (que
el propio robot dibujó), medís "en el idioma del robot", y la rotación del
papel desaparece del problema.

Un detalle que no es un bug: la L puede salir torcida o desalineada (por el
error del robot). Eso está bien — es justamente el error que queremos capturar,
y al alinear la regla a la L lo estamos midiendo.

### Tabla de medición

Para cada mancha anotá (x, y) en mm respecto al vértice de la L. No hace falta
que el origen coincida con (212, −12) si preferís medir desde otro punto fijo
del papel: el modelo absorbe la traslación. Lo único que NO se puede perdonar
es la rotación (regla alineada a la L).

| Landmark | Nominal (x, y) | Pasada 1 (x, y) | Pasada 2 (x, y) | Pasada 3 (x, y) |
|----------|----------------|-----------------|-----------------|-----------------|
| L01 | (215, -25) | | | |
| L02 | (260, -25) | | | |
| L03 | (305, -25) | | | |
| L04 | (215, -45) | | | |
| L05 | (260, -45) | | | |
| L06 | (305, -45) | | | |
| L07 | (215, -65) | | | |
| L08 | (260, -65) | | | |
| L09 | (305, -65) | | | |

### Precisión esperada

- El centro de una mancha de tinta de ~2-4 mm se mide a ~±0.5-1 mm con regla.
- La repetibilidad entre pasadas (el script te la calcula) te dice de
  antemano si el error es estable. Si algún landmark dispersa más de 3 mm
  entre pasadas, la post-IK estática no va a poder corregirlo: es problema
  mecánico, no de calibración.

## Procesamiento

Volcá las mediciones en `measurements/landmarks.csv` (columnas
`landmark, pass, x_mm, y_mm`; el script genera la plantilla si la pedís) y
ejecutá:

```bash
python3 fit_calibration.py --csv experiments/diagnostics/measurements/landmarks.csv
```

El script reporta:

1. **Repetibilidad** por landmark (dispersión entre pasadas).
2. **RMS del ajuste** affine (`measured = A·commanded + b`).
3. **RMS de validación leave-one-out** (predice cada landmark con el fit de
   los otros 11 — detecta sobreajuste).
4. **Recentralización** de `b` alrededor del centro de la retícula (260, −45),
   para que el centro quede fijo (la convención que espera el código web).
5. Escribe `web/public/post-ik-calibration.json` (versión 1).

Si el RMS de ajuste o la validación quedan por encima de ~2-3 mm, el error no
es affine estático y hay que decidir: modelo mecánico (DH/offsets) o resolver
el backlash direccional primero.

## Los 7 dibujos anteriores — qué aportaron y por qué no alcanzan

Los 7 dibujos SÍ sirvieron, pero para **diagnóstico**, no para coeficientes:

- Confirmaron el tipo de error dominante: **direccional/backlash** (separaciones
  ida-vuelta de 11–14 mm, deriva de retorno, ángulos ~81° en vez de 90°), no un
  error suave de escala.
- Eso orienta la decisión de fondo: si el error no es repetible, la post-IK
  estática no aplica.

Por qué no dan coeficientes numéricos directos: sus mediciones son distancias
entre puntos de una **cadena** (sin un sistema de coordenadas global ni una
referencia de alineación angular). Con cadenas locales sin ancla, la rotación y
la traslación del sistema de medición se confunden con la matriz A — por eso
las A que salieron son inestables (a11: 0.62 / 0.65 / 0.40 para el mismo tipo
de dato) y el RMS fue ~4 mm. La L de referencia de este protocolo resuelve
justamente esa ambigüedad.
