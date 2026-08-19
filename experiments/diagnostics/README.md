# Diagnóstico post-IK — Protocolo de medición

Set de 7 dibujos GCODE diseñados para **discriminar la causa del error de
dibujo** con instrumentos de mano (regla, escuadra, cartabón, transportador).
Cada patrón ataca UNA firma específica. Todos están validados contra el IK
real del modelo (`web/src/motion/diagnostics.test.ts`) — ninguno debería ser
bloqueado por reachability al cargarlos con **autofit OFF** (escala 1:1).

## Cómo cargar (importante)

1. F5 en la web (recargar): los planos de viaje ahora van a **z=120** (40mm de
   despegue). Si no recargás, el robot viaja a la altura vieja.
2. Cargar cada archivo con **autofit OFF** (modo escala 1:1, el toggle que ya
   existe en la web).
3. Usar un papel nuevo para cada archivo, **bien plano y fijo** (cinta).
4. Dibujar cada patrón **3 veces sobre el mismo papel** (recargando el mismo
   archivo) para evaluar repetibilidad.

## Instrumentos

- Regla rígida (mm)
- Escuadra y cartabón (ángulos rectos y 30/60°)
- Transportador (ángulos finos)
- Lápiz para marcar los puntos a medir

---

## Metodología para documentar los dibujos físicos

Cada resultado físico puede enviarse como una imagen acompañada de una breve
descripción. Para evitar confundir las marcas del papel con la trayectoria del
robot, se utilizará esta convención:

| Marca | Significado |
|-------|-------------|
| **Rojo** | Trayectoria real dibujada por el robot. |
| **Azul** | Anotación manual junto a determinadas marcas rojas de construcción. Indica trazas producidas durante los desplazamientos con el marcador insuficientemente elevado; **no representa una trayectoria independiente del robot**. |
| **Puntos negros y flechas numeradas** | Lugares de inicio y final, en el orden en que se ejecutaron. |
| **Símbolos tipo Γ** | Ubicación de un cambio de ángulo. No se debe asumir un valor numérico a menos que esté escrito explícitamente. |
| **Ángulos numéricos** | Mediciones o valores anotados por la persona que realizó la prueba. |

### Información mínima por imagen

1. Número y nombre del diagnóstico.
2. Longitud y tipo de marcador, si cambió.
3. Leyenda de colores, si se utilizaron colores adicionales.
4. Orden de ejecución mediante puntos o flechas numeradas.
5. Ángulos medidos numéricamente cuando sea relevante.
6. Líneas que coinciden, se separan, se deforman o aparecen duplicadas.
7. Marcas de construcción observadas durante los viajes, separadas de la
   trayectoria roja real.

Las imágenes se interpretarán primero según esta leyenda y después se
compararán con el patrón GCODE nominal. No se propondrán cambios en DH,
offsets o calibración basándose únicamente en una anotación ambigua.

---

## Patrones

| # | Archivo | Discrimina | Medir |
|---|---------|-----------|-------|
| 01 | `diag-01-overdraw-hv.gcode` | **Histéresis direccional** (backlash) | Separación entre trazo de ida y vuelta |
| 02 | `diag-02-radial-in-out.gcode` | **Direccionalidad** afuera→adentro vs adentro→afuera (síntoma del lado derecho) | Longitud de cada radio, desvío de la vuelta, ondulación |
| 03 | `diag-03-escala.gcode` | **Error de escala vs error constante** (modelo vs offsets) | Lados y diagonales de los 3 rectángulos |
| 04 | `diag-04-rectangulo-70x46.gcode` | **Firma trapezoide / plano** (la prueba de 5 puntos completa) | 4 lados, 4 ángulos, diagonales, **P1–P4 (el lado que faltó)** |
| 05 | `diag-05-ondulacion.gcode` | **Ondulación: dinámica vs estática** | Ondulación del lado derecho a 3 velocidades |
| 06 | `diag-06-repetibilidad.gcode` | **Repetibilidad** (¿el error es estable?) | Dispersión entre las 3 pasadas del cuadrado |
| 07 | `diag-07-angulos.gcode` | **Ángulos y acumulación en esquinas** | Ángulos con transportador, cierre del triángulo |

---

## Protocolo por patrón

### 01 — overdraw HV (histéresis)

Cada línea se dibuja de ida y vuelta SIN levantar. Si no hay backlash, los dos
trazos coinciden (uno solo). Si hay backlash, se ven **dos líneas paralelas**.

- Medir la separación (mm) entre los 2 trazos, en el medio y en los extremos.
- Registrar por separado: horizontales (y=const, mueve J1) y verticales
  (x=const, mueve J2/J3).
- **Lectura**: separación uniforme = backlash de transmisión; separación que
  crece hacia un extremo = flexión bajo carga; línea doble solo en un eje =
  ese eje tiene el juego.

### 02 — radial in/out (direccionalidad)

8 radios de 30mm desde el centro. El síntoma reportado es que el lado derecho
empeora "de afuera hacia adentro".

- Medir cada radio (nominal 30mm) y anotar cuál se pasa/corta.
- Observar el trazo de vuelta (extremo→centro): ¿se desvía lateralmente?
  ¿Cuánto? ¿En qué direcciones (E, NE, SE)?
- **Lectura**: asimetría E vs O = comportamiento distinto según la posición de
  J1 (lado derecho = brazo más extendido). Si la vuelta se desvía SIEMPRE en
  las direcciones con x>200 → el efecto es función de la extensión del brazo
  (flexión por gravedad), no solo del sentido.

### 03 — escala (modelo vs offset)

3 rectángulos concéntricos (30×30, 50×50, 70×50). Dwell de 500ms en cada
esquina → puntos bien marcados.

- Medir los 4 lados de cada rectángulo y las 2 diagonales.
- Calcular el error: `medido − nominal` para cada lado.
- **Lectura**:
  - Error **proporcional** al tamaño (ej. +10% en los 3) → error de modelo
    (tool frame, escala, plano inclinado).
  - Error **constante en mm** (ej. +3mm en los 3) → offset/backlash.

### 04 — rectángulo 140×92 + centro (firma completa)

El heredero de la prueba de 5 puntos, con la proporción 3:2 del patrón
150×100 original (escalado al área segura MG996R) y dwell largo (800ms) para
medir con precisión. Centro P0 en (260,−40); P1 abajo-izq, P2 abajo-der,
P3 arriba-der, P4 arriba-izq.

Medir TODO y anotar en la tabla:

| Segmento | Nominal | Medido | Δ |
|----------|---------|--------|---|
| P1–P2 (abajo, lado largo) | 140 | | |
| P4–P3 (arriba, lado largo) | 140 | | |
| P1–P4 (izquierda) | 92 | | |
| P2–P3 (derecha) | 92 | | |
| Diag P1–P3 | 167.6 | | |
| Diag P4–P2 | 167.6 | | |
| P0–P1 … P0–P4 | 83.8 (los 4) | | |

Ángulos internos (transportador): P1 __ P2 __ P3 __ P4 __ (nominal 90°).

- **Lectura**: ancho arriba ≠ ancho abajo (trapezoide) → plano inclinado /
  error de z del modelo. Ángulo ≠ 90° en un solo lado → direccional.
  **Este patrón completa la firma 4 que faltó medir (P1–P4).**

### 05 — ondulación (dinámica vs estática)

La misma línea 3 veces a 20, 40 y 60 mm/s, más una ida-vuelta rápida.

- Observar el lado derecho (x > 210 aprox): ¿la ondulación está en las 3
  pasadas? ¿Cambia con la velocidad?
- **Lectura**: crece con velocidad → inercia/resonancia (no corregible por
  post-IK estático; se mitiga con velocidad o mecánica). Igual en las 3 →
  backlash/flexión estática (corregible post-IK).

### 06 — repetibilidad (el test más importante)

El mismo cuadrado 50×50 dibujado 3 veces con pen up entre pasadas. Las 3
pasadas deberían caer encima.

- Medir la dispersión máxima de las marcas de cada esquina (la mayor distancia
  entre las 3 marcas del mismo vértice).
- **Lectura**: <1mm → error REPETIBLE → **post-IK viable**. >3mm → error no
  repetible (holgura variable, fricción, contacto) → post-IK NO sirve.

### 07 — ángulos y cierre

Triángulo rectángulo (catetos 60mm) + cuadrado 40×40 compartiendo vértice.

- Ángulos con transportador: los 3 del triángulo (90°, 45°, 45°) y los 4 del
  cuadrado (90°).
- Verificar que el triángulo cierra: la hipotenusa dibujada debe unir
  exactamente los extremos de los catetos.
- **Lectura**: si los ángulos de una misma figura son sistemáticos (ej. todos
  92°) → error de plano/modelo. Si el cierre falla pero los lados rectos
  están bien → error de esquina (backlash en reversión de J1/J2).

---

## Plan de decisión (qué hacemos con los números)

1. **Mediciones del robot** (antes de dibujar): ver sección siguiente.
2. **diag-06** primero: si la dispersión >3mm, **no hay post-IK que valga** —
   el siguiente paso es mecánico (holguras, sujeción del marcador).
3. Si es repetible:
   - **diag-03** decide modelo vs offset.
   - **diag-04** decide plano inclinado vs no.
   - **diag-01 + 02** cuantifican la histéresis direccional por eje → la
     tabla `Δq(q, signo)` para la etapa post-IK.
   - **diag-05** descarta/confirma el componente dinámico.
4. Con los números: diseño de la etapa post-IK (compensación direccional
   articular + mapa cartesiano `E(x,y)`) o corrección del modelo (tool frame,
   base, plano) según corresponda.

---

## Mediciones del robot (punto 1 del plan) — para corregir el modelo

Estas medidas se toman UNA vez, con la cinta métrica/regla, y validan o
corrigen los parámetros del modelo:

| Parámetro del modelo | Valor actual | Medir en el robot |
|----------------------|--------------|-------------------|
| Altura base (suelo→yaw) | `base_transform z=57` | Altura real del piso al eje del yaw |
| `d1` (yaw→J2) | 65 | Distancia real del punto del offset (15mm) al hombro (según el DH del modelo) |
| Largos de eslabones (brazo, antebrazo) | del DH | Medir entre ejes de rotación con el brazo extendido |
| Tool frame (muñeca→punta) | 117 | Distancia real del eje J5 a la punta del marcador (sobresaliente) |
| Altura muñeca→suelo en pose de dibujo | modelo: 137 (57+80) | Medir real en pose z=80: ¿coincide? Si NO → error de z en el modelo |
| Altura del papel sobre el suelo | — | Altura real de la mesa/papel |
| Paralelismo papel | — | ¿El papel está paralelo al plano XY del robot? (nivel de burbuja) |

**La medida clave es la altura muñeca→suelo**: si el modelo dice 137mm y el
robot real muestra otra cosa, TODOS los errores XY dependientes de la
configuración quedan explicados y se corrigen en el modelo, no en post-IK.
