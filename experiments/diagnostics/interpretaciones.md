# Interpretaciones de los diagnósticos físicos

Registro provisional de la evidencia física disponible para evaluar errores de
trayectoria después del IK. Este archivo documenta las iteraciones 1 y 2 sin
modificar el modelo, la calibración ni el código de producción.

## Cómo leer este registro

La interpretación sigue primero la leyenda de la persona que realizó las
pruebas y después compara el dibujo con el GCODE nominal. Se separan siempre:

- **Observado**: rasgo visible en la imagen o dato explícito del archivo.
- **Hipótesis**: explicación compatible con lo observado, todavía no
  demostrada.
- **No demostrado**: conclusión que no puede sostenerse sin mediciones
  adicionales.

### Leyenda aplicada

| Marca | Interpretación utilizada |
|---|---|
| Rojo | Dibujo real producido por el robot. |
| Azul | Marca añadida por la persona para señalar residuo de construcción causado por una elevación insuficiente del marcador. No es una trayectoria independiente del robot. |
| Puntos negros y flechas numeradas | Orden de inicio y final de los trazos. |
| Símbolos tipo gamma (`Γ`) | Lugares donde cambia el ángulo. No se asigna un valor numérico salvo que esté escrito. |
| Valores escritos | Mediciones o anotaciones explícitas de la persona. |

Las imágenes no incluyen, para estas dos iteraciones, una escala de medición
calibrada. Por tanto, las separaciones y derivas se describen cualitativamente
salvo cuando el GCODE o la anotación proporciona un valor nominal.

## Fuentes revisadas

- `experiments/diagnostics/README.md`.
- `experiments/diagnostics/diag-01-overdraw-hv.gcode`.
- `experiments/diagnostics/diag-02-radial-in-out.gcode`.
- `experiments/diagnostics/results/prueba1` (imagen de la primera iteración).
- `experiments/diagnostics/results/prueba2.pdf` (imagen de la segunda iteración).
- `experiments/diagnostics/results/prueba2.csv` (registro temporal de comandos o posiciones de servos asociado a la segunda iteración).

## Criterio de diagnóstico post-IK

Estas pruebas sólo pueden justificar una compensación post-IK si el error
cartesiano es suficientemente repetible y se puede relacionar con posición,
sentido de movimiento o historial. La evidencia buscada es:

| Aspecto | Pregunta |
|---|---|
| Patrón nominal | ¿La figura ejecutada corresponde a la geometría programada? |
| Geometría visible | ¿Qué líneas, extremos, centros y separaciones aparecen realmente? |
| Centro común | ¿Los trazos que deberían compartir un punto vuelven al mismo punto? |
| Deriva inicio/final | ¿El regreso termina donde empezó o queda desplazado? |
| Dirección/historial | ¿El error cambia entre ida, vuelta, afuera-adentro y adentro-afuera? |
| Repetibilidad | ¿Se repite el mismo error en pasadas o ciclos equivalentes? |
| Pérdida de contacto | ¿Hay discontinuidad o adelgazamiento del rojo, separada del residuo azul? |

Una coincidencia visual no prueba por sí sola que el error sea estable. Del
mismo modo, un trazo incompleto no prueba por sí solo un problema geométrico.

---

## Iteración 1: `diag-01-overdraw-hv.gcode`

**Resultado revisado:** `results/prueba1`  
**Marcador anotado:** longitud `7.5 cm`.  
**Objetivo nominal:** observar histéresis direccional mediante sobretrazado
horizontal y vertical sin levantar el marcador.

### Nominal del GCODE

- Tres líneas horizontales: `y = -55, -40, -25`, de `x=170` a `x=350` y
  regreso al mismo extremo.
- Tres líneas verticales: `x = 220, 260, 300`, de `y=-60` a `y=-20` y regreso.
- Cada ida y vuelta se ejecuta con `G1`, sin levantar entre ambos sentidos.
- El archivo declara el plano de dibujo como `z=0` relativo al programa y el
  comentario lo sitúa en el área física de dibujo `z=80`.

### Observado en la imagen

- El rojo muestra el dibujo del robot: se distinguen dos de las tres líneas
  verticales principales, aproximadamente paralelas y con separación casi
  uniforme, además de tres líneas horizontales principales. También se
  observan segmentos rojos más débiles o fragmentados en el lado
  izquierdo/inferior.
- Los puntos negros y flechas numeradas identifican extremos y orden de los
  trazos; no se interpretan como geometría adicional.
- Las marcas azules aparecen junto a varios trazos rojos y fueron añadidas para
  identificar residuos de los desplazamientos con el marcador insuficientemente
  elevado o trazos cuya forma resulta de especial interes. 
  No se contabilizan como una segunda trayectoria robotizada.
- Hay símbolos tipo `Γ` en ubicaciones de cambio de dirección. La imagen no
  escribe un valor angular numérico para ellos.
- Se distinguen dos de las tres líneas verticales rojas principales. Son
  aproximadamente paralelas y conservan una separación casi uniforme a lo
  largo de su recorrido, según la observación de la imagen.
- Esta separación describe la posición relativa entre esas dos líneas
  verticales visibles; no debe atribuirse automáticamente a la separación
  entre los trazos de ida y vuelta de una misma línea, porque la fotografía no
  permite identificar cada sobretrazado individual con suficiente certeza.
- Algunas otras líneas rojas parecen coincidir o quedar muy próximas.

### Comparación y evidencia post-IK

| Aspecto | Evaluación provisional |
|---|---|
| Patrón nominal | Hay correspondencia cualitativa con una retícula de tres horizontales y tres verticales, pero la imagen no permite reconstruir con precisión todos los seis segmentos. |
| Geometría visible | La figura principal está deformada por segmentos incompletos y por la concentración de tinta en ciertos cruces/extremos. No se dispone de longitudes medidas. |
| Centro común | No es una prueba de centro común; no aplica directamente. Los cruces sugieren referencias compartidas entre líneas, pero no permiten cuantificar su coincidencia. |
| Inicio/final | Los puntos numerados permiten seguir el orden, pero no hay medición de la separación entre inicio y final de cada sobretrazo. No se puede afirmar cierre exacto. |
| Dirección/historial | El diseño sí prueba ida contra vuelta en cada línea. La imagen muestra dos líneas verticales visibles, paralelas y con separación casi uniforme, pero no permite asignar esa separación a una pareja ida-vuelta concreta ni estimarla en milímetros. |
| Repetibilidad | La coincidencia visual de algunos trazos es compatible con un componente repetible, pero esta imagen corresponde a una ejecución y no sustituye `diag-06`. |
| Contacto | Los segmentos rojos débiles o fragmentados pueden ser pérdida de contacto o transferencia irregular de tinta. Deben mantenerse separados del azul; la imagen no permite decidir entre esas posibilidades. |

### Hipótesis, sin atribuir causa raíz

- **Hipótesis:** existe un error dependiente del sentido o del historial en
  algunos segmentos, porque el patrón fue diseñado para producir ese contraste
  y algunos trazos rojos no tienen la misma continuidad o apariencia.
- **Hipótesis:** parte de la aparente separación visual está dominada por
  contacto/tinta y no por la posición cartesiana real.
- **No demostrado:** backlash, flexión, error de un eje concreto, error de
  tool frame, error de DH o un valor de compensación post-IK.
- **No demostrado:** que la línea azul sea un segundo trazo del robot; según la
  leyenda, es residuo de construcción anotado manualmente.

### Disclaimer de la iteración 1

Esta interpretación es **provisional, no exacta y no exhaustiva**. Se basa en
la inspección visual de una imagen sin escala calibrada y en el GCODE nominal;
no constituye una medición metrológica ni permite afirmar una causa raíz.

### Datos que faltan para cerrar esta iteración

- Separación roja ida-vuelta en el centro y en ambos extremos de cada línea.
- Identificación inequívoca de cada uno de los tres trazos horizontales y
  verticales.
- Repetición del patrón sobre el mismo papel o, preferentemente, el protocolo
  de `diag-06` con dispersión de vértices.
- Registro separado de zonas de contacto perdido y de marcas azules de viaje.

---

## Iteración 2: `diag-02-radial-in-out`

**Resultado revisado:** `results/prueba2.pdf`  
**Marcador anotado:** longitud `7.5 cm`.  
**Objetivo nominal:** comparar los sentidos centro→extremo y extremo→centro en
ocho radios, con especial atención al lado derecho.

### Nominal del GCODE

- Centro nominal común: `(260,-40)`.
- Ocho direcciones: E, NE, N, NO, O, SO, S y SE.
- Cada radio se dibuja centro→extremo y luego extremo→centro sin levantar.
- Los extremos están a 40 mm del centro en el GCODE: por ejemplo, E termina
  en `(300,-40)` y NE en `(288.3,-11.7)`.

### Mismatch entre README y GCODE

El `README.md` dice en la sección de `diag-02` que son “8 radios de 30mm”,
mientras que el GCODE dice 40 mm y sus coordenadas confirman 40 mm. Esta
interpretación toma **40 mm como nominal efectivo del archivo ejecutado** y
deja el conflicto documentado; no se presenta una longitud física medida como
resultado de la imagen.

### Observado en la imagen

- El rojo no aparece como ocho radios limpios que vuelven exactamente a un
  único punto. Se ve una figura ramificada con varios trazos verticales y
  horizontales/oblicuos agrupados alrededor de más de una zona central.
- La anotación negra transcribe, de forma visible, secuencias como `2→1→2`,
  `2→3→4`, `4→5→4`, `4→6→4`, `2→7→4`, `2→8→2` y `2→9→2`, además de
  `11→10→11`. Algunas secuencias están anotadas como incompletas.
- La lectura por grupos sugiere que los primeros retornos usan el entorno del
  punto 2, varios trazos posteriores convergen alrededor del punto 4, otros
  regresan al entorno del punto 2 y el último grupo usa el entorno de 11/10.
  La transcripción exacta de algunos flechados es incierta por la imagen.
- Por tanto, los extremos y los retornos no parecen referirse de forma
  consistente a un único centro físico común, aunque no se pueden obtener
  distancias en mm a partir de la fotografía.
- En este caso las marcas azules muestran trazos donde se siguió cierta
  curvatura que no puede ser interpretada completamente como líneas de
  construcción sino como intentos de dibujo de líneas diagonales
- Hay trazos rojos incompletos o muy débiles en los grupos anotados como
  incompletos. Son evidencia de continuidad de contacto/transcripción
  deficiente, no una medición directa de deriva cartesiana.

### Comparación y evidencia post-IK

| Aspecto | Evaluación provisional |
|---|---|
| Patrón nominal | La figura visible no conserva de manera clara la estructura de ocho radios con un centro común y radio 40 mm. |
| Geometría visible | Hay varios grupos de trazos y varios puntos de convergencia anotados; la diferencia es cualitativamente mayor que una simple falta de tinta en un extremo. |
| Centro común | Es la señal principal: los grupos anotados alrededor de 2, 4 y 11/10 no parecen cerrar en el mismo punto. Esto es compatible con deriva del retorno o con una trayectoria que depende del historial. |
| Inicio/final | Las secuencias muestran retornos a puntos diferentes según el grupo y algunos retornos incompletos. No hay coordenadas medidas para convertir esa deriva en mm. |
| Dirección/historial | La organización centro→extremo→centro permite sospechar dependencia del sentido y del estado previo. La imagen no separa por sí sola si domina el sentido, la posición angular, la extensión radial o su combinación. |
| Repetibilidad | Sólo se observa una hoja/iteración; no se puede estimar dispersión entre repeticiones. La existencia de varios centros dentro de la misma ejecución sí merece repetición controlada. |
| Contacto | Los trazos rojos incompletos se registran como posible pérdida de contacto o transferencia insuficiente. El azul en este caso particular no sigue la interpretación de la leyenbda |

### Relación con `prueba2.csv`

El CSV tiene una cabecera de tiempo y posiciones de servos (`j1_us` a
`j5_us`, además de `g_us`) y 837 registros. El tiempo llega aproximadamente a
71.6 s. En el registro visible, `j4_us` permanece en 1524 y `g_us` en 1183,
mientras las demás posiciones cambian por bloques suaves y repetidos.

Esto puede servir para reconstruir el orden temporal de comandos si se dispone
de la correspondencia entre muestras, movimientos y puntos de la imagen. No
contiene coordenadas cartesianas medidas ni la posición real del marcador sobre
el papel; por sí solo no confirma cuánto se desplazó el centro físico ni cuál
servo produjo la desviación observada.

### Hipótesis, sin atribuir causa raíz

- **Hipótesis:** existe una componente importante de deriva de retorno o error
  dependiente del historial, porque los radios nominalmente concurrentes
  parecen agruparse alrededor de centros físicos distintos.
- **Hipótesis:** la magnitud del error podría depender también de la posición
  angular o de la extensión del brazo, pero esta imagen no permite separar esos
  factores.
- **Hipótesis:** los trazos incompletos pueden coexistir con el problema
  geométrico principal como un fenómeno de contacto distinto.
- **No demostrado:** que la causa sea backlash, flexión gravitacional,
  holgura, tool frame, DH, offset, control de altura o un servo específico.
- **No demostrado:** que las posiciones del CSV representen fielmente la
  posición efectiva del TCP o del marcador en cada instante.

### Disclaimer de la iteración 2

Esta interpretación es **provisional, no exacta y no exhaustiva**. La señal de
centros no comunes es visual y cualitativa, la transcripción de algunos
flechados tiene incertidumbre y el CSV no es una medición cartesiana. El
resultado orienta la siguiente medición, pero no identifica una causa raíz ni
autoriza todavía una compensación post-IK concreta.

### Datos que faltan para cerrar esta iteración

- Medir en mm cada extremo y cada retorno, identificando el punto de inicio y
  el punto de llegada de cada radio.
- Repetir los ocho radios varias veces, manteniendo papel, marcador, altura y
  orden constantes.
- Registrar por separado centro→extremo y extremo→centro, junto con dirección
  (E/NE/N/NO/O/SO/S/SE).
- Confirmar si los grupos centrados en 2, 4 y 11/10 persisten entre pasadas.
- Correlacionar los puntos numerados con marcas temporales del CSV antes de
  relacionar una deriva con una articulación.

---

## Resumen entre iteraciones

| Señal | Iteración 1 | Iteración 2 |
|---|---|---|
| Error geométrico dominante | No cuantificado; algunas líneas parecen coincidir, con continuidad irregular. | Deriva cualitativa del centro común, más evidente que una mera pérdida de tinta. |
| Dependencia de dirección/historial | Diseñada para medirla, pero sin separación roja medida concluyente. | Compatible con dependencia del retorno y del grupo previo; requiere repetición. |
| Repetibilidad | No evaluable con una sola imagen. | No evaluable entre pasadas; hay variación de centro dentro de la ejecución. |
| Contacto | Posibles segmentos rojos débiles/fragmentados. | Retornos incompletos separados de los residuos azules. |
| Acción recomendada | Medir separaciones ida-vuelta antes de corregir. | Repetir y medir centros/retornos; no ajustar DH ni post-IK todavía. |

## Plantilla para futuras iteraciones

Para cada nuevo resultado conservar esta ficha mínima:

1. Archivo GCODE y archivo/imagen de resultado.
2. Marcador, papel, altura y condiciones que hayan cambiado.
3. Leyenda de colores y confirmación del orden numerado.
4. Patrón nominal con longitudes y centros tomados del GCODE actual.
5. Observaciones visibles, separadas de hipótesis.
6. Mediciones en mm y tolerancia de lectura.
7. Centro común, deriva de inicio/final, sentido/historial y repetibilidad.
8. Contacto perdido separado de residuo de construcción.
9. Mismatch documental o de configuración encontrado.
10. Disclaimer explícito: interpretación provisional, no exacta y no
    exhaustiva.
