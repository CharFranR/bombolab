// =============================================================
// Análisis Cinemático y Dinámico del Robot FABRI Creator
// =============================================================

#set document(
  title: "Análisis Cinemático y Dinámico del Robot FABRI Creator",
  author: "Cruz, Reyes, Ruíz, Vanegas",
  date: datetime(year: 2026, month: 07, day: 30),
)

#set text(
  font: ("Times New Roman", "Liberation Serif"),
  size: 12pt,
  lang: "es",
)

#set par(
  justify: true,
  leading: 1em,
  first-line-indent: (amount: 1.27cm, all: true),
)

// Encabezados APA 7ma edición
#show heading.where(level: 1): it => {
  set text(weight: "bold", size: 12pt)
  set align(center)
  set par(first-line-indent: 0cm)
  block(above: 24pt, below: 12pt)[#it.body]
}

#show heading.where(level: 2): it => {
  set text(weight: "bold", size: 12pt)
  set par(first-line-indent: 0cm)
  block(above: 24pt, below: 12pt)[#it.body]
}

#show heading.where(level: 3): it => {
  set text(weight: "bold", style: "italic", size: 12pt)
  set par(first-line-indent: 0cm)
  block(above: 24pt, below: 12pt)[#it.body]
}

// =============================================================
// PORTADA INSTITUCIONAL
// =============================================================

#set page(
  numbering: none,
  header: none,
  margin: 0pt,
  background: image("portada.png", width: 100%, height: 100%, fit: "cover"),
)

#set par(first-line-indent: 0cm)

#align(center)[
  #v(2cm)
  #image("logo.png", width: 60%)
  #v(1cm)
  #text(weight: "bold", size: 20pt)[Análisis Cinemático y Dinámico del Robot FABRI Creator]
  #v(0.3cm)
  #text(size: 14pt)[Robótica]
  #v(3cm)
]

#pad(left: 2.54cm)[
  *Autores:*
  #v(0.2cm)
  Jean Carlos Cruz Rodríguez
  #linebreak()
  Oscar Francisco Reyes Guevara
  #linebreak()
  David Josué Ruíz Reyes
  #linebreak()
  Misael Antonio Vanegas Pacheco
  #v(0.5cm)
  *Docente:*
  #v(0.2cm)
  Ing. María Martha
  #v(0.5cm)
  *Fecha:*
  #v(0.2cm)
  30 de julio de 2026
]

#pagebreak()

// =============================================================
// CUERPO DEL DOCUMENTO
// =============================================================

#set page(
  numbering: "1",
  header: context {
    if counter(page).get().first() > 1 [
      #set text(size: 9pt, style: "italic")
      #h(1fr) Análisis Cinemático y Dinámico del Robot FABRI Creator
    ]
  },
  margin: (top: 2.5cm, bottom: 2.5cm, left: 2.8cm, right: 2.8cm),
)

#set par(first-line-indent: (amount: 1.27cm, all: true))

= Introducción

El FABRI Creator es un brazo robótico educativo de 5 grados de libertad (GDL) construido con servomotores MG996R  y MG90S y una placa Arduino Nano . Su modelo cinemático se basa en la convención de Denavit-Hartenberg (DH) estándar, que asigna cuatro parámetros por eslabón para describir la transformación entre sistemas de referencia consecutivos.

Este documento presenta el análisis cinemático y dinámico completo del robot FABRI Creator. Todas las expresiones se derivan a partir de los parámetros medidos directamente del robot físico y se evalúan numéricamente en la configuración home ($q_i = 0$, $i = 1, dots, 5$). Los valores numéricos se obtuvieron mediante una herramienta computacional que implementa el modelo cinemático y han sido contrastados con el comportamiento del robot real, como se detalla en la sección de verificación. El documento cubre: tabla DH, matrices de transformación homogénea y la MTH global del efector, conversión de la actitud a cuaternión unitario, formulación de la pose completa mediante cuaterniones duales, jacobiano geométrico del efector final, cinemática diferencial de cada eslabón (velocidades angulares y lineales de los centros de masa), análisis de centros de masa con parámetros másicos estimados, formulación lagrangiana, ecuaciones de energía cinética y potencial por eslabón, matriz de fuerzas centrípetas y de Coriolis, ecuaciones de potencia, análisis de singularidades cinemáticas y un caso de prueba numérico que evalúa todos los desarrollos en un vector de estado articular explícito.

= Parámetros de Denavit-Hartenberg

== Convención

Se emplea la convención DH estándar. La matriz de transformación del eslabón $i$ se construye como:

$ A_i = "Rot"_Z (theta_i) · "Trans"_Z (d_i) · "Trans"_X (a_i) · "Rot"_X (alpha_i) $

Los cuatro parámetros son: $theta_i$ (rotación alrededor de $Z_(i-1)$, contiene la variable articular para juntas rotacionales), $d_i$ (desplazamiento en $Z_(i-1)$), $a_i$ (desplazamiento en $X_i$, longitud del eslabón) y $alpha_i$ (rotación en $X_i$, torsión entre ejes $Z$). Para la junta 4 (Twist), la Ecuación 1 no es aplicable; su tratamiento específico se detalla en la Sección 3.

== Tabla DH del FABRI Creator

#set par(first-line-indent: 0cm)
#figure(
  table(
    columns: 6,
    align: center + horizon,
    stroke: (x, y) => (
      left: if x > 0 { 0.4pt },
      top: if y > 0 { 0.4pt },
    ),
    table.header[$i$][$alpha_i$][$a_i$ (mm)][$d_i$ (mm)][$theta_i$][Tipo],
    [1], [$-pi/2$], [15], [85], [$q_1$], [Rotacional $(Z_0)$],
    [2], [0], [120], [0], [$q_2 - pi/2$], [Rotacional $(Z_1)$],
    [3], [$-pi/2$], [90], [0], [$q_3 + pi/2$], [Rotacional $(Z_2)$],
    [4], [$pi/2$], [35], [15], [—], [Twist $(X_3)$],
    [5], [0], [0], [0], [$q_5$], [Rotacional $(Z_4)$],
  ),
  caption: [Tabla DH del robot FABRI Creator. Longitudes en milímetros, ángulos en radianes. La junta 4 (Twist) no sigue la Ecuación 1: rota alrededor del eje $X_3$ del antebrazo. Su variable articular $q_4$ se suma a $alpha_4$, y la traslación se aplica como $(a_4, d_4, 0)$ en el sistema local sin rotar (ver Ecuación 7).]
)
#set par(first-line-indent: (amount: 1.27cm, all: true))

Las juntas 1 (base, yaw), 2 (hombro) y 3 (codo) son rotacionales estándar. La junta 4 es de tipo Twist: rota alrededor del eje $X_3$ (dirección del antebrazo) y su traslación es $(a_4, d_4, 0) = (35, 15, 0)$ en el sistema local de la junta. La junta 5 (muñeca pitch) es rotacional estándar. Los offsets $q_2 - pi/2$ y $q_3 + pi/2$ aseguran que en home ($q_i = 0$) el brazo quede extendido verticalmente.

= Matrices de Transformación Homogénea

== Matriz Genérica (Juntas Rotacionales Estándar)

Para las juntas 1, 2, 3 y 5, la matriz homogénea $4 times 4$ se obtiene sustituyendo los parámetros en la Ecuación 1:

#set par(first-line-indent: 0cm)
$ A_i = mat(
  cos theta_i,  -sin theta_i cos alpha_i,   sin theta_i sin alpha_i,   a_i cos theta_i;
  sin theta_i,   cos theta_i cos alpha_i,  -cos theta_i sin alpha_i,   a_i sin theta_i;
  0,             sin alpha_i,               cos alpha_i,               d_i;
  0,             0,                         0,                         1;
) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

== Matriz de la Junta Twist (Eslabón 4)

La junta 4 del FABRI Creator es de tipo Twist: rota alrededor del eje $X$ del sistema local, no de $Z$. Su matriz de transformación se construye como:

$ A_4 = "Iso3"("traslación" = (a_4, d_4, 0), space "rotación" = "Rot"_X(alpha_4)) $

Donde $alpha_4 = q_4 + pi/2$. En forma matricial, con $a_4 = 35$, $d_4 = 15$:

#set par(first-line-indent: 0cm)
$ A_4 = mat(
  1,   0,        0,       35;
  0,  -sin q_4,  -cos q_4,  15;
  0,   cos q_4,  -sin q_4,   0;
  0,   0,        0,        1;
) space "Home:" space A_4(0) = mat(
  1,   0,   0,   35;
  0,   0,  -1,   15;
  0,   1,   0,    0;
  0,   0,   0,    1;
) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Nótese que la traslación es constante — no depende de $q_4$. Esto difiere de lo que produciría la fórmula RotX·TransX·TransY, donde la traslación sí rotaría con $alpha_4$. La Ecuación 7 refleja fielmente la implementación real del código.

== Matrices de los Eslabones Restantes

Sustituyendo los parámetros de la Tabla 1 en la Ecuación 3, y evaluando en home:

#set par(first-line-indent: 0cm)

*Eslabón 1* ($alpha_1 = -pi/2$, $a_1 = 15$, $d_1 = 85$, $theta_1 = q_1$):

$ A_1 = mat(
  cos q_1,  0,  -sin q_1,  15 cos q_1;
  sin q_1,  0,   cos q_1,  15 sin q_1;
  0,       -1,   0,        85;
  0,        0,   0,         1;
) space A_1(0) = mat(
  1,   0,   0,   15;
  0,   0,   1,    0;
  0,  -1,   0,   85;
  0,   0,   0,    1;
) $

*Eslabón 2* ($alpha_2 = 0$, $a_2 = 120$, $d_2 = 0$, $theta_2 = q_2 - pi/2$):

$ A_2 = mat(
  sin q_2,   cos q_2,   0,   120 sin q_2;
  -cos q_2,  sin q_2,   0,  -120 cos q_2;
  0,         0,         1,    0;
  0,         0,         0,    1;
) space A_2(0) = mat(
   0,   1,   0,     0;
  -1,   0,   0,  -120;
   0,   0,   1,     0;
   0,   0,   0,     1;
) $

*Eslabón 3* ($alpha_3 = -pi/2$, $a_3 = 90$, $d_3 = 0$, $theta_3 = q_3 + pi/2$):

$ A_3 = mat(
  -sin q_3,  0,  -cos q_3,  -90 sin q_3;
   cos q_3,  0,  -sin q_3,   90 cos q_3;
   0,       -1,   0,          0;
   0,        0,   0,          1;
) space A_3(0) = mat(
   0,   0,  -1,    0;
   1,   0,   0,   90;
   0,  -1,   0,    0;
   0,   0,   0,    1;
) $

*Eslabón 5* ($alpha_5 = 0$, $a_5 = 0$, $d_5 = 0$, $theta_5 = q_5$):

$ A_5 = mat(
  cos q_5,  -sin q_5,  0,  0;
  sin q_5,   cos q_5,  0,  0;
  0,         0,        1,  0;
  0,         0,        0,  1;
) space A_5(0) = mat(
  1,   0,   0,   0;
  0,   1,   0,   0;
  0,   0,   1,   0;
  0,   0,   0,   1;
) $

#set par(first-line-indent: (amount: 1.27cm, all: true))

== Cinemática Directa

La transformación acumulada del sistema $i$ al sistema mundo es:

$ bold(T)_(0,i)(bold(q)) = bold(T)_("base") · A_1 · A_2 · dots · A_i $

Donde $bold(T)_("base")$ es una traslación pura $(0, 0, 57)$ mm (altura de la base). El efector final incluye $bold(T)_("tool")$: traslación $(75, 0, 0)$ mm (portamarcador).

Multiplicando las matrices en cadena para home ($q_i = 0$) y verificando cada producto mediante multiplicación matricial explícita, se obtienen las siguientes posiciones (sin incluir la base de 57 mm, que se suma al componente $z$):

#set par(first-line-indent: 0cm)
- $bold(p)_1 = (15, 0, 85)$ mm
- $bold(p)_2 = (15, 0, 205)$ mm
- $bold(p)_3 = (105, 0, 205)$ mm
- $bold(p)_4 = (140, -15, 205)$ mm
- $bold(p)_5 = (140, -15, 205)$ mm
#set par(first-line-indent: (amount: 1.27cm, all: true))

Con la base: $z_i arrow.r z_i + 57$. Con la herramienta: $bold(p)_("ee") = bold(p)_5 + (75, 0, 0) = (215, -15, 205)$ mm (sin base) o $(215, -15, 262)$ mm (con base). El desplazamiento en $y = -15$ mm del frame 4 y 5 proviene del parámetro $d_4 = 15$ mm de la junta Twist, que introduce un offset lateral en el sistema de referencia del antebrazo.

Las matrices de rotación $bold(R)_(0,i)$ en home se presentan en la Sección 9.

== Interpretación Geométrica de las Matrices de los Eslabones

Las matrices $A_i$ no son meras tablas de números: cada columna tiene un significado cinemático, y los elementos $+-1$ que aparecen al evaluar en home son exactamente los senos y cosenos de las torsiones $alpha_i = +- pi/2$ y de los offsets $theta_i = +- pi/2$ de la Tabla 1. Esta subsección justifica cada elemento no trivial de las matrices presentadas.

=== Lectura de las Columnas

La parte de rotación de $A_i$ es el producto:

#set par(first-line-indent: 0cm)
$ "Rot"_Z (theta_i) · "Rot"_X (alpha_i) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

es decir, la rotación de la junta (alrededor de $Z_(i-1)$) seguida de la torsión fija del eslabón. Las columnas de la submatriz de rotación son los ejes del frame $i$ expresados en el frame $i-1$: columna 1 = $bold(X)_i$, columna 2 = $bold(Y)_i$, columna 3 = $bold(Z)_i$; la cuarta columna es el origen $bold(p)_i$.

Cuando la torsión es no nula, el patrón clásico de rotación de junta $[cos theta, -sin theta; sin theta, cos theta]$ no aparece como bloque contiguo: la torsión $"Rot"_X (alpha_i)$ permuta los destinos de los ejes $bold(Y)$ y $bold(Z)$, de modo que el patrón queda repartido entre la columna 1 y la columna 3. Por ejemplo, la rotación de la base no se ve "limpia" en $A_1$ precisamente por esta permutación.

=== Eslabón 1 — la rotación de la base (yaw)

La junta 1 es el yaw de la base: $theta_1 = q_1$ rota alrededor del eje vertical $Z_0$. Su rotación aparece en $A_1$ premultiplicada por la torsión $alpha_1 = -pi/2$:

#set par(first-line-indent: 0cm)
$ "Rot"_Z (q_1) · "Rot"_X (-pi/2) = mat(cos q_1, 0, -sin q_1; sin q_1, 0, cos q_1; 0, -1, 0) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

La rotación de la base queda repartida en las columnas: la columna 1, $(cos q_1, sin q_1, 0)$, es la primera columna de $"Rot"_Z (q_1)$ —la base girando sobre $X_0$— y la columna 3, $(-sin q_1, cos q_1, 0)$, es la segunda columna de $"Rot"_Z (q_1)$ —el eje del hombro $Z_1$ girando alrededor de la vertical cuando la base rota. El elemento $-1$ de la fila 3, columna 2 es $sin(alpha_1) = sin(-pi/2) = -1$: la torsión negativa, que acuesta el frame 1 (el eje $Z_1$ del hombro queda en $+Y$ y el eje $Y_1$ apunta a $-Z$). En home ($q_1 = 0$) la rotación de la base es la identidad y el giro no se observa; evaluando en $q_1 = pi/2$ el origen rota de $(15, 0, 85)$ a $(0, 15, 85)$ mm, lo que confirma el giro alrededor del eje vertical.

=== Eslabón 2 — el offset (no es torsión)

El elemento $-1$ de la fila 2, columna 1 de $A_2$ no proviene de una torsión ($alpha_2 = 0$) sino del offset de postura $theta_2 = q_2 - pi/2$: en home $sin(theta_2) = sin(-pi/2) = -1$. Es importante distinguir los dos orígenes posibles de los $-1$: la torsión del eslabón ($alpha$) o el offset angular de la tabla ($theta$).

=== Eslabón 3 — la inversión del antebrazo

Con $theta_3 = q_3 + pi/2$ y $alpha_3 = -pi/2$, en home $sin(theta_3) sin(alpha_3) = sin(pi/2) sin(-pi/2) = -1$ (fila 1, columna 3) y $sin(alpha_3) = -1$ (fila 3, columna 2). Geométricamente, la combinación de ambos parámetros invierte el frame 3 respecto al frame 0: $Z_3$ apunta hacia $-Z_0$ e $Y_3$ hacia $-Y_0$. Esa inversión es la que permite que el antebrazo cuelgue en la dirección de la gravedad y que el codo lo doble hacia abajo, que es la postura natural de trabajo del robot.

=== Eslabón 4 (twist) — la torsión variable

En home la matriz de la junta twist tiene $-1$ en la fila 2, columna 3: $-sin(alpha_4) = -sin(pi/2) = -1$, con $alpha_4 = q_4 + pi/2$. En esta junta la "torsión" es la propia variable articular (excepción declarada en la Tabla 1): el twist hace girar la muñeca alrededor del eje longitudinal del antebrazo.

=== Eslabón 5 — sin torsión ni offset

Con $alpha_5 = 0$ y $theta_5 = q_5$, en home la matriz es la identidad: no hay elementos negativos porque no intervienen senos de $+-pi/2$. Si el eslabón tuviera torsión, aparecerían $+-1$ en la fila 3.

=== Resumen: origen de cada elemento $-1$

#set par(first-line-indent: 0cm)
#figure(
  table(
    columns: 4,
    align: center + horizon,
    stroke: (x, y) => (
      left: if x > 0 { 0.4pt },
      top: if y > 0 { 0.4pt },
    ),
    table.header[Eslabón][Elemento][Origen trigonométrico][Efecto geométrico],
    [1], [fila 3, col 2 = $-1$], [$sin(alpha_1) = sin(-pi/2)$], [Torsión: $Z_1$ en $+Y$ (eje del hombro)],
    [2], [fila 2, col 1 = $-1$], [$sin(theta_2) = sin(-pi/2)$], [Offset de postura (home)],
    [3], [fila 1, col 3 = $-1$], [$sin(theta_3) sin(alpha_3) = -1$], [Inversión del frame 3],
    [3], [fila 3, col 2 = $-1$], [$sin(alpha_3) = -1$], [Torsión: $Z_3$ hacia $-Z_0$],
    [4], [fila 2, col 3 = $-1$], [$-sin(alpha_4)$, $alpha_4 = q_4 + pi/2$], [Twist sobre el eje del antebrazo],
    [5], [—], [$alpha_5 = 0$, $theta_5 = 0$], [Identidad en home],
  ),
  caption: [Origen de cada elemento $-1$ de las matrices de los eslabones en home.]
)
#set par(first-line-indent: (amount: 1.27cm, all: true))

La coherencia de estos signos se valida con la cinemática: las posiciones en home de la Tabla 2, el jacobiano de la Sección 6 y la herramienta computacional reproducen exactamente estos valores. Un signo erróneo en cualquier torsión u offset produciría posiciones o velocidades incompatibles con el robot físico.

= Cuaternión de la Actitud del Efector

== Motivación

Los ángulos de Euler y las matrices de rotación presentan limitaciones para el control y la generación de trayectorias: las representaciones de Euler sufren de singularidades de representación (gimbal lock) y las matrices requieren nueve parámetros con seis restricciones de ortonormalidad. El cuaternión unitario (cuaternión de rotación) es la representación estándar en robótica: cuatro parámetros con una única restricción de norma, sin singularidades de representación y con composición bilineal de bajo costo computacional.

Un cuaternión unitario se define como:

#set par(first-line-indent: 0cm)
$ bold(q)_r = (q_0, q_1, q_2, q_3) = (cos (theta/2), space sin (theta/2) bold(v)), quad || bold(q)_r || = 1 $
#set par(first-line-indent: (amount: 1.27cm, all: true))

donde $bold(v)$ es el eje de rotación unitario y $theta$ el ángulo de giro. La rotación de un vector $bold(p)$ se expresa como $bold(p)' = bold(q)_r ⊗ bold(p) ⊗ bold(q)_r^star$, donde $⊗$ denota el producto de Hamilton y $bold(q)_r^star = (q_0, -q_1, -q_2, -q_3)$ el conjugado. El par $(bold(q)_r, -bold(q)_r)$ representa la misma rotación; se elige siempre la representación que entrega el algoritmo de conversión.

== Conversión Matriz de Rotación a Cuaternión (Método de la Traza)

Dada la matriz de rotación $bold(R) in "SO"(3)$, el cuaternión unitario correspondiente se obtiene mediante el método de la traza (Shepperd). Para $tr(bold(R)) > 0$:

#set par(first-line-indent: 0cm)
$ S = 2 sqrt(tr(bold(R)) + 1), quad q_0 = S/4, quad q_1 = (R_(32) - R_(23))/S, quad q_2 = (R_(13) - R_(31))/S, quad q_3 = (R_(21) - R_(12))/S $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Cuando $tr(bold(R)) <= 0$ se emplea la rama correspondiente al mayor elemento diagonal, que maximiza la estabilidad numérica; todas las ramas producen el mismo cuaternión (salvo el signo global). La evaluación paso a paso sobre la actitud del caso de prueba se desarrolla en la sección de Caso de Prueba Numérico.

== Evaluación en Home

La actitud del efector en home es $bold(R)_(0,5)(0) = "Rot"_X (pi/2)$ (rotación de $pi/2$ alrededor del eje $X$). Aplicando el método de la traza: $tr(bold(R)) = 1$, $S = 2 sqrt(2)$, por lo que:

#set par(first-line-indent: 0cm)
$ bold(q)_r(0) = (sqrt(2)/2, -sqrt(2)/2, 0, 0) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

El signo negativo en $q_1$ es la representación que entrega el algoritmo; $-(sqrt(2)/2, -sqrt(2)/2, 0, 0)$ es la representación equivalente con eje $+X$.

= Cuaternión Dual de la Pose

== Motivación

Un cuaternión unitario codifica únicamente la orientación. La pose completa —rotación más traslación— requiere dos objetos. Los cuaterniones duales representan la transformación rígida $T = (bold(R), bold(t)) in "SE"(3)$ como un único elemento algebraico: el cuaternión dual unitario.

== Definición y Construcción desde la MTH

Un número dual es $hat(a) = a_r + epsilon a_d$, con $epsilon^2 = 0$ y $a_r, a_d in RR$ (partes real y dual). Un cuaternión dual es un cuaternión sobre el anillo de los números duales:

#set par(first-line-indent: 0cm)
$ hat(bold(q)) = bold(q)_r + epsilon bold(q)_d $
#set par(first-line-indent: (amount: 1.27cm, all: true))

donde $bold(q)_r$ y $bold(q)_d$ son cuaterniones. Dada la MTH del efector con rotación $bold(R)$ y traslación $bold(t) = (t_x, t_y, t_z)$:

#set par(first-line-indent: 0cm)
$ bold(q)_r = "cuaternión de" bold(R), quad bold(q)_d = 1/2 bold(t) ⊗ bold(q)_r $
#set par(first-line-indent: (amount: 1.27cm, all: true))

con $bold(t) = (0, t_x, t_y, t_z)$ (cuaternión puro) y $⊗$ el producto de Hamilton. El cuaternión dual resultante es unitario, $hat(bold(q)) ⊗ hat(bold(q))^star = 1$, y transforma un punto como $bold(p)' = hat(bold(q)) ⊗ bold(p) ⊗ hat(bold(q))^star$.

== Justificación para Control y Generación de Trayectorias

- *Representación compacta y libre de singularidades.* La pose se codifica con ocho escalares y una restricción de norma dual; no presenta las singularidades de representación de los ángulos de Euler ni la redundancia (16 parámetros, 6 restricciones) de las matrices homogéneas.

- *Interpolación de movimiento helicoidal (screw).* Todo desplazamiento rígido entre dos poses es un movimiento de tornillo (eje, ángulo y paso). La interpolación de cuaterniones duales con normalización —análoga dual del slerp, conocida como screw linear interpolation (Sclerp/DLERP)— genera trayectorias de tornillo uniformes en las que rotación y traslación avanzan acopladas. Esto es natural para trayectorias de herramienta, donde el FABRI Creator debe mantener la actitud del marcador mientras desplaza la punta.

- *Adecuación al control.* El error de pose entre el efector y la referencia se expresa como un cuaternión dual unitario (error de tornillo), lo que permite leyes de control directamente en el espacio de la tarea, sin desacoplar posición y orientación ni incurrir en las singularidades de representación de los ángulos de Euler.

- *Isomorfismo con SE(3).* El grupo de cuaterniones duales unitarios es isomorfo al grupo de movimientos rígidos; las operaciones de composición de poses (cinemática directa) y de velocidades de tornillo se preservan algebraicamente con el mismo producto bilineal.

== Evaluación en Home

En home la pose del efector es $bold(R) = "Rot"_X (pi/2)$ y $bold(t) = bold(p)_("ee") = (140, -15, 205)$ mm. Con el cuaternión de la Sección anterior:

#set par(first-line-indent: 0cm)
$ bold(q)_d(0) = 1/2 bold(t) ⊗ bold(q)_r(0) = (35 sqrt(2), 35 sqrt(2), -55 sqrt(2), 47.5 sqrt(2)) = (49.50, 49.50, -77.78, 67.18) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Verificación: $2 bold(q)_d ⊗ bold(q)_r^star = (140, -15, 205) = bold(p)_("ee")$, lo que confirma que $hat(bold(q))(0)$ reconstruye exactamente la pose home. La evaluación completa del caso de prueba se desarrolla en la sección de Caso de Prueba Numérico.

= Jacobiano Geométrico del Efector Final

== Definición

El jacobiano geométrico $bold(J) in RR^(6 times 5)$ satisface:

#set par(first-line-indent: 0cm)
$ mat(bold(v); bold(omega)) = bold(J)(bold(q)) · dot(bold(q)) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Donde $bold(v)$ y $bold(omega)$ son las velocidades lineal y angular del efector en coordenadas mundo. Para una junta rotacional genérica con eje $bold(zeta)_(i-1)$:

$ bold(J)_i = mat(bold(zeta)_(i-1) times (bold(p)_("ee") - bold(p)_(i-1)); bold(zeta)_(i-1)) $

== Ejes de Rotación en Home

Los ejes $bold(zeta)_(i-1)$ se extraen de las matrices de rotación acumuladas $bold(R)_(0,i-1)$: para juntas rotacionales estándar es la tercera columna ($bold(z)_(i-1)$); para la junta Twist es la primera columna de $bold(R)_(0,3)$. En home:

#set par(first-line-indent: 0cm)
$ bold(z)_0 = (0, 0, 1)^T space bold(z)_1 = (0, 1, 0)^T space bold(z)_2 = (0, 1, 0)^T space bold(x)_3 = (1, 0, 0)^T space bold(z)_4 = (0, 1, 0)^T $
#set par(first-line-indent: (amount: 1.27cm, all: true))

== Evaluación Numérica

Con $bold(p)_("ee") = (140, -15, 205)$ (sin base ni tool) y las posiciones de la Sección 3:

#set par(first-line-indent: 0cm)

*Columna 1* ($bold(z)_0$, $bold(p)_0 = (0, 0, 0)$):

$ bold(z)_0 times (bold(p)_("ee") - bold(p)_0) = (0, 0, 1) times (140, -15, 205) = (15, 140, 0) $

*Columna 2* ($bold(z)_1$, $bold(p)_1 = (15, 0, 85)$):

$ bold(z)_1 times (bold(p)_("ee") - bold(p)_1) = (0, 1, 0) times (125, -15, 120) = (120, 0, -125) $

*Columna 3* ($bold(z)_2$, $bold(p)_2 = (15, 0, 205)$):

$ bold(z)_2 times (bold(p)_("ee") - bold(p)_2) = (0, 1, 0) times (125, -15, 0) = (0, 0, -125) $

*Columna 4* ($bold(x)_3$, $bold(p)_3 = (105, 0, 205)$). La junta Twist tiene traslación constante en su implementación: el origen del frame 4 no se desplaza al rotar. Por tanto, la contribución lineal es nula:

$ bold(J)_4 = mat(bold(0); bold(x)_3) = (0, 0, 0, 1, 0, 0)^T $

*Columna 5* ($bold(z)_4$, $bold(p)_4 = (140, -15, 205)$):

$ bold(z)_4 times (bold(p)_("ee") - bold(p)_4) = (0, 1, 0) times (0, 0, 0) = (0, 0, 0) $

#set par(first-line-indent: (amount: 1.27cm, all: true))

La matriz jacobiana completa en home es:

#set par(first-line-indent: 0cm)
$ bold(J)_("home") = mat(
  15,      120,     0,   0,   0;
 140,        0,     0,   0,   0;
   0,     -125,  -125,   0,   0;
   0,        0,     0,   1,   0;
   0,        1,     1,   0,   1;
   1,        0,     0,   0,   0;
) space "mm/rad (lineal), adimensional (angular)" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Las filas 1-3 ($x, y, z$) corresponden a $bold(J)_v$ (velocidad lineal). Las filas 4-6 corresponden a $bold(J)_omega$ (velocidad angular). Nótese que la fila 4 ($omega_x$) solo tiene contribución de la junta 4 — esto es esperado en home porque los ejes $Z_0, Z_1, Z_2, Z_4$ son todos perpendiculares a $X$, mientras que $X_3$ (junta Twist) es el único eje alineado con $X$ mundo.


= Análisis de Singularidades

== Criterio General

La matriz jacobiana del FABRI Creator es $bold(J) in RR^(6 times 5)$, por lo que su rango máximo es 5. Una configuración es cinemáticamente singular cuando el rango del jacobiano cae por debajo del número de grados de libertad, es decir, cuando las cinco columnas son linealmente dependientes. El criterio algebraico equivalente es:

#set par(first-line-indent: 0cm)
$ det(bold(J)(bold(q))^T bold(J)(bold(q))) = 0 quad arrow.l.r quad "rank"(bold(J)(bold(q))) < 5 $
#set par(first-line-indent: (amount: 1.27cm, all: true))

En una configuración singular el robot pierde la capacidad de generar velocidad en al menos una dirección del espacio de la tarea.

== Estructura del Jacobiano del FABRI

Dos propiedades estructurales simplifican el análisis:

- *Columnas 4 y 5 con bloque lineal nulo.* El efector considerado es $bold(p)_("ee") = bold(p)_5 = bold(p)_4$: la junta Twist tiene traslación constante y la junta 5 tiene $a_5 = d_5 = 0$, por lo que la posición del efector no depende de $q_4$ ni de $q_5$. La parte lineal del jacobiano es $bold(J)_v = [bold(J)_("v3") | bold(0) bold(0)]$, con $bold(J)_("v3") in RR^(3 times 3)$.

- *Ejes de las juntas 2 y 3 siempre paralelos.* Al girar la junta 3 alrededor del eje $Z_2$, el eje $bold(z)_2$ permanece invariante, de modo que $bold(z)_2 = bold(z)_1$ en toda configuración. La contribución angular de ambas juntas es siempre la misma dirección, lo que limita el rango angular a 3 y reduce el rango máximo del jacobiano completo.

== Singularidad de Codo (Pérdida de Grado de Libertad de Posición)

Dado que la posición del efector depende únicamente de las juntas 1–3, la singularidad de posición se analiza con el determinante de $bold(J)_("v3")$:

#set par(first-line-indent: 0cm)
$ det(bold(J)_("v3") (bold(q))) = 0 $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Geométricamente, el determinante se anula cuando el antebrazo y el brazo quedan colineales en el plano de movimiento (configuración de codo extendido o codo doblado). En términos de la variable articular:

#set par(first-line-indent: 0cm)
$ q_3 = -pi/2 space ("codo extendido", theta_3 = 0) quad "o" quad q_3 = +pi/2 space ("codo doblado", theta_3 = pi) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Esta condición es independiente de $q_1$, $q_2$, $q_4$ y $q_5$. La verificación numérica confirma el rango $"rank"(bold(J)) = 4$ en los puntos singulares exactos:

#set par(first-line-indent: 0cm)
#figure(
  table(
    columns: 5,
    align: center + horizon,
    stroke: (x, y) => (
      left: if x > 0 { 0.4pt },
      top: if y > 0 { 0.4pt },
    ),
    table.header[$q_2$][$q_3$][$det(bold(J)_("v3"))$][$"rank"(bold(J))$][Configuración],
    [$0$], [$-pi/2$], [0], [4], [Codo extendido, brazo vertical],
    [$0$], [$pi/2$], [0], [4], [Codo doblado, brazo vertical],
    [$pi$], [$-pi/2$], [0], [4], [Codo extendido, brazo hacia abajo],
    [$pi/2$], [$-pi/2$], [0], [4], [Codo extendido, brazo horizontal],
  ),
  caption: [Configuraciones singulares verificadas numéricamente ($q_1 = q_4 = q_5 = 0$).]
)
#set par(first-line-indent: (amount: 1.27cm, all: true))

== Configuraciones Regulares

En home ($bold(q) = bold(0)$) el determinante vale $det(bold(J)_("v3")(0)) = 2.10 times 10^6 != 0$ y $"rank"(bold(J)) = 5$: la configuración home es regular. En el punto de prueba del Caso de Prueba Numérico, $det(bold(J)_("v3") (bold(q)_("test"))) = 2.38 times 10^6 != 0$, también regular.

== Consecuencias Prácticas

Cerca de $q_3 = +- pi/2$ el robot no puede generar velocidad en la dirección radial del plano de trabajo: el efector no puede alejarse ni aproximarse al hombro. En la planificación de trayectorias debe evitarse atravesar el codo alineado, o bien atravesarlo con velocidad articular nula. La degeneración estructural $bold(z)_2 = bold(z)_1$ no es evitable (es propia del diseño), pero no reduce el rango por sí sola: las configuraciones singulares del FABRI Creator son exclusivamente las de codo alineado.

= Cinemática Diferencial de los Eslabones

== Velocidad Angular

Propagación hacia adelante: $bold(omega)_i = bold(omega)_(i-1) + bold(zeta)_(i-1) dot(q)_i$, con $bold(omega)_0 = bold(0)$. Sustituyendo los ejes de la Ecuación 11:

$ bold(omega)_1 = (0, 0, 1)^T dot(q)_1 $

$ bold(omega)_2 = bold(omega)_1 + (0, 1, 0)^T dot(q)_2 = mat(0; dot(q)_2; dot(q)_1) $

$ bold(omega)_3 = bold(omega)_2 + (0, 1, 0)^T dot(q)_3 = mat(0; dot(q)_2 + dot(q)_3; dot(q)_1) $

$ bold(omega)_4 = bold(omega)_3 + (1, 0, 0)^T dot(q)_4 = mat(dot(q)_4; dot(q)_2 + dot(q)_3; dot(q)_1) $

$ bold(omega)_5 = bold(omega)_4 + (0, 1, 0)^T dot(q)_5 = mat(dot(q)_4; dot(q)_2 + dot(q)_3 + dot(q)_5; dot(q)_1) $

En forma matricial $bold(omega)_i = bold(J)_(omega, i) dot(bold(q))$:

#set par(first-line-indent: 0cm)
$ bold(J)_(omega, 1) = mat(0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 1, 0, 0, 0, 0) $

$ bold(J)_(omega, 2) = mat(0, 0, 0, 0, 0; 0, 1, 0, 0, 0; 1, 0, 0, 0, 0) $

$ bold(J)_(omega, 3) = mat(0, 0, 0, 0, 0; 0, 1, 1, 0, 0; 1, 0, 0, 0, 0) $

$ bold(J)_(omega, 4) = mat(0, 0, 0, 1, 0; 0, 1, 1, 0, 0; 1, 0, 0, 0, 0) $

$ bold(J)_(omega, 5) = mat(0, 0, 0, 1, 0; 0, 1, 1, 0, 1; 1, 0, 0, 0, 0) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Las filas 4-6 de $bold(J)_("home")$ (Ecuación 14) coinciden con $bold(J)_(omega, 5)$: la velocidad angular del efector es la del último eslabón.

== Velocidad Lineal del Centro de Masa (Jacobianos Parciales)

El jacobiano lineal parcial $bold(J)_("com", i)^((0)) in RR^(3 times 5)$ satisface $bold(v)_("com", i) = bold(J)_("com", i)^((0)) dot(bold(q))$. Se construye con la Ecuación 10, reemplazando $bold(p)_("ee")$ por la posición del centro de masa del eslabón $i$ en coordenadas mundo, y anulando las columnas $i+1, dots, 5$.

*Hipótesis de trabajo.* A falta de mediciones experimentales, se asume provisionalmente que el centro de masa de cada eslabón coincide con el origen de su sistema local: $bold(r)_("com", i)^((i)) = bold(0)$, por lo que $bold(r)_("com", i)^((0)) = bold(p)_i$. Esta hipótesis se aplica en todas las evaluaciones numéricas de esta sección y de las Secciones 11 y 12. Los valores deben recalcularse cuando se disponga de los centros de masa reales.

Bajo esta hipótesis, los jacobianos lineales parciales en home son:

#set par(first-line-indent: 0cm)
$ bold(J)_("com", 1)^((0)) = mat(0, 0, 0, 0, 0; 15, 0, 0, 0, 0; 0, 0, 0, 0, 0) $

$ bold(J)_("com", 2)^((0)) = mat(0, 120, 0, 0, 0; 15, 0, 0, 0, 0; 0, 0, 0, 0, 0) $

$ bold(J)_("com", 3)^((0)) = mat(0, 120, 0, 0, 0; 105, 0, 0, 0, 0; 0, -90, -90, 0, 0) $

$ bold(J)_("com", 4)^((0)) = mat(15, 120, 0, 0, 0; 140, 0, 0, 0, 0; 0, -125, -125, 0, 0) $

$ bold(J)_("com", 5)^((0)) = mat(15, 120, 0, 0, 0; 140, 0, 0, 0, 0; 0, -125, -125, 0, 0) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

La cuarta columna de $bold(J)_("com", 4)^((0))$ y $bold(J)_("com", 5)^((0))$ es nula porque la junta Twist, en la implementación del robot, tiene traslación constante: el origen del frame 4 no se desplaza al variar $q_4$. La quinta columna es nula en todos los jacobianos porque $bold(p)_5 = bold(p)_4$.

= Análisis de Centros de Masa

== Parámetros Másicos

Cada eslabón $i$ se caracteriza por tres propiedades a medir experimentalmente:

- $m_i$: masa (kg).
- $bold(r)_("com", i)^((i)) in RR^3$: vector al centro de masa en el sistema local (mm).
- $bold(I)_i in RR^(3 times 3)$: tensor de inercia respecto al COM en el sistema local (kg·mm²).

== Parámetros Másicos Estimados

El robot no se desarma para medición directa, por lo que los parámetros másicos se estiman a partir de las piezas impresas en PETG al 25% de relleno y los actuadores comerciales. La densidad del PETG es $rho approx 1.27$ g/cm³; con relleno del 25% más perímetros y tapas, la densidad efectiva de las piezas es $rho_("ef") approx 0.42$ g/cm³. Los actuadores son tres MG996R (~55 g cada uno) en las juntas 1–3 —base, hombro y codo, las de mayor par— y dos MG90S (~13 g cada uno) en las juntas 4–5 de la muñeca; el tercer MG90S acciona la herramienta y no participa del modelo de 5 GDL. A partir de los volúmenes aproximados de cada pieza y sumando el servo y herrajes que porta cada eslabón, se adoptan los siguientes valores de trabajo:

#set par(first-line-indent: 0cm)
#figure(
  table(
    columns: 6,
    align: center + horizon,
    stroke: (x, y) => (
      left: if x > 0 { 0.4pt },
      top: if y > 0 { 0.4pt },
    ),
    table.header[$i$][Descripción][$m_i$ (kg)][$L$ (mm)][$I_("xx") = I_("yy")$ (kg·mm²)][$I_("zz")$ (kg·mm²)],
    [1], [Base: estructura PETG + Arduino Nano + PCA9685 + servo MG996R], [$0.11$], [50], [33.9], [22.0],
    [2], [Hombro: brazo PETG + servo MG996R (codo)], [$0.10$], [120], [130.0], [20.0],
    [3], [Antebrazo PETG + servo MG90S (twist)], [$0.05$], [90], [38.8], [10.0],
    [4], [Muñeca PETG + servo MG90S (pitch)], [$0.04$], [50], [12.3], [8.0],
    [5], [Portamarcador + marcador], [$0.02$], [40], [4.7], [4.0],
  ),
  caption: [Parámetros másicos estimados. El tensor se modela como cilindro macizo equivalente de radio $r = 20$ mm y longitud $L$: $I_("xx") = I_("yy") = m(3 r^2 + L^2)/12$ y $I_("zz") = m r^2 / 2$, con $r$, $L$ en mm y $m$ en kg.]
)
#set par(first-line-indent: (amount: 1.27cm, all: true))

La masa total móvil es $sum m_i = 0.32$ kg, acorde con un brazo educativo de 5 GDL con actuadores MG996R y MG90S. Se mantiene la hipótesis $bold(r)_("com", i)^((i)) = bold(0)$ de la Sección anterior; los valores de $bold(I)_i$ se usan para la contribución rotacional de la energía cinética y la matriz de inercia.

== Posición del COM en Coordenadas Mundo

$ bold(r)_("com", i)^((0))(bold(q)) = bold(R)_(0,i)(bold(q)) · bold(r)_("com", i)^((i)) + bold(p)_i(bold(q)) $

Las matrices de rotación en home, evaluadas a partir de $bold(T)_(0,i)$, son:

#set par(first-line-indent: 0cm)
$ bold(R)_(0,1) = mat(1, 0, 0; 0, 0, 1; 0, -1, 0) space bold(R)_(0,2) = mat(0, 1, 0; 0, 0, 1; 1, 0, 0) $

$ bold(R)_(0,3) = mat(1, 0, 0; 0, -1, 0; 0, 0, -1) space bold(R)_(0,4) = bold(R)_(0,5) = mat(1, 0, 0; 0, 0, 1; 0, -1, 0) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

*Ilustración bajo $bold(r)_("com", i)^((i)) = bold(0)$.* Las alturas de los centros de masa en home (incluyendo la base de 57 mm) serían: $z_("com", 1) = 142$ mm, $z_("com", 2) = z_("com", 3) = 262$ mm, $z_("com", 4) = z_("com", 5) = 262$ mm. Nótese que el desplazamiento $y = -15$ mm de $bold(p)_4$ y $bold(p)_5$ no afecta la altura (componente $z$) en home porque la rotación $bold(R)_(0,4)$ mapea el eje $Y$ local al eje $Z$ mundo con signo negativo, pero con $y_("local") = -15$ se cancela parcialmente. Los valores exactos dependen de $bold(r)_("com", i)^((i))$ real.

= Formulación Lagrangiana

== Función Lagrangiana

$ cal(L)(bold(q), dot(bold(q))) = T(bold(q), dot(bold(q))) - V(bold(q)) = sum_(i=1)^5 T_i - sum_(i=1)^5 V_i $

== Ecuaciones de Euler-Lagrange

Para $i = 1, dots, 5$:

$ dif / (dif t) (dif cal(L)) / (dif dot(q)_i) - (dif cal(L)) / (dif q_i) = tau_i $

Dado que $V$ no depende de $dot(q)_i$, la ecuación se descompone como:

$ dif / (dif t) (dif T) / (dif dot(q)_i) - (dif T) / (dif q_i) + (dif V) / (dif q_i) = tau_i $

== Forma Matricial

Expresando la energía cinética como forma cuadrática $T = 1/2 dot(bold(q))^T bold(M)(bold(q)) dot(bold(q))$ y desarrollando la derivada temporal:

$ dif / (dif t) (dif T) / (dif dot(q)_i) = sum_(j=1)^5 M_(i j) dot.double(q)_j + sum_(j=1)^5 sum_(k=1)^5 (dif M_(i j)) / (dif q_k) dot(q)_k dot(q)_j $

Definiendo los símbolos de Christoffel $c_(i j k) = 1/2 ((dif M_(i j)) / (dif q_k) + (dif M_(i k)) / (dif q_j) - (dif M_(k j)) / (dif q_i))$, la ecuación de movimiento se reagrupa como:

$ sum_(j=1)^5 M_(i j) dot.double(q)_j + sum_(j=1)^5 sum_(k=1)^5 c_(i j k) dot(q)_j dot(q)_k + g_i(bold(q)) = tau_i $

En notación matricial compacta:

$ bold(M)(bold(q)) dot.double(bold(q)) + bold(C)(bold(q), dot(bold(q))) dot(bold(q)) + bold(g)(bold(q)) = bold(tau) $

Donde $C_(i j) = sum_(k=1)^5 c_(i j k) dot(q)_k$ y $bold(g) = nabla_(bold(q)) V$. La matriz $bold(C)$ no se desarrolla explícitamente por requerir las derivadas parciales de $bold(M)$ respecto a $bold(q)$, las cuales no están disponibles en forma cerrada sin los parámetros másicos completos. Su estructura se obtendría numéricamente una vez determinados $m_i$, $bold(r)_("com", i)^((i))$ e $bold(I)_i$.

= Energía Cinética por Eslabón

== Expresión General

$ T_i = 1/2 m_i bold(v)_("com", i)^T bold(v)_("com", i) + 1/2 bold(omega)_i^T bold(R)_(0,i) bold(I)_i bold(R)_(0,i)^T bold(omega)_i $

== Forma Matricial

Con $bold(v)_("com", i) = bold(J)_("com", i)^((0)) dot(bold(q))$ y $bold(omega)_i = bold(J)_(omega, i) dot(bold(q))$:

$ T_i = 1/2 dot(bold(q))^T bold(M)_i(bold(q)) dot(bold(q)) $

$ bold(M)_i = m_i bold(J)_("com", i)^((0) T) bold(J)_("com", i)^((0)) + bold(J)_(omega, i)^T bold(R)_(0,i) bold(I)_i bold(R)_(0,i)^T bold(J)_(omega, i) $

== Evaluación en Home (Parte Traslacional)

Bajo la hipótesis $bold(r)_("com", i)^((i)) = bold(0)$, usando los jacobianos de las Ecuaciones 17-21. El superíndice $"tras"$ indica que solo se incluye el primer sumando de la Ecuación 29 (contribución de la velocidad lineal del COM). La contribución rotacional se expresa simbólicamente como $bold(M)_i^("rot") = bold(J)_(omega, i)^T bold(R)_(0,i) bold(I)_i bold(R)_(0,i)^T bold(J)_(omega, i)$.

#set par(first-line-indent: 0cm)

*Eslabón 1 (Base):* Solo $dot(q)_1$.

$ bold(M)_1^("tras")(0) = m_1 mat(225, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0) $

*Eslabón 2 (Hombro):* $dot(q)_1$, $dot(q)_2$.

$ bold(M)_2^("tras")(0) = m_2 mat(225, 0, 0, 0, 0; 0, 14400, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0) $

*Eslabón 3 (Codo):* $dot(q)_1$, $dot(q)_2$, $dot(q)_3$, con acoplamiento $dot(q)_2 dot(q)_3$.

$ bold(M)_3^("tras")(0) = m_3 mat(11025, 0, 0, 0, 0; 0, 22500, 8100, 0, 0; 0, 8100, 8100, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0) $

*Eslabón 4 (Muñeca Roll):* $dot(q)_1$, $dot(q)_2$, $dot(q)_3$, $dot(q)_4$ con acoplamientos.

$ bold(M)_4^("tras")(0) = m_4 mat(19825, 1800, 0, 0, 0; 1800, 30025, 15625, 0, 0; 0, 15625, 15625, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0) $

*Eslabón 5 (Muñeca Pitch):* Ídem al eslabón 4 (igual jacobiano lineal bajo $bold(r)_("com", i)^((i)) = bold(0)$).

$ bold(M)_5^("tras")(0) = m_5 mat(19825, 1800, 0, 0, 0; 1800, 30025, 15625, 0, 0; 0, 15625, 15625, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0) $

#set par(first-line-indent: (amount: 1.27cm, all: true))

La matriz de inercia total es $bold(M)(bold(q)) = sum_(i=1)^5 [bold(M)_i^("tras")(bold(q)) + bold(M)_i^("rot")(bold(q))]$. Las matrices arriba indicadas dependen de la hipótesis $bold(r)_("com", i)^((i)) = bold(0)$ y deben recalcularse con los valores reales.

= Energía Potencial por Eslabón

== Expresión por Eslabón

$ V_i(bold(q)) = m_i g (z_("com", i)(bold(q)) / 1000) space "(J)" $

Con $g = 9.81$ m/s². La altura $z_("com", i)$ está en mm; la división por 1000 la convierte a metros.

== Evaluación en Home ($bold(r)_("com", i)^((i)) = bold(0)$)

$ V_1(0) = m_1 · 9.81 · 0.142 space "(J)" $

$ V_2(0) = m_2 · 9.81 · 0.262 space "(J)" $

$ V_3(0) = m_3 · 9.81 · 0.262 space "(J)" $

$ V_4(0) = m_4 · 9.81 · 0.262 space "(J)" $

$ V_5(0) = m_5 · 9.81 · 0.262 space "(J)" $

Sustituyendo las masas estimadas de la Sección 9:

#set par(first-line-indent: 0cm)
$ V_1(0) = 0.153 space "J" , quad V_2(0) = 0.257 space "J" , quad V_3(0) = 0.129 space "J" , quad V_4(0) = 0.103 space "J" , quad V_5(0) = 0.051 space "J" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

La energía potencial total en home es $V(0) = sum V_i(0) = 0.693$ J.

== Vector de Pares Gravitatorios

$ g_i(bold(q)) = (dif V) / (dif q_i) = g · 10^(-3) · sum_(j=1)^5 m_j (dif z_("com", j)) / (dif q_i) $

El factor $10^(-3)$ convierte mm a m. La derivada $dif z_("com", j) / dif q_i$ es la tercera fila de $bold(J)_("com", j)^((0))$. Con los jacobianos de las Ecuaciones 17-21:

$ bold(J)_("com", 1, z) = (0, 0, 0, 0, 0) space bold(J)_("com", 2, z) = (0, 0, 0, 0, 0) $

$ bold(J)_("com", 3, z) = (0, -90, -90, 0, 0) $

$ bold(J)_("com", 4, z) = (0, -125, -125, 0, 0) $

$ bold(J)_("com", 5, z) = (0, -125, -125, 0, 0) $

Por lo tanto, bajo $bold(r)_("com", i)^((i)) = bold(0)$:

#set par(first-line-indent: 0cm)
$ bold(g)(0) = g · 10^(-3) · mat(0; -90 m_3 - 125(m_4 + m_5); -90 m_3 - 125(m_4 + m_5); 0; 0) space "(N·m)" $

Con las masas de la Sección 9: $-90 m_3 - 125(m_4 + m_5) = -90(0.05) - 125(0.06) = -12.0$ mm·kg, por lo que:

#set par(first-line-indent: 0cm)
$ bold(g)(0) = 9.81 · 10^(-3) · (-12.0) = mat(0; -0.1177; -0.1177; 0; 0) space "(N·m)" $
#set par(first-line-indent: (amount: 1.27cm, all: true))
#set par(first-line-indent: (amount: 1.27cm, all: true))

La componente $g_4(0) = 0$ es consecuencia de que la traslación de la junta Twist es constante en la implementación: el origen del frame 4 no se desplaza al variar $q_4$, por lo que la junta 4 no realiza trabajo contra la gravedad bajo la hipótesis de centro de masa en el origen del frame.

= Matriz de Fuerzas Centrípetas y de Coriolis

== Construcción mediante Símbolos de Christoffel

La ecuación de movimiento en forma matricial es $bold(M)(bold(q)) dot.double(bold(q)) + bold(C)(bold(q), dot(bold(q))) dot(bold(q)) + bold(g)(bold(q)) = bold(tau)$. Los elementos de $bold(C)$ se construyen con los símbolos de Christoffel de primera especie asociados a la matriz de inercia:

#set par(first-line-indent: 0cm)
$ c_(i j k) = 1/2 ((dif M_(i j)) / (dif q_k) + (dif M_(i k)) / (dif q_j) - (dif M_(k j)) / (dif q_i)) , quad C_(i j) (bold(q), dot(bold(q))) = sum_(k=1)^5 c_(i j k) dot(q)_k $
#set par(first-line-indent: (amount: 1.27cm, all: true))

El término $C_(i j) dot(q)_j$ agrupa: para $i = j$, fuerzas centrífugas (proporcionales a $dot(q)_i^2$); para $i != j$, fuerzas de Coriolis (proporcionales al producto $dot(q)_i dot(q)_j$). Los coeficientes requieren las derivadas parciales de $bold(M)$ respecto a $bold(q)$, evaluadas numéricamente a partir de las masas e inercias de la Sección 9.

== Propiedad de Antisimetría

Con la elección estándar de Christoffel, la matriz $dot(bold(M)) - 2 bold(C)$ es antisimétrica:

#set par(first-line-indent: 0cm)
$ dot(bold(q))^T (dot(bold(M)) - 2 bold(C)) dot(bold(q)) = 0 $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Esto expresa que las fuerzas centrípetas y de Coriolis no realizan trabajo neto, coherente con el balance energético de la sección de Potencia.

== Evaluación Numérica

Con los parámetros de la Sección 9 y el punto de prueba $bold(q)_("test")$ del Caso de Prueba Numérico, se evalúa $bold(C)$ para un perfil de velocidades de ejemplo, del orden del movimiento nominal del robot: $dot(bold(q)) = (0, 0.6, -0.4, 0.3, 0.5)$ rad/s:

#set par(first-line-indent: 0cm)
$ bold(C)(bold(q)_("test"), dot(bold(q))) = mat(
  1677.9,  -70.5,  -24.7,   -1.0,   0.3;
     2.9,  407.9, -203.0,   -0.4,  -0.5;
     2.9,  611.6,    0.7,   -0.4,  -0.5;
    -2.5,    0.4,    0.4,    0.0,   0.4;
     0.3,   -0.5,   -0.5,   -0.4,   0.0;
) space "kg·mm²/s" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

El efecto sobre el vector de pares es $bold(C) dot(bold(q)) = (-32.6, 325.6, 366.3, 0.3, -0.2)$ kg·mm²/s², que convertido a N·m (factor $10^(-6)$) da $bold(C) dot(bold(q)) approx (-3.3 times 10^(-5), 3.3 times 10^(-4), 3.7 times 10^(-4), 2.6 times 10^(-7), -2.1 times 10^(-7))$ N·m.

Comparado con $bold(g)(bold(q)_("test")) = (0, -0.292, -0.118, 0, 0)$ N·m, el término centrípeto/Coriolis es dos órdenes de magnitud menor: a las velocidades típicas de los actuadores MG996R y MG90S la dinámica del FABRI está dominada por la gravedad y la inercia, y el término $bold(C) dot(bold(q))$ puede despreciarse en el diseño de controladores. En el Caso de Prueba Numérico (estático, $dot(bold(q)) = bold(0)$) la matriz es nula por construcción: $C_(i j) = sum c_(i j k) dot(q)_k = 0$.

= Ecuaciones de Potencia

La potencia mecánica instantánea en un sistema robótico de cadena abierta es la suma de los productos par-velocidad en cada junta.

== Potencia por Junta

$ P_i = tau_i · dot(q)_i space "(W)" $

Donde $tau_i$ está en N·m y $dot(q)_i$ en rad/s (watts = N·m/s). $P_i > 0$: trabajo motor; $P_i < 0$: trabajo resistivo o regeneración.

== Potencia Total

$ P(bold(q), dot(bold(q)), bold(tau)) = sum_(i=1)^5 tau_i dot(q)_i = bold(tau)^T dot(bold(q)) $

Multiplicando la ecuación de movimiento (Ecuación 28) por $dot(bold(q))^T$:

$ dot(bold(q))^T bold(M) dot.double(bold(q)) + dot(bold(q))^T bold(C) dot(bold(q)) + dot(bold(q))^T bold(g) = dot(bold(q))^T bold(tau) $

El primer término es $dif T / (dif t) - 1/2 dot(bold(q))^T dot(bold(M)) dot(bold(q))$. El segundo es idénticamente $1/2 dot(bold(q))^T dot(bold(M)) dot(bold(q))$ para la elección estándar de $bold(C)$ basada en Christoffel. El tercero es $dif V / (dif t)$. Por tanto, el balance energético global se reduce a:

$ dif / (dif t)(T + V) = bold(tau)^T dot(bold(q)) $

La potencia suministrada por los actuadores se invierte íntegramente en variar la energía mecánica total del sistema.

== Potencia por Eslabón

Desglosando por cuerpo rígido:

$ bold(tau)^T dot(bold(q)) = sum_(i=1)^5 [dif / (dif t)(T_i + V_i)] $

Cada eslabón contribuye individualmente al balance energético global. Esta descomposición, equivalente al principio de conservación para cada cuerpo del sistema, es útil para el análisis de eficiencia y dimensionamiento de actuadores. Su evaluación requiere los parámetros másicos completos.

= Caso de Prueba Numérico

== Vector de Estado Articular de Prueba

Se define el vector de estado articular explícito:

#set par(first-line-indent: 0cm)
$ bold(q)_("test") = (q_1, q_2, q_3, q_4, q_5) = (pi/6, pi/4, -pi/4, pi/3, pi/6) space "rad" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

La elección obedece a tres criterios: (i) ángulos notables, cuyos senos y cosenos admiten raíces exactas para el cálculo manual; (ii) todos dentro de los límites articulares del robot ($+-85°$ para las juntas 1–4 y entre $-115°$ y $55°$ para la muñeca pitch); (iii) configuración regular, con $det(bold(J)^T bold(J)) != 0$ (Sección 7). El estado es estático: $dot(bold(q)) = bold(0)$ y $dot.double(bold(q)) = bold(0)$.

== Evaluación de la MTH

Posiciones de los frames, paso a paso con los valores notables:

#set par(first-line-indent: 0cm)
$ bold(p)_1 = (15 cos (pi/6), 15 sin (pi/6), 85) = (15 sqrt(3)/2, 15/2, 85) = (12.99, 7.50, 85.00) $ space "mm"

$ bold(p)_2 = bold(p)_1 + bold(R)_(0,1) · (120 cos (pi/4), -120 sin (pi/4), 0) = (15 sqrt(3)/2 + 30 sqrt(6), 15/2 + 30 sqrt(2), 85 + 60 sqrt(2)) = (86.48, 49.93, 169.85) $ space "mm"

$ bold(p)_3 = bold(p)_2 + bold(R)_(0,2) · (45 sqrt(2), 45 sqrt(2), 0) = (86.48, 49.93, 169.85) + (77.94, 45.00, 0.00) = (164.42, 94.93, 169.85) $ space "mm"

$ bold(p)_4 = bold(p)_3 + bold(R)_(0,3) · (35, 15, 0) = (164.42, 94.93, 169.85) + (37.81, 4.51, 0.00) = (202.23, 99.44, 169.85) $ space "mm"
#set par(first-line-indent: (amount: 1.27cm, all: true))

Como la junta 5 tiene $a_5 = d_5 = 0$, $bold(p)_5 = bold(p)_4$ y la pose del efector es $bold(p)_("ee") = (202.23, 99.44, 169.85)$ mm. La MTH global del efector es:

#set par(first-line-indent: 0cm)
$ bold(T)_(0,5) (bold(q)_("test")) = mat(
  0.5335,  -0.8080,  -0.2500,  202.2282;
  0.8080,   0.3995,   0.4330,   99.4360;
  -0.2500,  -0.4330,   0.8660,  169.8528;
       0,        0,        0,        1;
) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

== Evaluación del Cuaternión Unitario

Aplicando el método de la traza a la rotación $bold(R)_(0,5)$ de la MTH anterior:

#set par(first-line-indent: 0cm)
$ tr(bold(R)) = 0.5335 + 0.3995 + 0.8660 = 1.7990 > 0 $

$ S = 2 sqrt(tr(bold(R)) + 1) = 2 sqrt(2.7990) = 3.3460 $

$ q_0 = S/4 = 0.8365 , quad q_1 = (R_(32) - R_(23))/S = (-0.4330 - 0.4330)/3.3460 = -0.2588 $

$ q_2 = (R_(13) - R_(31))/S = (-0.2500 + 0.2500)/3.3460 = 0 , quad q_3 = (R_(21) - R_(12))/S = (0.8080 + 0.8080)/3.3460 = 0.4829 $
#set par(first-line-indent: (amount: 1.27cm, all: true))

#set par(first-line-indent: 0cm)
$ bold(q)_r (bold(q)_("test")) = (0.8365, -0.2588, 0.0000, 0.4829) = (cos (pi/12) cos (pi/6), -sin (pi/12), 0, cos (pi/12) sin (pi/6)) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Verificación: $|| bold(q)_r ||^2 = 0.8365^2 + 0.2588^2 + 0.4829^2 = 0.6997 + 0.0670 + 0.2332 = 1.0000$, cuaternión unitario.

== Evaluación del Cuaternión Dual

Con $bold(t) = bold(p)_("ee") = (202.228, 99.436, 169.853)$ mm como cuaternión puro y el cuaternión anterior:

#set par(first-line-indent: 0cm)
$ bold(q)_d = 1/2 bold(t) ⊗ bold(q)_r = (-14.8460, 108.5956, -29.2250, 83.9103) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

El cuaternión dual de la pose es:

#set par(first-line-indent: 0cm)
$ hat(bold(q)) = bold(q)_r + epsilon bold(q)_d = (0.8365 - 14.8460 epsilon, -0.2588 + 108.5956 epsilon, 0 - 29.2250 epsilon, 0.4829 + 83.9103 epsilon) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Verificación: $2 bold(q)_d ⊗ bold(q)_r^star = (202.228, 99.436, 169.853) = bold(p)_("ee")$, el cuaternión dual reconstruye exactamente la pose del efector.

== Evaluación de la Matriz Jacobiana

Evaluando las columnas con los ejes $bold(z)_0 = (0, 0, 1)$, $bold(z)_1 = bold(z)_2 = (-1/2, sqrt(3)/2, 0)$, $bold(x)_3 = (sqrt(3)/2, 1/2, 0)$ y $bold(z)_4 = (-1/4, sqrt(3)/4, sqrt(3)/2)$ (coordenadas mundo):

#set par(first-line-indent: 0cm)
$ bold(J)(bold(q)_("test")) = mat(
  -99.44,   73.48,     0.00,   0.00,   0.00;
  202.23,   42.43,     0.00,   0.00,   0.00;
    0.00, -209.85,  -125.00,   0.00,   0.00;
    0.00,   -0.50,    -0.50,   0.87,  -0.25;
    0.00,    0.87,     0.87,   0.50,   0.43;
    1.00,    0.00,     0.00,   0.00,   0.87;
) space "mm/rad (lineal), adimensional (angular)" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Se verifica $"rank"(bold(J)) = 5$ y $det(bold(J)^T bold(J)) = 5.69 times 10^12 != 0$: la configuración de prueba es regular (Sección 7).

== Evaluación de los Pares Articulares

Con $dot(bold(q)) = dot.double(bold(q)) = bold(0)$, la ecuación de movimiento se reduce a $bold(tau) = bold(g)(bold(q))$. La matriz de inercia en el punto de prueba (kg·mm², Secciones 9 y 11) es:

#set par(first-line-indent: 0cm)
$ bold(M)(bold(q)_("test")) = mat(
  6058.2,    74.2,    -2.2,    0.0,    3.5;
    74.2,  6477.5,  2415.2,    0.0,    2.0;
    -2.2,  2415.2,  1397.0,    0.0,    2.0;
     0.0,     0.0,     0.0,   17.0,    0.0;
     3.5,     2.0,     2.0,    0.0,    4.0;
) space "kg·mm²" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

y $bold(C)(bold(q)_("test"), bold(0)) = bold(0)$ (Sección 13). Los pares gravitatorios se calculan a partir de las derivadas de las alturas de los centros de masa. Bajo la hipótesis $bold(r)_("com", i)^((i)) = bold(0)$, las derivadas no nulas en el punto de prueba son:

#set par(first-line-indent: 0cm)
$ (dif z_2)/(dif q_2) = -60 sqrt(2) , quad (dif z_3)/(dif q_2) = -(90 + 60 sqrt(2)) , quad (dif z_4)/(dif q_2) = (dif z_5)/(dif q_2) = -(125 + 60 sqrt(2)) $

$ (dif z_3)/(dif q_3) = -90 , quad (dif z_4)/(dif q_3) = (dif z_5)/(dif q_3) = -125 $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Para la junta 2 (hombro):

#set par(first-line-indent: 0cm)
$ g_2 = -9.81 · 10^(-3) [0.10 · 60 sqrt(2) + 0.05 · (90 + 60 sqrt(2)) + 0.06 · (125 + 60 sqrt(2))] = -9.81 · 10^(-3) · 29.819 = -0.2925 space "N·m" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Para la junta 3 (codo):

#set par(first-line-indent: 0cm)
$ g_3 = -9.81 · 10^(-3) [0.05 · 90 + 0.04 · 125 + 0.02 · 125] = -9.81 · 10^(-3) · 12.0 = -0.1177 space "N·m" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Las componentes restantes se anulan: $g_1 = 0$ (la junta 1 rota alrededor de la vertical y ninguna altura cambia), $g_4 = 0$ (la traslación de la junta Twist es constante) y $g_5 = 0$ (el centro de masa del eslabón 5 coincide con $bold(p)_4$, que no se desplaza con $q_5$). El vector de pares requeridos en el punto de prueba es:

#set par(first-line-indent: 0cm)
$ bold(tau) = bold(g)(bold(q)_("test")) = (0, -0.2925, -0.1177, 0, 0) space "N·m" $
#set par(first-line-indent: (amount: 1.27cm, all: true))

El resultado es coherente con la estructura del robot: solo las juntas 2 y 3 soportan el peso del brazo; el par del hombro supera al del codo porque el brazo extendido a $45°$ ejerce un mayor momento respecto al hombro.

== Resumen y Coincidencia con la Herramienta Computacional

Todos los valores de esta sección —MTH, cuaternión unitario, cuaternión dual, matriz jacobiana, matriz de inercia y pares articulares— fueron evaluados de forma independiente con la herramienta computacional del proyecto; la coincidencia es total en las cifras mostradas, lo que constituye la verificación cruzada del desarrollo manual.

= Verificación Numérica

Los valores numéricos presentados en las secciones anteriores se verificaron mediante una herramienta computacional que implementa el modelo cinemático completo del robot. A continuación se presentan los resultados obtenidos en la configuración home ($q_i = 0$, $i = 1, dots, 5$), los cuales coinciden con las expresiones analíticas derivadas en el documento.

== Posiciones de los Frames

#set par(first-line-indent: 0cm)
#figure(
  table(
    columns: 4,
    align: center + horizon,
    stroke: (x, y) => (
      left: if x > 0 { 0.4pt },
      top: if y > 0 { 0.4pt },
    ),
    table.header[$i$][$x$ (mm)][$y$ (mm)][$z$ (mm)],
    [1], [15.00], [0.00], [85.00],
    [2], [15.00], [0.00], [205.00],
    [3], [105.00], [0.00], [205.00],
    [4], [140.00], [$-15.00$], [205.00],
    [5], [140.00], [$-15.00$], [205.00],
  ),
  caption: [Posiciones de los frames en home (sin base ni tool).]
)
#set par(first-line-indent: (amount: 1.27cm, all: true))

Nótese que $x_5 = x_4$, $y_5 = y_4$ y $z_5 = z_4$ porque la junta 5 (muñeca pitch) está en home y su transformación es la identidad. El desplazamiento $y_4 = -15$ mm proviene del parámetro $d_4$ de la junta Twist.

== Jacobiano del Efector Final

La matriz jacobiana obtenida computacionalmente en home es:

#set par(first-line-indent: 0cm)
$ bold(J)_("home") = mat(
  15,      120,     0,   0,   0;
 140,        0,     0,   0,   0;
   0,     -125,  -125,   0,   0;
   0,        0,     0,   1,   0;
   0,        1,     1,   0,   1;
   1,        0,     0,   0,   0;
) $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Las columnas 1, 2, 3 y 5 corresponden a juntas rotacionales estándar (eje $Z$). La columna 4 corresponde a la junta Twist (eje $X$), con componente lineal nula porque su origen no se desplaza al rotar. Estos valores coinciden exactamente con la Ecuación 14.

== Ejes de Rotación

Los ejes de rotación de cada junta en home son:

#set par(first-line-indent: 0cm)
$ bold(z)_0 = (0, 0, 1)^T , quad bold(z)_1 = (0, 1, 0)^T , quad bold(z)_2 = (0, 1, 0)^T , quad bold(x)_3 = (1, 0, 0)^T , quad bold(z)_4 = (0, 1, 0)^T $
#set par(first-line-indent: (amount: 1.27cm, all: true))

Estos ejes, junto con las posiciones de la Tabla 2, producen el Jacobiano mostrado.



= Coincidencia de Datos Computacional

Para cumplir con el requisito de mostrar en consola los valores finales de la MTH, cuaterniones y pares dinámicos, la herramienta computacional del proyecto —el binario `test-case-report` de `bombolab-core`— evalúa el caso de prueba de la Sección 15 y vuelca su resultado. El comando de reproducción es:

#set par(first-line-indent: 0cm)
```text
cd crates/bombolab-core && cargo run --release --bin test-case-report
```
#set par(first-line-indent: (amount: 1.27cm, all: true))

La salida completa de la consola es:

#set par(first-line-indent: 0cm)
```text
======================================================================
  FABRI CREATOR - CASO DE PRUEBA NUMÉRICO (Sección 15 del reporte)
  q_test = (pi/6, pi/4, -pi/4, pi/3, pi/6) rad, estado estatico
======================================================================

=== 1. MTH GLOBAL DEL EFECTOR (mm) ===

  p_1 = (  12.9904,    7.5000,   85.0000)
  p_2 = (  86.4751,   49.9264,  169.8528)
  p_3 = ( 164.4174,   94.9264,  169.8528)
  p_4 = ( 202.2282,   99.4360,  169.8528)
  p_5 = ( 202.2282,   99.4360,  169.8528)

  T_0,5(q_test) =
    [     0.5335    -0.8080    -0.2500   202.2282 ]
    [     0.8080     0.3995     0.4330    99.4360 ]
    [    -0.2500    -0.4330     0.8660   169.8528 ]
    [     0.0000     0.0000     0.0000     1.0000 ]

=== 2. CUATERNIÓN UNITARIO DE R_0,5 ===

  q_r = (  0.8365,  -0.2588,  -0.0000,   0.4830)   |q_r| = 1.000000

=== 3. CUATERNIÓN DUAL DE LA POSE ===

  q_r = (  0.8365,  -0.2588,  -0.0000,   0.4830)
  q_d = (-14.8460, 108.5956, -29.2250,  83.9103)
  verificacion: 2*q_d x q_r* = ( 202.2282,   99.4360,  169.8528)  == p_ee

=== 4. JACOBIANA GEOMETRICA (6x5) ===

    [  -99.4360   73.4847   -0.0000    0.0000    0.0000 ]
    [  202.2282   42.4264    0.0000    0.0000    0.0000 ]
    [    0.0000 -209.8528 -125.0000    0.0000   -0.0000 ]
    [    0.0000   -0.5000   -0.5000    0.8660   -0.2500 ]
    [    0.0000    0.8660    0.8660    0.5000    0.4330 ]
    [    1.0000    0.0000    0.0000    0.0000    0.8660 ]
  det(J^T J) = 5.688e12  -> configuracion regular (rango 5)

=== 5. MATRIZ DE INERCIA M(q_test) (kg*mm^2) ===

    [    6058.2      74.2      -2.2       0.0       3.5 ]
    [      74.2    6477.5    2415.2       0.0       2.0 ]
    [      -2.2    2415.2    1397.0       0.0       2.0 ]
    [       0.0       0.0       0.0      17.0       0.0 ]
    [       3.5       2.0       2.0       0.0       4.0 ]

=== 6. PARES ARTICULARES (estatico: tau = g(q_test)) ===

  g(q_test) = (   0.0000,   -0.2925,   -0.1177,    0.0000,    0.0000) N*m

  Los valores coinciden con los cálculos manuales del reporte (Sección 15).
  OK: qp = qpp = 0 -> tau = g; M(q_test) y C(q_test,0) = 0 verifican M qpp + C qp + g = tau.
```
#set par(first-line-indent: (amount: 1.27cm, all: true))

La comparación de los valores clave entre el reporte y la consola es:

#set par(first-line-indent: 0cm)
#figure(
  table(
    columns: 3,
    align: center + horizon,
    stroke: (x, y) => (
      left: if x > 0 { 0.4pt },
      top: if y > 0 { 0.4pt },
    ),
    table.header[Magnitud][Reporte (Sección 15)][Consola (`test-case-report`)],
    [$bold(p)_("ee")$ (mm)], [(202.228, 99.436, 169.853)], [(202.2282, 99.4360, 169.8528)],
    [$bold(q)_r$], [(0.8365, −0.2588, 0.0000, 0.4829)], [(0.8365, −0.2588, −0.0000, 0.4830)],
    [$bold(q)_d$], [(−14.8460, 108.5956, −29.2250, 83.9103)], [(−14.8460, 108.5956, −29.2250, 83.9103)],
    [$det(bold(J)^T bold(J))$], [$5.69 times 10^12$], [5.688e12],
    [$bold(g)(bold(q)_("test"))$ (N·m)], [(0, −0.2925, −0.1177, 0, 0)], [(0.0000, −0.2925, −0.1177, 0.0000, 0.0000)],
  ),
  caption: [Comparación de valores clave entre el desarrollo manual (Sección 15) y la salida de consola de la herramienta computacional.]
)
#set par(first-line-indent: (amount: 1.27cm, all: true))

La coincidencia es total en las cifras mostradas, lo que demuestra la consistencia entre el desarrollo manual del caso de prueba y la implementación computacional del modelo.

= Anexos

#figure(
  image("robot-docs/5d36f1fc-a6ce-4c54-9692-dd8830453452.jpeg", width: 100%),
  caption: [Diagrama del robot FABRI Creator con sus dimensiones y sistemas de referencia.],
)

#figure(
  image("assets/image.png"),
  caption: [Resultado del motor cinemático - 1],
)

#figure(
  image("assets/image-1.png"),
  caption: [Resultado del motor cinemático - 2],
)



#pagebreak()
= Referencias

- Repositorio del proyecto: https://github.com/CharFranR/bombolab

Como prueba de la correcta implementación de lo solicitado se presenta tanto los motores de cinemática directa e inversa unicados en /crates/bombolab-core/src/ en las carpetas math/ y kinematics/

Además, para implementaciones puntuales se estructuró un archivo binario en /crates/bombolab-core/src/bin/test-case-report

Documentación extra específica del robot puede ser encontrada en robot-docs

