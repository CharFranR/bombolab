# Informe Técnico Bombolab — Secciones nuevas (listas para pegar)

> Texto redactado según el código real del repositorio (`bombolab-core`).
> Valores citados verificados: `IkSolver::new(200, 1.0, 0.05, 0.5)` → 200 iteraciones, tolerancia 1 mm, **λ = 0.05**, paso 0.5 rad.

---

## 4.7 Cinemática diferencial mediante cuaterniones duales

**a) Representación dual de la pose.** Todo marco (rotación 𝑅, traslación 𝑡) se representa por un cuaternión dual unitario

𝑞̂ = 𝑞𝑟 + ε 𝑞𝑑, con 𝑞𝑑 = ½ 𝑡 ⊗ 𝑞𝑟

donde 𝑞𝑟 es el cuaternión de rotación (obtenido por el método de Shepperd a partir de 𝑅) y ⊗ es el producto de Hamilton. La traslación se recupera con la identidad

𝑡 = 2 (𝑞𝑑 ⊗ 𝑞𝑟*)

Ambas expresiones son las que implementan `DualQuaternion::from_pose` y `DualQuaternion::translation` en `bombolab-core` (`math/quaternion.rs`).

**b) Derivada temporal y tornillo de velocidad.** Para una pose en movimiento 𝑞̂(𝑡), la derivada del cuaternión dual unitario satisface la ecuación diferencial dual

𝑞̂̇ = ½ ξ̂ ⊗ 𝑞̂ ⇔ ξ̂ = 2 𝑞̂̇ ⊗ 𝑞̂*

donde ξ̂ = 𝝎 + ε 𝒗 es el *tornillo* instantáneo de velocidad: 𝝎 es la velocidad angular y 𝒗 la velocidad lineal del origen del marco (Kavan et al., 2008; Kenwright, 2012). Esta ecuación es el análogo dual de 𝑞̇ = ½ 𝝎 ⊗ 𝑞 y constituye el núcleo de la cinemática diferencial: **la velocidad del efector se obtiene directamente del producto de cuaterniones duales, sin construir matrices homogéneas completas**.

**c) Cadena serial y Jacobiano.** La pose del efector es el producto de las transformaciones duales de cada eslabón, 𝑞̂₀ₙ = 𝑞̂₁ ⊗ … ⊗ 𝑞̂ₙ. Derivando con la regla del producto y usando 𝑞̂̇ᵢ = ½ 𝑞̇ᵢ ξ̂ᵢ ⊗ 𝑞̂ᵢ para una articulación revoluta de eje ξ̂ᵢ,

ξ̂ = 2 𝑞̂̇ ⊗ 𝑞̂* = Σᵢ 𝑞̇ᵢ (𝑞̂₁ ⊗ … ⊗ 𝑞̂ᵢ₋₁ ⊗ ξ̂ᵢ ⊗ 𝑞̂ᵢ₋₁* ⊗ … ⊗ 𝑞̂₁*)

es decir, el tornillo del efector es la **combinación lineal de los ejes articulares transformados al marco del efector**, y esos coeficientes son exactamente las columnas del Jacobiano geométrico:

𝐽ᵢ = [𝒛ᵢ₋₁ × (𝒑ₑ − 𝒑ᵢ₋₁); 𝒛ᵢ₋₁]

Esta es la construcción implementada en `geometric_jacobian` (`math/jacobian.rs`), que contempla además la articulación *twist* de la muñeca (eje 𝑋ᵢ₋₁ con pivote corregido en el origen del marco). La equivalencia entre la formulación dual y la matricial se verifica numéricamente: el test `dual_quaternion_pose_reconstruction` reconstruye la traslación de prueba (140, −15, 205) mm con error menor a 1e−9 mm (el binario `test-case-report` verifica la misma identidad 2·𝑞𝑑 ⊗ 𝑞𝑟* = 𝒑_ee sobre el caso 𝑞 = (𝜋/6, 𝜋/4, −𝜋/4, 𝜋/3, 𝜋/6)), y el Jacobiano completo se valida por diferencias finitas.

**d) Vínculo con la planificación sin singularidades.** La ecuación 𝑞̂̇ = ½ ξ̂ ⊗ 𝑞̂ es la que fundamenta la cinemática inversa diferencial: el solucionador de posición (DLS) resuelve Δ𝑞 = 𝐽ᵀ(𝐽𝐽ᵀ + λ²𝐼)⁻¹ 𝒆, donde λ = 0.05 amortigua los picos de velocidad cuando 𝐽 pierde rango. Así, el mismo Jacobiano deducido del formalismo dual alimenta el gate de singularidades (SVD, 𝜎ₘᵢₙ, 𝜅) que rechaza trayectorias *Block* antes de la ejecución.

---

## 4.8 Resolución de la cinemática inversa (texto completo, listo para pegar)

La cinemática inversa del FABRI Creator se resuelve en dos etapas desacopladas —posición (primeras tres articulaciones) y orientación (muñeca de 2 GDL)—, aprovechando la estructura del robot.

**Posición — DLS con regularización de Levenberg.** La posición del efector se resuelve por mínimos cuadrados amortiguados (DLS) con estructura tipo Levenberg–Marquardt: partiendo de una semilla (arranque en caliente para seguimiento), se itera el error 𝒆 = 𝒑ₜ − 𝒑ₑ(𝑞), se corrige Δ𝑞 = 𝐽ᵀ(𝐽𝐽ᵀ + λ²𝐼)⁻¹ 𝒆 con λ = 0.05, se limita el paso a 0.5 rad, se sujetan los límites articulares en cada iteración y se converge cuando ‖𝒆‖ < 1 mm (máximo 200 iteraciones).

**Orientación — muñeca analítica de 2 GDL.** Con 𝑞₁..𝑞₃ resueltos, se evalúa la rotación 𝑅₀₃ por cinemática directa y se descompone 𝑅₃₅ = 𝑅₀₃ᵀ 𝑅ₜₐᵣₑₜ. La condición de alcanzabilidad |𝑅₃₅(0,2)| ≤ tolerancia valida la estructura de muñeca, y las articulaciones se obtienen por funciones arctangente: 𝑞₄ = atan2(−𝑅₃₅(2,2), −𝑅₃₅(1,2)) y 𝑞₅ = atan2(−𝑅₃₅(0,1), 𝑅₃₅(0,0)), validando los límites de ambas articulaciones.

**Modo de dibujo.** Para el modo de dibujo se restringe la variedad 𝑞₄ = 0, 𝑞₅ = −(𝑞₂ + 𝑞₃), que mantiene el marcador vertical, y se emplea un Jacobiano reducido [𝐽₁, 𝐽₂ − 𝐽₅, 𝐽₃ − 𝐽₅] derivado por regla de la cadena.

---

## Checklist de correcciones verificadas (18/08/2026)

1. **λ = 0.1 → 0.05** en el párrafo original de cinemática inversa (quedó contradictorio con la sección nueva; el código usa `damping = 0.05`).
2. **Home del efector**: el informe dice (140, −15, 205) mm sin base/tool y TCP (236, 0, 314); los valores reales actuales son **p_ee = (160, 0, 185) mm** sin base/tool (binario `dynamics-report`) y **TCP = (277.0, 0.0, 242.0) mm** con base + herramienta (marcador perpendicular, verificado en el viewport de Bombolab).
3. **Dinámica estática** (valores del repo actual vía `test-case-report`):
   - g(home) = [0, −0.1344, −0.1344, 0, 0] N·m (el informe dice −0.1177)
   - g(q_test) = [0, −0.3092, −0.1344, 0, 0] N·m (el informe dice −0.2925 / −0.1177)
   - M(q_test) con M11 = 6803.2 kg·mm² (el informe dice 6477.5); M completa: [[6803.2, −2.2, −2.2, 0, 3.5], [−2.2, 7185.0, 2978.5, 0, 2.0], [−2.2, 2978.5, 1816.0, 0, 2.0], [0, 0, 0, 17.0, 0], [3.5, 2.0, 2.0, 0, 4.0]]
4. **Conteo de pruebas**: el repo tiene **157** tests Rust (no 158) y **19** archivos de prueba en web (no 14).
5. **Convención**: la Tabla 2 dice "según la conveniencia de Craig" y el texto dice "DH Estándar" — unificar (el marco conceptual ya habla de Craig).
6. **Índice de figuras**: la Figura 3 quedó desactualizada ("Plano de conjunto" vs "Piezas stl de conjunto" en el cuerpo).
7. **Estructura**: reordenar lo pegado — 4.7 como subsección propia antes de "Cinemática inversa", y 4.8 fusionada reemplazando el párrafo original (quedó intercalado con "Para el modo de dibujo").
8. **Objetivos con Taxonomía de Bloom** (la rúbrica lo pide explícitamente): agregar el nivel a cada objetivo específico.
9. **Tipeos**: "La vali dez del modelo", "𝑞4 y 𝑞5..", "C+ +" (arquitectura de software).