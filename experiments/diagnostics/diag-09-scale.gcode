G21 G90
; diag-09-scale: verificación de escala del robot — 3 líneas de longitud conocida.
; PROPÓSITO: medir la distorsión real de escala del robot, por eje.
; REQUIERE: modo calibración ACTIVADO (checkbox "dibujar SIN compensación post-IK").
; Medir con regla la longitud REAL de cada línea dibujada (del inicio al fin del trazo):
;   1. Línea H (horizontal): nominal 100 mm   -> cuánto mide de verdad?
;   2. Línea V (vertical):   nominal 50 mm    -> cuánto mide de verdad?
;   3. Diagonal:             nominal 109.7 mm (sqrt(100^2+50^2)) -> cuánto mide?
; Lectura:
;   - H mucho < 100  -> el eje X (base) se queda corto (escala < 1).
;   - H mucho > 100  -> el eje X se pasa (escala > 1).
;   - V mucho < 50   -> el eje Y (brazo) se queda corto.
;   - V mucho > 50   -> el eje Y se pasa.
;   - Si la diagonal NO es ≈ sqrt(H_medida^2 + V_medida^2) -> hay acoplamiento/rotación.
G0 X260 Y-45 Z5
; --- Línea H: 100 mm en X, y = -25 ---
G0 X210 Y-25
G0 Z0
G1 X310 Y-25 F1200
G0 Z5
; --- Línea V: 50 mm en Y, x = 260 ---
G0 X260 Y-20
G0 Z0
G1 X260 Y-70 F1200
G0 Z5
; --- Diagonal: (210,-25) -> (310,-70) ---
G0 X210 Y-25
G0 Z0
G1 X310 Y-70 F1200
G0 Z5
G0 X260 Y-45 Z5
; fin
