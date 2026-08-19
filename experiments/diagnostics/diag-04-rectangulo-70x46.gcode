G21 G90
; DIAG-04: rectángulo 140x92 + centro — la firma de la prueba de 5 puntos.
; Proporción 3:2 (150x100 escalado al área segura MG996R), con dwell LARGO
; (800ms) en los 5 puntos para medir POSICIONES y ÁNGULOS con precisión.
; Este patrón reproduce la geometría que dio 139mm arriba / 75-80mm de altura
; / 103° en P4 en la prueba anterior, ampliado al área nueva.
; Medir (nominal: x 190..330, y -86..6):
;   - P1-P2 (abajo) y P4-P3 (arriba): anchos nominales 140mm  <-- comparar
;   - P1-P4 (izq) y P2-P3 (der): alturas nominales 92mm
;   - 4 ángulos internos con transportador (nominal 90°)
;   - P0->P1..P4 y las 2 diagonales
; La firma TRAPEZOIDE = ancho arriba != ancho abajo (plano inclinado / modelo z).
; Área segura MG996R: x 160..360, y -130..30 (plano z=80).
G0 X260 Y-40 Z5
; --- esquina P1 (abajo-izquierda) ---
G0 X190 Y-86
G4 P800
G0 Z0
G1 X330 Y-86 F1200
G4 P800
; --- P2 (abajo-derecha) -> P3 (arriba-derecha) ---
G1 X330 Y6 F1200
G4 P800
; --- P3 -> P4 (arriba-izquierda) ---
G1 X190 Y6 F1200
G4 P800
; --- P4 -> P1 (cierre) ---
G1 X190 Y-86 F1200
G4 P800
G0 Z5
; --- centro P0 con cruce ---
G0 X260 Y-40
G4 P800
G0 Z0
G1 X255 Y-40 F1200
G1 X265 Y-40 F1200
G0 Z5
; --- diagonal P1->P3 (nominal 167.6mm) ---
G0 X190 Y-86
G4 P800
G0 Z0
G1 X330 Y6 F1200
G4 P800
G0 Z5
; --- diagonal P4->P2 ---
G0 X190 Y6
G4 P800
G0 Z0
G1 X330 Y-86 F1200
G4 P800
G0 Z5
