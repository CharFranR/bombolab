G21 G90
; DIAG-07: triángulo + cuadrado — ángulos con transportador y paralelismo.
; Un triángulo rectángulo (catetos 80mm) y un cuadrado 50x50 compartiendo el
; vértice, con dwell 600ms en cada vértice.
; Medir con TRANSPORTADOR todos los ángulos internos (nominales: 90°, 45°, 45°
; en el triángulo; 90° en el cuadrado) y las longitudes.
; - Ángulos ≠ 90° en lados que el robot dibuja en una sola dirección ->
;   error direccional/plano.
; - Si el triángulo NO cierra (hipotenusa no coincide con los catetos) ->
;   acumulación de error en las esquinas (backlash en reversión).
; Área segura MG996R: x 160..360, y -130..30 (plano z=80).
G0 X260 Y-45 Z5
; --- triángulo rectángulo: P1(190,-85) P2(270,-85) P3(190,-5) ---
G0 X190 Y-85
G4 P600
G0 Z0
G1 X270 Y-85 F1200
G4 P600
G1 X190 Y-5 F1200
G4 P600
G1 X190 Y-85 F1200
G4 P600
G0 Z5
; --- cuadrado 50x50 apoyado en P1: P1(190,-85) Q2(240,-85) Q3(240,-35) Q4(190,-35) ---
G0 X190 Y-85
G4 P600
G0 Z0
G1 X240 Y-85 F1200
G4 P600
G1 X240 Y-35 F1200
G4 P600
G1 X190 Y-35 F1200
G4 P600
G1 X190 Y-85 F1200
G4 P600
G0 Z5
