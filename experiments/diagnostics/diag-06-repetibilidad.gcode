G21 G90
; DIAG-06: cuadrado 100x100 dibujado 3 VECES sobre el mismo papel — REPETIBILIDAD.
; El mismo cuadrado se dibuja 3 pasadas (pen up entre pasadas), con dwell 500ms
; en cada esquina. Las 3 pasadas deben caer encima de la primera.
; Medir: la DISPERSIÓN máxima entre las 3 marcas de cada esquina (mm).
; - Dispersión < 1mm  -> error REPETIBLE: una etapa post-IK puede corregirlo.
; - Dispersión > 3mm  -> error NO repetible: post-IK NO sirve (es mecánico
;   variable: holgura, fricción, contacto del marcador).
; Área segura MG996R: x 160..360, y -130..30 (plano z=80).
G0 X260 Y-45 Z5
; --- pasada 1 ---
G0 X210 Y-95
G4 P500
G0 Z0
G1 X310 Y-95 F1200
G4 P500
G1 X310 Y5 F1200
G4 P500
G1 X210 Y5 F1200
G4 P500
G1 X210 Y-95 F1200
G4 P500
G0 Z5
; --- pasada 2 ---
G0 X210 Y-95
G4 P500
G0 Z0
G1 X310 Y-95 F1200
G4 P500
G1 X310 Y5 F1200
G4 P500
G1 X210 Y5 F1200
G4 P500
G1 X210 Y-95 F1200
G4 P500
G0 Z5
; --- pasada 3 ---
G0 X210 Y-95
G4 P500
G0 Z0
G1 X310 Y-95 F1200
G4 P500
G1 X310 Y5 F1200
G4 P500
G1 X210 Y5 F1200
G4 P500
G1 X210 Y-95 F1200
G4 P500
G0 Z5
