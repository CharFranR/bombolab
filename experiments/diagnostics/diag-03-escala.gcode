G21 G90
; DIAG-03: rectángulos concéntricos de 3 tamaños — discrimina error de ESCALA
; (modelo/tool frame, crece con el tamaño) vs error CONSTANTE (offsets/backlash).
; 50x50, 90x90 y 130x90 centrados en (260,-45), con dwell de 500ms en cada esquina
; para marcar los vértices a medir.
; Medir: los 4 lados de cada rectángulo + las 2 diagonales de cada uno.
; - Si el error mm crece proporcional al lado nominal -> error de modelo/escala.
; - Si el error es ~igual en los 3 tamaños -> offset/backlash constante.
; Área segura MG996R: x 160..360, y -130..30 (plano z=80).
G0 X260 Y-45 Z5
; --- rectángulo 50x50 (x 235..285, y -70..-20) ---
G0 X235 Y-70
G4 P500
G0 Z0
G1 X285 Y-70 F1200
G4 P500
G1 X285 Y-20 F1200
G4 P500
G1 X235 Y-20 F1200
G4 P500
G1 X235 Y-70 F1200
G4 P500
G0 Z5
; --- rectángulo 90x90 (x 215..305, y -90..0) ---
G0 X215 Y-90
G4 P500
G0 Z0
G1 X305 Y-90 F1200
G4 P500
G1 X305 Y0 F1200
G4 P500
G1 X215 Y0 F1200
G4 P500
G1 X215 Y-90 F1200
G4 P500
G0 Z5
; --- rectángulo 130x90 (x 195..325, y -90..0) ---
G0 X195 Y-90
G4 P500
G0 Z0
G1 X325 Y-90 F1200
G4 P500
G1 X325 Y0 F1200
G4 P500
G1 X195 Y0 F1200
G4 P500
G1 X195 Y-90 F1200
G4 P500
G0 Z5
