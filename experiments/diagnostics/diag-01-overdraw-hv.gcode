G21 G90
; DIAG-01: overdraw HV — histéresis direccional pura.
; Cada línea se dibuja de ida (→) y vuelta (←) SIN levantar el marcador.
; Si los dos trazos NO coinciden: hay backlash/histéresis en ese eje.
; Medir: separación entre trazo de ida y trazo de vuelta (mm), en el medio y en los extremos.
; Área segura MG996R: x 160..360, y -130..30 (plano z=80).
G0 X250 Y-40 Z5
; --- horizontales (y = -55, -40, -25): ida x170->x350, vuelta x350->x170 ---
G0 X170 Y-55
G4 P400
G0 Z0
G1 X350 F1200
G4 P400
G1 X170 F1200
G4 P400
G0 Z5
G0 X170 Y-40
G4 P400
G0 Z0
G1 X350 F1200
G4 P400
G1 X170 F1200
G4 P400
G0 Z5
G0 X170 Y-25
G4 P400
G0 Z0
G1 X350 F1200
G4 P400
G1 X170 F1200
G4 P400
G0 Z5
; --- verticales (x = 220, 260, 300): ida y-60->y-20, vuelta y-20->y-60 ---
G0 X220 Y-60
G4 P400
G0 Z0
G1 Y-20 F1200
G4 P400
G1 Y-60 F1200
G4 P400
G0 Z5
G0 X260 Y-60
G4 P400
G0 Z0
G1 Y-20 F1200
G4 P400
G1 Y-60 F1200
G4 P400
G0 Z5
G0 X300 Y-60
G4 P400
G0 Z0
G1 Y-20 F1200
G4 P400
G1 Y-60 F1200
G4 P400
G0 Z5
