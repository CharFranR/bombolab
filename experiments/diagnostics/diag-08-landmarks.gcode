G21 G90
; diag-08-landmarks: cuadrícula 3x3 de PUNTOS con dwell, 3 pasadas (9 puntos x3).
; Cada landmark se dibuja 3 veces; las 3 marcas caen juntas y NO hay que
; distinguirlas: se miden las 3 y el script promedia. Sin colitas ni colores.
; Referencia: L dibujada por el robot (origen y ejes). Medir respecto de su vértice.
G0 X260 Y-45 Z5
; --- Referencia: L con origen en (212,-12) ---
G0 X212 Y-12
G4 P300
G0 Z0
G1 X308.0 Y-12.0 F600
G1 X212.0 Y-12.0 F600
G1 X212.0 Y-68.0 F600
G1 X212.0 Y-12.0 F600
G4 P300
G0 Z5
; ==================== PASADA 1 ====================
; --- L01 (215,-25) pasada 1 ---
G0 X215 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L02 (260,-25) pasada 1 ---
G0 X260 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L03 (305,-25) pasada 1 ---
G0 X305 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L04 (215,-45) pasada 1 ---
G0 X215 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L05 (260,-45) pasada 1 ---
G0 X260 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L06 (305,-45) pasada 1 ---
G0 X305 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L07 (215,-65) pasada 1 ---
G0 X215 Y-65
G0 Z0
G4 P2000
G0 Z5
; --- L08 (260,-65) pasada 1 ---
G0 X260 Y-65
G0 Z0
G4 P2000
G0 Z5
; --- L09 (305,-65) pasada 1 ---
G0 X305 Y-65
G0 Z0
G4 P2000
G0 Z5
; ==================== PASADA 2 ====================
; --- L01 (215,-25) pasada 2 ---
G0 X215 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L02 (260,-25) pasada 2 ---
G0 X260 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L03 (305,-25) pasada 2 ---
G0 X305 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L04 (215,-45) pasada 2 ---
G0 X215 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L05 (260,-45) pasada 2 ---
G0 X260 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L06 (305,-45) pasada 2 ---
G0 X305 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L07 (215,-65) pasada 2 ---
G0 X215 Y-65
G0 Z0
G4 P2000
G0 Z5
; --- L08 (260,-65) pasada 2 ---
G0 X260 Y-65
G0 Z0
G4 P2000
G0 Z5
; --- L09 (305,-65) pasada 2 ---
G0 X305 Y-65
G0 Z0
G4 P2000
G0 Z5
; ==================== PASADA 3 ====================
; --- L01 (215,-25) pasada 3 ---
G0 X215 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L02 (260,-25) pasada 3 ---
G0 X260 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L03 (305,-25) pasada 3 ---
G0 X305 Y-25
G0 Z0
G4 P2000
G0 Z5
; --- L04 (215,-45) pasada 3 ---
G0 X215 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L05 (260,-45) pasada 3 ---
G0 X260 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L06 (305,-45) pasada 3 ---
G0 X305 Y-45
G0 Z0
G4 P2000
G0 Z5
; --- L07 (215,-65) pasada 3 ---
G0 X215 Y-65
G0 Z0
G4 P2000
G0 Z5
; --- L08 (260,-65) pasada 3 ---
G0 X260 Y-65
G0 Z0
G4 P2000
G0 Z5
; --- L09 (305,-65) pasada 3 ---
G0 X305 Y-65
G0 Z0
G4 P2000
G0 Z5
G0 X260 Y-45 Z5
; fin
