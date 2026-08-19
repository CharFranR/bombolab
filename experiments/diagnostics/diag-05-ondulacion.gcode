G21 G90
; DIAG-05: línea larga repetida a 3 velocidades — ondulación y su causa.
; La misma línea (y=-40, x 170->350) se dibuja 3 veces SIN levantar, a
; F1200 (20mm/s), F2400 (40mm/s) y F3600 (60mm/s).
; Medir: la ONDULACIÓN en el lado derecho (x>280 aprox) en cada pasada.
; - Si la ondulación crece con la velocidad -> dinámica (inercia/resonancia).
; - Si es igual en las 3 -> estática (backlash/flexión por gravedad).
; - Anotar también el error del punto final en cada pasada.
; Área segura MG996R: x 160..360, y -130..30 (plano z=80).
G0 X260 Y-40 Z5
G0 X170 Y-40
G4 P400
G0 Z0
; --- pasada 1: lenta (20 mm/s) ---
G1 X350 F1200
G4 P400
; --- pasada 2: media (40 mm/s) ---
G1 X170 F2400
G4 P400
; --- pasada 3: rápida (60 mm/s) ---
G1 X350 F3600
G4 P400
G0 Z5
; --- ida y vuelta rápida para ver histéresis de reversión ---
G0 X170 Y-40
G4 P400
G0 Z0
G1 X350 F3600
G4 P400
G1 X170 F3600
G4 P400
G0 Z5
