G21 G90
; DIAG-02: radial in/out — direccionalidad (afuera->adentro vs adentro->afuera).
; 8 radios de 40mm desde el centro (260,-40), cada uno dibujado centro->extremo
; y extremo->centro SIN levantar. Ataca DIRECTAMENTE el síntoma reportado:
; "el movimiento extraño del lado derecho, sobre todo de afuera hacia adentro".
; Medir: longitud de cada radio, desvío lateral del trazo de vuelta, y si la
; ondulación aparece al ir hacia el centro (E/SE/NE = lado derecho).
; Área segura MG996R: x 160..360, y -130..30 (plano z=80).
G0 X260 Y-40 Z5
; --- radio E (0°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X300 F1200
G4 P400
G1 X260 F1200
G4 P400
G0 Z5
; --- radio NE (45°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X288.3 Y-11.7 F1200
G4 P400
G1 X260 Y-40 F1200
G4 P400
G0 Z5
; --- radio N (90°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X260 Y0 F1200
G4 P400
G1 X260 Y-40 F1200
G4 P400
G0 Z5
; --- radio NO (135°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X231.7 Y-11.7 F1200
G4 P400
G1 X260 Y-40 F1200
G4 P400
G0 Z5
; --- radio O (180°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X220 F1200
G4 P400
G1 X260 F1200
G4 P400
G0 Z5
; --- radio SO (225°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X231.7 Y-68.3 F1200
G4 P400
G1 X260 Y-40 F1200
G4 P400
G0 Z5
; --- radio S (270°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X260 Y-80 F1200
G4 P400
G1 X260 Y-40 F1200
G4 P400
G0 Z5
; --- radio SE (315°) ---
G0 X260 Y-40
G4 P400
G0 Z0
G1 X288.3 Y-68.3 F1200
G4 P400
G1 X260 Y-40 F1200
G4 P400
G0 Z5
