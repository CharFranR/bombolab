G21 G90
; diag-10-radial: TCP desde cerca de la base hacia ADELANTE, con J1 fijo.
; Línea y=0 (dirección radial del brazo): un SOLO movimiento de (170,0) a (350,0)
; en el plano de dibujo z=80. J1 NO se mueve (verificado: q1=0 en todo el rango).
; Medir con la regla la LONGITUD REAL del trazo dibujado:
;   - 180 mm  -> el brazo reproduce la distancia pedida (escala correcta).
;   - mucho < 180  -> el brazo se queda corto (escala < 1).
;   - mucho > 180  -> el brazo se pasa (escala > 1).
; Modo calibración ACTIVADO (sin compensación post-IK).
G0 X170 Y0 Z5
G0 Z0
G1 X350 Y0 F1200
G0 Z5
G0 X260 Y0 Z5
; fin
