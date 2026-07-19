| Joint | Servo | Función     | Eje cinemático                 | Home (servo) | Home (q) | Offset | Límite servo | Límite q    | Dirección de giro |
| ----- | ----- | ----------- | ------------------------------ | ------------ | -------- | ------ | ------------ | ----------- | ------------------|
| J1    | S1    | Base (Yaw)  | Z                              | 90°          | 0°       | +90°   | 10°–170°     | -80° a +80° |    Anti Horario   |
| J2    | S2    | Shoulder    | Y (o eje equivalente según DH) | 90°          | 0°       | +90°   | 10°–170°     | -80° a +80° |    Anti Horario   |
| J3    | S3    | Elbow       | Y                              | 90°          | 0°       | +90°   | 10°–170°     | -80° a +80° |       Horario     |
| J4    | S4    | Wrist Roll  | X (según DH)                   | 90°          | 0°       | +90°   | 10°–170°     | -80° a +80° |       Horario     |
| J5    | S5    | Wrist Pitch | Y                              | 90°          | 0°       | +90°   | 10°–170°     | -80° a +80° |    Anti Horario   |


| Parámetro |                        Valor |
| --------- | ---------------------------: |
| Base → J1 |   57 mm      (Base Transform)|
| J1 → J2   |                      95 mm   |
| J2 → J3   |                     162 mm   |
| J3 → J4   |                     111 mm   |
| J4 → J5   |                      41 mm   |
| J5 → Tool |   75 mm    (Tool Transform)  |
