# CapyTown ESAN - RC-1 La Manzana del Tambo

Grupo: G1

Integrantes:
- AGUILAR CONTRERAS Angel Jesus
- CABALLERO SALAZAR Mattias Lincoln
- NECIOSUP SAAVEDRA Leslie Jazmin
- TICONA SANCHEZ Camila Danna

ROS_DOMAIN_ID: 1

b_eff final: 0.23000 m

## Estructura

```text
config/
└── wheel_params.yaml

capytown_esan/
└── error_odom.py

bags/
├── tambo_G_run1.bag/
├── tambo_G_run2.bag/
└── tambo_G_run3.bag/

plots/
├── trayectoria_run1.png
├── trayectoria_run1_intento_fallido.png
├── trayectoria_run1_iter1.png
├── trayectoria_run1_iter2.png
├── trayectoria_run1_iter3.png
├── trayectoria_run1_iter4.png
├── trayectoria_run1_iter5.png
├── trayectoria_run2.png
├── trayectoria_run2_intento_fallido.png
├── trayectoria_run2_iter1.png
├── trayectoria_run2_iter2.png
├── trayectoria_run2_iter3.png
├── trayectoria_run2_iter4.png
├── trayectoria_run2_iter5.png
├── trayectoria_run3.png
├── trayectoria_run3_intento_fallido.png
├── trayectoria_run3_iter1.png
├── trayectoria_run3_iter2.png
├── trayectoria_run3_iter3.png
├── trayectoria_run3_iter4.png
├── trayectoria_run3_iter5.png
├── trayectoria_corregida_final.png
└── resumen_error_cierre.png

calibration_log.csv
informe.pdf
```

## Ejecucion
https://github.com/user-attachments/assets/97d03a81-6cae-4ded-bd68-6aab40559967

## Notas

- `calibration_log.csv` registra el historial de calibracion de `b_eff` en la raiz del repositorio.
- `wheel_params.yaml` conserva el valor final adoptado y el historial comentado de calibracion.
- Los bags simulados se guardan como carpetas `bags/tambo_G_run*.bag/` con `odom.csv` y `metadata.txt` para reproducibilidad local.
](https://github.com/user-attachments/assets/97d03a81-6cae-4ded-bd68-6aab40559967)
