# capytown_esan — Guía de uso

**RC-1 · La Manzana del Tambo · CapyTown · Robótica 2026-I**
Universidad ESAN · Sección B · Prof. Marks Calderón Niquin

---

## Requisitos previos

- Docker con el contenedor ROS2 Humble corriendo
- El driver del robot publicando `/odom` y escuchando `/cmd_vel`
- `ROS_DOMAIN_ID` del grupo configurado en todas las terminales
- Robot sobre superficie plana

---

## 1. Compilar el paquete

```bash
# Dentro del contenedor Docker
cd /home/pi/yahboomcar_ws
colcon build --packages-select capytown_esan
source install/setup.bash
```

> Si ya compilaste antes, solo necesitas `source install/setup.bash` en cada terminal nueva.

---

## 2. Script: cuadrado 1 × 1 m

Mueve el robot en un cuadrado de 1 metro de lado (4 lados × avanzar + girar 90°).

### Parámetros internos

| Constante | Valor | Descripción |
|-----------|-------|-------------|
| `LINEAR_SPEED` | 0.2 m/s | Velocidad de avance |
| `SIDE_LENGTH` | 1.0 m | Largo del lado |
| `TURN_SPEED` | 0.5 rad/s | Velocidad de giro |

### Ejecución

```bash
# Terminal 1 — asegúrate de que el bringup esté corriendo
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py

# Terminal 2
source /home/pi/yahboomcar_ws/install/setup.bash
ros2 run capytown_esan square
```

El nodo se detiene solo al completar los 4 lados. Para interrumpir antes: `Ctrl+C`.

### Ajuste fino

Si el robot no recorre exactamente 1 m o no gira exactamente 90°, edita las constantes en
`capytown_esan/square.py`:

```python
LINEAR_SPEED = 0.2   # aumentar si avanza menos de 1 m
TURN_SPEED   = 0.5   # ajustar si el giro no llega a 90°
```

---

## 3. Script: calibración de b_eff

Calibra iterativamente el **track width efectivo** (`b_eff` / `wheel_separation`) usando
el método UMBmark. Corrige el error sistemático de giro en robots skid-steer.

### Preparación física

1. Coloca el robot en una superficie plana sin obstáculos.
2. Pega una tira de cinta masking en el piso alineada con la orientación inicial del robot
   (sirve como referencia visual para medir el ángulo real).
3. Ten a mano un transportador o usa el método de marca circular en el piso.

### Ejecución con parámetros por defecto

```bash
source /home/pi/yahboomcar_ws/install/setup.bash
ros2 run capytown_esan calibrate_beff
```

### Ejecución con parámetros personalizados

```bash
ros2 run capytown_esan calibrate_beff --ros-args \
    -p b_eff:=0.20 \
    -p target_angle_deg:=360.0 \
    -p angular_vel:=0.4 \
    -p log_path:=/root/calibration_log.csv \
    -p max_run_seconds:=90.0
```

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `b_eff` | `0.20` | Separación efectiva de ruedas en metros (valor inicial) |
| `target_angle_deg` | `360.0` | Ángulo objetivo de cada corrida |
| `angular_vel` | `0.5` | Velocidad angular en rad/s (positivo = izquierda) |
| `log_path` | `calibration_log.csv` | Archivo de registro de corridas |
| `max_run_seconds` | `60.0` | Timeout de seguridad por corrida |

### Flujo paso a paso

```
Corrida #1
│
├─ [ENTER] para iniciar
│
├─ Robot gira ~360° en sitio
│
├─ Robot se detiene solo
│
├─ Mides el ángulo real con transportador
│   └─ Ingresas el valor (ej: 348.5)
│
├─ El script muestra:
│     Reportado por /odom : 360.00°
│     Medido a mano       : 348.50°
│     Error               :   3.30 %
│     b_eff sugerido      : 0.20667 m
│
└─ ¿Iterar? [s/N]
    ├─ s → Corrida #2 con b_eff = 0.20667
    └─ N → Termina y muestra el valor final
```

### Criterio de convergencia

El script declara convergencia automáticamente cuando el **error < 1 %**.
Típicamente se alcanza en **3 a 5 iteraciones**.

### Aplicar el b_eff calibrado

Una vez obtenido el valor final, aplícalo de dos formas:

**En runtime (temporal):**
```bash
ros2 param set /yahboom_driver wheel_separation 0.20667
```

**En el YAML del bringup (permanente):**
```yaml
# config/wheel_params.yaml
wheel_separation: 0.20667
```

### Archivo de log

Cada corrida se registra en `calibration_log.csv`:

```
timestamp,run,b_eff_used,angle_reported_deg,angle_measured_deg,b_eff_new,error_pct
2026-05-28T10:00:00,1,0.20000,360.00,348.50,0.20667,3.30
2026-05-28T10:05:00,2,0.20667,360.00,357.20,0.20876,0.78
```

---

## Estructura del paquete

```
capytown_esan/
├── GUIA.md                        ← este archivo
├── package.xml
├── setup.py
├── resource/capytown_esan
├── launch/
└── capytown_esan/
    ├── __init__.py
    ├── square.py                  ← cuadrado 1×1 m
    └── calibrate_beff.py          ← calibración de b_eff
```
