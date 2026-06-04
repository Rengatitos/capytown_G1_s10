# RC-1 La Manzana del Tambo
**Reto Clasificatorio 1 · Escenario A — El Tambo · escala 0-20**

**Cinemática móvil · Odometría · Calibración de b_eff**  
Proyecto CapyTown — semana 10 · Robótica 2026-I  
Universidad ESAN · Prof. Marks Calderón Niquin  
**Entrega:** viernes de la semana 10  

*Este reto NO incluye señales ni obstáculos*

RC-1 es PURA cinemática y odometría: el robot recorre un cuadrado en lazo abierto. NO hay señales PARE, NO hay semáforo, NO hay karpinchus en la pista. Las señales aparecen en RC-3 (semana 12); los obstáculos en RC-4 (semana 13). Aquí solo importa que el robot se mueva con precisión y que su `/odom` diga la verdad sobre dónde está.

---

## 1. Resumen del reto
El robot debe recorrer una manzana cuadrada de 1.0 × 1.0 m en lazo abierto (sin sensores de corrección) partiendo y terminando en el mismo punto. La maniobra se repite 3 veces consecutivas, sin detener al robot entre repeticiones. Se mide el error de posición final con cinta métrica.

**Lo que el chaski aprende en esta semana:**
* Modelar el Yahboom MicroROS-Pi5 como skid-steer 4 ruedas y derivar la cinemática diferencial equivalente.
* Configurar el bringup ROS2: Wi-Fi, SSH, `ROS_DOMAIN_ID` único, `/cmd_vel` + `/odom` funcionando.
* Calibrar el parámetro `b_eff` (track efectivo) con el protocolo UMBmark simplificado.
* Recorrer una manzana cuadrada de 1×1 m en lazo abierto y medir el error de cierre.
* Grabar ros2 bag con `/odom`, `/cmd_vel` y `/tf`, y procesarlo para graficar la trayectoria estimada.
* Explicar por qué la odometría siempre deriva — y qué semanas futuras lo van a corregir.

**TPACK del reto**
* **CK** — modelo diferencial, propagación de error, frames TF2. 
* **TK** — ros2 bag, `/odom`, `/cmd_vel`, rqt_graph, `calibrate_beff.py`. 
* **PK** — predicción cuantitativa ANTES de medir; la discrepancia entre predicción y dato es el motor del aprendizaje.

---

## 2. El paquete `capytown_esan`
Todo el código del semestre vive en un solo paquete ROS2 Python: `capytown_esan`. Es el repositorio base que la cátedra mantiene durante las 6 semanas del proyecto (10-15). Cada grupo trabaja sobre un fork con el nombre `capytown_G<n>_s10..s15`.

### 2.1. Qué es el paquete
Es un paquete `ament_python` estándar de ROS2 Humble que reúne nodos, launchers, scripts de utilidad, archivos de configuración (YAML) y mapas. Cada semana agrega capas: en semana 10 solo usaremos el bringup y dos scripts auxiliares; las semanas 11–14 irán activando `lane_detector`, `sign_detector`, `obstacle_detector`, `route_planner`, etc.

### 2.2. Estructura del paquete
```text
capytown_esan_pkg/
├── launch/
│   ├── bringup.launch.py          # <- ESTA SEMANA
│   ├── perception.launch.py       # semanas 11+
│   └── competition.launch.py      # semana 15
├── capytown_esan/
│   ├── __init__.py
│   ├── teleop_key.py              # <- ESTA SEMANA
│   ├── calibrate_beff.py          # <- ESTA SEMANA (script de calibración)
│   ├── error_odom.py              # <- LO ESCRIBES TÚ (entregable)
│   ├── lane_detector.py           # semana 11
│   ├── lane_controller.py         # semana 11
│   ├── sign_detector.py           # semana 12
│   ├── obstacle_detector.py       # semana 13
│   ├── behavior_fsm.py            # semanas 12-14
│   ├── route_planner.py           # semana 14
│   └── intersection_handler.py    # semana 14
├── config/
│   ├── wheel_params.yaml          # <- ESTA SEMANA (b_eff, r, etc.)
│   ├── hsv_params.yaml            # semana 11
│   ├── pid_params.yaml            # semana 11
│   └── tile_graph.yaml            # semana 14
├── scripts/
│   └── hsv_tuner.py               # semana 11
├── maps/
│   └── capytown_full.pgm/.yaml    # semana 14+
├── package.xml
├── setup.py
└── README.md
```

### 2.3. Cómo obtener el paquete e instalarlo
```bash
# 1) Clonar el fork del grupo (lo crea cada grupo a partir del repo de la cátedra)
cd ~/ros2_ws/src
git clone https://github.com/<TU_USUARIO>/capytown_G<n>_s10.git capytown_esan

# 2) Compilar
cd ~/ros2_ws
colcon build --packages-select capytown_esan
source install/setup.bash

# 3) Verificar que ROS2 lo ve
ros2 pkg executables capytown_esan
```
Si todo está bien, deberías ver listados al menos:
* `capytown_esan calibrate_beff`
* `capytown_esan teleop_key`

### 2.4. Configuración por grupo
Cada grupo tiene un `ROS_DOMAIN_ID` único asignado por la cátedra (G1 → 1, G2 → 2, … G10 → 10). Exportar siempre antes de cualquier comando ROS2:
```bash
export ROS_DOMAIN_ID=<n>   # n = número del grupo
```
**Por qué ROS_DOMAIN_ID importa:** El lab tiene 10 robots transmitiendo por DDS simultáneamente. Sin dominios separados, todos los robots reciben los `/cmd_vel` de todos los grupos y se produciría caos. El `ROS_DOMAIN_ID` aísla cada grupo en su propia red lógica.

---

## 3. Scripts disponibles esta semana

| Script / Launch | Estado | Para qué sirve |
| :--- | :--- | :--- |
| `launch/bringup.launch.py` | Provisto | Lanza el driver del Yahboom: publica `/odom`, `/tf`, `/scan`; suscribe `/cmd_vel`. |
| `capytown_esan/teleop_key.py` | Provisto | Teleoperación por teclado (WASD). Útil para mover el robot a mano y verificar el bringup. |
| `capytown_esan/calibrate_beff.py` | Provisto | Calibración iterativa del track efectivo `b_eff` por protocolo UMBmark simplificado. Lee `b_eff` inicial, gira 360° en sitio, pide el ángulo medido, calcula el `b_eff` corregido. Ver guía anexa. |
| `capytown_esan/error_odom.py` | **LO ESCRIBES TÚ** | Script Python (entregable). Lee un ros2 bag con `/odom` + `/cmd_vel` y grafica la trayectoria estimada vs. la ideal del cuadrado. Calcula error de cierre. |
| `config/wheel_params.yaml` | Provisto | Parámetros del modelo cinemático: `wheel_radius` (r), `wheel_separation` (b_eff), `encoder_ticks_per_rev`. ESTE archivo es el que actualizas con el resultado de `calibrate_beff`. |

### 3.1. Cómo lanzar el bringup
```bash
# Vía SSH al Pi5
ssh capybara@<IP_DEL_PI5>
export ROS_DOMAIN_ID=<n>
ros2 launch capytown_esan bringup.launch.py
```
Al levantar el bringup deberías ver:
* `/odom` publicando a ~50 Hz (verifica con `ros2 topic hz /odom`).
* `/tf` con la transformación `odom` → `base_link` continua.
* `/cmd_vel` listo para recibir mensajes Twist.
* `/scan` publicando el LiDAR MS200 (aunque no lo usemos esta semana, ya debe estar OK).

### 3.2. Cómo lanzar calibrate_beff
El protocolo completo está en la Guía de calibración de b_eff (anexo). Resumen del lanzamiento:
```bash
ros2 run capytown_esan calibrate_beff --ros-args     -p b_eff:=0.24     -p target_angle_deg:=360.0     -p angular_vel:=0.5
# Sigue las instrucciones por terminal. Después de la convergencia,
# escribe el b_eff final en config/wheel_params.yaml y vuelve a lanzar bringup.
```

---

## 4. Reglas del reto

| Aspecto | Especificación |
| :--- | :--- |
| **Escenario** | A — El Tambo (16 tiles, 2.0 × 2.0 m). Solo se usa el tile vacío del centro y un cuadrado dibujado con cinta de 1 × 1 m sobre él. |
| **Trayectoria** | Cuadrado de 1.0 × 1.0 m. Lados rectos + 4 giros de 90° en las esquinas. Sentido a elección del grupo (horario o antihorario). |
| **Modo de control** | LAZO ABIERTO. No se permite usar cámara, LiDAR ni IMU para corrección — solo `/cmd_vel` y `/odom`. |
| **Velocidad recomendada** | 0.10 m/s lineal · 0.5 rad/s angular. Velocidades mayores son posibles, pero el slip lateral del skid-steer crece con la velocidad. |
| **Señales PARE** | NO HAY. Aparecen en RC-3 (semana 12). |
| **Semáforo** | NO HAY. Aparece en RC-3 (semana 12). |
| **Karpinchus / obstáculos** | NO HAY. Aparecen en RC-4 (semana 13). |
| **Repeticiones** | 3 vueltas consecutivas SIN detener al robot entre repeticiones. El error final se promedia sobre las 3. |
| **Medición del error** | Con cinta métrica desde el punto de partida marcado en el piso hasta el punto donde el robot terminó la última repetición. |
| **Tiempo de pista** | Cada grupo tiene 20 minutos de circuito + 25 minutos de calibración previa en mesa. |

---

## 5. Activación cognitiva — predicción antes de medir
Antes de encender el robot, el grupo escribe en el informe las respuestas a estas tres preguntas. La discrepancia entre predicción y medición es el primer dato científico que el grupo produce en CapyTown.
1. Si el robot recorre el cuadrado de 1 × 1 m sin corrección sensorial y vuelve al punto de partida, ¿cuánto error de posición esperamos acumular tras 3 vueltas? Dar un número en cm.
2. ¿Qué causará más error: las rectas o las esquinas? Justificar con la teoría del slip lateral del skid-steer.
3. Si reemplazáramos el Pi5 por un robot 10× más caro (con encoders ópticos de mayor resolución), ¿la odometría dejaría de derivar? ¿Por qué sí o por qué no?

---

## 6. Procedimiento del laboratorio

### 6.1. Setup (10 min)
1. Conectar la batería del Yahboom y esperar 30 s a que arranque.
2. SSH al Pi5: `ssh capybara@<IP_DEL_PI5>`
3. Exportar el `ROS_DOMAIN_ID` del grupo.
4. Lanzar el bringup: `ros2 launch capytown_esan bringup.launch.py`
5. Verificar en otra terminal: `ros2 topic hz /odom` debe reportar ~50 Hz.
6. Verificar transformaciones: `ros2 run tf2_tools view_frames` → debe aparecer `odom` → `base_link`.

### 6.2. Calibración de b_eff (25 min)
1. Pegar cinta masking en el piso marcando un punto de referencia y la orientación inicial del robot.
2. Lanzar `calibrate_beff` con `b_eff` inicial = ancho físico × 1.4 (≈ 0.24 m para Yahboom).
3. Seguir el ciclo: ENTER → gira → medir con transportador → introducir el valor → iterar.
4. Esperar a la convergencia (< 1 % de error) — típicamente 3-4 iteraciones.
5. Editar `config/wheel_params.yaml`: `wheel_separation = <b_eff final>`.
6. Reiniciar el bringup para que el firmware lea el nuevo valor.

### 6.3. Recorrido del cuadrado y captura de datos (20 min)
1. Dibujar con cinta masking en el piso un cuadrado de 1.0 × 1.0 m bien medido.
2. Posicionar el robot en una esquina marcando la posición inicial y la orientación con cinta.
3. Arrancar la grabación del bag: `ros2 bag record /odom /cmd_vel /tf -o tambo_G<n>_run1`
4. Lanzar la secuencia de movimiento (script propio o publicaciones manuales). Avanzar 1 m, girar 90°, repetir × 4 lados = una vuelta. Hacer 3 vueltas seguidas SIN detener al robot.
5. Detener el bag con `Ctrl+C` cuando termine la 3a vuelta.
6. Medir con cinta métrica el error final (Δx, Δy) y el error de orientación (Δθ con transportador).
7. Repetir el experimento 3 veces — generar bags `run1`, `run2`, `run3`.

### 6.4. Análisis y entrega (45 min)
1. Escribir `error_odom.py`: lee un ros2 bag, extrae la trayectoria de `/odom` y la grafica vs. el cuadrado ideal con matplotlib.
2. Generar un PNG por cada repetición con la trayectoria estimada superpuesta al cuadrado ideal.
3. Llenar la tabla del informe con: predicción inicial, `b_eff` final, error (Δx, Δy, Δθ) de cada repetición, error promedio.
4. Redactar la sección "Análisis de causa raíz": ¿por qué la deriva? ¿qué fuente domina?
5. Hacer commit final al fork `capytown_G<n>_s10` y crear un tag `v1.0-rc1`.

> **Snippet sugerido para mover el robot:**
> Para avanzar 1 m a 0.1 m/s = 10 s; para girar 90° a 0.5 rad/s = 3.14 s. Usar `ros2 topic pub` con `--rate 10 --times 100` (avance) y `--times 32` (giro). Frenar con un Twist cero entre cambios de comando.

---

## 7. Rúbrica de evaluación — escala 0 a 20
La nota de RC-1 se calcula sobre 20 puntos (escala ESAN). Se compone de tres bloques: ejecución técnica (11), informe (3), defensa técnica (2) y bonus opcional (4). El máximo alcanzable es 20.

| Componente | Criterio evaluado | Pts |
| :--- | :--- | :--- |
| **Ejecución técnica** | Setup completo: bringup OK, `/odom` publicando, `/cmd_vel` responde, `ROS_DOMAIN_ID` correcto. | 3 |
| | Calibración de `b_eff` documentada (`calibration_log.csv` + valor final adoptado en `wheel_params.yaml`). | 4 |
| | Robot completa las 3 vueltas al cuadrado sin intervención humana. | 2 |
| | Error de posición ≤ 15 cm promedio de las 3 repeticiones (medido con cinta métrica). | 2 |
| **Informe** | Informe técnico (1 página) con predicción inicial, tabla de errores, análisis de deriva y propuesta de corrección. | 3 |
| **Defensa** | Defensa técnica individual: 1 integrante al azar responde 4 preguntas técnicas × 0.5 pt (ver §8 — banco de preguntas). | 2 |
| **Subtotal base** | Suma sin bonus. | 16 |
| **BONUS** | Error ≤ 5 cm en TODAS las repeticiones (calibración fina de `b_eff` demostrada en pista). | +4 |
| **TOTAL** | Máximo alcanzable. | 20 |

*Equivalencia interna CapyTown: la nota /20 se mapea a /4 en la tabla acumulativa del campeonato (nota_20 × 0.2 = nota_4). Esto se hace automáticamente para sumar al Grand Prix.*

---

## 8. Defensa técnica — el primer Quipu
Al final del lab, el docente sortea 1 integrante por grupo. Esa persona responde 4 preguntas técnicas extraídas del banco de abajo. Cada respuesta correcta y completa vale 0.5 pt (total 2 pt). No se permite consultar el código ni el informe durante la defensa.

> **Regla del Quipu:** Las preguntas se hacen al integrante sorteado, pero el puntaje afecta a TODO el grupo. Esto fuerza que TODOS los miembros entiendan lo que hicieron — no solo el que escribió el código. Si el integrante no sabe responder, el grupo pierde el punto. Si responde bien, el grupo se beneficia.

### 8.1. Banco de preguntas RC-1
Las 4 preguntas se eligen del siguiente banco. Toca al docente seleccionarlas con criterio mixto (concepto + implementación + análisis).

**Categoría A — Cinemática y modelo del robot**
* ¿Por qué el Yahboom MicroROS-Pi5 es un skid-steer y no un diferencial puro? Da un argumento mecánico y otro geométrico.
* Si comandas `linear.x = 0.2` m/s y `angular.z = 1.0` rad/s, y tu `b_eff` es 0.20 m, ¿qué velocidad lineal toma cada rueda? Justifica con la fórmula.
* ¿Qué pasa físicamente con las 4 ruedas durante un giro en sitio? ¿Por qué eso produce slip?
* ¿Cuál es la diferencia entre el track físico W y el track efectivo `b_eff`? ¿Por qué `b_eff` es mayor que W?

**Categoría B — Calibración de b_eff**
* Explica con tus palabras el protocolo UMBmark simplificado. ¿Por qué giramos 360° y no 90° o 180°?
* Si la odometría reporta 360° pero el robot giró físicamente 350°, ¿`b_eff` actual es muy grande o muy pequeño? ¿En qué dirección lo corriges?
* ¿Cuál fue el `b_eff` final que adoptaron y cuántas iteraciones les tomó converger? ¿Cómo lo aplicaron al firmware?
* Si recalibran mañana en otro piso, ¿esperan obtener el mismo `b_eff`? ¿Por qué sí o por qué no?

**Categoría C — Implementación ROS2**
* ¿Qué tópico publica el bringup para reportar la pose del robot? ¿Cuál es su tipo de mensaje y a qué frecuencia publica?
* ¿Por qué cada grupo necesita un `ROS_DOMAIN_ID` único? ¿Qué pasa si dos grupos comparten dominio?
* Muéstrame el comando exacto que usaron para grabar el bag de la repetición 1.
* ¿Qué hace su `error_odom.py`? Explica los 3 pasos principales: lectura del bag, extracción de datos, gráfica.

**Categoría D — Análisis de la odometría**
* Tienen un error final promedio de X cm. ¿A qué fuente principal lo atribuyen y por qué? (Esperamos que mencionen slip lateral del skid-steer.)
* ¿Cuál creen que produjo más error: las rectas o las esquinas? ¿Coincide con su predicción del inicio?
* Si el robot tuviera un IMU adicional y lo integraran con un EKF, ¿el error final bajaría a cero? Justifica.
* Explica en una frase por qué se dice que la odometría es "continua pero deriva".

### 8.2. Criterio de evaluación por pregunta

| Calidad de la respuesta | Puntaje |
| :--- | :--- |
| Correcta, completa y con justificación técnica. | 0.5 pt |
| Correcta pero superficial (sin justificación profunda). | 0.3 pt |
| Parcialmente correcta o con confusión menor. | 0.2 pt |
| Incorrecta o "no sé". | 0 pt |

> **Recomendación para el grupo:** Antes de venir al lab: reúnanse 30 minutos y háganse las preguntas del banco unos a otros. Si los 4 integrantes no pueden responder al menos 12 de las 16 preguntas del banco, la defensa va a salir mal. La preparación en grupo cubre el riesgo del sorteo.

---

## 9. Entregable
Repositorio Git del grupo: `capytown_G<n>_s10` con tag `v1.0-rc1`, conteniendo:
* `config/wheel_params.yaml` — con el `b_eff` final adoptado.
* `capytown_esan/error_odom.py` — el script que escriben los alumnos.
* `bags/tambo_G<n>_run1.bag`, `run2.bag`, `run3.bag` — los 3 bags grabados.
* `calibration_log.csv` — historial de iteraciones del `calibrate_beff`.
* `plots/trayectoria_run1.png`, `run2.png`, `run3.png` — generados por `error_odom.py`.
* `informe.pdf` — 1 página máximo con: predicción inicial, tabla de errores, análisis de causa raíz, propuesta de corrección para semanas futuras.
* `README.md` actualizado con los nombres del grupo, el `ROS_DOMAIN_ID` y el `b_eff` final.

**Estructura del informe (1 página)**
Secciones obligatorias: 
(a) **Predicción inicial** — las 3 respuestas de la activación cognitiva. 
(b) **Calibración de b_eff** — tabla de iteraciones y valor final. 
(c) **Resultados experimentales** — tabla con Δx, Δy, Δθ de cada repetición + plots. 
(d) **Análisis de causa raíz** — qué fuente de error dominó y por qué. 
(e) **Propuesta de corrección** — qué semana de CapyTown va a resolver cada fuente identificada.

---

## Apéndice A. Comandos de referencia rápida

**Bringup y diagnóstico**
```bash
export ROS_DOMAIN_ID=<n>
ros2 launch capytown_esan bringup.launch.py
ros2 topic list
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 node list
```

**Teleoperación manual**
```bash
ros2 run capytown_esan teleop_key
```

**Comandos de movimiento desde CLI**
```bash
# Avanzar 1 m a 0.1 m/s (~ 10 s) y luego frenar
ros2 topic pub --rate 10 --times 100 /cmd_vel     geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" ; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# Girar 90 grados antihorario a 0.5 rad/s (~ 3.14 s)
ros2 topic pub --rate 10 --times 32 /cmd_vel     geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" ; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

**Grabación de datos**
```bash
# Grabar
ros2 bag record /odom /cmd_vel /tf -o tambo_G<n>_run1
# Inspeccionar
ros2 bag info tambo_G<n>_run1
ros2 bag play tambo_G<n>_run1
```

**Calibración de b_eff**
```bash
ros2 run capytown_esan calibrate_beff --ros-args     -p b_eff:=0.24     -p target_angle_deg:=360.0     -p angular_vel:=0.5
```

---

## Apéndice B. Esqueleto sugerido de `error_odom.py`
Este es el script que cada grupo debe escribir y entregar. Estructura recomendada (no se entrega esto al pie de la letra — la creatividad del grupo es parte del entregable):

```python
#!/usr/bin/env python3
"""error_odom.py — Analisis de la trayectoria de RC-1."""
import sys
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def read_odom(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions('', '')
    )
    xs, ys = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/odom':
            msg = deserialize_message(data, Odometry)
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)
    return xs, ys

def plot(xs, ys, out_png):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(xs, ys, label='trayectoria /odom', color='#B85042')
    ideal = [(0,0), (1,0), (1,1), (0,1), (0,0)]
    ax.plot([p[0] for p in ideal], [p[1] for p in ideal],
            '--', label='ideal', color='#5C6D3A')
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title('Trayectoria RC-1 — La Manzana del Tambo')
    fig.savefig(out_png, dpi=150)
    dx, dy = xs[-1] - xs[0], ys[-1] - ys[0]
    print(f'Error de cierre: dx={dx*100:+.2f} cm, dy={dy*100:+.2f} cm')

if __name__ == '__main__':
    xs, ys = read_odom(sys.argv[1])
    plot(xs, ys, sys.argv[1] + '_trayectoria.png')
```

---

## Apéndice C. Lecturas y anexos relacionados
* Plan oficial CapyTown — sección RC-1.
* Guía de calibración de b_eff (Anexo entregado por la cátedra).
* Diapositivas Semana 10 — Cinemática de ruedas.
* Slide "Movimiento de calibración" — flujo visual del protocolo UMBmark.
* REP-105 — `ros.org/reps/rep-0105.html`
* Siegwart, Nourbakhsh & Scaramuzza — *Introduction to Autonomous Mobile Robots*, cap. 3.
