#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
error_odom.py
=============

Analiza la trayectoria odometrica de RC-1 — La Manzana del Tambo.
Lee un ros2 bag con /odom_raw y grafica la trayectoria estimada vs el cuadrado ideal.

USO (con bag real del robot)
----------------------------
    python3 error_odom.py <ruta_al_bag>

    Ejemplo:
        python3 error_odom.py bags/tambo_G01_run1

MODO SIMULACION (sin ROS2 / sin bag)
-------------------------------------
    python3 error_odom.py --sim

    Genera 3 runs simulados con deriva realista y guarda los PNGs.

SALIDA
------
    <bag>_trayectoria.png   — trayectoria /odom_raw vs cuadrado ideal
    stdout                  — error de cierre (dx, dy, |d|) en cm

NOTA: el robot Yahboom MicroROS-Pi5 publica odometría en /odom_raw (no /odom).
Para grabar el bag:
    ros2 bag record /odom_raw /cmd_vel -o <nombre_bag>

RC-1 · CapyTown · Robótica 2026-I · Universidad ESAN
Prof. Marks Calderon Niquin
"""

import sys
import math
import os

import matplotlib
matplotlib.use("Agg")          # sin display — compatible con SSH
import matplotlib.pyplot as plt

# ------------------------------------------------------------------ #
# Lectura de bag (requiere ROS2 Humble + rosbag2_py instalado)
# ------------------------------------------------------------------ #

def read_odom_from_bag(bag_path):
    """
    Lee todos los mensajes /odom_raw de un ros2 bag (formato sqlite3).
    Retorna tres listas: xs, ys, thetas (en metros y radianes).
    """
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
    except ImportError:
        print("ERROR: rosbag2_py o rclpy no disponibles.")
        print("  Asegurate de tener ROS2 Humble instalado y el entorno cargado:")
        print("  source /opt/ros/humble/setup.bash")
        sys.exit(1)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions("", ""),
    )

    xs, ys, thetas = [], [], []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic in ("/odom", "/odom_raw"):
            msg = deserialize_message(data, Odometry)
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            thetas.append(math.atan2(siny, cosy))

    return xs, ys, thetas


# ------------------------------------------------------------------ #
# Modo simulacion (no requiere ROS2)
# ------------------------------------------------------------------ #

def simulate_odom(n_laps=3, dt=0.02,
                  linear_speed=0.10,
                  angular_speed=0.50,
                  turn_noise_std=0.015,
                  turn_bias=0.0,
                  angular_noise_std=0.002,
                  linear_noise_std=0.0015,
                  seed=42):
    """
    Simula datos de /odom_raw para n_laps vueltas al cuadrado de 1x1 m.

    Modela la deriva tipica de un skid-steer post-calibracion:
    - Ruido gaussiano en velocidad lineal (cuantizacion de encoder)
    - Ruido gaussiano en giro angular durante rectas
    - Error residual en cada giro de 90° (< 1% despues de calibrar b_eff)
    """
    import numpy as np
    np.random.seed(seed)

    side_time = 1.0 / linear_speed          # s — tiempo de avance 1 m
    turn_time = (math.pi / 2) / angular_speed  # s — tiempo de giro 90 deg

    x, y, theta = 0.0, 0.0, 0.0
    xs, ys, thetas = [x], [y], [theta]

    for _ in range(n_laps):
        for _side in range(4):
            # --- Avance recto ---
            n_steps = int(side_time / dt)
            for _ in range(n_steps):
                v   = linear_speed + np.random.normal(0, linear_noise_std)
                omg = np.random.normal(0, angular_noise_std)
                x     += v * math.cos(theta) * dt
                y     += v * math.sin(theta) * dt
                theta += omg * dt
                xs.append(x); ys.append(y); thetas.append(theta)

            # --- Giro 90 deg (con error residual post-calibracion) ---
            actual = math.pi / 2 + turn_bias + np.random.normal(0, turn_noise_std)
            omg_act = actual / turn_time
            n_steps = int(turn_time / dt)
            for _ in range(n_steps):
                theta += omg_act * dt
                xs.append(x); ys.append(y); thetas.append(theta)

    return xs, ys, thetas


# ------------------------------------------------------------------ #
# Visualizacion y calculo de error
# ------------------------------------------------------------------ #

def plot_trajectory(xs, ys, run_name, out_png):
    """
    Grafica la trayectoria /odom_raw vs el cuadrado ideal 1x1 m.
    Calcula el error de cierre y lo imprime en consola.
    """
    dx  = xs[-1] - xs[0]
    dy  = ys[-1] - ys[0]
    err = math.sqrt(dx**2 + dy**2) * 100      # cm

    fig, ax = plt.subplots(figsize=(7, 7))

    # Cuadrado ideal
    ideal = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
    ax.plot([p[0] for p in ideal], [p[1] for p in ideal],
            "--", color="#5C6D3A", lw=2, label="Trayectoria ideal")

    # Trayectoria odom
    ax.plot(xs, ys, color="#B85042", lw=1.2, alpha=0.85,
            label="Trayectoria /odom_raw")

    # Inicio y fin
    ax.plot(xs[0],  ys[0],  "o", ms=12, color="#2E86AB",
            zorder=5, label="Inicio (0, 0)")
    ax.plot(xs[-1], ys[-1], "s", ms=12, color="#E84855",
            zorder=5, label=f"Fin  dx={dx*100:+.1f} cm, dy={dy*100:+.1f} cm")

    # Flecha del error de cierre
    ax.annotate("", xy=(xs[-1], ys[-1]), xytext=(xs[0], ys[0]),
                arrowprops=dict(arrowstyle="->", color="#E84855", lw=2.5))

    # Recuadro con metricas
    info = (f"Error de cierre:\n"
            f"  dx = {dx*100:+.2f} cm\n"
            f"  dy = {dy*100:+.2f} cm\n"
            f"  |d| = {err:.2f} cm")
    ax.text(0.03, 0.97, info, transform=ax.transAxes, fontsize=10,
            va="top", bbox=dict(boxstyle="round", facecolor="lightyellow",
                                alpha=0.9))

    ax.set_xlim(-0.2, 1.3)
    ax.set_ylim(-0.2, 1.3)
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(
        f"Trayectoria RC-1 — La Manzana del Tambo\n{run_name} | 1 vuelta x 1x1 m"
    )
    ax.legend(loc="upper right", fontsize=9)

    plt.tight_layout()
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"[{run_name}] Error de cierre: dx={dx*100:+.2f} cm, "
          f"dy={dy*100:+.2f} cm, |d|={err:.2f} cm  ->  {out_png}")
    return dx, dy, err


# ------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------ #

def main():
    if len(sys.argv) < 2:
        print("Uso:")
        print("  Con bag real : python3 error_odom.py <ruta_bag>")
        print("  Simulacion   : python3 error_odom.py --sim")
        sys.exit(1)

    if sys.argv[1] == "--sim":
        # --- Modo simulacion: 3 runs con semillas distintas ---
        print("Modo simulacion activado (sin ROS2 requerido)")
        os.makedirs("plots", exist_ok=True)
        configs = [
            ("run1", 42,  0.003),
            ("run2", 7,  -0.002),
            ("run3", 123, 0.004),
        ]
        resultados = []
        for name, seed, bias in configs:
            xs, ys, _ = simulate_odom(seed=seed, turn_bias=bias)
            out_png = f"plots/trayectoria_{name}.png"
            dx, dy, err = plot_trajectory(xs, ys, name, out_png)
            resultados.append((name, dx * 100, dy * 100, err))

        # Resumen
        print()
        print(f"{'Run':<8} {'dx [cm]':>10} {'dy [cm]':>10} {'|d| [cm]':>10}")
        print("-" * 42)
        errores = []
        for name, dx, dy, err in resultados:
            print(f"{name:<8} {dx:>+10.2f} {dy:>+10.2f} {err:>10.2f}")
            errores.append(err)
        prom = sum(errores) / len(errores)
        print("-" * 42)
        print(f"{'Promedio':<8} {'':>10} {'':>10} {prom:>10.2f}")
        print()
        if prom <= 5.0:
            print(f"Error promedio = {prom:.2f} cm <= 5 cm  ->  BONUS +4 pts alcanzado!")
        elif prom <= 15.0:
            print(f"Error promedio = {prom:.2f} cm <= 15 cm  ->  2 pts asegurados.")
        else:
            print(f"Error promedio = {prom:.2f} cm  ->  Revisar calibracion b_eff.")

    else:
        # --- Modo bag real ---
        bag_path = sys.argv[1]
        print(f"Leyendo bag: {bag_path}")
        xs, ys, thetas = read_odom_from_bag(bag_path)

        if not xs:
            print("ERROR: no se encontraron mensajes /odom_raw en el bag.")
            print("  Verifica que grabaste con: ros2 bag record /odom_raw /cmd_vel -o <nombre>")
            print("  o con: ros2 bag info " + bag_path)
            sys.exit(1)

        print(f"Mensajes /odom_raw leidos: {len(xs)}")
        run_name = os.path.basename(bag_path)
        out_png  = bag_path + "_trayectoria.png"
        plot_trajectory(xs, ys, run_name, out_png)


if __name__ == "__main__":
    main()
