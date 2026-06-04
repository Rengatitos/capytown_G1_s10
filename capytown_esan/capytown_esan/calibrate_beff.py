#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_beff.py
=================

Script de calibración iterativa del parámetro b_eff (track width efectivo)
para el robot Yahboom MicroROS-Pi5 en configuración skid-steer 4 ruedas.

Anexo a RC-1 — La Manzana del Tambo · CapyTown · Robótica 2026-I
Universidad ESAN · Sección B · Prof. Marks Calderón Niquin

USO
---
    ros2 run capytown_esan calibrate_beff --ros-args \\
        -p b_eff:=0.20 \\
        -p target_angle_deg:=360.0 \\
        -p angular_vel:=0.5 \\
        -p log_path:=calibration_log.csv

O directamente:

    python3 calibrate_beff.py --ros-args -p b_eff:=0.20

CÓMO FUNCIONA
-------------
1.  El script publica en /cmd_vel un giro en sitio (angular.z = ω).
2.  Lee /odom continuamente, extrae el yaw del quaternion y acumula el ángulo.
3.  Cuando el yaw acumulado alcanza target_angle_deg → detiene al robot.
4.  Pregunta al usuario cuántos grados giró REALMENTE (medidos con
    transportador o marca en el piso).
5.  Calcula el b_eff corregido con la regla empírica del UMBmark:

        b_eff_new  =  b_eff_old  ×  ( θ_reportado / θ_medido )

6.  Registra la corrida en calibration_log.csv (timestamp · run · valores · error %).
7.  El usuario decide si itera (con el nuevo valor) o termina.

CONVERGENCIA
------------
El error suele bajar a < 2 % en 3–5 iteraciones. Cuando el error < 1 %
el script declara convergencia y reporta el b_eff final.

REQUISITOS
----------
- ROS2 Humble nativo (Ubuntu 22.04)
- El driver de bringup del Yahboom debe estar publicando /odom y aceptando /cmd_vel
- ROS_DOMAIN_ID del grupo configurado
- Una superficie plana y un punto de referencia visible (marca con cinta masking)

SALIDA
------
calibration_log.csv con todas las corridas. El último b_eff sugerido se debe
copiar al YAML del bringup del robot (típicamente config/wheel_params.yaml o
parámetro 'wheel_separation' del nodo del driver Yahboom).

AUTOR
-----
Marks Calderón Niquin — Robótica 2026-I ESAN — versión 1.0 (2026-05)
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ---------------------------------------------------------------
# Helpers matemáticos
# ---------------------------------------------------------------
def quaternion_to_yaw(q) -> float:
    """Quaternion → yaw (rotación en Z) en radianes."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(prev: float, curr: float) -> float:
    """
    Diferencia angular incremental entre dos lecturas de yaw, manejando
    el wraparound -π / π. Devuelve el delta SIGNADO.
    """
    diff = curr - prev
    if diff > math.pi:
        diff -= 2.0 * math.pi
    elif diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


# ---------------------------------------------------------------
# Nodo principal
# ---------------------------------------------------------------
class CalibrateBEff(Node):

    def __init__(self):
        super().__init__("calibrate_beff")

        # --- Parámetros configurables ---
        self.declare_parameter("b_eff", 0.20)
        self.declare_parameter("target_angle_deg", 360.0)
        self.declare_parameter("angular_vel", 0.5)
        self.declare_parameter("log_path", "calibration_log.csv")
        self.declare_parameter("max_run_seconds", 60.0)

        self.b_eff = float(self.get_parameter("b_eff").value)
        self.target_angle = math.radians(
            float(self.get_parameter("target_angle_deg").value)
        )
        self.angular_vel = float(self.get_parameter("angular_vel").value)
        self.log_path = str(self.get_parameter("log_path").value)
        self.max_run_seconds = float(self.get_parameter("max_run_seconds").value)

        # --- ROS interface ---
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        # --- Estado interno ---
        self.last_yaw = None
        self.accumulated_yaw = 0.0
        self.rotating = False
        self.run_count = 0
        self._t_start = None
        self._cycle_pending = True

        # --- Cabeceras de log ---
        self._init_log()

        # --- Banner inicial ---
        self._banner()

        # Timer principal: 10 Hz — publica /cmd_vel mientras self.rotating=True
        self.create_timer(0.1, self._control_loop)
        # Timer para arrancar la primera corrida (después de 1 s de warmup)
        self.create_timer(1.0, self._maybe_start_run)

    # ------------------------------------------------------------
    # Banner y log
    # ------------------------------------------------------------
    def _banner(self):
        L = self.get_logger()
        L.info("=" * 64)
        L.info("  calibrate_beff  —  Calibración iterativa del track efectivo")
        L.info("  RC-1 · La Manzana del Tambo · CapyTown · ESAN 2026-I")
        L.info("=" * 64)
        L.info(f"  b_eff inicial      : {self.b_eff:.5f} m")
        L.info(f"  ángulo objetivo    : {math.degrees(self.target_angle):.1f}°")
        L.info(f"  velocidad angular  : {self.angular_vel:.2f} rad/s")
        L.info(f"  tiempo máx/corrida : {self.max_run_seconds:.1f} s")
        L.info(f"  log file           : {os.path.abspath(self.log_path)}")
        L.info("=" * 64)

    def _init_log(self):
        if not os.path.exists(self.log_path):
            with open(self.log_path, "w", newline="") as f:
                csv.writer(f).writerow([
                    "timestamp", "run", "b_eff_used",
                    "angle_reported_deg", "angle_measured_deg",
                    "b_eff_new", "error_pct",
                ])

    # ------------------------------------------------------------
    # Ciclo de corrida
    # ------------------------------------------------------------
    def _maybe_start_run(self):
        if not self._cycle_pending:
            return
        self._cycle_pending = False
        self.run_count += 1

        L = self.get_logger()
        L.info("")
        L.info(f">>> Corrida #{self.run_count}  —  b_eff = {self.b_eff:.5f} m")
        L.info("    Marca con cinta masking la orientación INICIAL del robot.")
        L.info("    El robot va a girar ~%.0f° en sitio." %
               math.degrees(self.target_angle))
        try:
            input("    Presiona [ENTER] cuando estés listo para iniciar... ")
        except EOFError:
            pass

        self.last_yaw = None
        self.accumulated_yaw = 0.0
        self._t_start = self.get_clock().now().nanoseconds * 1e-9
        self.rotating = True

    def _control_loop(self):
        """A 10 Hz. Publica /cmd_vel mientras self.rotating=True."""
        if not self.rotating:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._t_start) > self.max_run_seconds:
            self.get_logger().warn(
                "Timeout de la corrida — deteniendo robot. Revisa /odom y velocidad."
            )
            self._finalize_run(timed_out=True)
            return

        twist = Twist()
        twist.angular.z = self.angular_vel
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------
    # Callback de odometría
    # ------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        if not self.rotating:
            return

        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.last_yaw is None:
            self.last_yaw = yaw
            return

        delta = angle_diff(self.last_yaw, yaw)
        if (self.angular_vel > 0 and delta > 0) or (self.angular_vel < 0 and delta < 0):
            self.accumulated_yaw += abs(delta)
        self.last_yaw = yaw

        if self.accumulated_yaw >= self.target_angle:
            self._finalize_run(timed_out=False)

    # ------------------------------------------------------------
    # Cierre de la corrida + pregunta al usuario
    # ------------------------------------------------------------
    def _finalize_run(self, timed_out=False):
        self.rotating = False
        for _ in range(8):
            self.cmd_pub.publish(Twist())

        L = self.get_logger()
        reported_deg = math.degrees(self.accumulated_yaw)

        if timed_out:
            L.warn(
                f"Corrida #{self.run_count} terminó por TIMEOUT. "
                f"Reportado por /odom: {reported_deg:.2f}°"
            )
        else:
            L.info("")
            L.info(f"Robot detenido. Reportado por /odom: {reported_deg:.2f}°")

        try:
            txt = input(
                "  ► Mide con transportador el ángulo REAL que giró el robot,\n"
                "    en grados (ej. 348.5):  "
            )
            measured_deg = float(txt)
        except (ValueError, EOFError):
            L.error("Valor inválido. Esta corrida no se guarda.")
            self._cycle_pending = True
            return

        if measured_deg <= 0:
            L.error("El ángulo medido debe ser > 0. Esta corrida no se guarda.")
            self._cycle_pending = True
            return

        new_beff = self.b_eff * (reported_deg / measured_deg)
        error_pct = abs(measured_deg - reported_deg) / reported_deg * 100.0

        L.info("-" * 64)
        L.info(f"  Reportado por /odom : {reported_deg:8.2f}°")
        L.info(f"  Medido a mano        : {measured_deg:8.2f}°")
        L.info(f"  Error                : {error_pct:8.2f} %")
        L.info(f"  b_eff actual         : {self.b_eff:.5f} m")
        L.info(f"  b_eff SUGERIDO       : {new_beff:.5f} m")
        L.info("-" * 64)

        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow([
                datetime.now().isoformat(timespec="seconds"),
                self.run_count,
                f"{self.b_eff:.5f}",
                f"{reported_deg:.2f}",
                f"{measured_deg:.2f}",
                f"{new_beff:.5f}",
                f"{error_pct:.2f}",
            ])

        if error_pct < 1.0:
            L.info("")
            L.info("Convergencia alcanzada (error < 1 %).")
            L.info(f"  b_eff FINAL = {new_beff:.5f} m")
            L.info("")
            L.info("  Pasa este valor al YAML del bringup, por ejemplo:")
            L.info(f"      wheel_separation: {new_beff:.5f}")
            L.info("  o en runtime:")
            L.info(f"      ros2 param set /yahboom_driver wheel_separation {new_beff:.5f}")
            rclpy.try_shutdown()
            return

        try:
            resp = input("  ¿Iterar otra corrida con el nuevo b_eff? [s/N]: ").strip().lower()
        except EOFError:
            resp = ""

        if resp.startswith("s"):
            self.b_eff = new_beff
            self._cycle_pending = True
        else:
            L.info("")
            L.info("Calibración terminada por el usuario.")
            L.info(f"Último b_eff sugerido: {new_beff:.5f} m")
            L.info(f"Log guardado en      : {os.path.abspath(self.log_path)}")
            rclpy.try_shutdown()


# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CalibrateBEff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
