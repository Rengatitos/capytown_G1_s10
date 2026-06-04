"""
movimiento_cuadrado.py
======================
Mueve el robot en un cuadrado de 1 x 1 m (1 vuelta, lazo abierto).

Estrategia basada en tiempo:
  - Avanzar a LINEAR_SPEED durante SIDE_TIME  segundos  → 1 m
  - Girar   a TURN_SPEED  durante TURN_TIME   segundos  → 90°

USO:
    source /opt/ros/humble/setup.bash
    source ~/yahboomcar_ws/install/setup.bash
    export ROS_DOMAIN_ID=20
    python3 movimiento_cuadrado.py

RC-1 · La Manzana del Tambo · CapyTown · Robótica 2026-I · ESAN
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


LINEAR_SPEED = 0.30         # m/s  — ajustar si el robot no recorre 1.5 m
SIDE_LENGTH  = 1.5          # m
SIDE_TIME    = SIDE_LENGTH / LINEAR_SPEED   # s  → 5.0 s

TURN_SPEED   = 0.50         # rad/s
TURN_ANGLE   = math.pi / 2  # 90°

# Tiempo de giro por cada esquina — ajustar individualmente hasta lograr 90° en cada una.
# Valor teorico: (pi/2) / 0.50 = 3.14 s
TURN_TIMES   = [3.9, 3.93, 4.5, 4.5]  # [giro1, giro2, giro3, giro4]

PAUSE_TIME   = 0.5          # s de freno entre avance y giro (evita momentum residual)


class SquareNode(Node):

    def __init__(self):
        super().__init__('square_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._phase   = 0     # 0: avanzar, 1: pausa, 2: girar
        self._side    = 0     # lados completados (0-3)
        self._elapsed = 0.0
        self._started = False

        self._dt   = 0.05     # 20 Hz
        self.timer = self.create_timer(self._dt, self._tick)

        # 0.5 s de warmup para que el publisher conecte, luego pide ENTER
        self._start_timer = self.create_timer(0.5, self._prompt_start)

        self.get_logger().info(
            'Cuadrado 1.5x1.5 m — LINEAR %.2f m/s  TURN %.2f rad/s  '
            't_avance=%.2fs  t_giros=%s'
            % (LINEAR_SPEED, TURN_SPEED, SIDE_TIME, TURN_TIMES)
        )

    def _prompt_start(self):
        """Pide ENTER una sola vez. Bloquea el spin hasta que el usuario responde."""
        self._start_timer.cancel()
        try:
            input('\n  Asegurate de que el bag ya esta grabando.\n'
                  '  Presiona ENTER para iniciar el cuadrado... ')
        except EOFError:
            pass
        self._started = True
        self.get_logger().info('Iniciando cuadrado...')

    def _tick(self):
        if not self._started:
            return

        if self._side >= 4:
            self._stop()
            self.get_logger().info('Cuadrado completado.')
            self.timer.cancel()
            return

        twist = Twist()
        if self._phase == 0:        # avanzar
            twist.linear.x = LINEAR_SPEED
            limit = SIDE_TIME
        elif self._phase == 1:      # pausa (cero velocidad)
            limit = PAUSE_TIME
        else:                       # girar
            twist.angular.z = TURN_SPEED
            limit = TURN_TIMES[self._side]

        self.pub.publish(twist)
        self._elapsed += self._dt

        if self._elapsed >= limit:
            self._stop()
            self._elapsed = 0.0
            if self._phase == 0:
                self._phase = 1     # avance → pausa
            elif self._phase == 1:
                self._phase = 2     # pausa → giro
            else:
                self._phase = 0     # giro → avance
                self._side += 1
                self.get_logger().info('Lado %d/4 completado' % self._side)

    def _stop(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SquareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
