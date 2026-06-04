"""
Mueve el robot en un cuadrado de 1 x 1 m.

Estrategia basada en tiempo:
  - Avanzar a LINEAR_SPEED durante SIDE_TIME  segundos  → 1 m
  - Girar   a TURN_SPEED  durante TURN_TIME   segundos  → 90°
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


LINEAR_SPEED = 1.7         # m/s
SIDE_LENGTH  = 1.0          # m
SIDE_TIME    = SIDE_LENGTH / LINEAR_SPEED   # 5.0 s

TURN_SPEED   = 1.4          # rad/s  (giro a la izquierda)
TURN_ANGLE   = math.pi / 2  # 90°
TURN_TIME    = TURN_ANGLE / TURN_SPEED      # ~3.14 s


class SquareNode(Node):
    def __init__(self):
        super().__init__('square_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # estado: 0=avanzar, 1=girar; repetir 4 veces
        self._phase   = 0   # 0: avanzar, 1: girar
        self._side    = 0   # cuántos lados completados
        self._elapsed = 0.0

        dt = 0.05  # 20 Hz
        self._dt = dt
        self.timer = self.create_timer(dt, self._tick)
        self.get_logger().info('Iniciando cuadrado 1x1 m — LINEAR %.1f m/s  TURN %.1f rad/s'
                               % (LINEAR_SPEED, TURN_SPEED))

    def _tick(self):
        if self._side >= 4:
            self._stop()
            self.get_logger().info('Cuadrado completado.')
            self.timer.cancel()
            return

        twist = Twist()

        if self._phase == 0:            # avanzar
            twist.linear.x = LINEAR_SPEED
            limit = SIDE_TIME
        else:                           # girar
            twist.angular.z = TURN_SPEED
            limit = TURN_TIME

        self.pub.publish(twist)
        self._elapsed += self._dt

        if self._elapsed >= limit:
            self._stop()
            self._elapsed = 0.0
            if self._phase == 0:
                self._phase = 1         # siguiente: girar
            else:
                self._phase = 0         # siguiente: avanzar
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
