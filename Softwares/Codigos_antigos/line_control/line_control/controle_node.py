#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ControleNode(Node):
    def __init__(self):
        super().__init__('controle_node')

        # Parâmetros do PID
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.1)

        # Parâmetros de velocidade
        self.declare_parameter('v_linear', 0.2)  # m/s
        self.declare_parameter('v_angular_max', 0.4)  # rad/s

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.v_linear = self.get_parameter('v_linear').value
        self.v_angular_max = self.get_parameter('v_angular_max').value

        # Assinante do erro
        self.error_sub = self.create_subscription(
            Float32,
            '/line/error',
            self.error_callback,
            10
        )

        # Publicador de cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variáveis do PID
        self.error_anterior = 0.0
        self.error_integral = 0.0
        self.last_time = None

        self.declare_parameter('error_max', 200.0)  # limite do erro (pixels)

        self.error_max = self.get_parameter('error_max').value

    def error_callback(self, msg: Float32):
        error = msg.data

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.01

        # Normalizar erro (opcional, mas ajuda no ajuste)
        error_norm = error / self.error_max

        # PID
        self.error_integral += error_norm * dt
        error_derivativo = (error_norm - self.error_anterior) / dt

        u = self.Kp * error_norm + self.Ki * self.error_integral + self.Kd * error_derivativo

        # Limitar velocidade angular
        u = max(-1.0, min(1.0, u))
        v_angular = u * self.v_angular_max

        # Publicar cmd_vel
        twist = Twist()
        twist.linear.x = self.v_linear
        twist.angular.z = v_angular

        self.cmd_pub.publish(twist)

        self.error_anterior = error_norm
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = ControleNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
