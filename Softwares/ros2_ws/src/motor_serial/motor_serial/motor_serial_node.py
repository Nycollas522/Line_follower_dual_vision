#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
import serial


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class MotorSerialOmni(Node):
    def __init__(self):
        super().__init__('motor_serial_omni')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_vx', 0.50)
        self.declare_parameter('max_vy', 0.50)
        self.declare_parameter('max_wz', 3.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value

        self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel_encoder_ticks', 10)
        self.wheel_pub = self.create_publisher(JointState, '/wheel_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_vel_callback,
            10,
        )

        self.lock = threading.Lock()
        self.last_twist = (0.0, 0.0, 0.0)

        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.0,
                write_timeout=0.1,
            )
            time.sleep(1.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as error:
            self.get_logger().fatal(f'Não foi possível abrir {self.port}: {error}')
            raise

        self.get_logger().info(f'ESP32 conectado em {self.port} @ {self.baudrate}')
        self.create_timer(0.005, self.read_serial)       # 200 Hz: drena a serial
        self.create_timer(0.05, self.send_twist)         # 20 Hz: watchdog do ESP32

    def write_line(self, line: str):
        try:
            with self.lock:
                self.ser.write(line.encode('ascii'))
                self.ser.flush()
        except serial.SerialException as error:
            self.get_logger().error(f'Falha na serial: {error}')

    def cmd_vel_callback(self, msg: Twist):
        vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        wz = max(-self.max_wz, min(self.max_wz, msg.angular.z))
        self.last_twist = (vx, vy, wz)

    def send_twist(self):
        vx, vy, wz = self.last_twist
        self.write_line(f'TWIST,{vx:.4f},{vy:.4f},{wz:.4f}\n')

    def read_serial(self):
        lines = []

        try:
            with self.lock:
                # Lê todas as linhas já disponíveis; evita atraso e buffer acumulado.
                while self.ser.in_waiting > 0:
                    raw = self.ser.readline()
                    if not raw:
                        break
                    line = raw.decode('utf-8', errors='replace').strip()
                    if line:
                        lines.append(line)
        except serial.SerialException as error:
            self.get_logger().error(f'Falha ao ler serial: {error}')
            return

        for line in lines:
            self.process_serial_line(line)

    def process_serial_line(self, line: str):
        if line.startswith('READY') or line == 'IMU,READY':
            self.get_logger().info(f'ESP32: {line}')
            return

        if line == 'IMU,NOT_FOUND' or line.startswith('ERR,'):
            self.get_logger().error(f'ESP32: {line}')
            return

        fields = line.split(',')

        try:
            if fields[0] == 'ENC' and len(fields) == 5:
                msg = Int32MultiArray()
                msg.data = [int(value) for value in fields[1:]]
                self.encoder_pub.publish(msg)
                return

            if fields[0] == 'WHEEL' and len(fields) == 5:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']
                msg.velocity = [float(value) for value in fields[1:]]
                self.wheel_pub.publish(msg)
                return

            if fields[0] == 'IMU' and len(fields) == 7:
                self.publish_imu([float(value) for value in fields[1:]])
                return

            if fields[0] == 'ODOM' and len(fields) == 7:
                self.publish_odom([float(value) for value in fields[1:]])
                return

        except ValueError:
            self.get_logger().warning(f'Telemetria inválida: {line}')

    def publish_imu(self, values):
        ax, ay, az, gx, gy, gz = values
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02,
        ]
        msg.linear_acceleration_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 0.25,
        ]
        self.imu_pub.publish(msg)

    def publish_odom(self, values):
        x, y, yaw, vx, vy, wz = values
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        msg.pose.covariance = [
            0.03, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.03, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
        ]
        msg.twist.covariance = [
            0.10, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.20,
        ]
        self.odom_pub.publish(msg)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        try:
            self.write_line('STOP\n')
            with self.lock:
                if self.ser.is_open:
                    self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialOmni()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
