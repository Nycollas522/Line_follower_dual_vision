#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisaoLinhaNode(Node):
    def __init__(self):
        super().__init__('visao_linha_node')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.centroid_pub = self.create_publisher(Point, '/line/centroid', 10)
        self.error_pub = self.create_publisher(Float32, '/line/error', 10)
        self.status_pub = self.create_publisher(Bool, '/line/status', 10)
        self.debug_pub = self.create_publisher(ImageMsg, '/line/debug_image', 10)

        self.x_reference = None

    def image_callback(self, msg: Image):
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape
        if self.x_reference is None:
            self.x_reference = w // 2

        roi = gray[h // 2:h, :]
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        _, mask = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        moments = cv2.moments(mask)
        status = False
        cx = 0
        cy = 0
        error = 0.0

        if moments['m00'] > 0:
            cx_roi = int(moments['m10'] / moments['m00'])
            cy_roi = int(moments['m01'] / moments['m00'])
            cx = cx_roi
            cy = cy_roi + h // 2
            error = float(cx - self.x_reference)
            status = True

        # Imagem de debug
        display = frame_bgr.copy()
        if status:
            cv2.circle(display, (cx, cy), 8, (0, 0, 255), -1)
            cv2.line(display, (self.x_reference, h//2), (self.x_reference, h), (255, 0, 0), 2)
            cv2.putText(display, f"erro={error:.1f}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Linha nao detectada", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Publicar debug
        debug_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
        self.debug_pub.publish(debug_msg)

        # Publicar centroid, error, status
        centroid_msg = Point()
        centroid_msg.x = float(cx)
        centroid_msg.y = float(cy)
        centroid_msg.z = 0.0

        error_msg = Float32()
        error_msg.data = error

        status_msg = Bool()
        status_msg.data = status

        self.centroid_pub.publish(centroid_msg)
        self.error_pub.publish(error_msg)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisaoLinhaNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
