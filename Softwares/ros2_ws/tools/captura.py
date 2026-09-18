import os, time
import cv2, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
S = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=1)
OUT = os.path.expanduser('~/quebra'); os.makedirs(OUT, exist_ok=True)
class N(Node):
    def __init__(self):
        super().__init__('captura'); self.br = CvBridge(); self.im = {}
        for k, t in (('inf_dbg','/line/debug_image'),
                     ('sup_dbg','/line_front/debug_image'),
                     ('sup_cru','/cam_front/image_raw'),
                     ('inf_cru','/cam_bottom/image_raw')):
            self.create_subscription(Image, t,
                                     lambda m,k=k: self.im.__setitem__(k,m), S)
rclpy.init(); n = N(); fim = time.time()+5
while time.time() < fim: rclpy.spin_once(n, timeout_sec=0.05)
for k, m in n.im.items():
    cv2.imwrite(f'{OUT}/{k}.png', n.br.imgmsg_to_cv2(m,'bgr8')); print(k,'ok')
n.destroy_node(); rclpy.shutdown()
