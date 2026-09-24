"""Fotografa a superior em angulos escolhidos, para INSPECAO VISUAL.

Numero de confianca nao prova que o detector achou a PISTA -- ele pode
estar casando com a quina de um movel. A imagem prova.
"""
import os
import subprocess
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from line_msgs.msg import HeadRequest, LineDetection
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

C = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
S = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=1)
ANGULOS = [float(a) for a in sys.argv[1:]] or [0.0]
OUT = os.path.expanduser('~/mira')
os.makedirs(OUT, exist_ok=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_seguro import trava_motores, liga_autonomia, devolve

# TRAVA OS MOTORES ANTES DE QUALQUER COISA. Mexer o servo exige ligar a
# autonomia, e em 21/09/2026 isso fez o robo ANDAR porque o modo estava
# em SEGUIDOR. trava_motores() levanta excecao se nao conseguir provar
# que o corpo esta desarmado.
# percepcao=True: PARADO desliga a percepcao, e sem ela este script
# nao recebe deteccao nem imagem de debug.
trava_motores(percepcao=True)
rclpy.init()
n = Node('olha')
br = CvBridge()
st = {'img': None, 'det': None}
n.create_subscription(Image, '/line_front/debug_image',
                      lambda m: st.__setitem__('img', m), S)
n.create_subscription(LineDetection, '/line_front/detection',
                      lambda m: st.__setitem__('det', m), C)
req = n.create_publisher(HeadRequest, '/head/request', C)
liga_autonomia(n, True)   # so o servo responde; o corpo esta travado

m = HeadRequest(); m.mode = HeadRequest.MODE_HOLD
for ang in ANGULOS:
    m.angle_deg = float(ang)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        m.header.stamp = n.get_clock().now().to_msg()
        req.publish(m); rclpy.spin_once(n, timeout_sec=0.02)
    if st['img'] is not None:
        nome = f'{OUT}/mira_{int(ang):+03d}.png'
        cv2.imwrite(nome, br.imgmsg_to_cv2(st['img'], 'bgr8'))
        d = st['det']
        print(f'{ang:+6.1f} deg -> {nome}   conf '
              f'{d.confidence if d else 0:.3f}')
m.angle_deg = 0.0
for _ in range(10):
    m.header.stamp = n.get_clock().now().to_msg()
    req.publish(m); rclpy.spin_once(n, timeout_sec=0.05)
liga_autonomia(n, False)
n.destroy_node(); rclpy.shutdown()
devolve()
