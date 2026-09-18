"""Mede o CAMPO DE VISAO sem depender do detector.

A sonda e a fracao da LARGURA DO QUADRO ocupada pela fita branca. Se a
resolucao apenas escala, a fracao nao muda. Se o sensor CORTA, a fita
passa a ocupar uma fracao maior.

Independe de calibracao, de confianca e de quantas bandas o detector
achou -- que foi o que estragou a primeira tentativa.
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

S = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=1)
rclpy.init()
n = Node('fov')
st = {'im': None}
n.create_subscription(Image, '/cam_bottom/image_raw',
                      lambda m: st.__setitem__('im', m), S)
fim = time.time() + 8.0
while time.time() < fim and st['im'] is None:
    rclpy.spin_once(n, timeout_sec=0.05)
if st['im'] is None:
    print('sem imagem')
    raise SystemExit(1)
bgr = CvBridge().imgmsg_to_cv2(st['im'], 'bgr8')
n.destroy_node()
rclpy.shutdown()

h, w = bgr.shape[:2]
g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
# Faixa central em altura, longe do chassi e da borda.
faixa = g[int(0.35 * h):int(0.55 * h)]
perfil = faixa.mean(axis=0)
lim, _ = cv2.threshold(
    faixa.astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
)
acima = perfil >= lim
# maior segmento contiguo claro = a fita
melhor = ini = 0
atual = None
for x in range(w):
    if acima[x] and atual is None:
        atual = x
    elif not acima[x] and atual is not None:
        if x - atual > melhor:
            melhor, ini = x - atual, atual
        atual = None
if atual is not None and w - atual > melhor:
    melhor, ini = w - atual, atual

rot = sys.argv[1] if len(sys.argv) > 1 else '?'
print(f'{rot}: quadro {w}x{h}')
print(f'  fita ocupa {melhor} px de {w} = {100.0*melhor/w:.2f}% da largura')
print(f'  centro da fita em {100.0*(ini+melhor/2)/w:.2f}% da largura')
cv2.imwrite(f'/home/bolt/fov_{rot}.png', bgr)
