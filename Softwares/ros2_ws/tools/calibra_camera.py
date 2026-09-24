#!/usr/bin/env python3
"""Calcula width_near_m / width_far_m medindo a fita na imagem.

O QUE ELE FAZ SOZINHO: mede a largura da fita, em pixels, ao longo da
profundidade do quadro. Como a largura REAL da fita e conhecida, cada
medida da a largura do campo naquela linha:

    campo(y) = largura_real * largura_do_quadro / largura_da_fita(y)

O QUE ELE NAO PODE ADIVINHAR: a que distancia do robo o quadro comeca e
termina. Isso e regua, e vai por argumento.

Uso:
    python3 logs/calibra_camera.py front  0.11 0.55
    python3 logs/calibra_camera.py bottom 0.11 0.16
                                   ^      ^    ^
                                   camera |    depth_far_m (borda de cima)
                                          depth_near_m (borda de baixo)

Roda com o robo PARADO sobre a fita, o mais alinhado possivel -- fita
inclinada mede 1/cos(theta) mais larga e contamina o resultado.
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
CAM = sys.argv[1] if len(sys.argv) > 1 else 'front'
DEPTH_NEAR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.11
DEPTH_FAR = float(sys.argv[3]) if len(sys.argv) > 3 else 0.55
FITA_M = float(sys.argv[4]) if len(sys.argv) > 4 else 0.020
TOPICO = f'/cam_{CAM}/image_raw'

rclpy.init()
n = Node('calibra')
st = {'im': None}
n.create_subscription(Image, TOPICO, lambda m: st.__setitem__('im', m), S)
fim = time.time() + 8.0
while time.time() < fim and st['im'] is None:
    rclpy.spin_once(n, timeout_sec=0.05)
if st['im'] is None:
    print(f'sem imagem em {TOPICO}')
    raise SystemExit(1)
bgr = CvBridge().imgmsg_to_cv2(st['im'], 'bgr8')
n.destroy_node()
rclpy.shutdown()

cv2.imwrite(f'/home/bolt/calib_{CAM}.png', bgr)
g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
h, w = g.shape
lim, _ = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
claro = float((g >= lim).mean())
# A pista pode ser fita clara em piso escuro OU o contrario. Decide pelo
# que e MINORIA: a fita e sempre o objeto fino no quadro.
fita_clara = claro < 0.5
print(f'camera {CAM}  quadro {w}x{h}  limiar {lim:.0f}  '
      f'fita {"CLARA" if fita_clara else "ESCURA"} sobre fundo oposto')
print(f'profundidade informada: {DEPTH_NEAR:.4f} m (base) a '
      f'{DEPTH_FAR:.4f} m (topo)\n')

print(' linha    largura da fita   campo estimado')
medidas = []
for i in range(10):
    y = int((i + 0.5) * h / 10)
    faixa = g[max(0, y - 4):y + 5]
    perfil = faixa.mean(axis=0)
    marca = perfil >= lim if fita_clara else perfil < lim
    melhor = atual = ini = melhor_ini = 0
    for x in range(w):
        if marca[x]:
            if atual == 0:
                ini = x
            atual += 1
            if atual > melhor:
                melhor, melhor_ini = atual, ini
        else:
            atual = 0
    if melhor < 3:
        print(f'  y{y:4d}     (fita nao encontrada)')
        continue
    campo = FITA_M * w / melhor
    # y=0 e o TOPO da imagem = o LONGE
    frac = y / float(h - 1)
    prof = DEPTH_FAR + (DEPTH_NEAR - DEPTH_FAR) * frac
    medidas.append((prof, campo))
    print(f'  y{y:4d}     {melhor:4d} px          {campo:.4f} m '
          f'(a {prof:.3f} m)')

if len(medidas) < 3:
    print('\nPoucas medidas. O robo esta sobre a fita e alinhado?')
    raise SystemExit(1)

# Ajuste linear campo(profundidade): e o modelo que o detector usa.
prof = np.array([m[0] for m in medidas])
campo = np.array([m[1] for m in medidas])
a, b = np.polyfit(prof, campo, 1)
w_near = a * DEPTH_NEAR + b
w_far = a * DEPTH_FAR + b
resid = float(np.sqrt(np.mean((np.polyval([a, b], prof) - campo) ** 2)))
print(f'\najuste linear: campo = {a:+.4f}*prof {b:+.4f}   '
      f'residuo {1000*resid:.2f} mm')
print(f'\n--- para o YAML (line_perception_{CAM}) ---')
print(f'    depth_near_m: {DEPTH_NEAR:.4f}')
print(f'    depth_far_m: {DEPTH_FAR:.4f}')
print(f'    width_near_m: {w_near:.4f}')
print(f'    width_far_m: {w_far:.4f}')
var = 100 * abs(w_far - w_near) / max(w_near, 1e-6)
if var < 3.0:
    print(f'\n(largura varia so {var:.1f}% -- vista praticamente')
    print(' ortografica; use o mesmo valor nos dois.)')
else:
    print(f'\n(largura varia {var:.0f}% entre perto e longe: ha')
    print(' perspectiva de verdade, os dois valores importam.)')
print(f'\nimagem salva em /home/bolt/calib_{CAM}.png -- CONFIRA que a')
print('fita medida e a da pista, e nao uma emenda de piso.')
