"""Tudo num disparo: liga autonomia, deixa a cabeca assentar, e captura
detecções das DUAS cameras + servo + as duas imagens no MESMO instante.

Medir em disparos separados ja produziu duas leituras contraditorias
nesta bancada -- o robo foi movido entre elas. O heading da inferior
denuncia a troca de cena, entao ele vai junto de tudo.
"""
import time
import cv2, rclpy
from cv_bridge import CvBridge
from line_msgs.msg import LineDetection
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_seguro import trava_motores, devolve

# TRAVA OS MOTORES ANTES DE QUALQUER COISA. Mexer o servo exige ligar a
# autonomia, e em 21/09/2026 isso fez o robo ANDAR porque o modo estava
# em SEGUIDOR. trava_motores() levanta excecao se nao conseguir provar
# que o corpo esta desarmado.
# percepcao=True porque o teste le as deteccoes; devolve(percepcao=False)
# porque aqui quem mira a cabeca E o seguidor -- o corpo segue travado.
trava_motores(percepcao=True)
devolve(percepcao=False)

C = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
S = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=1)
rclpy.init(); n = Node('quebra_teste'); br = CvBridge()
st = {'servo': 0.0, 'inf': None, 'sup': None, 'idbg': None, 'sdbg': None}
n.create_subscription(Float32, '/servo/state',
                      lambda m: st.__setitem__('servo', m.data), C)
n.create_subscription(LineDetection, '/line/detection',
                      lambda m: st.__setitem__('inf', m), C)
n.create_subscription(LineDetection, '/line_front/detection',
                      lambda m: st.__setitem__('sup', m), C)
n.create_subscription(Image, '/line/debug_image',
                      lambda m: st.__setitem__('idbg', m), S)
n.create_subscription(Image, '/line_front/debug_image',
                      lambda m: st.__setitem__('sdbg', m), S)
en = n.create_publisher(Bool, '/controle/enable', C)
fim = time.time() + 5
while time.time() < fim and en.get_subscription_count() < 3:
    rclpy.spin_once(n, timeout_sec=0.05)


def linha(rot):
    i, s = st['inf'], st['sup']
    print(f'  {rot:16} servo {st["servo"]:+6.1f}  '
          f'inf: head {(i.heading_error*57.3 if i else 0):+6.1f} '
          f'conf {(i.confidence if i else 0):.2f} '
          f'b{(i.bands_valid if i else 0)}  |  '
          f'sup: {"OK " if (s and s.valid) else "SEM"} '
          f'conf {(s.confidence if s else 0):.3f} '
          f'b{(s.bands_valid if s else 0)}')


b = Bool()
b.data = False
for _ in range(15):
    en.publish(b); rclpy.spin_once(n, timeout_sec=0.05)
t0 = time.time()
while time.time() - t0 < 2.0:
    rclpy.spin_once(n, timeout_sec=0.02)
print('ANTES (autonomia off, cabeca no centro):')
linha('centro')
cv2.imwrite('/home/bolt/quebra/T_sup_centro.png',
            br.imgmsg_to_cv2(st['sdbg'], 'bgr8'))

b.data = True
for _ in range(15):
    en.publish(b); rclpy.spin_once(n, timeout_sec=0.05)
t0 = time.time()
while time.time() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.02)
print('DEPOIS (autonomia on, cabeca mirando pela inferior):')
linha('mirando')
cv2.imwrite('/home/bolt/quebra/T_sup_virada.png',
            br.imgmsg_to_cv2(st['sdbg'], 'bgr8'))
cv2.imwrite('/home/bolt/quebra/T_inf.png',
            br.imgmsg_to_cv2(st['idbg'], 'bgr8'))
b.data = False
for _ in range(15):
    en.publish(b); rclpy.spin_once(n, timeout_sec=0.05)
n.destroy_node(); rclpy.shutdown()
devolve()   # desliga a percepcao que a trava ligou
