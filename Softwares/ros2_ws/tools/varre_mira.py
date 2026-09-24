"""Varre o servo e mede a deteccao da superior em cada angulo.

Cala o seguidor antes (head_control_enabled=false): ele publica
/head/request a 50 Hz e disputaria o servo -- armadilha ja documentada.
"""
import os
import subprocess
import sys
import time

import rclpy
from line_msgs.msg import HeadRequest, LineDetection
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

C = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
INI = float(sys.argv[1]) if len(sys.argv) > 1 else -40.0
FIM = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
PASSO = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_seguro import trava_motores, liga_autonomia, devolve

# TRAVA OS MOTORES ANTES DE QUALQUER COISA. Mexer o servo exige ligar a
# autonomia, e em 21/09/2026 isso fez o robo ANDAR porque o modo estava
# em SEGUIDOR. trava_motores() levanta excecao se nao conseguir provar
# que o corpo esta desarmado.
trava_motores()

rclpy.init()
n = Node('varre')
st = {'det': None, 'servo': 0.0}
n.create_subscription(LineDetection, '/line_front/detection',
                      lambda m: st.__setitem__('det', m), C)
n.create_subscription(Float32, '/servo/state',
                      lambda m: st.__setitem__('servo', m.data), C)
req = n.create_publisher(HeadRequest, '/head/request', C)

liga_autonomia(n, True)   # so o servo responde; o corpo esta travado

print(f'{"pedido":>7} {"servo":>7} {"valido":>7} {"conf":>6} {"bandas":>7} '
      f'{"lat_mm":>8} {"head":>7} {"contr":>6}')
m = HeadRequest()
m.mode = HeadRequest.MODE_HOLD
ang = INI
melhor = None
while ang <= FIM + 1e-6:
    m.angle_deg = float(ang)
    t0 = time.time()
    while time.time() - t0 < 2.5:
        m.header.stamp = n.get_clock().now().to_msg()
        req.publish(m)
        rclpy.spin_once(n, timeout_sec=0.02)
    d = st['det']
    if d is None:
        print(f'{ang:7.1f} {st["servo"]:7.1f}   sem deteccao')
    else:
        print(f'{ang:7.1f} {st["servo"]:7.1f} {str(d.valid):>7} '
              f'{d.confidence:6.3f} {d.bands_valid:7d} '
              f'{1000*d.lateral_error:8.1f} '
              f'{57.2958*d.heading_error:7.1f} {d.contrast:6.1f}')
        if d.valid and (melhor is None or d.confidence > melhor[1]):
            melhor = (ang, d.confidence, 1000*d.lateral_error, d.bands_valid)
    ang += PASSO

m.angle_deg = 0.0
for _ in range(10):
    m.header.stamp = n.get_clock().now().to_msg()
    req.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)
liga_autonomia(n, False)
n.destroy_node()
rclpy.shutdown()
devolve()
if melhor:
    print(f'\nmelhor: {melhor[0]:+.1f} deg  conf {melhor[1]:.3f}  '
          f'lat {melhor[2]:+.1f} mm  {melhor[3]} bandas')
