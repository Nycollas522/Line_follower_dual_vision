"""Prova fisica: o servo realmente gira a camera superior?

/servo/state e ECO do comando (verificado no firmware, SERVO_STATE
espelha servoAngle). Entao a unica evidencia de movimento real e a
IMAGEM mudar. Compara quadros em angulos opostos.
"""
import time
import cv2, numpy as np, rclpy
from cv_bridge import CvBridge
from line_msgs.msg import HeadRequest
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_seguro import trava_motores, devolve

# TRAVA OS MOTORES ANTES DE QUALQUER COISA. Mexer o servo exige ligar a
# autonomia, e em 21/09/2026 isso fez o robo ANDAR porque o modo estava
# em SEGUIDOR. trava_motores() levanta excecao se nao conseguir provar
# que o corpo esta desarmado.
# Tambem cala o seguidor no /head/request, senao ele disputa o servo.
trava_motores()

C = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
S = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST, depth=1)
rclpy.init(); n=Node('servo_prova'); br=CvBridge(); st={'img':None}
n.create_subscription(Image,'/cam_front/image_raw',
                      lambda m: st.__setitem__('img',m), S)
req=n.create_publisher(HeadRequest,'/head/request',C)
en=n.create_publisher(Bool,'/controle/enable',C)
fim=time.time()+5
while time.time()<fim and en.get_subscription_count()<3:
    rclpy.spin_once(n,timeout_sec=0.05)
# AUTORIDADE DA CABECA. line_follower_node publica /head/request a
# 50 Hz; sem calar ele, os dois pedidos brigam e o servo fica sendo
# puxado para o alvo dele. Foi exatamente isso que contaminou a
# primeira versao deste teste e me fez concluir, errado, que o servo
# nao girava a camera.
# trava_motores() ja calou o seguidor (head_control_enabled=false).
b=Bool(); b.data=True
for _ in range(15): en.publish(b); rclpy.spin_once(n,timeout_sec=0.05)

quadros={}
for ang in (-35.0, 0.0, 35.0):
    m=HeadRequest(); m.mode=HeadRequest.MODE_HOLD; m.angle_deg=float(ang)
    t0=time.time()
    while time.time()-t0 < 4.0:
        m.header.stamp=n.get_clock().now().to_msg()
        req.publish(m); rclpy.spin_once(n,timeout_sec=0.02)
    quadros[ang]=br.imgmsg_to_cv2(st['img'],'bgr8').copy()
    cv2.imwrite(f'/home/bolt/quebra/servo_{int(ang):+03d}.png', quadros[ang])
    print(f'  {ang:+6.1f} deg capturado')
b.data=False
for _ in range(15): en.publish(b); rclpy.spin_once(n,timeout_sec=0.05)
devolve()
n.destroy_node(); rclpy.shutdown()

print('\nDIFERENCA entre os quadros (0 = imagem identica):')
def dif(a,b):
    x=cv2.cvtColor(quadros[a],cv2.COLOR_BGR2GRAY).astype(np.float32)
    y=cv2.cvtColor(quadros[b],cv2.COLOR_BGR2GRAY).astype(np.float32)
    return float(np.abs(x-y).mean())
print(f'  -35 vs   0 : {dif(-35.0,0.0):6.2f} niveis de cinza')
print(f'    0 vs +35 : {dif(0.0,35.0):6.2f}')
print(f'  -35 vs +35 : {dif(-35.0,35.0):6.2f}')
print('\n  Um pan de 70 graus deveria dar dezenas de niveis.')
print('  Abaixo de ~3 e ruido de sensor: a camera NAO girou.')
