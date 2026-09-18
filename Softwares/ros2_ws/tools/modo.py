"""Troca o modo esperando a descoberta DDS.

Publicar antes do assinante conectar faz a mensagem cair no vazio -- a
armadilha ja documentada no README, e que eu repeti em 18/09/2026.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
ALVO = sys.argv[1].upper() if len(sys.argv) > 1 else 'PARADO'

rclpy.init()
n = Node('modo')
pub = n.create_publisher(String, '/robot/mode_request', Q)
est = {'m': '?'}
n.create_subscription(String, '/robot/mode',
                      lambda m: est.__setitem__('m', m.data), Q)

fim = time.time() + 8.0
while time.time() < fim and pub.get_subscription_count() < 1:
    rclpy.spin_once(n, timeout_sec=0.05)
if pub.get_subscription_count() < 1:
    print('mode_manager_node nao apareceu; abortando')
    raise SystemExit(1)

msg = String()
msg.data = ALVO
fim = time.time() + 10.0
while time.time() < fim and est['m'] != ALVO:
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.1)
print(f'modo pedido {ALVO} -> ativo {est["m"]}')
n.destroy_node()
rclpy.shutdown()
