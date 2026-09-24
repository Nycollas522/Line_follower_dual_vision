"""Varre o servo devagar por 60s para inspecao VISUAL do eixo.

Use junto com servo_prova.py: esta mostra se o EIXO gira, aquela mostra
se a CAMERA acompanha. /servo/state nao serve para nenhuma das duas --
e eco do comando (firmware: SERVO_STATE espelha servoAngle).
"""
import sys, time
import rclpy
from line_msgs.msg import HeadRequest
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
import os
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
SEG = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
rclpy.init(); n = Node('varredura')
req = n.create_publisher(HeadRequest, '/head/request', C)
en = n.create_publisher(Bool, '/controle/enable', C)
fim = time.time() + 5
while time.time() < fim and en.get_subscription_count() < 3:
    rclpy.spin_once(n, timeout_sec=0.05)
# AUTORIDADE DA CABECA. line_follower_node publica /head/request a
# 50 Hz; sem calar ele, os dois pedidos brigam e o servo fica sendo
# puxado para o alvo dele. Foi exatamente isso que contaminou a
# primeira versao deste teste e me fez concluir, errado, que o servo
# nao girava a camera.
# trava_motores() ja calou o seguidor (head_control_enabled=false).
b = Bool(); b.data = True
for _ in range(15):
    en.publish(b); rclpy.spin_once(n, timeout_sec=0.05)
print(f'varrendo -35 <-> +35 por {SEG:.0f}s -- olhe o EIXO do servo')
m = HeadRequest(); m.mode = HeadRequest.MODE_HOLD
t0 = time.time(); alvo = 35.0; troca = time.time() + 3.0
while time.time() - t0 < SEG:
    if time.time() > troca:
        alvo = -alvo; troca = time.time() + 3.0
        print(f'  -> {alvo:+.0f} deg')
    m.header.stamp = n.get_clock().now().to_msg()
    m.angle_deg = float(alvo)
    req.publish(m); rclpy.spin_once(n, timeout_sec=0.02)
b.data = False
for _ in range(15):
    en.publish(b); rclpy.spin_once(n, timeout_sec=0.05)
devolve()
n.destroy_node(); rclpy.shutdown()
print('parado')
