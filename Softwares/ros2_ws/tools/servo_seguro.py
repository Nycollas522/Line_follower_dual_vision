"""Autoridade do servo COM TRAVA DE MOVIMENTO.

Para mexer o servo e preciso ligar a autonomia -- o head_servo_node tem
watchdog que so aceita comando com ela ligada. Mas autonomia sozinha nao
basta para o robo andar: quem publica /cmd_vel_auto e o seguidor, e ele
so faz isso com body_control_enabled=true.

INCIDENTE DE 21/09/2026: o operador pediu para nao mexer os motores. Eu
coloquei o modo em SEGUIDOR (para ligar a percepcao), e o mode_manager
liga body_control junto. Depois um script de varredura ligou a autonomia
para mexer o servo -- e o robo ANDOU, com o operador tendo que tira-lo
do chao.

Por isso este modulo NAO confia em disciplina: ele DESLIGA body_control
e o modo antes de ligar a autonomia, confere, e recusa continuar se a
confirmacao nao vier.

Use assim em qualquer script que mexa no servo:

    from servo_seguro import trava_motores, devolve
    trava_motores()      # levanta RuntimeError se nao conseguir travar
    ...                  # mexa o servo a vontade
    devolve()
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

C = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)


def _param(node, nome, valor):
    r = subprocess.run(
        ['ros2', 'param', 'set', node, nome, valor],
        capture_output=True, text=True, timeout=20,
    )
    return 'successful' in r.stdout


def _le(node, nome):
    r = subprocess.run(
        ['ros2', 'param', 'get', node, nome],
        capture_output=True, text=True, timeout=20,
    )
    return r.stdout.strip().split()[-1] if r.stdout.strip() else '?'


def trava_motores():
    """Garante que o corpo nao anda, e PROVA isso antes de devolver."""
    # 1. Modo PARADO: o mode_manager desliga body_control e desarma.
    subprocess.run(['python3', '/home/bolt/ros2_ws/logs/modo.py', 'PARADO'],
                   capture_output=True, timeout=60)
    # 2. Cinto e suspensorio: desliga body_control direto tambem, caso o
    #    mode_manager nao esteja no ar.
    _param('/line_follower_node', 'body_control_enabled', 'false')
    # 3. Cala o seguidor no /head/request, senao ele disputa o servo.
    _param('/line_follower_node', 'head_control_enabled', 'false')

    # 4. CONFERE. Sem confirmacao, nao segue.
    body = _le('/line_follower_node', 'body_control_enabled')
    if body != 'False':
        raise RuntimeError(
            f'body_control_enabled={body}: nao consegui travar os motores. '
            'ABORTANDO em vez de arriscar mover o robo.'
        )
    print('motores travados (body_control_enabled=False, modo PARADO)')


def liga_autonomia(node, ligar=True):
    """Liga/desliga a autonomia -- so o servo responde, o corpo nao."""
    pub = node.create_publisher(Bool, '/controle/enable', C)
    fim = time.time() + 6.0
    while time.time() < fim and pub.get_subscription_count() < 2:
        rclpy.spin_once(node, timeout_sec=0.05)
    msg = Bool()
    msg.data = bool(ligar)
    for _ in range(15):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)


def devolve():
    """Devolve a autoridade da cabeca ao seguidor."""
    _param('/line_follower_node', 'head_control_enabled', 'true')
    print('autoridade da cabeca devolvida (motores seguem travados)')
