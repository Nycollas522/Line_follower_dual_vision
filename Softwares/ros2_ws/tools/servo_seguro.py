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

Se o script precisa de DETECCAO ou imagem de debug, use
trava_motores(percepcao=True). O modo PARADO desliga a percepcao, e por
isso ela e religada direto nos dois nos, com o modo continuando em PARADO.
Colocar o modo em SEGUIDOR e desligar body_control depois seria uma
corrida contra o mode_manager, que liga body_control nessa troca.
devolve() desliga a percepcao de novo.
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


PERCEPCAO = ('/line_perception_bottom', '/line_perception_front')
_percepcao_ligada = False


def trava_motores(percepcao=False):
    """Garante que o corpo nao anda, e PROVA isso antes de devolver."""
    global _percepcao_ligada
    # 1. Modo PARADO: o mode_manager desliga body_control e desarma.
    subprocess.run(['python3', '/home/bolt/ros2_ws/logs/modo.py', 'PARADO'],
                   capture_output=True, timeout=60)
    # 2. Cinto e suspensorio: desliga body_control direto tambem, caso o
    #    mode_manager nao esteja no ar.
    _param('/line_follower_node', 'body_control_enabled', 'false')
    # 3. Cala o seguidor no /head/request, senao ele disputa o servo.
    _param('/line_follower_node', 'head_control_enabled', 'false')

    # 4. CONFERE, duas vezes com intervalo: um pedido atrasado do
    #    mode_manager ainda poderia religar body_control logo depois.
    for tentativa in range(2):
        if tentativa:
            time.sleep(2.0)
        body = _le('/line_follower_node', 'body_control_enabled')
        if body != 'False':
            raise RuntimeError(
                f'body_control_enabled={body}: nao consegui travar os '
                'motores. ABORTANDO em vez de arriscar mover o robo.'
            )
    print('motores travados (body_control_enabled=False, modo PARADO)')

    # 5. So depois da trava provada: percepcao, se pedida.
    if percepcao:
        for no in PERCEPCAO:
            if not _param(no, 'enabled', 'true'):
                raise RuntimeError(f'nao consegui ligar a percepcao em {no}')
        _percepcao_ligada = True
        print('percepcao ligada nas duas cameras (modo segue PARADO)')


def liga_autonomia(node, ligar=True):
    """Liga/desliga a autonomia -- so o servo responde, o corpo nao.

    CONFIRMA pelo /autonomy/state do motor_serial_node, que e a
    autoridade. Em 24/09/2026 esta funcao esperava so 2 dos 3
    assinantes de /controle/enable: o motor_serial ainda nao tinha sido
    descoberto, perdeu o pedido, e o /autonomy/state=False dele (5 Hz)
    desfez o "ligado" nos outros dois. A cabeca nao se mexeu e uma
    varredura inteira de calibracao saiu com a imagem parada.
    Ligar sem confirmacao levanta RuntimeError; desligar so avisa.
    """
    pub = node.create_publisher(Bool, '/controle/enable', C)
    estado = {'v': None}
    sub = node.create_subscription(
        Bool, '/autonomy/state',
        lambda m: estado.__setitem__('v', bool(m.data)), C)
    try:
        fim = time.time() + 6.0
        while (ligar and time.time() < fim
               and pub.get_subscription_count() < 3):
            rclpy.spin_once(node, timeout_sec=0.05)
        msg = Bool()
        msg.data = bool(ligar)
        fim = time.time() + 5.0
        while time.time() < fim:
            pub.publish(msg)
            estado['v'] = None
            t0 = time.time()
            # espera a PROXIMA publicacao do estado (5 Hz), nao uma velha
            while time.time() - t0 < 0.5 and estado['v'] is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            if estado['v'] == bool(ligar):
                return
        if ligar:
            raise RuntimeError(
                'motor_serial_node nao confirmou autonomia ligada em 5 s '
                '(/autonomy/state); a cabeca nao se mexeria.')
        print('AVISO: autonomia desligada sem confirmacao do motor_serial')
    finally:
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)


def devolve(percepcao=True):
    """Devolve a autoridade da cabeca ao seguidor.

    Com percepcao=True (padrao), tambem desliga a percepcao que
    trava_motores(percepcao=True) tiver ligado, deixando o PARADO coerente.
    """
    global _percepcao_ligada
    _param('/line_follower_node', 'head_control_enabled', 'true')
    print('autoridade da cabeca devolvida (motores seguem travados)')
    if percepcao and _percepcao_ligada:
        for no in PERCEPCAO:
            _param(no, 'enabled', 'false')
        _percepcao_ligada = False
        print('percepcao desligada de novo')
