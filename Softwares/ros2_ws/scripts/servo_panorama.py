#!/usr/bin/env python3
"""Varredura de ambientacao da camera superior.

Gira o pan por varios angulos e salva uma imagem em cada um, montando
um panorama da pista.

Para que serve: a camera superior tem meio-FOV projetado de apenas
~16.4 graus, enquanto uma curva de raio 33cm exige 29.6 graus de
bearing. Ou seja, com a cabeca parada a linha SAI do quadro em curva.
Esta varredura mostra, na pratica, o que a cabeca alcanca em cada
angulo -- util para decidir max_angle_deg e scan_amplitude_deg.

SEGURANCA: nao move os motores. Publica so em /head/request. Mas o
head_servo_node exige autonomia LIGADA para aceitar qualquer modo
diferente de CENTER, entao este script liga a autonomia e a desliga no
final. Rode com o robo em local seguro, ou com os motores sem tensao.

AUTORIDADE: com a autonomia ligada o line_follower_node tambem publica
em /head/request, a ~50 Hz -- muito mais rapido que a varredura, entao
ele venceria e a cabeca seguiria a linha em vez do angulo pedido (foi
exatamente o que aconteceu na primeira execucao). Por isso o script
pede head_control_enabled=false ao seguidor antes de varrer, e devolve
no final. O watchdog do head_servo_node segue valendo o tempo todo.

Uso:
    python3 scripts/servo_panorama.py            # -30..+30 de 10 em 10
    python3 scripts/servo_panorama.py 40 8       # -40..+40 de 8 em 8
    python3 scripts/servo_panorama.py 20 40 2    # +20..+40 de 2 em 2

A forma com tres argumentos varre uma faixa assimetrica -- util para
achar batente mecanico, comparando a diferenca entre imagens vizinhas:
onde a cabeca para de andar, a diferenca desaba.
"""
import os
import sys
import time

import cv2
from cv_bridge import CvBridge
from line_msgs.msg import HeadRequest
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=1)
CTRL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST, depth=10)

if len(sys.argv) > 3:
    INICIO, FIM, PASSO = (float(a) for a in sys.argv[1:4])
else:
    amp = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    PASSO = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    INICIO, FIM = -amp, amp
OUT = os.path.expanduser(os.environ.get('PAN_OUT', '~/servo_panorama'))


class Panorama(Node):
    def __init__(self):
        super().__init__('servo_panorama')
        self.bridge = CvBridge()
        self.head_pub = self.create_publisher(HeadRequest, '/head/request', CTRL)
        self.enable_pub = self.create_publisher(Bool, '/controle/enable', CTRL)
        self.frame = None
        self.servo = None
        self.create_subscription(
            Image, '/cam_front/image_raw',
            lambda m: setattr(self, 'frame', m), SENSOR)
        self.create_subscription(
            Float32, '/servo/state',
            lambda m: setattr(self, 'servo', m.data), CTRL)
        self.params = self.create_client(
            SetParameters, '/line_follower_node/set_parameters')

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_enable(self, value):
        deadline = time.time() + 5.0
        while (time.time() < deadline
               and self.enable_pub.get_subscription_count() < 3):
            rclpy.spin_once(self, timeout_sec=0.05)
        msg = Bool()
        msg.data = value
        for _ in range(10):
            self.enable_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin(0.8)

    def head_control(self, follower_owns):
        """Liga/desliga a publicacao de /head/request pelo seguidor.

        Sem isso os dois publicam no mesmo topico e o seguidor, muito
        mais rapido, vence. Devolve True se o seguidor confirmou.
        """
        if not self.params.wait_for_service(timeout_sec=3.0):
            print('  ! line_follower_node nao respondeu ao set_parameters; '
                  'a varredura pode ser sobreposta pelo seguidor')
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='head_control_enabled',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=bool(follower_owns)))]
        fut = self.params.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        ok = bool(res and res.results and res.results[0].successful)
        if not ok:
            print('  ! nao consegui mudar head_control_enabled')
        return ok

    def aponta(self, angulo, assentar=1.6):
        """Manda a cabeca para um angulo e espera assentar."""
        req = HeadRequest()
        req.mode = HeadRequest.MODE_HOLD
        req.angle_deg = float(angulo)
        end = time.time() + assentar
        while time.time() < end:
            req.header.stamp = self.get_clock().now().to_msg()
            self.head_pub.publish(req)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    node = Panorama()
    try:
        node.set_enable(True)
        node.head_control(False)   # a varredura assume a cabeca
        node.spin(0.6)             # deixa o ultimo pedido do seguidor expirar
        angulos = []
        a = INICIO
        while a <= FIM + 1e-6:
            angulos.append(round(a, 1))
            a += PASSO

        print(f'>>> Varredura de {INICIO:+.0f} a {FIM:+.0f} '
              f'graus, passo {PASSO:g} <<<')
        for ang in angulos:
            node.aponta(ang)
            if node.frame is None:
                print(f'  {ang:+6.1f} deg -> sem frame da camera')
                continue
            img = node.bridge.imgmsg_to_cv2(
                node.frame, desired_encoding='bgr8')
            medido = node.servo if node.servo is not None else float('nan')
            nome = f'pan_{ang:+06.1f}.png'.replace('+', 'p').replace('-', 'm')
            cv2.imwrite(os.path.join(OUT, nome), img)
            print(f'  pedido {ang:+6.1f} deg | medido {medido:+6.1f} deg '
                  f'-> {nome}')
    finally:
        print('>>> Voltando ao centro e devolvendo a cabeca <<<')
        node.head_control(True)
        req = HeadRequest()
        req.mode = HeadRequest.MODE_CENTER
        for _ in range(30):
            node.head_pub.publish(req)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.set_enable(False)
        print(f'imagens em {OUT}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
