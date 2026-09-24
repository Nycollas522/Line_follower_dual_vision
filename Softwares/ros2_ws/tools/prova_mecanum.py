#!/usr/bin/env python3
"""Prova se as rodas estao no X, medindo em vez de olhando.

COMO FUNCIONA: cada movimento puro tem uma assinatura conhecida nas
quatro rodas. Se uma roda estiver espelhada, a assinatura dela quebra --
e o teste diz QUAL roda e em QUE movimento.

    frente (vx)  ->  todas iguais e positivas
    strafe (vy)  ->  FL -  FR +  RL +  RR -
    giro   (wz)  ->  FL -  FR +  RL -  RR +

A do strafe e a que denuncia rolete espelhado: e o unico movimento que
depende da ORIENTACAO do rolete, e nao so do sentido do motor.

SEGURANCA: o robo ANDA. Rode com as RODAS SUSPENSAS -- assim o teste
mede tracao pura, sem o chao mascarar com atrito. E com as rodas no ar
nao importa se a cinematica estiver errada.

Uso:
    python3 logs/prova_mecanum.py           # 0.10 m/s
    python3 logs/prova_mecanum.py 0.08
"""
import statistics as st
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

C = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
WZ = 0.8
NOMES = ('FL', 'FR', 'RL', 'RR')

# (rotulo, vx, vy, wz, sinal esperado por roda)
ETAPAS = [
    ('frente', V, 0.0, 0.0, (+1, +1, +1, +1)),
    ('strafe direita', 0.0, -V, 0.0, (+1, -1, -1, +1)),
    ('strafe esquerda', 0.0, +V, 0.0, (-1, +1, +1, -1)),
    ('giro esquerda', 0.0, 0.0, WZ, (-1, +1, -1, +1)),
]


class Prova(Node):
    def __init__(self):
        super().__init__('prova_mecanum')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', C)
        self.en = self.create_publisher(Bool, '/controle/enable', C)
        self.amostras = []
        self.coletando = False
        self.create_subscription(JointState, '/wheel_states', self._ws, C)

    def _ws(self, msg):
        if self.coletando and len(msg.velocity) >= 4:
            self.amostras.append(list(msg.velocity[:4]))

    def espera_descoberta(self):
        fim = time.time() + 6.0
        while time.time() < fim and self.cmd.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.05)

    def dirige(self, vx, vy, wz, seg, coletar=False):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.coletando = coletar
        if coletar:
            self.amostras.clear()
        fim = time.time() + seg
        while time.time() < fim:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.coletando = False

    def para(self):
        for _ in range(25):
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def avalia(rotulo, esperado, amostras):
    if len(amostras) < 5:
        print(f'  {rotulo}: poucas amostras ({len(amostras)})')
        return [f'{rotulo}: sem telemetria']
    colunas = list(zip(*amostras))
    medias = [st.mean(c) for c in colunas]
    ref = st.median([abs(m) for m in medias])
    problemas = []
    print(f'  {rotulo:16} medido      esperado   situacao')
    for i, roda in enumerate(NOMES):
        m = medias[i]
        if ref > 1e-3 and abs(m) < 0.3 * ref:
            estado = 'FRACA'
            problemas.append(f'{rotulo}: {roda} quase parada')
        elif m * esperado[i] < 0 and abs(m) > 0.3 * ref:
            estado = '<== SENTIDO ERRADO'
            problemas.append(f'{rotulo}: {roda} girou ao contrario')
        else:
            estado = 'ok'
        print(f'    {roda:12} {m:+8.4f}   {esperado[i]:+8d}   {estado}')
    return problemas


def main():
    rclpy.init()
    n = Prova()
    achados = []
    try:
        print('>>> O ROBO VAI ANDAR. Use com as RODAS SUSPENSAS. <<<')
        n.espera_descoberta()
        b = Bool()
        b.data = False          # teleop: autonomia DESLIGADA de proposito
        for _ in range(15):
            n.en.publish(b)
            rclpy.spin_once(n, timeout_sec=0.05)
        for rotulo, vx, vy, wz, esperado in ETAPAS:
            n.dirige(vx, vy, wz, 0.7)                  # vence a inercia
            n.dirige(vx, vy, wz, 1.5, coletar=True)    # so entao mede
            n.para()
            achados += avalia(rotulo, esperado, list(n.amostras))
            print()
            time.sleep(0.4)
    finally:
        n.para()
        n.destroy_node()
        rclpy.shutdown()

    print('=' * 58)
    if achados:
        print('PROBLEMAS:')
        for p in achados:
            print(f'  - {p}')
        print('\nSe o erro aparece SO no strafe e nao na frente nem no giro,')
        print('e rolete espelhado: o motor gira certo, a roda e que empurra')
        print('para o lado errado. Confira o X olhando o robo de cima.')
    else:
        print('As quatro rodas bateram com a cinematica do X nas 4 etapas.')


if __name__ == '__main__':
    main()
