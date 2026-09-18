#!/usr/bin/env python3
"""Confere se as QUATRO rodas respondem ao comando.

POR QUE ESTE SCRIPT EXISTE (10/09/2026): um mau contato em duas rodas
passou um dia inteiro sem ser detectado. As corridas instrumentadas
gravavam vx/wz de /cmd_vel_auto -- o que o controlador PEDIU -- e nunca
/odom ou /wheel_states, o que o robo FEZ. O sintoma na pista era o robo
"andar angulado" e deslizar, que num chassi mecanum e assinatura direta
de tracao assimetrica, mas foi atribuido a controle.

O teste manda comandos de forma conhecida e compara com o encoder de
cada roda. Uma roda com mau contato aparece como velocidade muito
abaixo das outras, ou intermitente (desvio alto no tempo).

SEGURANCA: o robo ANDA. Use com as rodas suspensas se puder -- assim o
teste isola a tracao do atrito com o chao. No chao ele avanca alguns
centimetros por etapa.

Uso:
    python3 scripts/teste_rodas.py            # frente, tras e giro
    python3 scripts/teste_rodas.py 0.10       # com outra velocidade
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

CTRL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST, depth=10)
V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.08
WZ = 0.6
# (rotulo, vx, wz, sinal esperado por roda [fl, fr, rl, rr])
ETAPAS = [
    ('frente', V, 0.0, (+1, +1, +1, +1)),
    ('re', -V, 0.0, (-1, -1, -1, -1)),
    ('giro esquerda', 0.0, WZ, (-1, +1, -1, +1)),
    ('giro direita', 0.0, -WZ, (+1, -1, +1, -1)),
]
NOMES = ('FL', 'FR', 'RL', 'RR')


class Teste(Node):
    """Publica twist e coleta a velocidade das quatro rodas."""

    def __init__(self):
        super().__init__('teste_rodas')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', CTRL)
        self.en = self.create_publisher(Bool, '/controle/enable', CTRL)
        self.amostras = []
        self.coletando = False
        self.create_subscription(JointState, '/wheel_states', self._ws, CTRL)

    def _ws(self, msg):
        if self.coletando and len(msg.velocity) >= 4:
            self.amostras.append(list(msg.velocity[:4]))

    def enable(self, valor):
        # Espera a descoberta: publicar antes dela faz a mensagem cair no
        # vazio e a autonomia nunca mudar de estado.
        fim = time.time() + 5.0
        while (time.time() < fim
               and self.en.get_subscription_count() < 2):
            rclpy.spin_once(self, timeout_sec=0.05)
        msg = Bool()
        msg.data = valor
        for _ in range(15):
            self.en.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def dirige(self, vx, wz, segundos, coletar=False):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        self.coletando = coletar
        if coletar:
            self.amostras.clear()
        fim = time.time() + segundos
        while time.time() < fim:
            self.cmd.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.coletando = False

    def para(self):
        for _ in range(20):
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def avalia(nome, esperado, amostras):
    """Imprime a leitura de cada roda e devolve os problemas achados."""
    if len(amostras) < 5:
        print(f'  {nome}: poucas amostras ({len(amostras)})')
        return [f'{nome}: sem telemetria']

    problemas = []
    colunas = list(zip(*amostras))
    medias = [st.mean(c) for c in colunas]
    desvios = [st.pstdev(c) for c in colunas]
    referencia = st.median([abs(m) for m in medias])

    print(f'  {nome:14} media(m/s)   desvio   |  esperado  situacao')
    for i, roda in enumerate(NOMES):
        m, d = medias[i], desvios[i]
        sinal_ok = (m * esperado[i] > 0) or abs(m) < 1e-4
        if referencia > 1e-3 and abs(m) < 0.35 * referencia:
            estado = 'FRACA/MORTA'
            problemas.append(f'{nome}: {roda} muito abaixo das outras')
        elif not sinal_ok:
            estado = 'SENTIDO INVERTIDO'
            problemas.append(f'{nome}: {roda} girou ao contrario')
        elif referencia > 1e-3 and d > 0.5 * referencia:
            estado = 'INTERMITENTE'
            problemas.append(f'{nome}: {roda} oscila muito (mau contato?)')
        else:
            estado = 'ok'
        print(f'    {roda:12} {m:+9.4f} {d:8.4f}   |  {esperado[i]:+8d}  '
              f'{estado}')
    if referencia > 1e-3:
        espalha = max(abs(m) for m in medias) - min(abs(m) for m in medias)
        print(f'    espalhamento entre rodas: {espalha:.4f} m/s '
              f'({100 * espalha / referencia:.0f}% da mediana)')
    return problemas


def main():
    rclpy.init()
    node = Teste()
    achados = []
    try:
        print('>>> O ROBO VAI ANDAR. Mao no botao do ESP32 <<<')
        node.enable(True)
        for nome, vx, wz, esperado in ETAPAS:
            node.dirige(vx, wz, 0.6)                 # vence a inercia
            node.dirige(vx, wz, 1.5, coletar=True)   # so entao mede
            node.para()
            achados += avalia(nome, esperado, list(node.amostras))
            print()
            time.sleep(0.4)
    finally:
        node.para()
        node.enable(False)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

    print('=' * 60)
    if achados:
        print('PROBLEMAS ENCONTRADOS:')
        for p in achados:
            print(f'  - {p}')
        print('\nMau contato costuma aparecer como INTERMITENTE (desvio alto)')
        print('ou FRACA/MORTA. Mexa no chicote da roda e rode de novo.')
    else:
        print('As quatro rodas responderam de forma coerente.')


if __name__ == '__main__':
    main()
