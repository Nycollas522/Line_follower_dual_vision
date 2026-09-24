#!/usr/bin/env python3
"""Compara giroscopio e encoders girando POR COMANDO, no chao.

O teste a mao foi grosseiro: giro irregular e IMU a 5 Hz. Aqui a
velocidade e constante e conhecida, entao as duas fontes sao comparadas
em regime -- que e o unico jeito de derivar yawEncoderWeight em vez de
chutar.

Assinaturas:
  ganho ~1      as duas medem a mesma rotacao; o encoder e confiavel
  ganho < 1     o encoder le MAIS que a realidade -> patinagem
  ganho > 1     o encoder le MENOS -> erro de escala (ticksRev ou K)

SEGURANCA: gira no PROPRIO EIXO (vx=vy=0), entao nao translada e nao
precisa de espaco alem do proprio robo. Ainda assim, ANDA. Para em cada
troca de etapa e no fim, inclusive se der erro.
"""
import math
import statistics as st
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
K = 0.165
VELOCIDADES = [0.4, -0.4, 0.8, -0.8, 1.2, -1.2]


class Teste(Node):
    def __init__(self):
        super().__init__('imu_enc_motor')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', Q)
        self.janela = []
        self.pares = []
        self.coletando = False
        self.create_subscription(Imu, '/imu/data_raw', self._imu, Q)
        self.create_subscription(JointState, '/wheel_states', self._ws, Q)

    def _ws(self, m):
        if len(m.velocity) >= 4:
            fl, fr, rl, rr = m.velocity[:4]
            self.janela.append((-fl + fr - rl + rr) / (4.0 * K))

    def _imu(self, m):
        # Pareia cada amostra da IMU com a MEDIA dos encoders na mesma
        # janela -- sem isso, comparar 5 Hz com 48 Hz mede desalinhamento
        # e nao discordancia.
        if not self.janela:
            return
        enc = sum(self.janela) / len(self.janela)
        self.janela.clear()
        if self.coletando:
            self.pares.append((m.angular_velocity.z, enc))

    def gira(self, wz, seg, coletar=False):
        t = Twist()
        t.angular.z = float(wz)
        self.coletando = coletar
        if coletar:
            self.pares.clear()
        fim = time.time() + seg
        while time.time() < fim:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.coletando = False

    def para(self):
        for _ in range(30):
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    n = Teste()
    linhas = []
    try:
        print('>>> O ROBO VAI GIRAR NO PROPRIO EIXO. Mao no botao. <<<\n')
        fim = time.time() + 6.0
        while time.time() < fim and n.cmd.get_subscription_count() < 1:
            rclpy.spin_once(n, timeout_sec=0.05)
        print(f'{"wz pedido":>10} {"giro":>8} {"encoder":>9} {"ganho":>7} '
              f'{"n":>4}')
        for wz in VELOCIDADES:
            n.gira(wz, 1.2)                    # entra em regime
            n.gira(wz, 3.0, coletar=True)      # so entao mede
            n.para()
            p = list(n.pares)
            if len(p) < 6:
                print(f'{wz:10.2f}   poucas amostras ({len(p)})')
                continue
            g = st.mean(x[0] for x in p)
            e = st.mean(x[1] for x in p)
            ganho = g / e if abs(e) > 1e-6 else float('nan')
            linhas.append((wz, g, e, ganho))
            print(f'{wz:10.2f} {g:8.3f} {e:9.3f} {ganho:7.3f} {len(p):4d}')
            time.sleep(0.5)
    finally:
        n.para()
        n.destroy_node()
        rclpy.shutdown()

    if len(linhas) < 3:
        print('\nDados insuficientes.')
        return
    ganhos = [x[3] for x in linhas]
    gm = st.mean(ganhos)
    print(f'\nganho medio {gm:.3f}  (desvio {st.pstdev(ganhos):.3f})')
    # regressao giro = a*encoder, sem intercepto: e a escala pura
    num = sum(x[1] * x[2] for x in linhas)
    den = sum(x[2] * x[2] for x in linhas)
    a = num / den if den > 1e-9 else float('nan')
    print(f'escala por regressao: giro = {a:.3f} * encoder')
    print()
    if abs(a - 1.0) <= 0.05:
        print('  AS DUAS FONTES CONCORDAM (dentro de 5%).')
        print('  Nao ha patinagem sistematica: o encoder e confiavel para')
        print('  wz, e o peso deve ser decidido por RUIDO, nao por vies.')
    elif a < 0.95:
        print(f'  O encoder le {100*(1/a - 1):.0f}% A MAIS que o giroscopio.')
        print('  Assinatura de PATINAGEM: a roda gira mais que o robo.')
        print('  -> manter peso alto no giroscopio faz sentido.')
    else:
        print(f'  O encoder le {100*(a - 1):.0f}% A MENOS que o giroscopio.')
        print('  Isso NAO e patinagem -- e erro de escala.')
        print('  -> confira ticksRev e K (halfL+halfW) antes do peso.')


if __name__ == '__main__':
    main()
