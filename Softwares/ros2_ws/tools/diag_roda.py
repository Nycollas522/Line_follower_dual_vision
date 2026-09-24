#!/usr/bin/env python3
"""Onde a roda deixa de alcancar o alvo: PID, saturacao ou atrito?

O firmware publica os tres sinais que separam os casos:
    TARGET  o que a cinematica pediu para a roda
    WHEEL   o que a roda de fato fez
    PWM     quanto o driver mandou

    PWM saturado no pwmLimit  -> falta autoridade: subir o limite
    PWM baixo e roda devagar  -> o PID nao esta insistindo
    PWM alto e roda parada    -> atrito/carga mecanica

Gira no PROPRIO EIXO (vx=vy=0). Nao translada.
"""
import statistics as st
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
VELS = [0.3, 0.5, 0.8, 1.2]


class D(Node):
    def __init__(self):
        super().__init__('diag_roda')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', Q)
        self.w = self.t = self.p = None
        self.amostras = []
        self.coletando = False
        self.create_subscription(JointState, '/wheel_states', self._ws, Q)
        self.create_subscription(JointState, '/wheel_targets', self._tg, Q)
        self.create_subscription(Int32MultiArray, '/wheel_pwm', self._pw, Q)

    def _ws(self, m):
        if len(m.velocity) >= 4:
            self.w = list(m.velocity[:4])
            if self.coletando and self.t and self.p:
                self.amostras.append((list(self.t), self.w, list(self.p)))

    def _tg(self, m):
        if len(m.velocity) >= 4:
            self.t = list(m.velocity[:4])

    def _pw(self, m):
        if len(m.data) >= 4:
            self.p = list(m.data[:4])

    def gira(self, wz, seg, coletar=False):
        t = Twist()
        t.angular.z = float(wz)
        self.coletando = coletar
        if coletar:
            self.amostras.clear()
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
    n = D()
    try:
        fim = time.time() + 6.0
        while time.time() < fim and n.cmd.get_subscription_count() < 1:
            rclpy.spin_once(n, timeout_sec=0.05)
        print('>>> GIRA NO PROPRIO EIXO. Mao no botao. <<<\n')
        print(f'{"wz":>5} {"alvo":>8} {"medido":>8} {"%":>5} '
              f'{"PWM":>6} {"satur":>7}')
        for wz in VELS:
            n.gira(wz, 1.2)
            n.gira(wz, 2.5, coletar=True)
            n.para()
            a = list(n.amostras)
            if len(a) < 5:
                print(f'{wz:5.2f}  poucas amostras ({len(a)})')
                continue
            alvo = st.mean(st.mean(abs(x) for x in s[0]) for s in a)
            med = st.mean(st.mean(abs(x) for x in s[1]) for s in a)
            pwm = st.mean(st.mean(abs(x) for x in s[2]) for s in a)
            pmax = max(max(abs(x) for x in s[2]) for s in a)
            sat = 100.0 * sum(
                1 for s in a for x in s[2] if abs(x) >= 179
            ) / (4 * len(a))
            print(f'{wz:5.2f} {alvo:8.4f} {med:8.4f} '
                  f'{100*med/max(alvo,1e-9):5.0f} {pwm:6.0f} {sat:6.1f}%'
                  f'   pico {pmax}')
            time.sleep(0.4)
    finally:
        n.para()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
