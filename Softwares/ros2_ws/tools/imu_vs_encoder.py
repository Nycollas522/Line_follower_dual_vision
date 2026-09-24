#!/usr/bin/env python3
"""Compara guinada do GIROSCOPIO com a dos ENCODERS, girando a mao.

Duas perguntas de uma vez:

  1. SINAL: girando para a direita (horario), a REP-103 manda z para
     cima e positivo anti-horario, entao os dois devem ficar negativos.

  2. CONCORDANCIA: com as rodas no X correto, os encoders deveriam
     medir a rotacao tao bem quanto o giroscopio. Se concordarem, o
     peso de 85% no giroscopio (yawEncoderWeight=0.15) perde a
     justificativa que eu dei -- "roletes patinam por projeto" -- e
     precisa ser refeito.

Calcula a wz dos encoders AQUI, a partir de /wheel_states, para nao
depender da fusao que o firmware ja aplica em /odom.

Sem motores: o robo e girado a mao, apoiado no chao.
"""
import math
import statistics as st
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
K = 0.165          # halfL + halfW, do firmware
DUR = 40.0

rclpy.init()
n = Node('imu_enc')
st_d = {'g': 0.0, 'e': 0.0}
par = []


# ALINHAMENTO TEMPORAL. A IMU chega a 5 Hz e os encoders a 48 Hz.
# Parear "ultimo valor de cada" compara um giroscopio com ate 200 ms de
# atraso contra um encoder fresco, e num giro a mao que muda de sentido
# isso destroi a correlacao -- foi o que estragou a primeira medicao.
#
# Aqui cada amostra da IMU e pareada com a MEDIA dos encoders no
# intervalo desde a amostra anterior: as duas passam a representar a
# mesma janela de tempo.
janela = []


def _ws(m):
    if len(m.velocity) >= 4:
        fl, fr, rl, rr = m.velocity[:4]
        # inversa do modelo do firmware: wz -> (-,+,-,+)/(4k)
        janela.append((-fl + fr - rl + rr) / (4.0 * K))


def _imu(m):
    if not janela:
        return
    enc = sum(janela) / len(janela)
    janela.clear()
    giro = m.angular_velocity.z
    if abs(enc) > 0.05 or abs(giro) > 0.05:
        par.append((giro, enc))


n.create_subscription(Imu, '/imu/data_raw', _imu, Q)
n.create_subscription(JointState, '/wheel_states', _ws, Q)

print('>>> GIRE O ROBO A MAO, no chao, para os DOIS lados. 40 s. <<<')
print('    Devagar: a IMU so chega a 5 Hz.')
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.01)
n.destroy_node()
rclpy.shutdown()

if len(par) < 20:
    print(f'\nSo {len(par)} amostras com giro. Girou no chao, com as')
    print('rodas encostadas? Precisa de giro amplo para os dois lados.')
    raise SystemExit(1)

g = [x[0] for x in par]
e = [x[1] for x in par]
mg, me = st.mean(g), st.mean(e)
num = sum((a - mg) * (b - me) for a, b in par)
den = math.sqrt(sum((a - mg) ** 2 for a in g) * sum((b - me) ** 2 for b in e))
r = num / den if den > 1e-9 else 0.0
ganho = (sum(a * b for a, b in par) / sum(b * b for b in e)
         if sum(b * b for b in e) > 1e-9 else 0.0)
res = [a - b for a, b in par]

print(f'\n{len(par)} amostras com giro de verdade')
print(f'  correlacao giroscopio x encoder : {r:+.3f}')
print(f'  ganho (giro por unidade de enc) : {ganho:+.3f}')
print(f'  |giro| medio  {st.mean(abs(x) for x in g):.3f} rad/s')
print(f'  |enc|  medio  {st.mean(abs(x) for x in e):.3f} rad/s')
print(f'  discordancia  media {st.mean(abs(x) for x in res):.4f}  '
      f'desvio {st.pstdev(res):.4f} rad/s')

print()
if r > 0.85:
    print('  SINAL CORRETO e os dois concordam bem.')
elif r < -0.85:
    print('  SINAL INVERTIDO: precisa de imuYawSign = -1.')
else:
    print('  correlacao fraca -- gire mais devagar e mais amplo.')

if abs(ganho - 1.0) < 0.15 and r > 0.85:
    print('  GANHO ~1: o encoder mede a rotacao tao bem quanto o giro.')
    print('  -> yawEncoderWeight (0.15) merece ser refeito: a')
    print('     justificativa de "roletes patinam" nao vale mais.')
elif r > 0.85:
    print(f'  ganho {ganho:.2f}: as duas fontes discordam em escala.')
    print('  -> confira ticksRev e o K (halfL+halfW) antes de mexer no peso.')
