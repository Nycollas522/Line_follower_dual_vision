"""Descobre o SINAL do eixo de guinada da IMU, sem ligar motor.

O robo e girado A MAO. Os encoders dao wz com sinal conhecido (a
cinematica mecanum ja esta verificada); o giroscopio da yawRate com
sinal a determinar. Se os dois concordarem durante o giro, o sinal
configurado esta certo.

ATENCAO: gire o robo APOIADO NO CHAO, para as rodas rolarem e os
encoders registrarem. Levantado no ar, so o giroscopio responde e a
comparacao nao existe.
"""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
rclpy.init()
n = Node('sinal_imu')
d = {'g': 0.0, 'e': 0.0}
par = []
n.create_subscription(Imu, '/imu/data_raw',
                      lambda m: d.__setitem__('g', m.angular_velocity.z), Q)
n.create_subscription(Odometry, '/odom',
                      lambda m: d.__setitem__('e', m.twist.twist.angular.z), Q)

print('>>> GIRE O ROBO A MAO, no chao, uns 90 graus para cada lado')
print('>>> 20 segundos. Pode ir de um lado para o outro.')
t0 = time.time()
while time.time() - t0 < 20.0:
    rclpy.spin_once(n, timeout_sec=0.02)
    if abs(d['e']) > 0.05 or abs(d['g']) > 0.05:
        par.append((d['g'], d['e']))
n.destroy_node()
rclpy.shutdown()

if len(par) < 20:
    print(f'\nSo {len(par)} amostras com giro. O robo girou mesmo, no chao?')
    raise SystemExit(1)

g = [x[0] for x in par]
e = [x[1] for x in par]
mg = sum(g) / len(g)
me = sum(e) / len(e)
num = sum((a - mg) * (b - me) for a, b in par)
den = math.sqrt(sum((a - mg) ** 2 for a in g) *
                sum((b - me) ** 2 for b in e))
r = num / den if den > 1e-9 else 0.0
concorda = sum(1 for a, b in par if a * b > 0)

print(f'\n{len(par)} amostras com giro de verdade')
print(f'  correlacao giroscopio x encoder: {r:+.3f}')
print(f'  amostras com o MESMO sinal: {100*concorda/len(par):.0f}%')
# ganho: quanto o giroscopio le para cada unidade do encoder
ganho = (sum(a * b for a, b in par) /
         sum(b * b for b in e)) if sum(b * b for b in e) > 1e-9 else 0.0
print(f'  ganho giro/encoder: {ganho:+.2f}')
print()
if r > 0.7:
    print('  SINAL CORRETO (+1). Os dois concordam.')
elif r < -0.7:
    print('  SINAL INVERTIDO. Precisa de imuYawSign = -1.')
else:
    print('  INCONCLUSIVO -- correlacao fraca. Gire mais devagar e mais')
    print('  amplo, garantindo que as rodas rolem no chao.')
