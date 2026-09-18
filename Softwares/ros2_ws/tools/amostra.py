import statistics as st, sys, time
import rclpy
from line_msgs.msg import LineDetection
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
CTRL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST, depth=10)
rclpy.init(); n = Node('amostra'); d = {}
for t in ('/line/detection', '/line_front/detection'):
    d[t] = []
    n.create_subscription(LineDetection, t,
                          lambda m, t=t: d[t].append(m), CTRL)
t0 = time.time()
while time.time() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.05)
n.destroy_node(); rclpy.shutdown()
for t, v in d.items():
    ok = [m for m in v if m.valid]
    if not ok:
        print(f'{t}: {len(v)} quadros, NENHUM valido'); continue
    print(f'{t:26} {len(v):4d} quadros  validos {100*len(ok)/len(v):5.1f}%  '
          f'conf {st.mean(m.confidence for m in ok):.3f}  '
          f'bandas {st.mean(m.bands_valid for m in ok):.2f}  '
          f'lat {1000*st.mean(m.lateral_error for m in ok):+6.1f}mm  '
          f'curv_ok {sum(m.curvature_valid for m in ok)}/{len(ok)}')
