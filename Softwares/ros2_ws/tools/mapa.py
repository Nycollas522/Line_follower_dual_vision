"""Mapa do circuito a partir de um CSV da corrida.py.

Pose: usa as colunas x_m/y_m/yaw_deg (integradas pelo FIRMWARE, encoders
+ giroscopio) quando existem; CSVs antigos, sem elas, sao integrados aqui
a partir de odom_vx/odom_vy/odom_wz. A linha e projetada no mundo a
partir do erro lateral da camera inferior, na banda mais proxima.

FECHAMENTO DA VOLTA: com mais de ~330 graus de giro acumulado, o robo
passou de novo pelo ponto de partida. A distancia entre as duas passagens
e o erro de giro nesse ponto sao o erro acumulado do dead reckoning --
e o numero que diz se o mapa serve (poucos cm) ou nao.

Uso:
    python3 logs/mapa.py logs/corrida_AAAAMMDD_HHMMSS.csv [saida.png]
"""
import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

LOOK = 0.099        # m a frente do centro: banda mais proxima da inferior
LIMIAR_QUEBRA = 18  # graus de heading da inferior


def carrega(caminho):
    with open(caminho) as fh:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(fh)]


def poses(linhas):
    """(x, y, yaw) por linha, com a origem e o yaw zerados na partida."""
    if 'x_m' in linhas[0]:
        x0, y0 = linhas[0]['x_m'], linhas[0]['y_m']
        th0 = math.radians(linhas[0]['yaw_deg'])
        c, s = math.cos(-th0), math.sin(-th0)
        saida, anterior, acumulado = [], th0, 0.0
        for r in linhas:
            dx, dy = r['x_m'] - x0, r['y_m'] - y0
            th = math.radians(r['yaw_deg'])
            # desenrola: o firmware da o yaw em +/-180
            acumulado += math.atan2(math.sin(th - anterior),
                                    math.cos(th - anterior))
            anterior = th
            saida.append((c * dx - s * dy, s * dx + c * dy, acumulado))
        return saida, 'pose do firmware'
    x = y = th = 0.0
    saida = [(0.0, 0.0, 0.0)]
    for a, b in zip(linhas, linhas[1:]):
        dt = b['t'] - a['t']
        meio = th + 0.5 * a['odom_wz'] * dt   # ponto medio do giro
        x += (a['odom_vx'] * math.cos(meio) - a['odom_vy'] * math.sin(meio)) * dt
        y += (a['odom_vx'] * math.sin(meio) + a['odom_vy'] * math.cos(meio)) * dt
        th += a['odom_wz'] * dt
        saida.append((x, y, th))
    return saida, 'integrado das velocidades'


def fechamento(traj, dist):
    """Menor distancia ao ponto de partida depois de uma volta."""
    giro = traj[-1][2]
    if abs(giro) < math.radians(330):
        return None
    candidatos = [i for i, d in enumerate(dist)
                  if abs(traj[i][2]) > math.radians(300)]
    if not candidatos:
        return None
    i = min(candidatos, key=lambda k: math.hypot(traj[k][0], traj[k][1]))
    erro_giro = abs(traj[i][2]) - 2 * math.pi
    return i, math.hypot(traj[i][0], traj[i][1]), math.degrees(erro_giro)


def main():
    csv_path = sys.argv[1]
    png = sys.argv[2] if len(sys.argv) > 2 else csv_path.rsplit('.', 1)[0] + '_mapa.png'
    linhas = carrega(csv_path)
    traj, fonte = poses(linhas)
    dist = [0.0]
    for p, q in zip(traj, traj[1:]):
        dist.append(dist[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))

    linha, quebras = [], []
    for k, r in enumerate(linhas):
        x, y, th = traj[k]
        if r['valid'] and r['bandas'] >= 3:
            ly = -r['lat_mm'] / 1000.0     # +lat = linha a direita = -y
            linha.append((x + LOOK * math.cos(th) - ly * math.sin(th),
                          y + LOOK * math.sin(th) + ly * math.cos(th)))
        if k and abs(r['head_deg']) > LIMIAR_QUEBRA >= abs(linhas[k - 1]['head_deg']):
            quebras.append((x, y, r['t']))

    print(f'fonte da pose: {fonte}')
    print(f'distancia {dist[-1]:.2f} m   giro total '
          f'{math.degrees(traj[-1][2]):+.0f} graus   quebras {len(quebras)}')
    fecha = fechamento(traj, dist)
    if fecha:
        i, gap, erro = fecha
        print(f'VOLTA FECHADA em t={linhas[i]["t"]:.1f}s: perimetro ~{dist[i]:.2f} m, '
              f'erro de posicao {100 * gap:.1f} cm, erro de giro {erro:+.1f} graus')
    else:
        print('volta NAO fechou (menos de ~330 graus): rode a corrida por mais '
              'tempo para medir o erro acumulado.')

    fig, ax = plt.subplots(figsize=(8, 8))
    if linha:
        ax.plot(*zip(*linha), '.', ms=2, color='0.6', label='linha (camera inferior)')
    ax.plot([p[0] for p in traj], [p[1] for p in traj], '-', lw=1.5,
            color='tab:blue', label=f'centro do robo ({fonte})')
    ax.plot(0, 0, 'o', color='tab:green', ms=9, label='inicio')
    ax.plot(traj[-1][0], traj[-1][1], 's', color='tab:red', ms=9, label='fim')
    for qx, qy, t in quebras:
        ax.plot(qx, qy, '^', color='tab:orange', ms=7)
        ax.annotate(f'{t:.0f}s', (qx, qy), fontsize=8, color='tab:orange')
    if fecha:
        i = fecha[0]
        ax.plot([0, traj[i][0]], [0, traj[i][1]], 'r--', lw=1)
    ax.set_aspect('equal')
    ax.grid(alpha=.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'{csv_path.split("/")[-1]} -- {dist[-1]:.1f} m, '
                 f'giro {math.degrees(traj[-1][2]):+.0f} graus')
    fig.savefig(png, dpi=110, bbox_inches='tight')
    print(f'mapa em {png}')


if __name__ == '__main__':
    main()
