#!/usr/bin/env python3
"""Corrida instrumentada: liga a autonomia, grava, desliga.

Nao toca em nenhuma autoridade de publicacao -- o seguidor fica com o
controle inteiro, como na pista. So observa.

GRAVA COMANDO **E** ATUACAO. Isto nao e detalhe: em 10/09/2026 um mau
contato em duas rodas passou um dia inteiro sem ser detectado porque as
corridas gravavam apenas vx/wz de /cmd_vel_auto -- o que o controlador
PEDIU. Nunca /odom nem /wheel_states, o que o robo FEZ. O sintoma era o
robo "andar angulado" e deslizar, assinatura direta de tracao
assimetrica num mecanum, e foi atribuido a controle.

GRAVA AS DUAS CAMERAS, pelo mesmo motivo e pelo mesmo erro repetido:
na primeira corrida com a superior religada (10/09/2026) o resultado
saiu ambiguo -- uma quebra melhorou, outra piorou -- e nao deu para
atribuir nada, porque a saida da superior nao estava sendo gravada.
Se o preview esta influenciando a corrida, ele TEM de estar no CSV.

Uso:
    python3 scripts/corrida.py            # 45 s
    python3 scripts/corrida.py 20         # 20 s
"""
import csv
import os
import statistics as st
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from line_msgs.msg import LineDetection
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

CTRL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST, depth=10)
DURACAO = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
# Timestamp e local persistente: o caminho fixo em /tmp ja fez uma
# corrida sobrescrever a anterior, e sem o CSV antigo nao da para
# comparar duas configuracoes -- as quebras nao se repetem iguais.
CSV = os.path.expanduser(
    time.strftime('~/ros2_ws/logs/corrida_%Y%m%d_%H%M%S.csv')
)
RODAS = ('fl', 'fr', 'rl', 'rr')
# halfL + halfW do chassi (Config::HALF_L + HALF_W no firmware)
K_MECANUM = 0.165


class Corrida(Node):
    """Assina percepcao, comando e atuacao ao mesmo tempo."""

    def __init__(self):
        super().__init__('corrida')
        self.en = self.create_publisher(Bool, '/controle/enable', CTRL)
        self.b = None
        self.servo = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.ovx = 0.0
        self.ovy = 0.0
        self.owz = 0.0
        self.rodas = [0.0, 0.0, 0.0, 0.0]
        self.create_subscription(LineDetection, '/line/detection',
                                 lambda m: setattr(self, 'b', m), CTRL)
        # A camera superior alimenta o freio antecipado e a recuperacao.
        # Sem grava-la nao da para separar o que ela causou do que a
        # pista causou -- as quebras nao se repetem iguais entre corridas.
        self.f = None
        self.create_subscription(LineDetection, '/line_front/detection',
                                 lambda m: setattr(self, 'f', m), CTRL)
        self.create_subscription(Float32, '/servo/state',
                                 lambda m: setattr(self, 'servo', m.data),
                                 CTRL)
        self.create_subscription(Twist, '/cmd_vel_auto', self._tw, CTRL)
        self.create_subscription(Odometry, '/odom', self._od, CTRL)
        self.create_subscription(JointState, '/wheel_states', self._ws, CTRL)
        # Bateria fraca faz o robo nao entregar o comando, e o sintoma se
        # confunde com erro de controle. MEDIDO em 10/09/2026: a 11.13V
        # ele entregou 35% do vx pedido, contra 80% numa corrida anterior.
        self.volts = 0.0
        self.create_subscription(Float32, '/battery/voltage',
                                 lambda m: setattr(self, 'volts', m.data),
                                 CTRL)

    def _tw(self, m):
        self.vx = m.linear.x
        self.wz = m.angular.z

    def _od(self, m):
        self.ovx = m.twist.twist.linear.x
        self.ovy = m.twist.twist.linear.y
        self.owz = m.twist.twist.angular.z

    def _ws(self, m):
        if len(m.velocity) >= 4:
            self.rodas = list(m.velocity[:4])

    def enable(self, valor):
        fim = time.time() + 5.0
        while (time.time() < fim
               and self.en.get_subscription_count() < 3):
            rclpy.spin_once(self, timeout_sec=0.05)
        msg = Bool()
        msg.data = valor
        for _ in range(15):
            self.en.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def relata_percepcao(linhas, hz):
    ok = [r for r in linhas if r['valid']]
    perdas = len(linhas) - len(ok)
    print(f'\namostras {len(linhas)}  validas {len(ok)} '
          f'({100 * len(ok) / len(linhas):.1f}%)  '
          f'sem linha {100 * perdas / len(linhas):.1f}%')
    if ok:
        mod = sorted(abs(r['lat_mm']) for r in ok)
        print(f'|lat|: p50 {mod[len(mod) // 2]:.1f}  '
              f'p90 {mod[int(0.9 * len(mod))]:.1f}  '
              f'max {mod[-1]:.1f} mm   '
              f'(vies {st.mean(r["lat_mm"] for r in ok):+.2f})')
        print(f'conf media {st.mean(r["conf"] for r in ok):.3f}  '
              f'bandas {st.mean(r["bandas"] for r in ok):.2f}')
    seq = []
    cur = 0
    for r in linhas:
        if not r['valid']:
            cur += 1
        else:
            if cur:
                seq.append(cur)
            cur = 0
    if cur:
        seq.append(cur)
    if seq:
        print(f'episodios de perda: {len(seq)}  '
              f'duracoes(s): {sorted(round(d / hz, 2) for d in seq)}')


def relata_preview(linhas):
    """A camera superior: ela viu a quebra antes da inferior?"""
    ok = [r for r in linhas if r['f_valid']]
    if not ok:
        print('\nPREVIEW: camera superior sem deteccao valida na corrida.')
        return
    print(f'\nPREVIEW (camera superior): valida em '
          f'{100 * len(ok) / len(linhas):.1f}% dos quadros')
    print(f'  confianca {st.mean(r["f_conf"] for r in ok):.3f}  '
          f'bandas {st.mean(r["f_bandas"] for r in ok):.2f}  '
          f'curvatura valida em '
          f'{100 * sum(r["f_curv_ok"] for r in ok) / len(ok):.0f}%')
    # A pergunta que importa: quando a inferior viu a quebra, a superior
    # ja tinha visto? Mede-se pelo ADIANTAMENTO do pico de heading.
    def picos(chave, limiar):
        saida, dentro = [], False
        for r in linhas:
            alto = abs(r[chave]) > limiar
            if alto and not dentro:
                dentro, ini = True, r['t']
            elif not alto and dentro:
                dentro = False
                saida.append(ini)
        return saida
    inf = picos('head_deg', 18.0)
    sup = picos('f_head_deg', 18.0)
    if not inf or not sup:
        print('  sem quebras suficientes para medir antecipacao.')
        return
    print('  antecipacao da superior sobre a inferior, por quebra:')
    for t_inf in inf:
        antes = [t for t in sup if t < t_inf]
        if antes:
            print(f'    quebra em t={t_inf:5.1f}s  '
                  f'superior avisou {t_inf - antes[-1]:5.2f}s antes')
        else:
            print(f'    quebra em t={t_inf:5.1f}s  '
                  'superior NAO avisou antes')


def relata_atuacao(linhas):
    """Comandado contra medido. E aqui que mau contato aparece."""
    movendo = [r for r in linhas if abs(r['vx']) > 0.02]
    if not movendo:
        print('\nATUACAO: o robo nunca recebeu comando de avanco.')
        return
    volts = [r['volts'] for r in linhas if r['volts'] > 1.0]
    if volts:
        print(f'\nBATERIA: {volts[0]:.2f}V no inicio -> {volts[-1]:.2f}V no fim'
              f'  (minimo {min(volts):.2f}V)')
        if min(volts) < 11.4:
            print('  <== BAIXA. Abaixo disso o robo nao entrega o comando e')
            print('      o sintoma se confunde com erro de controle.')
    cvx = st.mean(r['vx'] for r in movendo)
    mvx = st.mean(r['odom_vx'] for r in movendo)
    mvy = st.mean(abs(r['odom_vy']) for r in movendo)
    print(f'\nATUACAO (nos {len(movendo)} quadros com comando > 0.02 m/s):')
    entregue = 100 * mvx / cvx if cvx else 0
    print(f'  vx  comandado {cvx:+.3f}  medido {mvx:+.3f}  '
          f'({entregue:.0f}% do pedido)', end='')
    if entregue >= 60:
        print()
    elif volts and min(volts) < 11.4:
        print('  <== BAIXO e bateria caiu: e energia')
    else:
        print('  <== BAIXO com bateria OK: comando lento demais para')
        print('      vencer o atrito, ou carga mecanica')
    print(f'  vy  medido {mvy:.3f} m/s  -- o seguidor NUNCA comanda vy;')
    print('      valor alto = escorregando ou tracao assimetrica')
    # Compara cada roda com a CINEMATICA, nao com as outras cruas: numa
    # corrida real a velocidade varia porque o comando varia, e usar o
    # desvio bruto acusa "intermitente" nas quatro (falso positivo que
    # esta versao ja cometeu). O que denuncia mau contato e o RESIDUO
    # medio contra o previsto -- uma roda so fica destoando das demais.
    #   mecanum, vy=0:  FL=RL=vx-K*wz    FR=RR=vx+K*wz
    sinais = {'fl': -1, 'fr': +1, 'rl': -1, 'rr': +1}
    print('  roda    medido   esperado   residuo   desvio')
    residuos = []
    for nome in RODAS:
        med = [r[nome] for r in movendo]
        esp = [r['vx'] + sinais[nome] * K_MECANUM * r['wz'] for r in movendo]
        res = [m - e for m, e in zip(med, esp)]
        residuos.append(st.mean(res))
        print(f'  {nome.upper():6} {st.mean(med):+8.4f} {st.mean(esp):+9.4f} '
              f'{st.mean(res):+9.4f} {st.pstdev(res):8.4f}')
    espalha = max(residuos) - min(residuos)
    tipico = st.median([abs(r['vx']) for r in movendo])
    print(f'  espalhamento dos residuos: {espalha:.4f} m/s', end='')
    if tipico > 1e-3 and espalha > 0.25 * tipico:
        # A suspeita e a roda que mais SE AFASTA das outras, nao a de
        # menor residuo -- essa e a melhor. (Este erro ja apontou RL,
        # justamente a que mais entregava.)
        mediana = st.median(residuos)
        desvios = [abs(x - mediana) for x in residuos]
        pior = RODAS[desvios.index(max(desvios))]
        print(f'  <== ASSIMETRICO, suspeite de {pior.upper()}')
    else:
        print('  (rodas simetricas)')
    comum = st.mean(residuos)
    if tipico > 1e-3 and abs(comum) > 0.30 * tipico:
        print(f'  todas {comum:+.4f} abaixo do previsto: '
              'escorregamento ou carga, nao mau contato')


def main():
    rclpy.init()
    node = Corrida()
    linhas = []
    try:
        print(f'>>> AUTONOMIA LIGADA por {DURACAO:.0f}s -- '
              'mao no botao do ESP32 <<<')
        node.enable(True)
        t0 = time.time()
        while time.time() - t0 < DURACAO:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.b is None:
                continue
            linha = {
                't': round(time.time() - t0, 3),
                'valid': int(node.b.valid),
                'conf': round(node.b.confidence, 3),
                'lat_mm': round(node.b.lateral_error * 1000, 2),
                'head_deg': round(node.b.heading_error * 57.2958, 2),
                'bandas': int(node.b.bands_valid),
                'contraste': round(node.b.contrast, 1),
                'larg_mm': round(node.b.line_width * 1000, 2),
                'vx': round(node.vx, 4),
                'wz': round(node.wz, 4),
                'odom_vx': round(node.ovx, 4),
                'odom_vy': round(node.ovy, 4),
                'odom_wz': round(node.owz, 4),
                'volts': round(node.volts, 2),
                'servo': round(node.servo, 2),
            }
            frente = node.f
            linha.update({
                'f_valid': int(frente.valid) if frente else 0,
                'f_conf': round(frente.confidence, 3) if frente else 0.0,
                'f_lat_mm': round(frente.lateral_error * 1000, 2)
                            if frente else 0.0,
                'f_head_deg': round(frente.heading_error * 57.2958, 2)
                              if frente else 0.0,
                'f_curv': round(frente.curvature, 3) if frente else 0.0,
                'f_curv_ok': int(frente.curvature_valid) if frente else 0,
                'f_bandas': int(frente.bands_valid) if frente else 0,
            })
            for nome, valor in zip(RODAS, node.rodas):
                linha[nome] = round(valor, 4)
            linhas.append(linha)
    finally:
        print('>>> DESLIGANDO AUTONOMIA <<<')
        node.enable(False)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

    if not linhas:
        print('sem dados')
        return
    os.makedirs(os.path.dirname(CSV), exist_ok=True)
    with open(CSV, 'w', newline='') as fh:
        escritor = csv.DictWriter(fh, fieldnames=list(linhas[0]))
        escritor.writeheader()
        escritor.writerows(linhas)

    relata_percepcao(linhas, len(linhas) / DURACAO)
    relata_preview(linhas)
    relata_atuacao(linhas)
    print(f'\nCSV em {CSV}')


if __name__ == '__main__':
    main()
