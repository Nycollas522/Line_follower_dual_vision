"""Testes da politica de apontamento da cabeca.

Cobrem os dois modos de falha observados na pista e que so aparecem
com o robo andando -- caros de reproduzir e faceis de errar de novo:

  1. a cabeca ficar PARALELA a linha em vez de olhar para onde a pista
     vai (miravamos o erro da banda mais proxima);
  2. a cabeca ficar TRAVADA na saida da curva no angulo que tinha
     dentro dela (o integrador visual nao tinha vazamento), o que
     realimenta a direcao do corpo por servo_bearing_gain.

O no e de ROS, mas as duas leis sao aritmetica pura: exercitamos
_erro_de_mira e o passo do integrador sem subir rclpy, com dublês
minimos no lugar do no.
"""

import math

import pytest

from robot_bringup.head_servo_node import HeadServoNode


class _Deteccao:
    """line_msgs/LineDetection com so o que a lei de mira consome."""

    def __init__(
        self,
        lateral_error=0.0,
        heading_error=0.0,
        curvature=0.0,
        lookahead_distance=0.24,
        heading_valid=True,
        curvature_valid=True,
    ):
        self.valid = True
        self.lateral_error = lateral_error
        self.heading_error = heading_error
        self.curvature = curvature
        self.lookahead_distance = lookahead_distance
        self.heading_valid = heading_valid
        self.curvature_valid = curvature_valid


class _Cabeca:
    """Estado minimo do no para exercitar a lei sem subir rclpy."""

    def __init__(self, preview=0.34, max_heading_deg=50.0, leak=2.0,
                 hold=1.5):
        self.preview_distance = preview
        self.preview_max_heading = math.radians(max_heading_deg)
        self.track_leak_tau = leak
        self.track_hold_tau = hold
        self.max_angle = 45.0
        self._tracked_angle = 0.0

    _erro_de_mira = HeadServoNode._erro_de_mira
    _clamp = HeadServoNode._clamp


def _passo_visual(cabeca, erro_m, ganho=300.0, periodo=0.05):
    """Um ciclo do integrador visual do MODE_TRACK, com vazamento."""
    if cabeca.track_leak_tau > 0.0:
        cabeca._tracked_angle *= max(
            0.0, 1.0 - periodo / cabeca.track_leak_tau
        )
    cabeca._tracked_angle = cabeca._clamp(
        cabeca._tracked_angle + ganho * erro_m * periodo
    )
    return cabeca._tracked_angle


# --- lei de mira ---------------------------------------------------


def test_linha_reta_e_centrada_nao_desvia_a_mira():
    """Sem heading nem curvatura, mirar longe da o mesmo que mirar perto."""
    cabeca = _Cabeca()
    assert cabeca._erro_de_mira(_Deteccao(lateral_error=0.0)) == 0.0


def test_linha_reta_deslocada_mantem_o_deslocamento():
    cabeca = _Cabeca()
    erro = cabeca._erro_de_mira(_Deteccao(lateral_error=0.03))
    assert erro == pytest.approx(0.03)


def test_linha_inclinada_projeta_para_onde_a_pista_vai():
    """O ponto do erro e o problema que estavamos corrigindo.

    Linha passando sob o robo (erro perto = 0) mas inclinando para a
    direita: mirar a banda proxima manda a cabeca para o centro e ela
    fica paralela a pista. Mirando 0.34 m a frente, a cabeca ve que a
    linha ja saiu para a direita e acompanha.
    """
    cabeca = _Cabeca(preview=0.34)
    det = _Deteccao(
        lateral_error=0.0,
        heading_error=math.radians(20.0),
        curvature=0.0,
        lookahead_distance=0.24,
    )
    esperado = math.tan(math.radians(20.0)) * (0.34 - 0.24)
    assert cabeca._erro_de_mira(det) == pytest.approx(esperado)
    assert cabeca._erro_de_mira(det) > 0.0   # para a direita, como a linha


def test_curvatura_entra_com_o_quadrado_da_distancia():
    cabeca = _Cabeca(preview=0.44)
    det = _Deteccao(lateral_error=0.0, curvature=2.0, lookahead_distance=0.24)
    avanco = 0.44 - 0.24
    assert cabeca._erro_de_mira(det) == pytest.approx(
        0.5 * 2.0 * avanco * avanco
    )


def test_curva_a_esquerda_mira_a_esquerda():
    cabeca = _Cabeca(preview=0.44)
    det = _Deteccao(lateral_error=0.0, curvature=-2.0, lookahead_distance=0.24)
    assert cabeca._erro_de_mira(det) < 0.0


def test_sem_heading_valido_nao_extrapola_heading():
    """Degradar para o comportamento antigo e melhor que inventar."""
    cabeca = _Cabeca()
    det = _Deteccao(
        lateral_error=0.02,
        heading_error=math.radians(30.0),
        heading_valid=False,
        curvature_valid=False,
    )
    assert cabeca._erro_de_mira(det) == pytest.approx(0.02)


def test_heading_absurdo_e_saturado():
    """tan() explode perto de 90 graus; a mira nao pode ir junto."""
    cabeca = _Cabeca(preview=0.34, max_heading_deg=50.0)
    det = _Deteccao(
        lateral_error=0.0,
        heading_error=math.radians(89.0),
        curvature_valid=False,
    )
    teto = math.tan(math.radians(50.0)) * (0.34 - 0.24)
    assert cabeca._erro_de_mira(det) == pytest.approx(teto)


def test_preview_desligado_volta_a_mirar_a_banda_proxima():
    cabeca = _Cabeca(preview=0.0)
    det = _Deteccao(lateral_error=0.01, heading_error=math.radians(30.0))
    assert cabeca._erro_de_mira(det) == pytest.approx(0.01)


def test_banda_proxima_alem_da_mira_nao_extrapola_para_tras():
    """Se o detector so ve alem do ponto de mira, nao inventamos o meio."""
    cabeca = _Cabeca(preview=0.20)
    det = _Deteccao(
        lateral_error=0.01,
        heading_error=math.radians(30.0),
        lookahead_distance=0.24,
    )
    assert cabeca._erro_de_mira(det) == pytest.approx(0.01)


# --- anti-windup do integrador visual -------------------------------


def test_erro_sustentado_ainda_gira_a_cabeca():
    """O vazamento nao pode matar o rastreio normal.

    Com vazamento o integrador vira um atraso de primeira ordem: para
    erro constante o angulo tende a tau*ganho*erro, subindo com
    1 - exp(-t/tau). Aqui: 2.0 * 300 * 0.02 = 12 graus de regime.
    """
    cabeca = _Cabeca(leak=2.0)
    for _ in range(20):        # 20 * 0.05 s = 1.0 s
        _passo_visual(cabeca, 0.02)
    regime = 2.0 * 300.0 * 0.02
    esperado = regime * (1.0 - math.exp(-1.0 / 2.0))
    assert cabeca._tracked_angle == pytest.approx(esperado, rel=0.05)

    for _ in range(200):       # deixa assentar
        _passo_visual(cabeca, 0.02)
    assert cabeca._tracked_angle == pytest.approx(regime, rel=0.05)


def test_sem_erro_o_angulo_visual_volta_sozinho():
    """O modo de falha da saida de curva: angulo velho de pe sozinho."""
    cabeca = _Cabeca(leak=2.0)
    for _ in range(40):
        _passo_visual(cabeca, 0.03)
    dentro_da_curva = cabeca._tracked_angle
    assert dentro_da_curva > 10.0
    for _ in range(120):          # 6 s de reta, erro zero
        _passo_visual(cabeca, 0.0)
    assert cabeca._tracked_angle < 0.2 * dentro_da_curva


def test_sem_vazamento_o_angulo_da_curva_fica_travado():
    """Documenta por que o vazamento existe."""
    cabeca = _Cabeca(leak=0.0)
    for _ in range(40):
        _passo_visual(cabeca, 0.03)
    travado = cabeca._tracked_angle
    for _ in range(120):
        _passo_visual(cabeca, 0.0)
    assert cabeca._tracked_angle == pytest.approx(travado)


def test_integrador_respeita_o_limite_de_angulo():
    cabeca = _Cabeca(leak=2.0)
    for _ in range(400):
        _passo_visual(cabeca, 1.0)
    assert cabeca._tracked_angle <= cabeca.max_angle


# --- segurar o angulo quando a superior perde a linha ------------------
#
# MEDIDO em 10/09/2026: a superior fica invalida em 13.8% do tempo (24.7s
# de 180), e 5 dos 6 episodios longos -- 17.0s somados -- sao seguidos de
# uma quebra em ate 3s. Zerar a parte visual mandava a cabeca para o
# CENTRO exatamente ali. E centrada ela nao reencontra: a validade da
# superior e 94.9% com a cabeca entre +8 e +20 graus contra 85.0%
# centrada. Confirmado com o robo parado sobre uma quebra: a inferior
# lia th=+24deg com 5/5 bandas e a superior lia SEM LINHA em 203 de 203
# quadros, com a cabeca apontada para frente.


def _passo_perdido(cabeca, periodo=0.05):
    """Um ciclo do MODE_TRACK SEM deteccao fresca da superior."""
    if cabeca.track_hold_tau > 0.0:
        cabeca._tracked_angle *= max(
            0.0, 1.0 - periodo / cabeca.track_hold_tau
        )
    else:
        cabeca._tracked_angle = 0.0
    return cabeca._tracked_angle


def test_perda_curta_quase_nao_move_a_cabeca():
    """A mediana das perdas e 0.10s -- essas sao piscadas, nao quebras."""
    cabeca = _Cabeca(hold=1.5)
    cabeca._tracked_angle = 20.0
    for _ in range(2):            # 0.10 s
        _passo_perdido(cabeca)
    assert cabeca._tracked_angle > 0.9 * 20.0


def test_perda_longa_decai_mas_nao_salta_para_o_centro():
    """1.5s de perda: ainda aponta para o lado certo, so que menos."""
    cabeca = _Cabeca(hold=1.5)
    cabeca._tracked_angle = 20.0
    for _ in range(30):           # 1.5 s
        _passo_perdido(cabeca)
    assert 3.0 < cabeca._tracked_angle < 15.0


def test_perda_muito_longa_volta_ao_centro():
    """O outro extremo: linha sumiu de vez, nao segurar para sempre."""
    cabeca = _Cabeca(hold=1.5)
    cabeca._tracked_angle = 20.0
    for _ in range(200):          # 10 s
        _passo_perdido(cabeca)
    assert cabeca._tracked_angle < 1.0


def test_hold_desligado_reproduz_o_comportamento_antigo():
    """Documenta a regressao que o parametro existe para evitar."""
    cabeca = _Cabeca(hold=0.0)
    cabeca._tracked_angle = 20.0
    assert _passo_perdido(cabeca) == 0.0


def test_o_lado_e_preservado_na_perda():
    cabeca = _Cabeca(hold=1.5)
    cabeca._tracked_angle = -18.0
    for _ in range(10):
        _passo_perdido(cabeca)
    assert cabeca._tracked_angle < 0.0
