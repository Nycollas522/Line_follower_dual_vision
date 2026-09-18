"""Testes da lei de controle e da maquina de estados.

O valor destes testes e cobrir os caminhos de seguranca -- perda de
linha, recuperacao, parada -- que sao caros e arriscados de exercitar
no robo real e que so aparecem em condicoes que nao se reproduzem
por vontade propria (uma sombra exatamente no lugar certo).
"""

import math

import pytest

from robot_bringup.line_controller import (
    ControllerConfig,
    FollowerController,
    HeadMode,
    Observation,
    State,
)

DT = 0.02


def good(lateral=0.0, heading=0.0, curvature=0.0, confidence=0.9,
         age=0.0) -> Observation:
    return Observation(
        valid=True,
        confidence=confidence,
        lateral_error=lateral,
        heading_error=heading,
        curvature=curvature,
        heading_valid=True,
        curvature_valid=True,
        age=age,
    )


def lost() -> Observation:
    return Observation(valid=False, confidence=0.0, age=math.inf)


def run(controller, observation, seconds, start=100.0, enabled=True,
        preview=None, servo=0.0, yaw_rate=None, obedece=True):
    """Roda o controlador por N segundos e devolve o ultimo comando.

    obedece=False simula atrito estatico: o robo NAO gira, por mais que
    o controlador mande. Foi o caso real que quebrou a primeira versao
    do alinhamento.
    """
    steps = max(1, int(seconds / DT))
    command = None
    for index in range(steps):
        if yaw_rate is not None:
            medido = yaw_rate
        elif obedece:
            medido = command.wz if command is not None else 0.0
        else:
            medido = 0.0
        command = controller.update(
            now=start + index * DT,
            enabled=enabled,
            obs=observation,
            preview=preview,
            servo_angle_deg=servo,
            odom_yaw_rate=medido,
        )
    return command


def make_controller(**overrides) -> FollowerController:
    config = ControllerConfig()
    for name, value in overrides.items():
        setattr(config, name, value)
    return FollowerController(config)


# ----------------------------------------------------------------------
# Sinais -- o bug mais caro do sistema anterior
# ----------------------------------------------------------------------
def test_linha_a_direita_gira_para_a_direita():
    """lateral_error > 0 (linha a direita) deve dar angular.z NEGATIVO,
    porque em ROS z positivo e giro anti-horario = para a esquerda.
    A versao anterior somava sem inverter e fugia da linha."""
    controller = make_controller()
    command = run(controller, good(lateral=+0.03), 0.5)

    assert command.wz < 0.0


def test_linha_a_esquerda_gira_para_a_esquerda():
    controller = make_controller()
    command = run(controller, good(lateral=-0.03), 0.5)

    assert command.wz > 0.0


def test_orientacao_para_a_direita_gira_para_a_direita():
    controller = make_controller()
    command = run(controller, good(lateral=0.0, heading=+0.4), 0.5)

    assert command.wz < 0.0


def test_curvatura_a_direita_produz_feedforward_para_a_direita():
    sem_ff = make_controller(k_feedforward=0.0)
    com_ff = make_controller(k_feedforward=1.0)

    a = run(sem_ff, good(curvature=+4.0), 1.0)
    b = run(com_ff, good(curvature=+4.0), 1.0)

    assert b.wz < a.wz   # mais negativo = vira mais para a direita


# ----------------------------------------------------------------------
# Velocidade adaptativa
# ----------------------------------------------------------------------
def test_reta_limpa_acelera_ate_v_max():
    """v_max so e alcancado com confianca 1.0: a propria confianca entra
    na severidade, entao evidencia parcial ja custa velocidade."""
    controller = make_controller()
    command = run(controller, good(confidence=1.0), 3.0)

    assert command.state == State.FOLLOWING
    assert command.vx == pytest.approx(controller.config.v_max, abs=0.005)


def test_confianca_parcial_ja_custa_velocidade():
    plena = run(make_controller(), good(confidence=1.0), 3.0)
    parcial = run(make_controller(), good(confidence=0.85), 3.0)

    assert parcial.state == State.FOLLOWING
    assert parcial.vx < plena.vx


def test_curva_fechada_reduz_velocidade_sem_parar():
    controller = make_controller()
    command = run(controller, good(lateral=0.05, curvature=8.0), 3.0)

    assert command.vx < controller.config.v_max
    assert command.vx >= controller.config.v_min - 1e-6


def test_confianca_baixa_reduz_velocidade():
    rapido = run(make_controller(), good(confidence=1.0), 3.0)
    lento = run(make_controller(), good(confidence=0.20), 3.0)

    assert lento.vx < rapido.vx
    assert lento.state == State.DEGRADED


# ----------------------------------------------------------------------
# Limite cinematico do chassi mecanum
# ----------------------------------------------------------------------
def test_giro_nunca_excede_o_limite_de_tracao():
    """Com arc_ratio < 1 nenhuma roda pode inverter de sentido, senao o
    robo pirueta no lugar em vez de descrever a curva.

    A invariante vale para os estados que descrevem ARCO ANDANDO. Giro
    deliberado no lugar (ALIGNING, RECOVERING) e outra manobra e tem o
    seu proprio teto -- ver test_giro_parado_e_excecao_deliberada.
    """
    config = ControllerConfig()
    config.align_on_start = False        # queremos FOLLOWING, nao ALIGNING
    controller = FollowerController(config)
    command = run(controller, good(lateral=0.20, heading=1.0), 3.0)

    assert command.state in (State.FOLLOWING, State.DEGRADED)
    limite = config.arc_ratio * command.vx / config.wheel_base_k
    assert abs(command.wz) <= limite + 1e-6
    assert abs(command.wz) <= config.wz_max


def test_giro_parado_e_excecao_deliberada():
    """ALIGNING gira no lugar: as rodas SE OPOEM de proposito.

    O teto arc_ratio*v/k existe para nao inverter roda EM MOVIMENTO; com
    vx=0 ele daria zero e impediria qualquer giro. Aqui o que precisa ser
    respeitado e o maximo de roda do firmware.
    """
    config = ControllerConfig()
    controller = FollowerController(config)
    command = run(controller, good(heading=math.radians(30.0)), 0.5)

    assert command.state == State.ALIGNING
    assert command.vx == pytest.approx(0.0, abs=1e-6)
    assert abs(command.wz) <= config.align_wz + 1e-6
    roda = abs(command.vx) + config.wheel_base_k * abs(command.wz)
    assert roda <= config.wheel_max_mps + 1e-6


def test_roda_mais_rapida_respeita_o_maximo_do_firmware():
    config = ControllerConfig()
    controller = FollowerController(config)
    command = run(controller, good(lateral=0.20), 3.0)

    roda_rapida = command.vx + config.wheel_base_k * abs(command.wz)
    assert roda_rapida <= config.wheel_max_mps + 1e-6


# ----------------------------------------------------------------------
# Perda de linha e seguranca
# ----------------------------------------------------------------------
def test_perda_de_um_frame_nao_para_o_robo():
    """Requisito 5: sombra ou frame ruim nao pode virar parada brusca."""
    controller = make_controller()
    run(controller, good(), 2.0)

    command = controller.update(
        now=102.02, enabled=True, obs=lost(), preview=None
    )

    assert command.state == State.COASTING
    assert command.vx > 0.0


def test_perda_persistente_para_de_avancar():
    controller = make_controller()
    run(controller, good(), 2.0)
    command = run(controller, lost(), 1.0, start=102.0)

    assert command.state == State.RECOVERING
    assert command.vx == pytest.approx(0.0, abs=0.01)


def test_recuperacao_gira_para_o_lado_por_onde_a_linha_saiu():
    controller = make_controller()
    run(controller, good(lateral=+0.04), 2.0)   # linha saiu pela direita
    command = run(controller, lost(), 1.0, start=102.0)

    assert command.state == State.RECOVERING
    assert command.wz < 0.0   # gira para a direita, atras da linha


def test_ultima_leitura_ruidosa_nao_muda_o_lado_da_recuperacao():
    """Medido no robo real (pista do laboratorio, duas tentativas): bem
    na transicao para a perda, a leitura "fresca" (fresh so exige
    confidence_min) as vezes vem com o sinal invertido -- a curva aperta,
    a camera perde a linha por um instante, e o ultimo frame antes de
    cair pra COASTING pode ser esse frame ruim. Casos reais: confianca
    caiu 0.52->0.15 e 0.53->0.24, com o sinal de lateral_error invertendo
    nos dois exatamente nessa transicao. A recuperacao tem que ancorar no
    lado da ULTIMA leitura CONFIAVEL (>= confidence_ok), nao nesse ultimo
    pixel ruidoso."""
    controller = make_controller()
    # Curva bem negociada: linha consistentemente a esquerda, confiavel.
    run(controller, good(lateral=-0.015, confidence=0.90), 2.0)
    # Bem no instante da perda: sinal inverte, mas com confianca baixa
    # (ainda "fresh", pois confidence_min=0.15, mas abaixo de
    # confidence_ok=0.45 -- exatamente o padrao medido no robo).
    ruidosa = good(lateral=+0.005, confidence=0.20)
    command = run(controller, ruidosa, 1.0, start=102.0)
    assert command.state in (State.DEGRADED, State.FOLLOWING)

    command = run(controller, lost(), 1.0, start=103.0)
    assert command.state == State.RECOVERING
    # Se ancorasse no pixel ruidoso (lateral=+0.005, direita), giraria
    # para a direita (wz<0). Ancorado na leitura confiavel anterior
    # (esquerda), tem que girar para a esquerda (wz>0).
    assert command.wz > 0.0


def test_leitura_confiavel_ainda_atualiza_o_lado_normalmente():
    """Contraprova do teste acima: uma leitura fresca e CONFIAVEL deve
    continuar atualizando o lado normalmente -- a correcao e so para
    leituras marginais, nao trava o lado no valor mais antigo para
    sempre."""
    controller = make_controller()
    run(controller, good(lateral=-0.015, confidence=0.90), 2.0)
    run(controller, good(lateral=+0.020, confidence=0.90), 1.0, start=102.0)

    command = run(controller, lost(), 1.0, start=103.0)
    assert command.state == State.RECOVERING
    assert command.wz < 0.0   # ancorado na leitura confiavel mais recente (direita)


def test_recuperacao_pede_varredura_da_cabeca():
    controller = make_controller()
    run(controller, good(), 2.0)
    command = run(controller, lost(), 1.0, start=102.0)

    assert command.head_mode == HeadMode.SCAN


def test_parada_segura_apos_o_limite_de_recuperacao():
    """Requisito 7: existe um limite de tempo, e depois dele o robo para."""
    controller = make_controller(recovery_time=1.0)
    run(controller, good(), 2.0)
    command = run(controller, lost(), 4.0, start=102.0)

    assert command.state == State.SAFE_STOP
    # O requisito e sobre o CORPO: passado o limite, o robo para.
    assert command.vx == pytest.approx(0.0, abs=1e-6)
    assert command.wz == pytest.approx(0.0, abs=1e-6)
    # A cabeca, ao contrario, continua procurando -- mudanca deliberada,
    # ver test_safe_stop_continua_procurando_a_linha. Parar o corpo e
    # seguranca; cegar a cabeca era so desperdicio.
    assert command.head_mode == HeadMode.SCAN


def test_camera_travada_e_tratada_como_perda():
    """A deteccao chega, mas o stamp e antigo: camera congelada. Na versao
    anterior isso mantinha o ultimo comando para sempre."""
    controller = make_controller()
    run(controller, good(), 2.0)

    congelada = good(age=1.0)
    command = run(controller, congelada, 2.0, start=102.0)

    assert command.state in (State.RECOVERING, State.SAFE_STOP)
    assert command.vx == pytest.approx(0.0, abs=0.01)


def test_relock_exige_varias_deteccoes_seguidas():
    controller = make_controller(relock_frames=3, recovery_time=5.0)
    run(controller, good(), 2.0)
    run(controller, lost(), 1.0, start=102.0)
    assert controller.state == State.RECOVERING

    primeira = controller.update(now=103.0, enabled=True, obs=good())
    assert primeira.state == State.RECOVERING

    controller.update(now=103.02, enabled=True, obs=good())
    terceira = controller.update(now=103.04, enabled=True, obs=good())
    assert terceira.state == State.FOLLOWING


# ----------------------------------------------------------------------
# Autonomia desligada
# ----------------------------------------------------------------------
def test_desabilitar_zera_comando_e_centraliza_cabeca():
    controller = make_controller()
    run(controller, good(lateral=0.04), 2.0)

    command = controller.update(now=103.0, enabled=False, obs=good())

    assert command.state == State.IDLE
    assert command.vx == 0.0
    assert command.wz == 0.0
    assert command.head_mode == HeadMode.CENTER


def test_nao_anda_antes_de_ver_a_linha_pela_primeira_vez():
    controller = make_controller()
    command = run(controller, lost(), 2.0)

    assert command.state == State.IDLE
    assert command.vx == 0.0


# ----------------------------------------------------------------------
# Camera superior
# ----------------------------------------------------------------------
def test_preview_nao_domina_a_camera_inferior():
    """Mesmo com uma curvatura absurda no preview, o feedforward dele fica
    abaixo do teto configurado."""
    config = ControllerConfig()
    controller = FollowerController(config)
    preview = good(curvature=50.0, confidence=1.0)

    command = run(controller, good(), 3.0, preview=preview)

    teto = config.preview_ff_max_ratio * config.wz_max
    assert abs(command.preview_ff) <= teto + 1e-9


def test_preview_ignorado_com_a_cabeca_girada():
    """Com o pan fora do centro, o deslocamento na imagem mistura 'a linha
    desviou' com 'a camera girou'. Sem calibracao camera-servo, o preview
    deixa de valer como direcao."""
    controller = make_controller()
    preview = good(curvature=6.0, confidence=1.0)

    command = run(controller, good(), 2.0, preview=preview, servo=30.0)

    assert not command.preview_active
    assert command.preview_ff == 0.0


def test_preview_reduz_velocidade_antes_da_curva_chegar():
    """Requisito 3: antecipar a curva antes de ela entrar na camera de baixo."""
    sem = run(make_controller(), good(), 3.0)
    com = run(
        make_controller(), good(), 3.0,
        preview=good(curvature=7.0, confidence=1.0),
    )

    assert com.vx < sem.vx


def test_preview_desligado_nao_afeta_nada():
    ligado = run(
        make_controller(use_preview=True), good(), 3.0,
        preview=good(curvature=7.0, confidence=1.0),
    )
    desligado = run(
        make_controller(use_preview=False), good(), 3.0,
        preview=good(curvature=7.0, confidence=1.0),
    )

    assert desligado.vx > ligado.vx
    assert not desligado.preview_active


# ----------------------------------------------------------------------
# Metricas
# ----------------------------------------------------------------------
def test_metricas_contam_perdas_e_erro_medio():
    controller = make_controller(recovery_time=0.5)
    run(controller, good(lateral=0.02), 2.0)
    run(controller, lost(), 2.0, start=102.0)

    assert controller.loss_events == 1
    assert controller.mean_abs_error == pytest.approx(0.02, abs=1e-6)


# ----------------------------------------------------------------------
# Preview de heading (quina/reta angulada, curvatura invalida)
# ----------------------------------------------------------------------
def corner_preview(heading=0.0, confidence=0.9) -> Observation:
    """Preview tipico de uma quina real: heading valido mas curvatura
    invalida (o ajuste quadratico nao serve pra uma dobra angular -- ver
    preview_heading_gain). Modela o que foi medido no robo: a camera
    superior lia heading_error grande e estavel numa quina, enquanto a
    curvatura oscilava/era rejeitada."""
    return Observation(
        valid=True,
        confidence=confidence,
        lateral_error=0.0,
        heading_error=heading,
        curvature=0.0,
        heading_valid=True,
        curvature_valid=False,
        age=0.0,
    )


def test_preview_heading_antecipa_quina_mesmo_sem_curvatura_valida():
    """O caso real medido no robo: curvatura do preview invalida (quina,
    nao curva), mas heading_error grande e estavel. Isso sozinho ja deve
    reduzir velocidade antes da camera inferior ver a quina chegando."""
    sem_preview = run(make_controller(), good(), 3.0)
    com_preview = run(
        make_controller(), good(), 3.0,
        preview=corner_preview(heading=math.radians(25.0), confidence=0.7),
    )

    # Freia mesmo com a DIRECAO do preview desligada: o papel de
    # velocidade e independente e continua valendo (ver as chaves
    # preview_*_enabled em ControllerConfig).
    assert com_preview.vx < sem_preview.vx

    # Ja o preview_active marca a contribuicao no GIRO, que so existe
    # com o papel de direcao habilitado.
    com_direcao = run(
        make_controller(preview_steering_enabled=True), good(), 3.0,
        preview=corner_preview(heading=math.radians(25.0), confidence=0.7),
    )
    assert com_direcao.preview_active


def test_preview_heading_gira_para_o_lado_certo():
    """heading positivo do preview (quina para a direita) deve produzir
    feedforward NEGATIVO -- mesma convencao de sinal do heading proprio
    (k_heading), so que antecipado pela camera de cima."""
    controller = make_controller(
        k_feedforward=0.0, preview_steering_enabled=True
    )
    comando = run(
        controller, good(), 1.0,
        preview=corner_preview(heading=math.radians(20.0), confidence=0.9),
    )

    assert comando.preview_ff < 0.0


def test_preview_heading_e_curvatura_somados_respeitam_o_mesmo_teto():
    """Curvatura e heading do preview, ambos fortes ao mesmo tempo, nao
    podem somar acima do teto -- o teto vale para a contribuicao TOTAL
    do preview, nao para cada termo em separado."""
    config = ControllerConfig()
    controller = FollowerController(config)
    preview_forte = Observation(
        valid=True, confidence=1.0, lateral_error=0.0,
        heading_error=math.radians(40.0), curvature=50.0,
        heading_valid=True, curvature_valid=True, age=0.0,
    )

    comando = run(controller, good(), 2.0, preview=preview_forte)

    teto = config.preview_ff_max_ratio * config.wz_max
    assert abs(comando.preview_ff) <= teto + 1e-9


def test_preview_heading_tambem_ignorado_com_cabeca_girada():
    controller = make_controller()
    comando = run(
        controller, good(), 2.0,
        preview=corner_preview(heading=math.radians(25.0)), servo=30.0,
    )

    assert not comando.preview_active
    assert comando.preview_ff == 0.0


# ----------------------------------------------------------------------
# Bearing pelo servo (cabeca rastreando a linha)
# ----------------------------------------------------------------------
def test_use_servo_bearing_pede_head_mode_track():
    """Com use_servo_bearing ligado, o seguimento normal pede TRACK em
    vez de CENTER -- e a cabeca precisa estar rastreando para o angulo
    dela significar algo."""
    controller = make_controller(use_servo_bearing=True)
    comando = run(controller, good(), 1.0, preview=good(confidence=0.8))

    assert comando.head_mode == HeadMode.TRACK


def test_realimentacao_desligada_nao_congela_a_cabeca():
    """Comportamento MUDADO de proposito.

    Antes use_servo_bearing=false mandava a cabeca para o centro, o que
    juntava duas decisoes numa flag so: "o corpo usa o angulo da
    cabeca" e "a cabeca se mexe". Quem congela a cabeca agora e
    head_tracking_enabled -- ver test_cabeca_pode_ser_congelada_sozinha.
    """
    controller = make_controller(use_servo_bearing=False)
    comando = run(controller, good(), 1.0, preview=good(confidence=0.8))

    assert comando.head_mode == HeadMode.TRACK
    assert comando.preview_ff == pytest.approx(0.0)


def test_servo_bearing_gira_para_o_lado_certo():
    """servo_angle_deg positivo (cabeca virada a direita) deve produzir
    feedforward NEGATIVO -- mesma convencao de todo o resto do sistema."""
    controller = make_controller(use_servo_bearing=True, k_feedforward=0.0)
    comando = run(
        controller, good(), 1.0,
        preview=good(confidence=0.8), servo=20.0,
    )

    assert comando.preview_ff < 0.0
    assert comando.preview_active


def test_servo_bearing_nao_exige_cabeca_centrada():
    """Ao contrario do preview de imagem, o bearing do servo so funciona
    justamente QUANDO a cabeca esta girada -- nao pode ser bloqueado pelo
    mesmo teste de preview_center_tol_deg."""
    controller = make_controller(use_servo_bearing=True, k_feedforward=0.0)
    comando = run(
        controller, good(), 1.0,
        preview=good(confidence=0.8), servo=30.0,  # bem fora do centro
    )

    assert comando.preview_active
    assert comando.preview_ff != 0.0


def test_servo_bearing_exige_preview_fresco_e_confiavel():
    """Sem deteccao valida da camera superior, nao ha bearing para usar
    -- head_mode ainda pede TRACK (a cabeca deve continuar tentando
    procurar), mas o feedforward nao pode vir do nada."""
    controller = make_controller(use_servo_bearing=True, k_feedforward=0.0)
    comando = run(controller, good(), 1.0, preview=None, servo=20.0)

    assert not comando.preview_active
    assert comando.preview_ff == 0.0
    assert comando.head_mode == HeadMode.TRACK


def test_servo_bearing_continua_ativo_em_degraded():
    """Ao contrario do preview de imagem (explicitamente desligado em
    DEGRADED), o bearing do servo e uma leitura independente da camera
    inferior fraca, e continua valendo."""
    controller = make_controller(use_servo_bearing=True, k_feedforward=0.0)
    comando = run(
        controller, good(confidence=0.20), 1.0,
        preview=good(confidence=0.8), servo=20.0,
    )

    assert comando.state == State.DEGRADED
    assert comando.preview_active


def test_servo_bearing_respeita_o_mesmo_teto_do_preview():
    config = ControllerConfig(use_servo_bearing=True, servo_bearing_gain=10.0)
    controller = FollowerController(config)
    comando = run(controller, good(), 2.0, preview=good(confidence=1.0), servo=40.0)

    teto = config.preview_ff_max_ratio * config.wz_max
    assert abs(comando.preview_ff) <= teto + 1e-9


# ----------------------------------------------------------------------
# Offset de regime em curva -- o "segue a linha mas fica paralelo a ela"
# ----------------------------------------------------------------------
def _offset_de_regime(k_ff, kappa=3.0, seconds=25.0):
    """Simula o robo numa curva de curvatura constante e devolve o erro
    lateral de regime.

    Modelo de seguimento de linha com os DOIS estados acoplados:
        e_ponto     = v * theta          (o erro lateral cresce com o
                                          heading relativo a tangente)
        theta_ponto = wz + v * kappa     (o heading relativo muda com o
                                          deficit entre o wz comandado e
                                          o wz que a curva exige, -v*kappa)

    Em regime (e_ponto = theta_ponto = 0) sai a formula fechada:
        e = (1 - k_ff) * v * kappa / k_lateral
    """
    controller = make_controller(k_feedforward=k_ff, v_max=0.15, v_min=0.15,
                                 curvature_filter_tau=0.05)
    e = 0.0
    theta = 0.0
    t = 100.0
    cmd = None
    for _ in range(int(seconds / DT)):
        obs = good(lateral=e, heading=theta, curvature=kappa, confidence=1.0)
        cmd = controller.update(now=t, enabled=True, obs=obs)
        v = cmd.vx
        e += v * theta * DT
        theta += (cmd.wz + v * kappa) * DT
        t += DT
    return e, cmd


def test_feedforward_baixo_deixa_o_robo_paralelo_a_linha():
    """Com k_ff pequeno, o feedback so consegue gerar o wz que falta
    MANTENDO erro -- o robo anda paralelo a linha, nao em cima dela.
    Formula: e_regime = (1 - k_ff) * v * kappa / k_lateral."""
    e_baixo, _ = _offset_de_regime(k_ff=0.15)
    e_alto, _ = _offset_de_regime(k_ff=0.90)

    assert abs(e_baixo) > 0.010          # mais de 10mm: meia fita fora
    assert abs(e_alto) < abs(e_baixo) / 3   # feedforward correto derruba muito
    # O valor analitico exato aqui e 5.000mm ((1-0.9)*v*kappa/k_lateral),
    # entao o limiar nao pode ser 0.005: a simulacao converge PARA ele e
    # a comparacao vira uma disputa de ponto flutuante. Ver
    # test_offset_bate_com_a_formula_analitica para a checagem exata.
    assert abs(e_alto) <= 0.0055         # sobra ~5mm, nao dezenas


def test_offset_bate_com_a_formula_analitica():
    """A simulacao tem que concordar com e = (1-k_ff)*v*kappa/k_lateral,
    que e a conta usada para justificar o valor de k_feedforward."""
    k_ff, kappa, v, k_lat = 0.30, 3.0, 0.15, 9.0
    e_sim, _ = _offset_de_regime(k_ff=k_ff, kappa=kappa)
    e_teorico = (1.0 - k_ff) * v * kappa / k_lat

    assert abs(abs(e_sim) - e_teorico) < 0.30 * e_teorico


def test_curvatura_filtrada_ignora_pico_de_um_frame():
    """Um pico isolado de curvatura (o ruido que nos fez baixar o ganho)
    nao pode virar comando cheio: o filtro tem que absorve-lo."""
    controller = make_controller(k_feedforward=0.90,
                                 curvature_filter_tau=0.15)
    run(controller, good(curvature=0.0, confidence=1.0), 1.0)
    antes = controller._curvature_filtered

    controller.update(now=101.02, enabled=True,
                      obs=good(curvature=11.0, confidence=1.0))
    depois = controller._curvature_filtered

    assert abs(depois - antes) < 11.0 * 0.25   # absorveu a maior parte


def test_curvatura_filtrada_decai_quando_fica_invalida():
    """Ao sair da curva, a curvatura fica invalida. Segurar o ultimo
    valor manteria o robo curvando depois da curva acabar."""
    controller = make_controller(curvature_filter_tau=0.10)
    run(controller, good(curvature=4.0, confidence=1.0), 2.0)
    assert abs(controller._curvature_filtered) > 1.0

    sem_curva = good(confidence=1.0)
    sem_curva.curvature_valid = False
    run(controller, sem_curva, 1.5, start=103.0)

    assert abs(controller._curvature_filtered) < 0.5


# ----------------------------------------------------------------------
# Antecipacao geometrica da cabeca
# ----------------------------------------------------------------------
def test_cabeca_antecipa_a_curva_pela_geometria():
    """Numa curva a direita, a cabeca tem que apontar para a DIREITA
    antes de a linha sair do quadro -- bearing = atan(D*kappa/2)."""
    controller = make_controller(use_servo_bearing=True,
                                 curvature_filter_tau=0.05)
    cmd = run(controller, good(curvature=+3.0, confidence=1.0), 2.0,
              preview=good(confidence=0.9))

    assert cmd.head_mode == HeadMode.TRACK
    assert cmd.head_angle > 5.0     # aponta mesmo, nao fica no centro

    esperado = math.degrees(math.atan(0.375 * 3.0 * 0.5)) * 0.8
    assert abs(cmd.head_angle - esperado) < 3.0


def test_cabeca_antecipa_para_o_lado_certo_na_curva_a_esquerda():
    controller = make_controller(use_servo_bearing=True,
                                 curvature_filter_tau=0.05)
    cmd = run(controller, good(curvature=-3.0, confidence=1.0), 2.0,
              preview=good(confidence=0.9))

    assert cmd.head_angle < -5.0


def test_sem_curva_a_cabeca_nao_antecipa():
    controller = make_controller(use_servo_bearing=True,
                                 curvature_filter_tau=0.05)
    cmd = run(controller, good(curvature=0.0, confidence=1.0), 2.0,
              preview=good(confidence=0.9))

    assert abs(cmd.head_angle) < 1.0


# ----------------------------------------------------------------------
# Papeis da camera superior separados por risco
#
# Antes os tres papeis (direcao, velocidade, recuperacao) viviam atras
# de use_preview, e use_servo_bearing controlava ao mesmo tempo a
# realimentacao no corpo e o movimento da cabeca. Isso escondeu dois
# problemas na pista: desligar a direcao matava a recuperacao junto, e
# um teste A/B da realimentacao mudava duas variaveis de uma vez.
# ----------------------------------------------------------------------


def test_direcao_do_preview_desligada_por_padrao():
    """Medido na pista: com ela ligada o robo perdia 32% do tempo."""
    controller = make_controller()
    assert controller.config.preview_steering_enabled is False


def test_direcao_do_preview_desligada_nao_muda_o_giro():
    reto = make_controller(preview_steering_enabled=False)
    curvo = make_controller(preview_steering_enabled=True)
    preview = good(curvature=3.0, confidence=0.9)
    sem = run(reto, good(), 1.0, preview=preview)
    com = run(curvo, good(), 1.0, preview=preview)
    assert sem.wz == pytest.approx(0.0, abs=1e-6)
    assert abs(com.wz) > 0.05          # a chave realmente faz efeito


def test_velocidade_ainda_antecipa_a_curva_com_direcao_desligada():
    """O papel barato tem de sobreviver ao desligamento do caro."""
    controller = make_controller(
        preview_steering_enabled=False, preview_speed_enabled=True
    )
    reta = run(controller, good(), 3.0, preview=good(curvature=0.0))
    controller.reset()
    curva = run(controller, good(), 3.0, preview=good(curvature=5.0))
    assert curva.vx < reta.vx


def test_velocidade_do_preview_pode_ser_desligada_sozinha():
    ligado = make_controller(preview_speed_enabled=True)
    desligado = make_controller(preview_speed_enabled=False)
    preview = good(curvature=5.0)
    com = run(ligado, good(), 3.0, preview=preview)
    sem = run(desligado, good(), 3.0, preview=preview)
    assert sem.vx > com.vx


def test_recuperacao_sobrevive_a_direcao_desligada():
    """O bug real: desligar a direcao matava a dica de recuperacao.

    A camera superior so escolhe o LADO do giro em RECOVERING -- nunca
    a magnitude -- entao nao tem como desestabilizar a direcao.
    """
    controller = make_controller(
        preview_steering_enabled=False, preview_recovery_enabled=True
    )
    run(controller, good(lateral=0.0), 0.5)
    achou_a_direita = good(lateral=0.05, confidence=0.9)
    command = run(controller, lost(), 3.0, preview=achou_a_direita)
    assert command.state == State.RECOVERING
    assert command.wz < 0.0            # gira para a direita, onde ela esta


def test_recuperacao_pode_ser_desligada_sozinha():
    controller = make_controller(preview_recovery_enabled=False)
    run(controller, good(lateral=0.0), 0.5)
    command = run(controller, lost(), 3.0, preview=good(lateral=0.05))
    assert command.state == State.RECOVERING
    assert command.head_mode != HeadMode.HOLD   # nao segura na superior


def test_chave_mestra_desliga_os_tres_papeis():
    controller = make_controller(
        use_preview=False,
        preview_steering_enabled=True,
        preview_speed_enabled=True,
        preview_recovery_enabled=True,
    )
    run(controller, good(lateral=0.0), 0.5)
    command = run(controller, lost(), 3.0, preview=good(lateral=0.05))
    assert command.head_mode != HeadMode.HOLD


def test_cabeca_rastreia_mesmo_sem_realimentar_o_corpo():
    """A configuracao que mediu 0% de perda na pista.

    A cabeca continua enquadrando a curva (util para velocidade e
    recuperacao) sem injetar o proprio angulo no giro do corpo.
    """
    controller = make_controller(
        head_tracking_enabled=True, use_servo_bearing=False
    )
    command = run(controller, good(curvature=2.0), 1.0)
    assert command.head_mode == HeadMode.TRACK
    assert abs(command.head_angle) > 1.0


def test_desligar_a_realimentacao_nao_congela_mais_a_cabeca():
    """Antes use_servo_bearing=false mandava a cabeca para o centro."""
    controller = make_controller(
        head_tracking_enabled=True, use_servo_bearing=False
    )
    command = run(controller, good(curvature=3.0), 1.0)
    assert command.head_mode == HeadMode.TRACK


def test_cabeca_pode_ser_congelada_sozinha():
    controller = make_controller(head_tracking_enabled=False)
    command = run(controller, good(curvature=3.0), 1.0)
    assert command.head_mode == HeadMode.CENTER
    assert command.head_angle == pytest.approx(0.0)


def test_safe_stop_continua_procurando_a_linha():
    """MEDIDO na pista: em SAFE_STOP a cabeca ficava parada no centro.

    Num episodio de perda de 9.9s a cabeca varreu so os 3s de
    RECOVERING e depois esperou cega por ~7s. Como o corpo ja esta
    parado neste estado, varrer e ganho puro de percepcao.
    """
    controller = make_controller(scan_in_safe_stop=True)
    run(controller, good(), 0.5)
    command = run(controller, lost(), 8.0)
    assert command.state == State.SAFE_STOP
    assert command.head_mode == HeadMode.SCAN


def test_safe_stop_nao_move_o_corpo_enquanto_procura():
    """A varredura so e aceitavel porque o corpo esta parado."""
    controller = make_controller(scan_in_safe_stop=True)
    run(controller, good(), 0.5)
    command = run(controller, lost(), 8.0)
    assert command.state == State.SAFE_STOP
    assert command.vx == pytest.approx(0.0, abs=1e-6)
    assert command.wz == pytest.approx(0.0, abs=1e-6)


def test_varredura_em_safe_stop_pode_ser_desligada():
    controller = make_controller(scan_in_safe_stop=False)
    run(controller, good(), 0.5)
    command = run(controller, lost(), 8.0)
    assert command.state == State.SAFE_STOP
    assert command.head_mode == HeadMode.CENTER


def test_idle_nunca_varre():
    """IDLE e 'desligado', nao 'procurando' -- a cabeca fica no centro."""
    controller = make_controller(scan_in_safe_stop=True)
    command = run(controller, good(), 1.0, enabled=False)
    assert command.state == State.IDLE
    assert command.head_mode == HeadMode.CENTER


# ----------------------------------------------------------------------
# Atraso de transporte do feedforward e amortecimento constante
#
# Os dois vieram de medicao na pista (09/09/2026): o robo virava cedo
# demais e entrava por dentro da curva, e o amortecimento caia conforme
# ele acelerava na reta.
# ----------------------------------------------------------------------


def _entra_na_curva(controller, curvatura, segundos, vx_alvo=0.15):
    """Reta e depois curva, com a curvatura aparecendo A FRENTE."""
    passos = max(1, int(segundos / DT))
    wz = []
    for i in range(passos):
        cmd = controller.update(
            now=200.0 + i * DT, enabled=True,
            obs=good(lateral=0.0, heading=0.0, curvature=curvatura),
        )
        wz.append(cmd.wz)
    return wz


def test_feedforward_nao_vira_no_primeiro_instante_da_curva():
    """O sintoma medido: virar antes de chegar na curva.

    Cenario real: o robo vem de uma RETA e a curvatura aparece no
    horizonte da camera. Com o atraso de transporte essa leitura ainda
    nao comanda giro -- ela so vale quando o robo chegar naquele trecho.
    """
    def sem_atraso_e_com_atraso(distancia):
        controller = make_controller(
            curvature_eval_distance_m=distancia,
            k_lateral=0.0, k_heading=0.0, k_damping=0.0,
            heading_gain_scheduling=False,
        )
        # reta primeiro: enche o historico de curvatura com zeros
        for i in range(100):
            controller.update(now=200.0 + i * DT, enabled=True,
                              obs=good(curvature=0.0))
        # agora a curva aparece no horizonte
        wz = []
        for i in range(10):
            cmd = controller.update(
                now=202.0 + i * DT, enabled=True, obs=good(curvature=5.0))
            wz.append(cmd.wz)
        return wz

    com = sem_atraso_e_com_atraso(0.085)
    sem = sem_atraso_e_com_atraso(0.0)
    # vindo da reta, o atraso segura o feedforward: nada de virar ainda
    assert all(abs(v) < 1e-9 for v in com)
    # sem atraso, o giro comeca no ato -- o comportamento que tirava o
    # robo da linha por dentro da curva
    assert abs(sem[-1]) > 0.01


def test_feedforward_chega_depois_do_atraso():
    """Atrasado nao e cancelado: passado o tempo de percurso, ele age."""
    controller = make_controller(
        curvature_eval_distance_m=0.085, k_lateral=0.0, k_heading=0.0,
        k_damping=0.0, heading_gain_scheduling=False,
    )
    wz = _entra_na_curva(controller, curvatura=5.0, segundos=4.0)
    assert wz[-1] < -0.05            # curva a direita -> wz negativo


def test_atraso_desligado_vira_no_ato():
    """Documenta o comportamento anterior, para comparacao."""
    controller = make_controller(
        curvature_eval_distance_m=0.0, k_lateral=0.0, k_heading=0.0,
        k_damping=0.0, heading_gain_scheduling=False,
    )
    wz = _entra_na_curva(controller, curvatura=5.0, segundos=0.2)
    assert abs(wz[-1]) > abs(wz[0])  # ja reagiu na largada


def test_atraso_e_maior_quando_o_robo_anda_devagar():
    """distancia/velocidade: mais devagar, mais tempo ate chegar la."""
    rapido = make_controller(v_min=0.20, v_max=0.20)
    devagar = make_controller(v_min=0.05, v_max=0.05)
    for c in (rapido, devagar):
        c.update(now=300.0, enabled=True, obs=good())
    wz_r = _entra_na_curva(rapido, 5.0, 0.6)
    wz_d = _entra_na_curva(devagar, 5.0, 0.6)
    # o mais devagar ainda nao aplicou tanto feedforward quanto o rapido
    assert abs(wz_d[-1]) < abs(wz_r[-1])


def test_amortecimento_fica_constante_com_a_velocidade():
    """O ponto do agendamento: zeta igual em toda a faixa de velocidade."""
    cfg = ControllerConfig()
    controller = FollowerController(cfg)
    for v in (0.07, 0.15, 0.22):
        k = controller._ganho_de_heading(v)
        wn = math.sqrt(v * cfg.k_lateral)
        zeta = (k + cfg.k_damping * v) / (2.0 * wn)
        assert zeta == pytest.approx(cfg.target_damping, abs=0.05)


def test_agendamento_nunca_devolve_ganho_negativo():
    """Invariante MUDADA em 10/09/2026.

    Antes o agendamento tinha piso em k_heading. Com a montagem
    apontada para baixo o erro passou a ser medido a ~11cm A FRENTE, e
    k_lateral*L ja da ganho de orientacao de graca -- descontar isso
    exige poder descer abaixo do k_heading nominal. O piso que resta e
    zero: negativo seria realimentacao positiva de orientacao.
    """
    cfg = ControllerConfig()
    controller = FollowerController(cfg)
    for v in (0.0, 0.01, 0.05, 0.07, 0.15, 0.22):
        for L in (0.0, 0.11, 0.5):
            assert controller._ganho_de_heading(v, L) >= 0.0


def test_agendamento_desligado_devolve_o_ganho_fixo():
    controller = make_controller(heading_gain_scheduling=False)
    for v in (0.05, 0.15, 0.22):
        assert controller._ganho_de_heading(v) == pytest.approx(1.6)


def test_sem_agendamento_o_amortecimento_cai_com_a_velocidade():
    """Documenta o problema que o agendamento resolve."""
    cfg = ControllerConfig()
    cfg.heading_gain_scheduling = False
    controller = FollowerController(cfg)
    zetas = []
    for v in (0.07, 0.22):
        k = controller._ganho_de_heading(v)
        wn = math.sqrt(v * cfg.k_lateral)
        zetas.append((k + cfg.k_damping * v) / (2.0 * wn))
    assert zetas[0] > 1.0 > zetas[1]


# ----------------------------------------------------------------------
# ALIGNING: girar parado antes de andar
#
# MEDIDO na pista: largado a 25 graus, o robo atravessava a linha e saia
# a -49.8mm. O limite e geometrico (raio minimo 20.6cm, constante com a
# velocidade), entao nenhum ganho resolve -- so nao avancar enquanto
# gira.
# ----------------------------------------------------------------------


def test_largado_torto_alinha_parado_antes_de_andar():
    controller = make_controller(align_on_start=True, align_heading_deg=18.0)
    command = run(controller, good(heading=math.radians(25.0)), 0.3)
    assert command.state == State.ALIGNING
    assert command.vx == pytest.approx(0.0, abs=1e-6)
    assert command.wz < 0.0          # linha inclina a direita -> gira a direita


def test_alinhamento_gira_para_o_lado_certo_a_esquerda():
    controller = make_controller(align_on_start=True)
    command = run(controller, good(heading=math.radians(-25.0)), 0.3)
    assert command.state == State.ALIGNING
    assert command.wz > 0.0


def test_largado_reto_nao_perde_tempo_alinhando():
    controller = make_controller(align_on_start=True, align_heading_deg=18.0)
    command = run(controller, good(heading=math.radians(3.0)), 0.5)
    assert command.state == State.FOLLOWING
    assert command.vx > 0.0


def test_alinhou_entao_comeca_a_andar():
    controller = make_controller(align_on_start=True, align_exit_deg=7.0)
    run(controller, good(heading=math.radians(25.0)), 0.3)
    command = run(controller, good(heading=math.radians(2.0)), 1.0,
                  start=101.0)
    assert command.state == State.FOLLOWING
    assert command.vx > 0.0


def test_alinhamento_nunca_fica_preso():
    """Se o heading nao cair (leitura ruim), o timeout solta o robo."""
    controller = make_controller(align_on_start=True, align_timeout=1.0)
    run(controller, good(heading=math.radians(25.0)), 0.3)
    command = run(controller, good(heading=math.radians(25.0)), 3.0,
                  start=101.0)
    assert command.state != State.ALIGNING


def test_nao_para_para_alinhar_no_meio_da_curva():
    """So dispara na PARTIDA. Em curva o heading tambem passa de 20 graus,
    e parar ali seria pior que o problema que isto resolve."""
    controller = make_controller(align_on_start=True, align_heading_deg=18.0)
    run(controller, good(heading=0.0), 1.0)          # ja seguindo
    command = run(controller, good(heading=math.radians(25.0), curvature=4.0),
                  1.0, start=101.0)
    assert command.state != State.ALIGNING
    assert command.vx > 0.0


def test_alinhamento_pode_ser_desligado():
    controller = make_controller(align_on_start=False)
    command = run(controller, good(heading=math.radians(25.0)), 0.3)
    assert command.state != State.ALIGNING


def test_perder_a_linha_alinhando_nao_aborta_a_manobra():
    """Comportamento MUDADO de proposito.

    Antes, perder a linha alinhando caia em COASTING. Mas o robo esta
    PARADO durante o alinhamento -- nao ha nada de que se proteger, e
    abortar deixava o robo torto. A manobra agora segue ate o alvo
    travado, a rotacao medida ou o timeout. Ver
    test_alinhamento_sobrevive_a_perder_a_linha_no_giro.
    """
    controller = make_controller(align_on_start=True, align_timeout=30.0)
    run(controller, good(heading=math.radians(40.0)), 0.3)
    command = run(controller, lost(), 0.1, start=101.0)
    assert command.state == State.ALIGNING
    assert command.vx == pytest.approx(0.0, abs=1e-6)


def test_alinhamento_sobrevive_a_perder_a_linha_no_giro():
    """MEDIDO: girando no lugar a banda mais proxima so aguenta ~9 graus.

    Perder a linha no meio da manobra e esperado, nao motivo para
    abortar: o robo esta parado e o alvo ja foi travado na entrada.
    """
    controller = make_controller(align_on_start=True)
    cmd = run(controller, good(heading=math.radians(40.0)), 0.2)
    assert cmd.state == State.ALIGNING
    # a linha some no MEIO do giro (a 0.9 rad/s, 40 graus levam ~0.78s)
    cmd = run(controller, lost(), 0.2, start=101.0)
    assert cmd.state == State.ALIGNING
    assert cmd.vx == pytest.approx(0.0, abs=1e-6)
    assert abs(cmd.wz) > 0.1          # continua girando


def test_alinhamento_termina_pela_rotacao_cumprida():
    """Sem visao, quem encerra e o alvo travado, nao o timeout."""
    controller = make_controller(align_on_start=True, align_timeout=30.0)
    run(controller, good(heading=math.radians(25.0)), 0.2)
    cmd = run(controller, lost(), 3.0, start=101.0)
    assert cmd.state != State.ALIGNING     # cumpriu o alvo e saiu


def test_alinhamento_gira_para_o_lado_do_alvo_travado():
    controller = make_controller(align_on_start=True)
    cmd = run(controller, good(heading=math.radians(25.0)), 0.3)
    assert cmd.wz < 0.0
    outro = make_controller(align_on_start=True)
    cmd = run(outro, good(heading=math.radians(-25.0)), 0.3)
    assert cmd.wz > 0.0


def test_alinhamento_nao_gira_indefinidamente_sem_visao():
    """Guarda de overshoot: nunca muito alem do alvo travado."""
    controller = make_controller(
        align_on_start=True, align_timeout=30.0, align_overshoot_guard=1.25
    )
    run(controller, good(heading=math.radians(20.0)), 0.2)
    cmd = run(controller, lost(), 10.0, start=101.0)
    assert cmd.state != State.ALIGNING


def test_angulo_pequeno_ainda_usa_a_visao():
    """Ate align_closed_loop_deg a banda proxima aguenta: visao decide."""
    controller = make_controller(
        align_on_start=True, align_heading_deg=5.0,
        align_closed_loop_deg=9.0, align_exit_deg=2.0,
    )
    cmd = run(controller, good(heading=math.radians(8.0)), 0.2)
    assert cmd.state == State.ALIGNING
    cmd = run(controller, good(heading=math.radians(1.0)), 0.3, start=101.0)
    assert cmd.state in (State.FOLLOWING, State.DEGRADED)


def test_atrito_estatico_nao_engana_o_alinhamento():
    """O bug real medido na pista, agora travado por teste.

    Com atrito estatico o robo nao gira mesmo comandado. A versao
    anterior integrava o wz COMANDADO, concluia que tinha girado e
    soltava o robo ainda torto (medido: saiu a -23.7 graus). Agora o
    progresso vem do giro MEDIDO nos encoders.
    """
    controller = make_controller(align_on_start=True, align_timeout=30.0)
    parado = good(heading=math.radians(25.0))
    cmd = run(controller, parado, 0.2)
    assert cmd.state == State.ALIGNING
    # 3 segundos comandando giro, mas o robo nao obedece
    cmd = run(controller, parado, 3.0, start=101.0, obedece=False)
    assert cmd.state == State.ALIGNING       # nao declara vitoria falsa
    assert cmd.vx == pytest.approx(0.0, abs=1e-6)


def test_visao_encerra_o_alinhamento_mesmo_com_alvo_grande():
    """A visao e criterio primario sempre que existir."""
    controller = make_controller(align_on_start=True, align_exit_deg=7.0)
    cmd = run(controller, good(heading=math.radians(40.0)), 0.2)
    assert cmd.state == State.ALIGNING
    cmd = run(controller, good(heading=math.radians(2.0)), 0.3, start=101.0)
    assert cmd.state in (State.FOLLOWING, State.DEGRADED)


def test_sem_visao_usa_o_giro_medido():
    controller = make_controller(align_on_start=True, align_timeout=30.0)
    run(controller, good(heading=math.radians(20.0)), 0.2)
    # cego, mas girando de verdade a 0.9 rad/s
    cmd = run(controller, lost(), 2.0, start=101.0, yaw_rate=0.9)
    assert cmd.state != State.ALIGNING


# ----------------------------------------------------------------------
# Montagem apontada para baixo (10/09/2026): o erro passou a ser medido
# a ~11cm A FRENTE do robo, e isso muda o amortecimento efetivo.
# ----------------------------------------------------------------------


def _zeta(cfg, controller, v, lookahead):
    """Amortecimento efetivo, ja contando o que a montagem oferece."""
    k = controller._ganho_de_heading(v, lookahead)
    efetivo = k + cfg.k_lateral * lookahead
    wn = math.sqrt(v * cfg.k_lateral)
    return (efetivo + cfg.k_damping * v) / (2.0 * wn)


def test_lookahead_da_montagem_entra_no_amortecimento():
    """Sem descontar, o agendamento empilha e sobreamortece.

    So vale onde heading_gain_min nao encosta (v de cruzeiro). Abaixo
    disso o piso manda -- de proposito, ver
    test_piso_do_ganho_impede_o_robo_de_andar_torto.
    """
    cfg = ControllerConfig()
    controller = FollowerController(cfg)
    for v in (0.15, 0.22):
        assert _zeta(cfg, controller, v, 0.11) == pytest.approx(
            cfg.target_damping, abs=0.05
        )


def test_sem_lookahead_o_agendamento_continua_valendo():
    """Montagem antiga (erro medido junto ao robo) nao muda de resposta."""
    cfg = ControllerConfig()
    controller = FollowerController(cfg)
    for v in (0.07, 0.15, 0.22):
        assert _zeta(cfg, controller, v, 0.0) == pytest.approx(
            cfg.target_damping, abs=0.05
        )


def test_ganho_de_heading_nunca_inverte_de_sinal():
    """Negativo seria realimentacao positiva de orientacao."""
    controller = make_controller()
    for L in (0.0, 0.11, 1.0, 5.0):
        for v in (0.0, 0.05, 0.15, 0.22):
            assert controller._ganho_de_heading(v, L) > 0.0


def test_piso_do_ganho_impede_o_robo_de_andar_torto():
    """MEDIDO na pista (10/09/2026): "andou quase que angulado".

    Com o erro medido a L metros A FRENTE, o controlador pode anular
    e_medido = e_centro + L*theta GIRANDO em vez de endireitar, e
    estaciona em theta = k_lat*e/(k_lat*L + k_head). Sem piso, o
    agendamento levava k_head a ZERO em baixa velocidade e o angulo de
    estacionamento dobrava. O piso existe para limitar esse angulo, nao
    para amortecer.
    """
    cfg = ControllerConfig()
    controller = FollowerController(cfg)
    L = 0.115
    offset = 0.020

    def angulo_estacionado(k_head):
        return math.degrees(
            cfg.k_lateral * offset / (cfg.k_lateral * L + k_head)
        )

    # parado, onde o agendamento pediria ~zero
    k = controller._ganho_de_heading(0.003, L)
    assert k >= cfg.heading_gain_min
    assert angulo_estacionado(k) < 5.5

    # sem o piso o angulo quase dobra
    assert angulo_estacionado(0.0) > 9.0

    # em velocidade de cruzeiro o piso nao encosta
    assert controller._ganho_de_heading(0.22, L) > cfg.heading_gain_min


def test_agendamento_desligado_ignora_o_lookahead():
    controller = make_controller(heading_gain_scheduling=False)
    for L in (0.0, 0.11, 0.5):
        assert controller._ganho_de_heading(0.15, L) == pytest.approx(1.6)


def test_cabeca_desligada_nao_varre_em_recovering():
    """MEDIDO na pista (10/09/2026): com head_tracking_enabled=false o
    servo ainda varria, porque RECOVERING/SAFE_STOP forcavam SCAN por
    conta propria. Com a camera superior fora do suporte isso puxava o
    cabo. A chave passou a ser mestra do movimento da cabeca."""
    controller = make_controller(
        head_tracking_enabled=False, scan_in_recovery=True
    )
    run(controller, good(), 0.5)
    command = run(controller, lost(), 1.0, start=101.0)
    assert command.state == State.RECOVERING
    assert command.head_mode == HeadMode.CENTER


def test_cabeca_desligada_nao_varre_em_safe_stop():
    controller = make_controller(
        head_tracking_enabled=False, scan_in_safe_stop=True
    )
    run(controller, good(), 0.5)
    command = run(controller, lost(), 8.0, start=101.0)
    assert command.state == State.SAFE_STOP
    assert command.head_mode == HeadMode.CENTER


def test_cabeca_ligada_ainda_varre_ao_perder():
    """A varredura continua valendo quando a cabeca esta habilitada."""
    controller = make_controller(
        head_tracking_enabled=True, scan_in_recovery=True
    )
    run(controller, good(), 0.5)
    command = run(controller, lost(), 1.0, start=101.0)
    assert command.state == State.RECOVERING
    assert command.head_mode == HeadMode.SCAN


# ----------------------------------------------------------------------
# Lado da busca decidido pela linha PROJETADA
#
# MEDIDO na pista (10/09/2026): entrando em curva, as bandas morrem por
# HEADING enquanto o erro lateral ainda esta em +3 a +5mm -- ruido. O
# robo girava para o lado errado: "perde primeiro, depois vira pro lado
# perdido e acha".
# ----------------------------------------------------------------------


def test_lado_da_busca_segue_o_heading_quando_o_lateral_e_ruido():
    """O caso real: lateral quase zero, heading claro para a direita."""
    controller = make_controller()
    # linha praticamente centrada, mas inclinando forte para a DIREITA
    run(controller, good(lateral=0.003, heading=math.radians(14.0),
                         confidence=0.9), 1.0)
    command = run(controller, lost(), 3.0, start=101.0)
    assert command.state == State.RECOVERING
    assert command.wz < 0.0          # gira para a DIREITA, onde a linha foi


def test_lado_da_busca_segue_o_heading_para_a_esquerda():
    controller = make_controller()
    run(controller, good(lateral=-0.003, heading=math.radians(-14.0),
                         confidence=0.9), 1.0)
    command = run(controller, lost(), 3.0, start=101.0)
    assert command.state == State.RECOVERING
    assert command.wz > 0.0


def test_lateral_grande_ainda_manda_no_lado_da_busca():
    """Com offset real e heading nulo, quem decide continua sendo ele."""
    controller = make_controller()
    run(controller, good(lateral=0.030, heading=0.0, confidence=0.9), 1.0)
    command = run(controller, lost(), 3.0, start=101.0)
    assert command.wz < 0.0          # linha a direita -> busca a direita


def test_projecao_desligada_volta_a_usar_so_o_lateral():
    controller = make_controller(recovery_projection_m=0.0)
    # lateral levemente negativo, heading forte para a direita
    run(controller, good(lateral=-0.002, heading=math.radians(20.0),
                         confidence=0.9), 1.0)
    command = run(controller, lost(), 3.0, start=101.0)
    assert command.wz > 0.0          # so o lateral manda: busca a esquerda


def test_heading_absurdo_nao_explode_a_projecao():
    """tan() perto de 90 graus: a projecao satura em vez de estourar."""
    controller = make_controller()
    run(controller, good(lateral=0.001, heading=math.radians(89.0),
                         confidence=0.9), 1.0)
    command = run(controller, lost(), 3.0, start=101.0)
    assert command.state == State.RECOVERING
    assert abs(command.wz) <= controller.config.recovery_wz + 1e-6


# ----------------------------------------------------------------------
# Freio de quebra pelo residuo do ajuste
# ----------------------------------------------------------------------


def _obs_residuo(residuo, heading=0.0, valido=True):
    o = good(heading=heading, confidence=0.9)
    o.fit_residual = residuo
    o.residual_valid = valido
    return o


def test_quebra_freia_o_robo():
    controller = make_controller(ref_residual=0.0028)
    reta = run(controller, _obs_residuo(0.0), 3.0)
    controller.reset()
    quebra = run(controller, _obs_residuo(0.0030), 3.0)
    assert quebra.vx < reta.vx


def test_robo_torto_em_reta_NAO_e_freado_pelo_residuo():
    """O ponto do sinal: heading alto com residuo baixo nao e quebra.

    Se este teste falhar, o freio de quebra vira freio de
    desalinhamento e atrasa a recuperacao da reta.
    """
    # align_on_start=False: 25 graus dispararia ALIGNING e zeraria vx
    # por outro motivo, escondendo o que este teste mede.
    # ref_heading alto neutraliza o freio de heading, deixando so o
    # residuo como diferenca entre os dois casos.
    controller = make_controller(
        ref_residual=0.0028, ref_heading=10.0, align_on_start=False
    )
    # heading grande, mas as bandas sobre a mesma reta
    torto = run(controller, _obs_residuo(0.0, heading=math.radians(25.0)), 3.0)
    controller.reset()
    reto = run(controller, _obs_residuo(0.0, heading=0.0), 3.0)
    assert torto.vx == pytest.approx(reto.vx, rel=0.05)


def test_residuo_invalido_nao_freia():
    controller = make_controller(ref_residual=0.0028)
    com = run(controller, _obs_residuo(0.010, valido=False), 3.0)
    controller.reset()
    sem = run(controller, _obs_residuo(0.0), 3.0)
    assert com.vx == pytest.approx(sem.vx, rel=0.01)


def test_freio_de_quebra_pode_ser_desligado():
    controller = make_controller(ref_residual=0.0)
    com = run(controller, _obs_residuo(0.010), 3.0)
    controller.reset()
    sem = run(controller, _obs_residuo(0.0), 3.0)
    assert com.vx == pytest.approx(sem.vx, rel=0.01)


def test_quebra_grande_leva_a_v_min():
    controller = make_controller(ref_residual=0.0028)
    command = run(controller, _obs_residuo(0.010), 4.0)
    assert command.vx == pytest.approx(controller.config.v_min, abs=1e-3)


# --- mira da cabeca vinda da camera inferior --------------------------
#
# MEDIDO em 10/09/2026 com o robo parado sobre uma quebra: a inferior
# lia th=+24deg com 5/5 bandas, e a superior lia SEM LINHA em 203 de 203
# quadros -- a pista virou e saiu do campo dela. Com a cabeca centrada
# nao ha o que ler; a unica saida e apontar para onde a inferior diz que
# a pista vai.


def _obs_mira(heading_deg=0.0, lateral=0.0, lookahead=0.115,
              heading_valid=True, curvature=0.0, curvature_valid=False):
    return Observation(
        valid=True,
        lateral_error=lateral,
        heading_error=math.radians(heading_deg),
        heading_valid=heading_valid,
        lookahead_distance=lookahead,
        curvature=curvature,
        curvature_valid=curvature_valid,
    )


def _controlador_mira(**overrides):
    cfg = ControllerConfig(preview_lookahead_m=0.375,
                           servo_lookahead_gain=1.0)
    for nome, valor in overrides.items():
        setattr(cfg, nome, valor)
    return FollowerController(cfg), cfg


def test_linha_reta_centrada_mira_em_frente():
    ctrl, _ = _controlador_mira()
    assert ctrl._mira_da_cabeca(_obs_mira()) == pytest.approx(0.0, abs=0.1)


def test_heading_da_inferior_vira_a_cabeca_para_o_mesmo_lado():
    """O caso da quebra medida: heading positivo -> mira positiva."""
    ctrl, _ = _controlador_mira()
    assert ctrl._mira_da_cabeca(_obs_mira(heading_deg=24.0)) > 8.0


def test_a_mira_cresce_com_o_heading():
    ctrl, _ = _controlador_mira()
    anterior = -99.0
    for graus in (0.0, 10.0, 24.0, 40.0):
        atual = ctrl._mira_da_cabeca(_obs_mira(heading_deg=graus))
        assert atual > anterior
        anterior = atual


def test_quebra_para_a_esquerda_mira_a_esquerda():
    ctrl, _ = _controlador_mira()
    assert ctrl._mira_da_cabeca(_obs_mira(heading_deg=-24.0)) < -8.0


def test_sem_curvatura_a_mira_nao_e_mais_zero():
    """A regressao que motivou a mudanca.

    Com curvature_enabled=false a curvatura e sempre 0. O termo antigo
    era proporcional a ela, entao a mira colapsava para o centro
    justamente na quebra, onde a superior mais precisa estar virada.
    """
    ctrl, _ = _controlador_mira()
    obs = _obs_mira(heading_deg=24.0, curvature=0.0, curvature_valid=False)
    assert abs(ctrl._mira_da_cabeca(obs)) > 5.0


def test_heading_invalido_usa_so_o_erro_lateral():
    """Degradar para o que se sabe e melhor que extrapolar."""
    ctrl, _ = _controlador_mira()
    obs = _obs_mira(heading_deg=45.0, lateral=0.02, heading_valid=False)
    esperado = math.degrees(math.atan(0.02 / 0.375))
    assert ctrl._mira_da_cabeca(obs) == pytest.approx(esperado, abs=0.1)


def test_heading_absurdo_e_saturado():
    """tan() explode perto de 90 graus; a mira nao pode ir junto."""
    ctrl, _ = _controlador_mira()
    teto = ctrl._mira_da_cabeca(_obs_mira(heading_deg=60.0))
    assert ctrl._mira_da_cabeca(_obs_mira(heading_deg=89.0)) == pytest.approx(
        teto, abs=0.1
    )


def test_curvatura_valida_ainda_soma():
    """Desligada hoje, mas se voltar tem de melhorar a estimativa."""
    ctrl, _ = _controlador_mira()
    reto = ctrl._mira_da_cabeca(_obs_mira(heading_deg=10.0))
    curvo = ctrl._mira_da_cabeca(
        _obs_mira(heading_deg=10.0, curvature=2.0, curvature_valid=True)
    )
    assert curvo > reto


def test_ganho_escala_a_mira():
    ctrl, cfg = _controlador_mira()
    cheio = ctrl._mira_da_cabeca(_obs_mira(heading_deg=24.0))
    cfg.servo_lookahead_gain = 0.5
    assert ctrl._mira_da_cabeca(_obs_mira(heading_deg=24.0)) == pytest.approx(
        cheio / 2.0, abs=0.05
    )
