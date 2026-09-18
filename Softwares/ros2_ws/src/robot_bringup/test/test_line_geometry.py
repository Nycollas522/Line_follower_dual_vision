"""Testes da deteccao geometrica com imagens sinteticas.

Sinteticas de proposito: uma linha desenhada tem posicao conhecida com
precisao de pixel, entao da para afirmar que o erro em metros esta certo.
Para regressao com imagens reais, grave um rosbag e alimente frames
extraidos na mesma funcao process().
"""

import math

import cv2
import numpy as np
import pytest

from robot_bringup.line_geometry import (
    CameraGeometry,
    DetectorConfig,
    LineDetector,
)

WIDTH = 320
HEIGHT = 240


def make_config(**overrides) -> DetectorConfig:
    config = DetectorConfig(
        roi_top=0.0,
        roi_bottom=1.0,
        # Os frames sinteticos ja sao gerados na orientacao correta.
        # As cameras do robo real estao invertidas -- ver o teste
        # test_rotate_180_inverte_o_lado, que cobre esse caso.
        rotate_180=False,
        work_width=160,
        bands=5,
        geometry=CameraGeometry(
            depth_near_m=0.035,
            depth_far_m=0.135,
            width_near_m=0.100,
            width_far_m=0.100,   # sem perspectiva: facilita a conferencia
        ),
    )
    for name, value in overrides.items():
        setattr(config, name, value)
    return config


# Com width_near_m = 0.100 e line_width_m = 0.020, a linha ocupa 20% da
# largura da imagem -- 64 px em 320. Desenhar mais fino que isso faz o
# detector rejeitar por implausibilidade de largura, que e o comportamento
# desejado, mas nao e o que estes testes querem exercitar.
LINE_THICKNESS = 64


def make_frame(offset_px: int = 0, slope: float = 0.0,
               thickness: int = LINE_THICKNESS,
               background: int = 60, line: int = 230) -> np.ndarray:
    """Linha reta clara sobre fundo escuro. slope em px por px de altura."""
    frame = np.full((HEIGHT, WIDTH, 3), background, dtype=np.uint8)
    for row in range(HEIGHT):
        centre = WIDTH // 2 + offset_px + int(slope * (row - HEIGHT // 2))
        left = max(0, centre - thickness // 2)
        right = min(WIDTH, centre + thickness // 2)
        frame[row, left:right] = line
    return frame


def test_linha_centrada_da_erro_quase_zero():
    detector = LineDetector(make_config())
    observation = detector.process(make_frame(offset_px=0))

    assert observation.valid
    assert observation.bands_valid == 5
    assert abs(observation.lateral_error) < 0.003
    assert observation.confidence > 0.5


def test_linha_a_direita_tem_erro_positivo():
    """Convencao central do sistema: +x = direita. Se este teste inverter,
    o robo foge da linha em vez de segui-la."""
    detector = LineDetector(make_config())
    observation = detector.process(make_frame(offset_px=+64))

    assert observation.valid
    assert observation.lateral_error > 0.0
    # 64 px de 320 = 20% da largura = 20% de 0.10 m = 0.020 m.
    assert observation.lateral_error == pytest.approx(0.020, abs=0.004)


def test_linha_a_esquerda_tem_erro_negativo():
    detector = LineDetector(make_config())
    observation = detector.process(make_frame(offset_px=-64))

    assert observation.valid
    assert observation.lateral_error < 0.0


def test_inclinacao_produz_heading_com_sinal_correto():
    """A linha se afasta para a direita -> heading positivo."""
    detector = LineDetector(make_config())
    # slope negativo em px por linha: no topo (mais longe) fica a direita.
    observation = detector.process(make_frame(slope=-0.4))

    assert observation.valid
    assert observation.heading_valid
    assert observation.heading_error > 0.05


def test_frame_uniforme_nao_inventa_linha():
    """Otsu sempre parte o histograma, ate numa parede lisa. O guarda de
    contraste e o que impede uma deteccao fantasma."""
    detector = LineDetector(make_config())
    frame = np.full((HEIGHT, WIDTH, 3), 90, dtype=np.uint8)
    frame += np.random.default_rng(0).integers(
        -3, 3, frame.shape, dtype=np.int8
    ).astype(np.uint8)

    observation = detector.process(frame)

    assert not observation.valid
    assert observation.confidence == 0.0


def test_piso_todo_claro_e_rejeitado():
    detector = LineDetector(make_config())
    frame = np.full((HEIGHT, WIDTH, 3), 40, dtype=np.uint8)
    frame[:, : int(WIDTH * 0.8)] = 220   # 80% claro: nao e uma linha

    observation = detector.process(frame)

    assert not observation.valid
    assert observation.reject_reason == 'area'


def test_uma_banda_so_nao_produz_orientacao():
    """Requisito explicito: linha vista so perto conduz com cautela, mas
    nao pode inventar para onde a pista vai."""
    config = make_config(bands=5)
    detector = LineDetector(config)
    frame = np.full((HEIGHT, WIDTH, 3), 60, dtype=np.uint8)
    # Linha apenas nos ultimos 12% da altura (banda mais proxima).
    half = LINE_THICKNESS // 2
    frame[int(HEIGHT * 0.88):, WIDTH // 2 - half: WIDTH // 2 + half] = 230

    observation = detector.process(frame)

    assert observation.bands_valid == 1
    assert observation.valid
    assert not observation.heading_valid
    assert not observation.curvature_valid
    assert observation.heading_error == 0.0
    assert observation.confidence < 0.5


def test_continuidade_rejeita_distrator_maior():
    """Um reflexo com area maior que a linha nao pode ganhar a disputa.
    Era exatamente o que 'max(contours, key=contourArea)' fazia."""
    config = make_config(track_weight=0.8)
    detector = LineDetector(config)

    # Trava o detector na linha verdadeira, a esquerda.
    for _ in range(3):
        observation = detector.process(make_frame(offset_px=-50))
    assert observation.valid
    tracked = observation.lateral_error

    # Agora aparece um borrao com MAIS AREA que a linha, do outro lado.
    frame = make_frame(offset_px=-50)
    frame[:, 240:WIDTH] = 235
    observation = detector.process(frame)

    assert observation.valid
    # Continua na linha estreita da esquerda, nao pula para o borrao.
    assert observation.lateral_error == pytest.approx(tracked, abs=0.006)


def test_curvatura_tem_sinal_de_curva_a_direita():
    config = make_config(bands=6, min_bands_for_curvature=4)
    detector = LineDetector(config)

    frame = np.full((HEIGHT, WIDTH, 3), 60, dtype=np.uint8)
    half = LINE_THICKNESS // 2
    for row in range(HEIGHT):
        # y_ratio 0 no topo = mais longe. Quanto mais longe, mais a direita.
        far = 1.0 - row / float(HEIGHT)
        centre = int(WIDTH // 2 + 70 * far * far)
        frame[row, max(0, centre - half): min(WIDTH, centre + half)] = 230

    observation = detector.process(frame)

    assert observation.valid
    assert observation.curvature_valid
    assert observation.curvature > 0.0


def test_custo_por_frame_e_compativel_com_o_raspberry():
    """Nao e um benchmark, e uma trava: se alguem reintroduzir uma
    operacao O(area) no pipeline, este teste avisa."""
    detector = LineDetector(make_config())
    frame = cv2.resize(make_frame(offset_px=20), (640, 480))

    for _ in range(5):
        observation = detector.process(frame)

    assert observation.valid
    assert observation.processing_time < 0.030


def test_rotate_180_inverte_o_lado():
    """As duas cameras do robo estao montadas de cabeca para baixo.
    Com rotate_180 ligado, o mesmo frame precisa dar o erro com o sinal
    trocado -- e e por isso que o parametro existe: sem ele, "perto" e
    "longe" trocam de lugar e o robo foge da linha."""
    direto = LineDetector(make_config(rotate_180=False))
    invertido = LineDetector(make_config(rotate_180=True))

    frame = make_frame(offset_px=+64)
    a = direto.process(frame)
    b = invertido.process(frame)

    assert a.valid and b.valid
    assert a.lateral_error > 0.0
    assert b.lateral_error < 0.0
    assert b.lateral_error == pytest.approx(-a.lateral_error, abs=0.004)


def test_flanco_escuro_derruba_candidato_encostado_na_borda():
    """Um pedaco de piso claro que encosta na borda do quadro nao pode
    ser confundido com a linha: nao ha como confirmar que ele termina ali."""
    config = make_config(rotate_180=False, flank_weight=0.7, track_weight=0.0)
    detector = LineDetector(config)

    frame = np.full((HEIGHT, WIDTH, 3), 60, np.uint8)
    # Linha legitima, ladeada de escuro dos dois lados.
    frame[:, 150:214] = 230
    # Faixa clara da mesma largura, mas colada na borda direita.
    frame[:, WIDTH - 64:] = 230

    observation = detector.process(frame)

    assert observation.valid
    # Fica na linha do meio, nao na faixa da borda.
    assert observation.lateral_error < 0.01


def test_desalinhamento_grande_nao_produz_curvatura_espuria():
    """Medido no robo: linha RETA, robo desalinhado ~23 graus na
    partida, produzia curvatura de +7 a +10 1/m -- artefato da projecao
    das bandas no referencial girado do robo, nao curva real. Isso
    prendia severity=1.0 (vx no minimo) e, por tabela, o teto de giro
    (proporcional a vx), atrasando a propria correcao do heading."""
    detector = LineDetector(make_config(rotate_180=False))

    # Linha reta no mundo, mas o "robo" (a imagem) esta rotacionado:
    # simula com uma inclinacao forte e constante (slope), que e
    # exatamente o efeito de fotografar uma reta de um angulo.
    frame = make_frame(slope=-0.55)  # ~28 graus de inclinacao na imagem

    observation = detector.process(frame)

    assert observation.valid
    assert observation.heading_valid
    assert abs(observation.heading_error) > math.radians(15)
    assert not observation.curvature_valid


def test_desalinhamento_pequeno_ainda_permite_curvatura():
    """O guarda so bloqueia heading grande; com o robo quase alinhado,
    uma curva real ainda deve ser medida normalmente."""
    config = make_config(bands=6, min_bands_for_curvature=4)
    detector = LineDetector(config)

    frame = np.full((HEIGHT, WIDTH, 3), 60, dtype=np.uint8)
    half = LINE_THICKNESS // 2
    for row in range(HEIGHT):
        far = 1.0 - row / float(HEIGHT)
        centre = int(WIDTH // 2 + 70 * far * far)
        frame[row, max(0, centre - half): min(WIDTH, centre + half)] = 230

    observation = detector.process(frame)

    assert observation.valid
    assert abs(observation.heading_error) < math.radians(17)
    assert observation.curvature_valid


def test_compensacao_de_inclinacao_ajuda_linha_diagonal():
    """A varredura mede a linha na horizontal: inclinada em theta, ela
    mede largura_real/cos(theta). Medido na pista: a 60 graus uma fita
    de 20mm mediu 40mm, e o score caiu de ~0.95 para 0.34 -- o detector
    punia a linha por 'larga demais' exatamente na curva."""
    frame = make_frame(slope=-0.55)   # ~28 graus na imagem

    sem = LineDetector(make_config(rotate_180=False,
                                   max_tilt_width_factor=1.0))
    com = LineDetector(make_config(rotate_180=False,
                                   max_tilt_width_factor=3.0))
    for _ in range(5):
        obs_sem = sem.process(frame)
        obs_com = com.process(frame)

    assert obs_sem.valid and obs_com.valid
    media_sem = sum(s.score for s in obs_sem.samples) / len(obs_sem.samples)
    media_com = sum(s.score for s in obs_com.samples) / len(obs_com.samples)
    assert media_com > media_sem


def test_compensacao_nao_muda_nada_em_linha_reta():
    """Com a linha alinhada, cos(theta)=1 e a compensacao tem que ser
    inerte -- ela nao pode afrouxar o criterio de largura em reta."""
    frame = make_frame(offset_px=0)
    sem = LineDetector(make_config(rotate_180=False,
                                   max_tilt_width_factor=1.0))
    com = LineDetector(make_config(rotate_180=False,
                                   max_tilt_width_factor=3.0))
    for _ in range(5):
        obs_sem = sem.process(frame)
        obs_com = com.process(frame)

    assert obs_sem.bands_valid == obs_com.bands_valid
    assert abs(obs_sem.confidence - obs_com.confidence) < 0.02


def test_heading_guardado_expira_depois_de_perder_a_linha():
    """Se a linha se perde com heading grande, o valor guardado nao pode
    continuar afrouxando o criterio de largura indefinidamente."""
    detector = LineDetector(make_config(rotate_180=False, track_memory=3))
    for _ in range(4):
        detector.process(make_frame(slope=-0.55))
    assert abs(detector._last_heading) > math.radians(10)

    vazio = np.full((HEIGHT, WIDTH, 3), 90, dtype=np.uint8)
    for _ in range(6):
        detector.process(vazio)

    assert detector._miss_count > detector.config.track_memory


def test_curvatura_pode_ser_desligada():
    """Montagem apontada para baixo: a base de profundidade e curta
    demais para medir curvatura. MEDIDO em 10/09/2026, mesma cena: o
    detector deu +10.10 1/m e o ajuste direto na imagem deu -15.61 1/m.
    Sinais opostos -- o sinal esta abaixo do erro. Melhor nao publicar
    do que publicar ruido que vira giro no feedforward."""
    frame = np.full((HEIGHT, WIDTH, 3), 60, dtype=np.uint8)
    for y in range(HEIGHT):
        # linha levemente curva
        cx = WIDTH // 2 + int(0.004 * (y - HEIGHT) ** 2)
        frame[y, max(0, cx - 12):cx + 12] = 235

    config = DetectorConfig(curvature_enabled=True)
    ligado = LineDetector(config).process(frame)
    config_off = DetectorConfig(curvature_enabled=False)
    desligado = LineDetector(config_off).process(frame)

    assert ligado.valid and desligado.valid
    # a posicao continua sendo medida normalmente
    assert desligado.lateral_error == pytest.approx(
        ligado.lateral_error, abs=1e-9
    )
    assert desligado.heading_valid == ligado.heading_valid
    # so a curvatura some
    assert desligado.curvature_valid is False
    assert desligado.curvature == pytest.approx(0.0)


def _quadro_com_linha(pontos):
    """Desenha uma linha branca passando pelos pontos (y, cx) dados."""
    frame = np.full((HEIGHT, WIDTH, 3), 60, dtype=np.uint8)
    ys = [p[0] for p in pontos]
    xs = [p[1] for p in pontos]
    for y in range(HEIGHT):
        cx = int(np.interp(y, ys, xs))
        frame[y, max(0, cx - 12):cx + 12] = 235
    return frame


def test_residuo_e_quase_zero_em_reta_mesmo_com_o_robo_torto():
    """O caso que o heading sozinho NAO distingue.

    Robo torto numa reta da heading alto, mas as bandas ficam todas
    sobre a mesma reta -- residuo ~zero. Se este teste falhar, o freio
    de quebra vai frear a toa so por desalinhamento.
    """
    # reta inclinada: cx varia linearmente com y
    frame = _quadro_com_linha([(0, WIDTH // 2 - 40), (HEIGHT - 1, WIDTH // 2 + 40)])
    obs = LineDetector(DetectorConfig()).process(frame)

    assert obs.valid and obs.residual_valid
    assert abs(obs.heading_error) > math.radians(5.0)   # esta torto mesmo
    assert obs.fit_residual < 0.002                     # mas e reta


def test_residuo_sobe_numa_quebra():
    """Duas retas anguladas: metade das bandas de cada lado da reta."""
    meio = HEIGHT // 2
    frame = _quadro_com_linha([
        (0, WIDTH // 2 - 60), (meio, WIDTH // 2), (HEIGHT - 1, WIDTH // 2 - 60),
    ])
    obs = LineDetector(DetectorConfig()).process(frame)

    assert obs.valid and obs.residual_valid
    # 0.0033 medido nesta geometria sintetica; o limiar real da pista
    # se calibra com o robo sobre uma quebra de verdade.
    assert obs.fit_residual > 0.003


def test_quebra_da_residuo_maior_que_reta_torta():
    """A comparacao que o freio usa."""
    torta = _quadro_com_linha(
        [(0, WIDTH // 2 - 40), (HEIGHT - 1, WIDTH // 2 + 40)]
    )
    meio = HEIGHT // 2
    quebra = _quadro_com_linha([
        (0, WIDTH // 2 - 60), (meio, WIDTH // 2), (HEIGHT - 1, WIDTH // 2 - 60),
    ])
    r_torta = LineDetector(DetectorConfig()).process(torta)
    r_quebra = LineDetector(DetectorConfig()).process(quebra)

    assert r_quebra.fit_residual > 3.0 * r_torta.fit_residual


def test_residuo_invalido_com_menos_de_tres_bandas():
    """Com 2 pontos a reta passa exata: o residuo seria sempre zero e
    daria a falsa impressao de 'reta confirmada'."""
    config = DetectorConfig(bands=2)
    frame = _quadro_com_linha(
        [(0, WIDTH // 2), (HEIGHT - 1, WIDTH // 2)]
    )
    obs = LineDetector(config).process(frame)
    assert obs.residual_valid is False


# --- achatamento de fundo (reflexo / gradiente de iluminacao) ---------
#
# MEDIDO na pista em 10/09/2026: a camera inferior pega um reflexo e a
# mediana do piso cai de 151 (longe) para 106 (perto) -- 45 contagens,
# mais 19 de inclinacao lateral, contra 78 de contraste linha/piso. O
# Otsu que a banda distante pedia era 168 e o da proxima 131; o global
# saiu 155 e fundiu a linha com o piso vizinho na banda distante, que
# passou a medir 39.9mm no lugar de 19.9mm e foi rejeitada por largura.


def make_frame_com_gradiente(topo: int, base: int, linha: int) -> np.ndarray:
    """Linha clara sobre fundo em RAMPA, como o reflexo da pista real."""
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    esquerda = WIDTH // 2 - LINE_THICKNESS // 2
    direita = WIDTH // 2 + LINE_THICKNESS // 2
    for row in range(HEIGHT):
        fundo = topo + (base - topo) * row / (HEIGHT - 1)
        frame[row, :] = int(round(fundo))
        frame[row, esquerda:direita] = linha
    return frame


def test_gradiente_forte_quebra_o_limiar_unico():
    """Documenta a falha que motivou o achatamento.

    Sem achatar, a rampa de fundo faz o Otsu global servir so metade da
    ROI: a banda do extremo claro se funde com o piso e cai.
    """
    frame = make_frame_com_gradiente(170, 95, 200)
    obs = LineDetector(make_config(background_kernel_ratio=0.0)).process(frame)
    assert obs.bands_valid < obs.bands_total


def test_achatamento_recupera_as_bandas_perdidas_no_gradiente():
    frame = make_frame_com_gradiente(170, 95, 200)
    obs = LineDetector(make_config(background_kernel_ratio=3.0)).process(frame)
    assert obs.bands_valid == obs.bands_total
    assert obs.confidence > 0.70
    # A largura tem de continuar sendo a real: o top-hat nao pode
    # emagrecer a linha para "consertar" a banda.
    assert obs.line_width == pytest.approx(0.020, abs=0.002)


def test_achatamento_nao_muda_a_medida_sem_gradiente():
    """Ligar a correcao nao pode custar nada onde nao havia problema."""
    frame = make_frame(offset_px=30)
    sem = LineDetector(make_config(background_kernel_ratio=0.0)).process(frame)
    com = LineDetector(make_config(background_kernel_ratio=3.0)).process(frame)
    assert com.lateral_error == pytest.approx(sem.lateral_error, abs=1e-4)
    assert com.bands_valid == sem.bands_valid


@pytest.mark.parametrize('graus', [0.0, 30.0, 45.0, 60.0])
def test_achatamento_nao_come_a_linha_inclinada(graus):
    """A janela tem de superar a linha mais larga que a varredura ve.

    Uma linha inclinada em theta mede largura/cos(theta) na horizontal --
    o DOBRO a 60 graus. Se a janela do top-hat nao cobrisse isso, a
    correcao apagaria a linha justamente na curva.
    """
    frame = make_frame(slope=math.tan(math.radians(graus)))
    sem = LineDetector(make_config(background_kernel_ratio=0.0)).process(frame)
    com = LineDetector(make_config(background_kernel_ratio=3.0)).process(frame)
    assert com.bands_valid >= sem.bands_valid


def test_janela_estreita_demais_e_ignorada():
    """Guarda: janela menor que a linha comeria a linha, entao nao roda."""
    detector = LineDetector(make_config(background_kernel_ratio=0.5))
    gray = np.full((40, 160), 120, dtype=np.uint8)
    gray[:, 70:90] = 200
    assert np.array_equal(detector._achata_fundo(gray), gray)


def test_achatamento_desligado_devolve_a_imagem_original():
    detector = LineDetector(make_config(background_kernel_ratio=0.0))
    gray = np.full((40, 160), 120, dtype=np.uint8)
    gray[:, 70:90] = 200
    assert np.array_equal(detector._achata_fundo(gray), gray)


def test_achatamento_zera_o_fundo_e_preserva_a_linha():
    detector = LineDetector(make_config(background_kernel_ratio=3.0))
    gray = np.full((40, 160), 120, dtype=np.uint8)
    gray[:, 70:90] = 200
    plano = detector._achata_fundo(gray)
    assert plano[:, :50].mean() < 1.0          # piso vira zero
    assert plano[:, 75:85].mean() > 50.0       # a linha sobrevive
