#!/usr/bin/env python3
"""Deteccao geometrica de linha branca em piso cinza/escuro.

Modulo puro: nao importa rclpy. Recebe um frame BGR e devolve erro lateral
em metros, orientacao em radianos e curvatura em 1/m, mais uma confianca
honesta (zero quando nao ha deteccao, reduzida quando ha pouca evidencia).

Por que nao "maior contorno + centroide":
  - findContours percorre a ROI inteira e o maior contorno e trivialmente
    ganho por um reflexo grande no piso;
  - o centroide de um contorno em curva fica entre os dois ramos da linha,
    num ponto onde a linha nao esta;
  - nao ha nada no resultado que diga se a deteccao e plausivel.

O que este modulo faz no lugar:
  1. Recorta a ROI e reduz para uma largura de trabalho fixa (CPU do Pi).
  2. Binariza com Otsu, que se adapta a iluminacao, e valida o resultado
     com dois guardas: contraste minimo e fracao maxima de area branca.
  3. Amostra N bandas horizontais. Em cada banda, acha os segmentos
     contiguos ("runs") de pixels claros -- O(largura), nao O(area).
  4. Pontua cada run por largura plausivel e por continuidade com o frame
     anterior, e escolhe um por banda.
  5. Converte cada ponto para o chao usando 4 medidas de regua por camera
     (profundidade e largura visiveis) e ajusta um polinomio para extrair
     erro lateral, orientacao e curvatura em unidades fisicas.

Convencao de sinais (unica em todo o sistema):
  +x  = direita do robo   -> lateral_error > 0 significa linha a direita
  +heading  = a linha se afasta inclinando para a direita
  +curvature = a pista curva para a direita
A traducao para ROS (wz > 0 = girar a esquerda) e feita uma unica vez,
em line_controller.py.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class CameraGeometry:
    """Geometria da camera medida com regua, sem calibracao intrinseca.

    A ROI e tratada como um trapezio no chao: profundidade conhecida e
    largura que cresce com a distancia. Isso e uma homografia simplificada
    -- suficiente para dar unidade fisica aos ganhos e muito mais barato
    de calibrar do que um chessboard.

    depth_near_m / depth_far_m: distancia a frente do robo das bordas
    inferior e superior da ROI. Para a camera de baixo isso ja e conhecido
    (comeca a enxergar a 3.5 cm, alcance de 10 cm -> 0.035 e 0.135).

    width_near_m / width_far_m: largura de chao coberta pela imagem inteira
    nessas duas distancias. ESTAS SAO AS DUAS MEDIDAS QUE PRECISAM DE
    REGUA. Ver o procedimento em config/line_follower.yaml.
    """

    depth_near_m: float = 0.035
    depth_far_m: float = 0.135
    width_near_m: float = 0.090
    width_far_m: float = 0.160
    # Deslocamento lateral da camera em relacao ao eixo do robo. Positivo
    # se a camera esta montada a direita do centro.
    camera_x_offset_m: float = 0.0

    def depth_at(self, y_ratio: float) -> float:
        """y_ratio 0 = topo da ROI (mais longe), 1 = base (mais perto)."""
        return self.depth_far_m + (self.depth_near_m - self.depth_far_m) * y_ratio

    def width_at(self, y_ratio: float) -> float:
        return self.width_far_m + (self.width_near_m - self.width_far_m) * y_ratio

    def to_ground_x(self, x_norm: float, y_ratio: float) -> float:
        """x_norm em [-1, 1] (borda esquerda a direita) -> metros."""
        return x_norm * 0.5 * self.width_at(y_ratio) - self.camera_x_offset_m


@dataclass
class DetectorConfig:
    """Parametros da deteccao. Os defaults sao para a camera inferior."""

    # Recorte vertical da imagem usado como ROI.
    roi_top: float = 0.30
    roi_bottom: float = 1.00
    # Camera montada de cabeca para baixo. VERIFICADO NA BANCADA: as duas
    # cameras deste robo estao rotacionadas 180 graus.
    # Sem corrigir, "mais longe" e "mais perto" trocam de lugar (o
    # lookahead aponta para tras) e o sinal do erro lateral inverte.
    rotate_180: bool = True
    # Peso do teste de "linha ladeada por escuro". A pista real e uma
    # linha branca com fita preta dos dois lados sobre piso claro; exigir
    # que o candidato seja mais claro que os dois vizinhos e o que separa
    # a linha do piso, que tambem e claro.
    flank_weight: float = 0.7
    # A varredura mede a linha na HORIZONTAL. Se a linha esta inclinada
    # em theta (curva, ou robo desalinhado), a largura medida cresce por
    # 1/cos(theta) -- geometria pura, nao ruido. MEDIDO NA PISTA: numa
    # curva com theta=60deg, a fita de 20mm mediu 40.0mm (fator 2.00,
    # exatamente 1/cos(60)), e o score da banda caiu de ~0.95 para 0.34.
    # Sem compensar isso, o detector penaliza a linha por "larga demais"
    # justamente na curva, que e quando ele mais precisa acertar.
    # O limite evita explodir perto de 90 graus (a 71deg o fator ja e 3x).
    max_tilt_width_factor: float = 3.0
    # Largura de trabalho apos o downscale. 160 px mantem a linha com
    # varios pixels de largura e derruba o custo de CPU ~16x em 640x480.
    work_width: int = 160
    # Numero de bandas horizontais amostradas dentro da ROI.
    bands: int = 5
    # Altura de cada banda, em fracao da altura da ROI.
    band_height: float = 0.10
    # Largura esperada da linha no chao, em metros, e tolerancia relativa.
    # Serve para rejeitar reflexos estreitos e faixas de piso claro largas.
    line_width_m: float = 0.020
    line_width_tolerance: float = 0.75
    # Guardas de binarizacao.
    min_contrast: float = 25.0        # separacao minima linha/fundo (0-255)
    max_white_fraction: float = 0.45  # acima disso a ROI e piso claro, nao linha
    # Continuidade temporal: peso da predicao e largura da janela de busca
    # (em fracao da largura da ROI) no primeiro frame apos uma deteccao.
    track_weight: float = 0.6
    track_sigma: float = 0.25
    # Quantos frames a predicao continua valendo depois de uma falha,
    # alargando a janela a cada frame perdido.
    track_memory: int = 6
    # Score minimo para aceitar um run como pertencente a linha.
    min_band_score: float = 0.25
    # Abertura morfologica (px na imagem de trabalho). 0 desliga.
    open_kernel: int = 3
    # ACHATAMENTO DE FUNDO antes do limiar. 0 desliga.
    #
    # POR QUE EXISTE (10/09/2026): a camera inferior pega um reflexo de
    # luz e o piso fica com um GRADIENTE de iluminacao. MEDIDO no quadro
    # capturado: a mediana do piso cai de 151 (longe) para 106 (perto),
    # 45 contagens, mais 19 de inclinacao lateral -- contra apenas 78 de
    # contraste entre a linha e o piso. Um limiar de Otsu unico nao serve
    # os dois extremos: o Otsu que a banda mais distante pediria era 168
    # e o da mais proxima 131, e o global saiu em 155. Resultado, o piso
    # ao lado da linha na banda distante (151) encostava no limiar e se
    # fundia com ela: a banda 0 media 39.9mm em vez de 19.9mm, o termo de
    # largura a matava e a deteccao caia para 4/5 bandas.
    #
    # A correcao e um top-hat com kernel HORIZONTAL: para cada linha da
    # imagem, o fundo e o minimo local numa janela mais larga que a fita,
    # e subtrai-lo remove qualquer iluminacao suave -- vertical de vez, e
    # lateral na proporcao da janela. MEDIDO: as cinco bandas voltaram a
    # medir 19.9mm e o branco espurio em piso vazio caiu de 34.3% p/ 12.5%.
    #
    # O limiar passa a sair da imagem achatada, mas o CONTRASTE e o TESTE
    # DE FLANCO continuam lendo o gray original -- eles medem fisica da
    # cena (linha branca ladeada de fita preta), nao artefato de limiar.
    #
    # O valor e razao sobre a largura ESPERADA da linha em pixels. Tem de
    # superar a linha mais larga que a varredura horizontal pode ver: uma
    # linha inclinada em theta mede largura/cos(theta), entao a 60 graus
    # ja e o DOBRO. 3.0 deixa 50% de folga sobre esse caso -- estreitar
    # demais faz o top-hat comer a linha justamente na curva.
    background_kernel_ratio: float = 3.0
    # Minimo de bandas para tentar ajuste quadratico (curvatura).
    # Curvatura so faz sentido se a BASE de profundidade for longa o
    # bastante. A incerteza vai com 8*sigma/L^2: com a montagem apontada
    # para baixo (L = 22.5mm) e sigma = 0.04mm, isso da 0.63 1/m de ruido,
    # e efeitos de borda levam a varios 1/m -- enquanto uma curva real de
    # 50cm de raio vale 2 1/m. MEDIDO em 10/09/2026 na mesma cena: o
    # detector reportou +10.10 1/m e o ajuste direto na imagem deu
    # -15.61 1/m. Sinais OPOSTOS: o sinal esta abaixo do erro.
    # Com False, curvature_valid nunca sobe e o feedforward de curvatura
    # simplesmente nao age -- em vez de agir sobre ruido.
    curvature_enabled: bool = True
    min_bands_for_curvature: int = 4
    # Minimo de bandas para o HEADING valer. A curvatura sempre teve
    # essa guarda; o heading nao tinha, e bastavam 2 bandas.
    #
    # POR QUE EXISTE (22/09/2026): uma reta por 2 pontos nao tem
    # redundancia -- se uma banda escorrega, o angulo gira sem limite. E
    # numa QUEBRA as bandas que sobrevivem costumam cair em ramos
    # DIFERENTES da curva, o que da um heading que nao aponta para lugar
    # nenhum. Capturado em corrida: na primeira quebra o heading foi de
    # -11.7 para +29.8 graus em 0.6 s, com 2 bandas e confianca 0.09, e
    # o controlador esterçou a fundo (wz saturado em -0.900) em cima
    # disso. O robo saiu para a esquerda, como o operador relatou.
    #
    # Com 3 bandas ha um grau de redundancia, e o residuo do ajuste
    # (fit_residual) passa a significar alguma coisa.
    #
    # Quando o heading fica invalido o controlador degrada para esterçar
    # so pelo erro LATERAL -- que com 2 bandas ainda e uma posicao
    # honesta, ao contrario de uma inclinacao.
    min_bands_for_heading: int = 3
    # Limite fisico de curvatura aceito; acima disso o ajuste e ruido.
    max_curvature: float = 12.0
    # Acima deste heading, a curvatura ajustada deixa de ser confiavel:
    # com o robo desalinhado, a projecao das bandas no referencial dele
    # (girado em relacao a linha real) faz ate uma linha RETA parecer
    # curva. Medido no robo: um heading de 23 graus (so desalinhamento
    # de partida, sem curva nenhuma na pista) produziu leituras de
    # curvatura de +7 a +10 1/m. Acima deste limite, curvature_valid
    # fica False -- e a curvatura nao entra em severidade nem feedforward.
    max_heading_for_curvature: float = 0.30  # rad, ~17 graus
    geometry: CameraGeometry = field(default_factory=CameraGeometry)


@dataclass
class BandSample:
    y_ratio: float
    x_norm: float
    width_norm: float
    score: float
    depth_m: float
    x_m: float
    width_m: float


@dataclass
class LineObservation:
    valid: bool = False
    confidence: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    curvature: float = 0.0
    # Residuo RMS do ajuste de reta -- ver o comentario em _fit().
    fit_residual: float = 0.0
    residual_valid: bool = False
    lookahead_distance: float = 0.0
    line_width: float = 0.0
    bands_valid: int = 0
    bands_total: int = 0
    heading_valid: bool = False
    curvature_valid: bool = False
    contrast: float = 0.0
    processing_time: float = 0.0
    samples: list = field(default_factory=list)
    mask: np.ndarray | None = None
    reject_reason: str = ''


def _runs(flags: np.ndarray) -> list[tuple[int, int]]:
    """Segmentos contiguos de True em um vetor booleano.

    Retorna pares [inicio, fim) em indices de coluna. Implementado com
    np.diff sobre a versao inteira do vetor: um unico passo em C, sem
    laco Python sobre pixels.
    """
    if flags.size == 0:
        return []
    padded = np.concatenate(([0], flags.view(np.int8), [0]))
    edges = np.diff(padded)
    starts = np.flatnonzero(edges > 0)
    ends = np.flatnonzero(edges < 0)
    return list(zip(starts.tolist(), ends.tolist()))


class LineDetector:
    """Detector com estado (continuidade temporal entre frames)."""

    def __init__(self, config: DetectorConfig):
        self.config = config
        self._kernel: np.ndarray | None = None
        self._kernel_size = -1
        # Cache do kernel do achatamento de fundo (ver
        # background_kernel_ratio): a janela so muda se a
        # geometria ou a resolucao de trabalho mudarem.
        self._fundo_kernel = None
        self._fundo_janela = -1
        self.reset()

    def reset(self) -> None:
        """Esquece a linha anterior. Chamado ao habilitar a autonomia."""
        self._last_fit: np.ndarray | None = None
        self._last_heading = 0.0
        self._miss_count = 0

    # ------------------------------------------------------------------
    # Predicao de continuidade
    # ------------------------------------------------------------------
    def _predict(self, y_ratio: float) -> float | None:
        """x_norm esperado nesta banda, a partir do ajuste do frame anterior."""
        if self._last_fit is None or self._miss_count > self.config.track_memory:
            return None
        return float(np.polyval(self._last_fit, y_ratio))

    def _search_sigma(self) -> float:
        """Janela de busca, que alarga a cada frame sem deteccao."""
        return self.config.track_sigma * (1.0 + 0.5 * self._miss_count)

    # ------------------------------------------------------------------
    # Pipeline
    # ------------------------------------------------------------------
    def _achata_fundo(self, gray: np.ndarray) -> np.ndarray:
        """Remove a iluminacao de fundo, preservando a linha.

        Top-hat com kernel horizontal: o fundo de cada linha da imagem e
        o minimo local numa janela mais larga que a fita, entao a fita
        (estreita) sobrevive e qualquer rampa suave some. Devolve o
        proprio gray quando desligado ou quando a janela nao cabe.
        """
        cfg = self.config
        if cfg.background_kernel_ratio <= 0.0:
            return gray

        # Largura esperada da linha, em pixels da imagem de trabalho, no
        # meio da ROI. Vem da geometria, entao a mesma razao vale para as
        # duas cameras mesmo com campos de visao muito diferentes.
        largura_m = max(cfg.geometry.width_at(0.5), 1e-3)
        linha_px = (cfg.line_width_m / largura_m) * gray.shape[1]
        janela = int(round(cfg.background_kernel_ratio * linha_px)) | 1
        # Precisa caber na imagem e ser mais larga que a linha; fora
        # disso o top-hat comeria a propria linha.
        if janela >= gray.shape[1] or janela <= linha_px:
            return gray

        if self._fundo_janela != janela:
            self._fundo_kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT, (janela, 1)
            )
            self._fundo_janela = janela
        return cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, self._fundo_kernel)

    def _binarize(self, roi_bgr: np.ndarray):
        """ROI colorida -> (mascara binaria, contraste, motivo de rejeicao)."""
        cfg = self.config

        height, width = roi_bgr.shape[:2]
        scale = cfg.work_width / float(width)
        work_height = max(4, int(round(height * scale)))
        # INTER_AREA ja faz a media dos pixels descartados: substitui o
        # blur gaussiano que o codigo antigo aplicava depois, de graca.
        small = cv2.resize(
            np.ascontiguousarray(roi_bgr),
            (cfg.work_width, work_height),
            interpolation=cv2.INTER_AREA,
        )
        if small.ndim == 3:
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        else:
            gray = small

        # Achata o fundo ANTES do limiar. Sem isso um gradiente de
        # iluminacao maior que meio contraste faz um limiar unico servir
        # so metade da ROI (ver background_kernel_ratio).
        para_limiar = self._achata_fundo(gray)

        # Otsu escolhe o limiar a partir do histograma do proprio frame,
        # entao acompanha sombra e mudanca de lampada sem reajuste manual.
        threshold, mask = cv2.threshold(
            para_limiar, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        foreground = mask > 0
        white_fraction = float(np.count_nonzero(foreground)) / foreground.size
        if white_fraction < 1e-4 or white_fraction > cfg.max_white_fraction:
            # Nada claro, ou claro demais para ser uma linha: Otsu sempre
            # parte o histograma em dois, mesmo numa parede lisa, entao
            # este guarda e o que impede uma "linha" inventada.
            return gray, mask, 0.0, 'area'

        contrast = float(gray[foreground].mean() - gray[~foreground].mean())
        if contrast < cfg.min_contrast:
            return gray, mask, contrast, 'contraste'

        if cfg.open_kernel > 1:
            if self._kernel_size != cfg.open_kernel:
                size = cfg.open_kernel | 1
                self._kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (size, size)
                )
                self._kernel_size = cfg.open_kernel
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

        return gray, mask, contrast, ''

    def _flank_term(
        self, profile: np.ndarray, start: int, end: int,
        flank_px: int, contrast: float
    ) -> float:
        """Quanto o candidato e mais claro que os dois vizinhos imediatos.

        A pista real e uma linha branca com fita preta dos dois lados
        sobre piso claro. Sem este teste, um pedaco de piso e tao "branco"
        quanto a linha e ganha a disputa sempre que aparecer com largura
        parecida. Exigir queda de brilho NOS DOIS LADOS tambem descarta
        automaticamente regioes que encostam na borda da imagem.
        """
        interior = float(profile[start:end].mean())
        scale = max(contrast, 1.0)

        def drop(neighbour: np.ndarray) -> float:
            if neighbour.size == 0:
                # Encostou na borda do quadro: nao ha como confirmar que a
                # regiao termina ali. Penaliza sem eliminar.
                return 0.0
            return float(
                np.clip((interior - neighbour.mean()) / scale, 0.0, 1.0)
            )

        left = drop(profile[max(0, start - flank_px):start])
        right = drop(profile[end:min(profile.size, end + flank_px)])
        raw = (0.25 + 0.75 * left) * (0.25 + 0.75 * right)
        return (1.0 - self.config.flank_weight) + self.config.flank_weight * raw

    def _sample_bands(
        self, mask: np.ndarray, gray: np.ndarray, contrast: float
    ) -> list[BandSample]:
        cfg = self.config
        geometry = cfg.geometry
        rows, cols = mask.shape
        half_band = max(1, int(round(rows * cfg.band_height * 0.5)))

        ideal_width_norm_ref = cfg.line_width_m
        # Fator de inclinacao, do ultimo heading valido. Usar o frame
        # anterior evita uma segunda passada de deteccao: a inclinacao
        # muda devagar comparada a taxa de quadros.
        # Se a linha se perdeu, o heading guardado envelhece junto com a
        # predicao: seguir esperando uma linha 39% mais larga (heading de
        # 44 graus) depois de voltar para a reta penalizaria a deteccao
        # correta. Mesma regra de validade usada em _predict().
        heading_ref = (
            self._last_heading
            if self._miss_count <= cfg.track_memory
            else 0.0
        )
        tilt = 1.0 / max(
            math.cos(heading_ref), 1.0 / cfg.max_tilt_width_factor
        )
        ideal_width_ref_tilted = ideal_width_norm_ref * tilt
        samples: list[BandSample] = []
        sigma = self._search_sigma()

        for index in range(cfg.bands):
            y_ratio = (index + 0.5) / cfg.bands
            row = int(round(y_ratio * (rows - 1)))
            row0 = max(0, row - half_band)
            row1 = min(rows, row + half_band + 1)

            band = mask[row0:row1] > 0
            # Voto de maioria na vertical: um pixel ruidoso isolado nao
            # sobrevive, mas a linha (presente em todas as sublinhas) sim.
            votes = band.sum(axis=0)
            on = votes >= max(1, (row1 - row0 + 1) // 2)

            candidates = _runs(on)
            if not candidates:
                continue

            # Perfil de brilho da banda, calculado uma vez e reaproveitado
            # por todos os candidatos dela.
            profile = gray[row0:row1].mean(axis=0)

            width_at = geometry.width_at(y_ratio)
            # Largura esperada da linha nesta banda, em fracao da imagem,
            # JA COMPENSADA pela inclinacao do frame anterior: uma linha
            # inclinada em theta atravessa a banda na diagonal e mede
            # largura_real/cos(theta) na varredura horizontal.
            ideal_width_norm = (
                ideal_width_ref_tilted / max(width_at, 1e-3)
            )
            flank_px = max(2, int(round(ideal_width_norm * cols * 0.6)))
            prediction = self._predict(y_ratio)

            best: BandSample | None = None
            for start, end in candidates:
                width_norm = (end - start) / float(cols)
                center = 0.5 * (start + end) / float(cols)
                x_norm = center * 2.0 - 1.0

                # Plausibilidade de largura. Gaussiana em log garante que
                # "metade" e "o dobro" da largura ideal sao penalizados
                # igualmente -- em pixels a escala e multiplicativa.
                ratio = width_norm / max(ideal_width_norm, 1e-6)
                width_term = float(
                    np.exp(
                        -(np.log(max(ratio, 1e-6)) ** 2)
                        / (2.0 * max(cfg.line_width_tolerance, 1e-3) ** 2)
                    )
                )

                if prediction is None:
                    track_term = 1.0
                else:
                    track_term = float(
                        np.exp(-((x_norm - prediction) ** 2) / (2.0 * sigma ** 2))
                    )

                flank_term = self._flank_term(
                    profile, start, end, flank_px, contrast
                )

                # Multiplicativo, nao aditivo: um run com largura absurda
                # ou sem borda escura e rejeitado por mais bem posicionado
                # que esteja.
                score = width_term * flank_term * (
                    cfg.track_weight * track_term + (1.0 - cfg.track_weight)
                )

                if best is None or score > best.score:
                    best = BandSample(
                        y_ratio=y_ratio,
                        x_norm=x_norm,
                        width_norm=width_norm,
                        score=score,
                        depth_m=geometry.depth_at(y_ratio),
                        x_m=geometry.to_ground_x(x_norm, y_ratio),
                        width_m=width_norm * width_at,
                    )

            if best is not None and best.score >= cfg.min_band_score:
                samples.append(best)

        return samples

    def _fit(self, samples: list[BandSample], observation: LineObservation) -> None:
        """Ajusta x(d) e extrai erro, orientacao e curvatura."""
        cfg = self.config
        depths = np.array([s.depth_m for s in samples], dtype=np.float64)
        xs = np.array([s.x_m for s in samples], dtype=np.float64)
        weights = np.array([s.score for s in samples], dtype=np.float64)

        # A banda mais proxima e a de menor profundidade: e ela que ancora
        # o alinhamento atual. Nunca extrapolamos para uma banda ausente.
        nearest = int(np.argmin(depths))
        observation.lateral_error = float(xs[nearest])
        observation.lookahead_distance = float(depths[nearest])
        observation.line_width = float(
            np.median([s.width_m for s in samples])
        )

        if len(samples) < max(2, cfg.min_bands_for_heading):
            # Uma banda so: da para saber onde a linha esta, nao para onde
            # ela vai. Publicar heading aqui seria inventar informacao.
            observation.heading_valid = False
            observation.curvature_valid = False
            return

        linear = np.polyfit(depths, xs, 1, w=weights)
        observation.heading_error = float(np.arctan(linear[0]))
        observation.heading_valid = True

        # QUEBRA DE RETA: o quanto as bandas fogem da reta ajustada.
        # Numa reta -- inclusive com o robo TORTO -- as bandas ficam
        # todas sobre a mesma reta e o residuo e ~zero. Numa quebra
        # (reta -> reta anguladas, sem arco entre elas) metade das
        # bandas fica de cada lado e o residuo salta. E o sinal que o
        # heading sozinho nao da: ele sobe nos DOIS casos.
        # Precisa de 3+ bandas: com 2 a reta passa exata por definicao.
        if len(samples) >= 3:
            previsto = np.polyval(linear, depths)
            peso = np.maximum(weights, 1e-9)
            residuo = float(
                np.sqrt(np.sum(peso * (xs - previsto) ** 2) / np.sum(peso))
            )
            observation.fit_residual = residuo
            observation.residual_valid = True

        heading_trustworthy = (
            abs(observation.heading_error) <= cfg.max_heading_for_curvature
        )
        if (
            cfg.curvature_enabled
            and heading_trustworthy
            and len(samples) >= max(3, cfg.min_bands_for_curvature)
        ):
            quad = np.polyfit(depths, xs, 2, w=weights)
            slope = 2.0 * quad[0] * depths[nearest] + quad[1]
            curvature = 2.0 * quad[0] / (1.0 + slope * slope) ** 1.5
            if abs(curvature) <= cfg.max_curvature:
                observation.curvature = float(curvature)
                observation.curvature_valid = True

    def _confidence(
        self, samples: list[BandSample], observation: LineObservation
    ) -> float:
        cfg = self.config
        # Cobertura: quantas bandas viram a linha.
        coverage = len(samples) / float(cfg.bands)
        # Qualidade media dos runs escolhidos (largura + continuidade).
        quality = float(np.mean([s.score for s in samples]))
        # Contraste: satura em 3x o minimo exigido.
        contrast_term = min(
            1.0, observation.contrast / max(3.0 * cfg.min_contrast, 1e-3)
        )
        # Continuidade com o passado recente.
        continuity = 1.0 / (1.0 + 0.5 * self._miss_count)
        # A banda mais proxima e a que o controlador usa como erro. Sem ela
        # o seguimento e cego onde mais importa.
        has_nearest = any(
            s.y_ratio > (cfg.bands - 1.0) / cfg.bands for s in samples
        )
        nearest_term = 1.0 if has_nearest else 0.5

        value = coverage * quality * contrast_term * continuity * nearest_term
        return float(min(1.0, max(0.0, value)))

    def process(self, frame_bgr: np.ndarray) -> LineObservation:
        """Processa um frame e devolve a observacao geometrica."""
        started = time.perf_counter()
        cfg = self.config
        observation = LineObservation(bands_total=cfg.bands)

        if cfg.rotate_180:
            # Vista invertida: nao ha copia aqui, so strides negativos.
            # A copia acontece uma vez so, no resize da ROI.
            frame_bgr = frame_bgr[::-1, ::-1]

        height = frame_bgr.shape[0]
        top = int(round(height * cfg.roi_top))
        bottom = int(round(height * cfg.roi_bottom))
        top = max(0, min(height - 2, top))
        bottom = max(top + 2, min(height, bottom))
        roi = frame_bgr[top:bottom]

        gray, mask, contrast, reason = self._binarize(roi)
        observation.contrast = contrast
        observation.mask = mask

        if reason:
            observation.reject_reason = reason
            self._miss_count += 1
            observation.processing_time = time.perf_counter() - started
            return observation

        samples = self._sample_bands(mask, gray, contrast)
        observation.samples = samples
        observation.bands_valid = len(samples)

        if not samples:
            observation.reject_reason = 'sem bandas'
            self._miss_count += 1
            observation.processing_time = time.perf_counter() - started
            return observation

        self._fit(samples, observation)
        observation.valid = True
        observation.confidence = self._confidence(samples, observation)

        # Atualiza a predicao para o proximo frame, em coordenadas de
        # imagem (y_ratio, x_norm), que e onde a busca acontece.
        if len(samples) >= 2:
            self._last_fit = np.polyfit(
                [s.y_ratio for s in samples],
                [s.x_norm for s in samples],
                1,
            )
        else:
            self._last_fit = np.array([0.0, samples[0].x_norm])
        if observation.heading_valid:
            self._last_heading = observation.heading_error
        self._miss_count = 0

        observation.processing_time = time.perf_counter() - started
        return observation


def draw_debug(
    frame_bgr: np.ndarray,
    observation: LineObservation,
    config: DetectorConfig,
    label: str = 'bottom',
) -> np.ndarray:
    """Sobrepoe a deteccao no frame original. So chamado quando alguem
    esta inscrito no topico de debug -- desenhar custa CPU."""
    if config.rotate_180:
        # O debug precisa mostrar o que o detector viu, nao o que o
        # sensor entregou -- senao os pontos aparecem espelhados.
        frame_bgr = frame_bgr[::-1, ::-1]
    display = np.ascontiguousarray(frame_bgr)
    height, width = display.shape[:2]
    top = int(round(height * config.roi_top))
    bottom = int(round(height * config.roi_bottom))
    roi_height = max(1, bottom - top)

    cv2.rectangle(display, (0, top), (width - 1, bottom - 1), (0, 255, 255), 1)
    center_x = int(
        round(
            (0.5 + config.geometry.camera_x_offset_m
             / max(config.geometry.width_near_m, 1e-3)) * width
        )
    )
    cv2.line(display, (center_x, top), (center_x, bottom), (255, 0, 0), 1)

    for sample in observation.samples:
        px = int(round((sample.x_norm * 0.5 + 0.5) * width))
        py = int(round(top + sample.y_ratio * roi_height))
        colour = (0, 200, 0) if sample.score > 0.6 else (0, 165, 255)
        cv2.circle(display, (px, py), 4, colour, -1)
        half = max(1, int(round(sample.width_norm * width * 0.5)))
        cv2.line(display, (px - half, py), (px + half, py), colour, 1)

    if observation.valid:
        text = (
            f'{label} e={observation.lateral_error * 1000:+.0f}mm '
            f'th={np.degrees(observation.heading_error):+.0f}deg '
            f'k={observation.curvature:+.2f} c={observation.confidence:.2f}'
        )
        colour = (0, 255, 0)
    else:
        text = f'{label} SEM LINHA ({observation.reject_reason})'
        colour = (0, 0, 255)

    cv2.putText(
        display, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
        cv2.LINE_AA
    )
    cv2.putText(
        display,
        f'bandas {observation.bands_valid}/{observation.bands_total} '
        f'contraste {observation.contrast:.0f} '
        f'cpu {observation.processing_time * 1000:.1f}ms',
        (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1,
        cv2.LINE_AA
    )
    return display
