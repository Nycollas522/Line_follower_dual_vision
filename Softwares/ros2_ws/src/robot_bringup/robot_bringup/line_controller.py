#!/usr/bin/env python3
"""Lei de controle e maquina de estados do seguidor de linha.

Modulo puro: nao importa rclpy nem cv2. Recebe observacoes ja em unidades
fisicas e devolve um comando de velocidade. Isso permite testar a logica
de seguranca (perda de linha, recuperacao, parada) sem robo e sem camera.

CONVENCAO DE SINAIS -- o ponto onde a versao anterior errava.
  A percepcao usa +x = direita, entao lateral_error > 0 = linha a direita.
  ROS usa angular.z > 0 = giro anti-horario = virar a ESQUERDA.
  Logo, para ir atras de uma linha a direita e preciso wz NEGATIVO.
  A inversao acontece uma unica vez, em _steering(), no sinal de menos.

LEI DE CONTROLE
  wz = -(k_y * e_y + k_theta * e_theta + k_d * d(e_y)/dt) - k_ff * kappa * v
  Nao ha termo integral. O erro lateral nao tem offset permanente a
  compensar (um desalinhamento de camera se corrige em camera_x_offset_m),
  e o integral so acumularia durante curvas longas e durante os instantes
  em que a linha some -- exatamente onde ele estraga a resposta. O PID de
  roda no ESP32 ja cuida do erro de regime de velocidade.

LIMITE CINEMATICO
  O chassi e mecanum 4x4. Com vy = 0, a velocidade de cada roda e
  v_roda = vx +/- K * wz, com K = halfL + halfW = 0.165 m (do firmware).
  Duas restricoes saem dai, e as duas sao aplicadas:
    |wz| <= (v_roda_max - vx) / K   -> nao saturar o driver de roda
    |wz| <= arc_ratio * vx / K      -> nenhuma roda inverte de sentido,
                                       que e o que faz o robo piruetar no
                                       lugar em vez de descrever a curva.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


class State(IntEnum):
    IDLE = 0
    FOLLOWING = 1
    DEGRADED = 2
    COASTING = 3
    RECOVERING = 4
    SAFE_STOP = 5
    ALIGNING = 6


STATE_NAMES = {
    State.IDLE: 'IDLE',
    State.FOLLOWING: 'FOLLOWING',
    State.DEGRADED: 'DEGRADED',
    State.COASTING: 'COASTING',
    State.RECOVERING: 'RECOVERING',
    State.SAFE_STOP: 'SAFE_STOP',
    State.ALIGNING: 'ALIGNING',
}


class HeadMode(IntEnum):
    CENTER = 0
    HOLD = 1
    SCAN = 2
    TRACK = 3  # rastreia a linha (camera superior); ver use_servo_bearing


@dataclass
class ControllerConfig:
    # --- velocidade -------------------------------------------------
    v_max: float = 0.22           # m/s em reta com linha confiavel
    v_min: float = 0.07           # m/s no pior caso ainda seguindo
    accel: float = 0.35           # m/s^2, rampa de subida
    decel: float = 0.60           # m/s^2, rampa de descida (pode ser maior)

    # --- direcao ----------------------------------------------------
    k_lateral: float = 9.0        # (rad/s) por metro de erro lateral
    k_heading: float = 1.6        # (rad/s) por radiano de orientacao
    k_damping: float = 0.35       # (rad/s) por (m/s) de variacao do erro
    # AGENDAMENTO DO GANHO DE ORIENTACAO POR VELOCIDADE.
    # A dinamica de duas variaveis (e_ponto = v*theta, theta_ponto = wz)
    # com esta lei de controle da:
    #     e_2pontos + (k_heading + k_damping*v)*e_ponto + v*k_lateral*e = 0
    # logo  wn = sqrt(v*k_lateral)  e
    #       zeta = (k_heading + k_damping*v) / (2*sqrt(v*k_lateral)).
    # Com k_heading FIXO o amortecimento cai quando o robo acelera --
    # justamente na reta, que e onde ele acelera:
    #     v=0.07 -> zeta 1.02 (sem sobressinal)
    #     v=0.15 -> zeta 0.71
    #     v=0.22 -> zeta 0.60 (sobressinal ~9.5%)
    # Agendando k_heading = 2*zeta_alvo*sqrt(v*k_lateral) - k_damping*v
    # o amortecimento fica constante em toda a faixa. Nunca desce
    # abaixo de k_heading, entao o comportamento em baixa velocidade
    # (ja bem ajustado) fica identico.
    heading_gain_scheduling: bool = True
    target_damping: float = 1.0   # 1.0 = critico, sem sobressinal
    # PISO do ganho agendado. O agendamento persegue amortecimento
    # constante, e amortecimento so faz sentido com o robo ANDANDO: a
    # dinamica lateral e e_ponto = v*theta, que morre com v. Parado, o
    # robo nao consegue corrigir offset lateral -- so girar.
    #
    # E ai esta a armadilha: com o erro medido a L metros a frente, o
    # controlador pode anular e_medido = e_centro + L*theta GIRANDO, e
    # estaciona em theta = -k_lat*e_centro/(k_lat*L + k_head). Sem piso,
    # o agendamento leva k_head a ZERO em baixa velocidade e o robo fica
    # torto de proposito, com a mira em cima da linha e o corpo nao.
    # MEDIDO NA PISTA (10/09/2026): "andou quase que angulado" -- 10 graus
    # com 20mm de offset, e ele passou 50% do teste com vx=0.003.
    #
    # 1.0 limita esse angulo a ~5 graus para 20mm de offset:
    #   theta = k_lat*e / (k_lat*L + k_head) = 9*0.02/(1.035+1.0) = 0.088 rad
    # e so age abaixo de v ~ 0.13 m/s, onde o agendamento pediria menos.
    # Em velocidade de cruzeiro o piso nao encosta e zeta continua 1.0.
    heading_gain_min: float = 1.0
    # Fracao da curvatura geometrica aplicada como feedforward.
    #
    # POR QUE 1.0 E O VALOR FISICAMENTE CORRETO: para percorrer um arco
    # de curvatura kappa a velocidade v, a cinematica exige exatamente
    # omega = v*kappa. Com k_ff = 1.0 esse termo e fornecido inteiro pelo
    # feedforward e o feedback so precisa corrigir o residuo.
    #
    # COM k_ff < 1.0 O ROBO FICA PARALELO A LINHA, NAO EM CIMA DELA:
    # em regime permanente numa curva, o feedback precisa gerar a parte
    # que falta, e a unica forma de um controlador sem integral gerar
    # sinal e mantendo erro. Sai a conta:
    #     e_regime = (1 - k_ff) * v * kappa / k_lateral
    # Medido na pista: k_ff=0.15, v=0.15, kappa=1.06 (raio 94cm) da
    # 15mm de offset -- numa fita de 20mm, a linha fica na BORDA do
    # eixo do robo. Numa curva de raio 33cm da 42mm, o dobro da fita.
    #
    # O motivo de termos baixado para 0.15 era ruido na curvatura, nao
    # o conceito. A correcao certa e filtrar a curvatura
    # (curvature_filter_tau) e devolver o ganho ao valor fisico.
    k_feedforward: float = 0.90
    # Constante de tempo do filtro passa-baixa da curvatura usada no
    # feedforward. A curvatura crua salta entre frames (baseline curto);
    # a curvatura REAL da pista muda devagar comparada a 50Hz de
    # controle, entao filtrar custa pouca fase e remove o ruido que
    # tornava o feedforward perigoso.
    curvature_filter_tau: float = 0.15
    # ATRASO DE TRANSPORTE DO FEEDFORWARD DE CURVATURA.
    # A curvatura e ajustada sobre uma janela A FRENTE do robo (a camera
    # inferior ve de 3.5 a 13.5 cm, centro ~8.5 cm), mas o feedforward
    # omega = v*kappa vale para a curvatura SOB o robo. Aplicar a leitura
    # crua faz o robo virar cedo demais e entrar por dentro da curva.
    # MEDIDO NA PISTA (09/09/2026): em t=1.00s a curvatura ja marcava
    # +0.45 e o wz ja virava para a direita, mas o heading da linha ainda
    # era +0.6 graus -- o robo continuava na reta. Ele saiu da linha por
    # dentro (lat foi a -34mm) e so entao o feedback reagiu para a
    # esquerda, o que na pista se ve como "virou para a esquerda numa
    # curva para a direita".
    # A correcao e usar a curvatura medida ha (distancia / velocidade)
    # segundos: e a leitura daquele trecho de pista quando ele ainda
    # estava a frente. A 0.20 m/s isso e 0.43 s; a 0.07 m/s, 1.2 s.
    # 0.0 desliga o atraso e volta ao comportamento anterior.
    curvature_eval_distance_m: float = 0.085
    curvature_delay_max_s: float = 1.5  # s
    derivative_tau: float = 0.08  # s, filtro passa-baixa da derivada
    wz_max: float = 2.5           # rad/s, teto absoluto
    wz_accel: float = 6.0         # rad/s^2, casado com maxWzAccel do firmware

    # --- limite cinematico ------------------------------------------
    wheel_base_k: float = 0.165   # m, halfL + halfW (Config::K do firmware)
    wheel_max_mps: float = 0.70   # m/s, Config::MAX_WHEEL_MPS
    arc_ratio: float = 0.80       # 1.0 = permite roda interna parada

    # --- escala de velocidade ---------------------------------------
    ref_lateral: float = 0.045    # m de erro que ja justifica v_min
    ref_heading: float = 0.55     # rad de orientacao que ja justifica v_min
    ref_curvature: float = 6.0
    # Residuo do ajuste de reta que ja justifica andar em v_min. E o
    # freio de QUEBRA. O valor tem de ser MEDIDO na pista: ponha o robo
    # numa reta e depois numa quebra e compare /line/detection. A ordem
    # esperada, para uma janela de 54mm, e residuo ~ L*tan(D/2)/8:
    #   reta      -> ~0
    #   quebra 30 -> 1.8mm
    #   quebra 45 -> 2.8mm
    # 0.0 desliga o freio de quebra.
    ref_residual: float = 0.0028    # 1/m de curvatura que ja justifica v_min

    # --- confianca e perda ------------------------------------------
    confidence_ok: float = 0.45   # acima disso: FOLLOWING
    confidence_min: float = 0.15  # abaixo disso a deteccao nao e usada
    vision_timeout: float = 0.20  # s sem deteccao valida -> COASTING
    coast_time: float = 0.25      # s de COASTING antes de RECOVERING
    coast_speed_ratio: float = 0.5  # fracao de v_min mantida no coast
    recovery_time: float = 3.0    # s de busca antes da parada segura
    recovery_wz: float = 0.9      # rad/s do giro de busca
    # Distancia usada para projetar a linha ao decidir o LADO da busca.
    # Ver o comentario onde _last_side e atualizado: o erro lateral cru
    # e ruido no instante da perda em curva; quem carrega a informacao
    # e o heading. 0.05m = a ordem da janela de profundidade da camera.
    recovery_projection_m: float = 0.05
    relock_frames: int = 3        # deteccoes seguidas para voltar a seguir

    # --- alinhamento parado (ALIGNING) ------------------------------
    # MEDIDO NA PISTA (09/09/2026): largado a 25 graus da linha, o robo
    # NAO consegue se alinhar andando para frente. O motivo e geometrico,
    # nao de ganho: o teto de giro sem inverter roda e
    # arc_ratio*v/wheel_base_k, logo a curvatura maxima do caminho e
    # arc_ratio/wheel_base_k = 4.85 1/m -- raio minimo de 20.6 cm,
    # CONSTANTE, independente da velocidade. Desacelerar nao aperta a
    # curva. Para girar 25 graus nesse raio o desvio lateral e
    # R*(1-cos25) = 19 mm, que somado ao offset inicial passa dos +/-19mm
    # que a banda mais proxima enxerga. Na medicao o robo atravessou a
    # linha e saiu a -49.8mm, com a confianca caindo de 0.88 para 0.03.
    #
    # Girar PARADO nao tem desvio lateral nenhum. E a mesma manobra que
    # RECOVERING ja faz com seguranca, so que aqui de forma deliberada e
    # antes de comecar a andar.
    #
    # So dispara na PARTIDA (saindo de IDLE), nunca durante o
    # seguimento: em curva o heading tambem passa de 20 graus, e parar
    # para girar no meio de uma curva seria pior que o problema.
    align_on_start: bool = True
    align_heading_deg: float = 18.0   # acima disso, alinha antes de andar
    align_exit_deg: float = 7.0       # histerese: sai do alinhamento aqui
    # 0.9 e nao 0.6: com 0.6 a velocidade de roda fica em
    # wheel_base_k*wz = 0.099 m/s, abaixo do atrito estatico. MEDIDO NA
    # PISTA: o robo ficou 1.2s parado comandando giro antes de destravar.
    # 0.9 e o mesmo valor de recovery_wz, que sabidamente gira.
    align_wz: float = 0.9             # rad/s, teto do giro parado
    align_timeout: float = 4.0        # s, nunca fica preso alinhando
    # A manobra NAO depende de visao continua. MEDIDO NA PISTA: girando
    # no lugar, 12 graus moveram o erro lateral em 24.8mm -- ou seja, o
    # ponto observado esta a ~125mm do centro de rotacao (a camera fica a
    # frente dele). Com a banda mais proxima cobrindo +/-19mm, o giro so
    # corrige ~8.7 graus antes de a linha sair dela. Por isso o alvo de
    # rotacao e TRAVADO na entrada, a partir do heading medido com boa
    # confianca, e o progresso e estimado integrando o wz comandado.
    # Perder a linha no meio do giro deixou de ser motivo para abortar:
    # o robo esta parado, e a manobra e limitada pelo alvo e pelo timeout.
    align_closed_loop_deg: float = 9.0   # ate aqui, confia na visao
    align_overshoot_guard: float = 1.25  # nunca gira mais que isto x alvo

    # --- camera superior (preview) ----------------------------------
    # A camera superior tem TRES papeis, com riscos muito diferentes, e
    # cada um tem a sua chave. Antes eles viviam todos atras de
    # use_preview, entao desligar a direcao (o papel nocivo) matava
    # junto a recuperacao (o papel mais util e mais seguro) sem avisar.
    #
    #   direcao     -> entra no laco fechado de wz. RUIDO VIRA GIRO.
    #                  MEDIDO na pista: com ele ligado o robo perdia a
    #                  linha 32% do tempo e o vies de curva chegou a
    #                  -16mm; desligado, 0% de perda e vies +0.46mm.
    #                  Fica DESLIGADO ate haver evidencia do contrario.
    #   velocidade  -> so freia antes da curva. Errar custa tempo, nao
    #                  rastreio: nao realimenta a direcao.
    #   recuperacao -> escolhe o LADO para onde girar quando a linha ja
    #                  foi perdida. Nunca define magnitude, so o sinal,
    #                  e so age em RECOVERING. Risco minimo.
    #
    # use_preview continua valendo como chave-mestra: false desliga os
    # tres de uma vez.
    use_preview: bool = True
    preview_steering_enabled: bool = False
    preview_speed_enabled: bool = True
    preview_recovery_enabled: bool = True
    preview_timeout: float = 0.40      # s
    preview_confidence_min: float = 0.40
    # MEMORIA DO PREVIEW. Na quebra a camera superior ve a curva ~1,2 s
    # antes da inferior perder a linha, mas quando a recuperacao comeca
    # ela ja publica valid=False -- nao e mensagem velha, e mensagem sem
    # deteccao, entao preview_timeout nao segura nada. Sem isso o lado do
    # giro cai em _last_side da inferior, que na quebra e cara-ou-coroa:
    # medido na pista, a MESMA primeira quebra girou para lados opostos
    # em duas corridas seguidas. Guardar o ultimo lado utilizavel da
    # superior por alguns segundos usa a informacao que ja existia.
    # 2.0 s a 0.10 m/s = 20 cm percorridos, da ordem do que a superior
    # enxerga a frente (19,8 a 25,3 cm), entao a leitura ainda descreve
    # onde o robo esta agora. Vale SO em RECOVERING e so define o LADO.
    preview_memory_time: float = 2.0   # s
    preview_ff_gain: float = 0.35      # fracao da curvatura do preview
    # Ganho do heading do preview, aplicado direto (rad/s por rad), na
    # mesma forma de k_heading. Existe porque curvatura so faz sentido
    # para uma curva suave: numa quina real (reta -> reta anguladas,
    # sem arco entre elas) o ajuste quadratico de curvatura explode
    # (medido no robo: chegou a +11 1/m numa "curva" que era na verdade
    # uma quina de ~30 graus), mas o heading_error da propria camera
    # superior mede o angulo da quina de forma direta e estavel, porque
    # e so uma inclinacao lida nas bandas, nao uma curvatura ajustada.
    preview_heading_gain: float = 0.40
    # teto do preview (curvatura+heading+bearing somados), fracao de wz_max
    preview_ff_max_ratio: float = 0.30
    preview_center_tol_deg: float = 8.0  # pan maximo para confiar no preview
    preview_speed_gain: float = 1.0    # peso do preview na reducao de velocidade

    # --- bearing pelo servo (camera superior rastreando a linha) ----
    # Ideia diferente do preview acima: em vez de inferir heading a
    # partir do deslocamento na imagem (que fica ruim quando o pan se
    # move), a cabeca RASTREIA a linha ativamente (loop proprio no
    # head_servo_node) e o ANGULO DO SERVO em si vira o sinal -- uma
    # leitura geometrica direta e inequivoca, sem precisar combinar
    # deslocamento de imagem com angulo de pan. So funciona quando a
    # cabeca esta de fato rastreando (head_mode=TRACK), por isso
    # substitui o preview centrado (que exige cabeca parada) durante o
    # seguimento normal quando habilitado.
    # ATENCAO: esta chave controla SO a realimentacao do angulo da
    # cabeca no giro do corpo. Quem manda a cabeca rastrear e
    # head_tracking_enabled, abaixo. Antes as duas coisas estavam na
    # mesma flag, entao desligar a realimentacao congelava a cabeca no
    # centro junto -- e um teste A/B que so queria isolar a
    # realimentacao acabava mudando duas variaveis ao mesmo tempo.
    use_servo_bearing: bool = False
    servo_bearing_gain: float = 1.2   # rad/s por rad de angulo do servo
    # CHAVE-MESTRA do movimento da cabeca. Com False a cabeca fica
    # CENTRADA em TODOS os estados -- inclusive a varredura de
    # RECOVERING e SAFE_STOP, que antes ignoravam esta chave e moviam o
    # servo assim mesmo. Isso surpreendeu na pista (10/09/2026): com a
    # camera superior fora do suporte, desliguei o rastreio e mesmo
    # assim o servo varreu, porque o robo passou 44% do tempo em
    # estados de perda.
    # Independente de a DIRECAO usar ou nao esse angulo (use_servo_bearing).
    head_tracking_enabled: bool = True
    # Distancia media que a camera superior observa. Vem das medidas do
    # projeto: ela enxerga de 20.5cm a 54.5cm, logo o centro do que ela
    # olha esta a ~37.5cm a frente do robo.
    preview_lookahead_m: float = 0.375
    # ANTECIPACAO GEOMETRICA DA CABECA. Sem isso o servo so reage ao
    # erro que JA apareceu na imagem, com controle de taxa -- ou seja,
    # sempre atrasado, e numa curva a linha sai do quadro antes dele
    # alcancar. Com a curvatura ja medida pela camera inferior da para
    # prever onde a linha estara: num arco de curvatura kappa, o ponto
    # a D metros a frente esta deslocado ~D^2*kappa/2, o que corresponde
    # a um bearing de atan(D*kappa/2). Para uma curva de raio 33cm
    # (kappa=3) e D=0.375m isso da 29 graus de antecipacao.
    # 1.0 aponta exatamente onde a geometria preve; abaixo disso sobra
    # trabalho para o loop visual do servo, que corrige o residuo.
    servo_lookahead_gain: float = 0.8

    # --- servo -------------------------------------------------------
    scan_in_recovery: bool = True
    # Continuar varrendo a cabeca depois que RECOVERING desistiu e caiu
    # em SAFE_STOP. O corpo ja esta parado, entao nao ha risco: e so
    # deixar o robo continuar PROCURANDO em vez de esperar cego.
    scan_in_safe_stop: bool = True


@dataclass
class Observation:
    """Snapshot de uma LineDetection ja validada pelo no."""

    valid: bool = False
    confidence: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    curvature: float = 0.0
    heading_valid: bool = False
    curvature_valid: bool = False
    # Residuo RMS do ajuste de reta nas bandas: grande so quando ha uma
    # QUEBRA dentro da janela. Ver o comentario em line_geometry._fit().
    fit_residual: float = 0.0
    residual_valid: bool = False
    # A que distancia a FRENTE do robo o erro lateral foi medido. Vem do
    # detector, entao acompanha a montagem sozinho -- se a camera mudar
    # de lugar, o controlador percebe sem ninguem reeditar parametro.
    lookahead_distance: float = 0.0
    age: float = math.inf


@dataclass
class Command:
    vx: float = 0.0
    wz: float = 0.0
    state: State = State.IDLE
    severity: float = 0.0
    wz_limit: float = 0.0
    preview_active: bool = False
    preview_ff: float = 0.0
    head_mode: HeadMode = HeadMode.CENTER
    head_angle: float = 0.0


class FollowerController:
    """Supervisor + controlador. Um passo por ciclo, com dt medido."""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.state = State.IDLE
        self._vx = 0.0
        self._wz = 0.0
        self._error_prev: float | None = None
        self._derivative = 0.0
        self._curvature_filtered = 0.0
        self._curvature_delayed = 0.0
        self._align_alvo = 0.0       # rad a girar, travado na entrada
        self._align_girado = 0.0     # rad ja girados (medidos)
        self._odom_yaw_rate: float | None = None
        # (instante, curvatura filtrada) para o atraso de transporte
        self._curvature_hist: list[tuple[float, float]] = []
        self._state_since = 0.0
        self._now = 0.0
        self._last_side = 1.0        # +1 = linha saiu pela direita
        # ultimo lado visto pela superior enquanto ela era utilizavel
        self._preview_side: float | None = None
        self._preview_side_time = 0.0
        self._relock_count = 0
        self._seen_line = False
        self._scan_phase = 0.0
        self._scan_direction = 1.0
        # metricas
        self.loss_events = 0
        self.last_recovery_time = 0.0
        self._recovery_started = 0.0
        self.error_sum = 0.0
        self.error_count = 0
        self.distance = 0.0

    # ------------------------------------------------------------------
    def _enter(self, state: State) -> None:
        if state == self.state:
            return
        if state == State.RECOVERING:
            self.loss_events += 1
            self._recovery_started = self._now
        if self.state == State.RECOVERING and state in (
            State.FOLLOWING, State.DEGRADED
        ):
            self.last_recovery_time = self._now - self._recovery_started
        self.state = state
        self._state_since = self._now

    def _elapsed(self) -> float:
        return self._now - self._state_since

    # ------------------------------------------------------------------
    def _mira_da_cabeca(self, obs: Observation) -> float:
        """Para onde a cabeca deve apontar, segundo a camera INFERIOR.

        O head_servo_node soma isto ao proprio loop visual: este termo
        faz o movimento grosso, a imagem da superior refina. Quando a
        superior perde a linha, este termo e a UNICA referencia que
        sobra -- por isso ele precisa continuar valendo alguma coisa.

        POR QUE MUDOU (10/09/2026): antes vinha so da CURVATURA da
        inferior. A curvatura foi desligada com evidencia cruzada
        (curvature_enabled), entao o termo virou zero fixo e a mira
        colapsou para o centro. MEDIDO na pista, com a inferior parada
        em cima de uma quebra: a inferior lia th=+24deg, 5/5 bandas,
        e a superior lia SEM LINHA em 203 de 203 quadros -- a pista
        tinha virado e saido do campo dela, restando um fragmento de
        ~35px na borda. Com a cabeca centrada nao havia o que ler.

        O heading da inferior nao tem esse problema: e medido, e vivo, e
        aponta justamente para onde a pista vai. Projeta-se a linha ate
        a distancia de preview e tira-se o angulo dali:

            lateral(D) = e + (D - L) * tan(theta)
            mira       = atan(lateral(D) / D)

        onde L e a profundidade que a propria inferior ja enxerga, para
        nao contar duas vezes o trecho que ela ja mediu.

        A curvatura continua somando quando for valida -- se ela voltar
        a ser confiavel, melhora a estimativa em vez de substitui-la.
        """
        cfg = self.config
        distancia = max(cfg.preview_lookahead_m, 1e-3)
        lateral = obs.lateral_error
        if obs.heading_valid:
            # tan() explode perto de 90 graus; a mira nao pode ir junto.
            limite = math.radians(60.0)
            heading = max(-limite, min(limite, obs.heading_error))
            avanco = max(0.0, distancia - obs.lookahead_distance)
            lateral += avanco * math.tan(heading)
        if obs.curvature_valid:
            lateral += 0.5 * obs.curvature * distancia * distancia
        bearing = math.atan(lateral / distancia)
        return math.degrees(bearing) * cfg.servo_lookahead_gain

    def _severity(
        self,
        obs: Observation,
        preview: Observation | None,
        servo_bearing_active: bool = False,
        servo_angle_deg: float = 0.0,
    ) -> float:
        """0 = reta limpa e rapida, 1 = pior caso ainda seguindo.

        Combina o que ja aconteceu (erro, orientacao) com o que esta por
        vir (curvatura da propria camera e do preview) e com a qualidade
        da evidencia. E continuo: nao ha degrau de velocidade em curva.
        """
        cfg = self.config
        terms = [
            abs(obs.lateral_error) / cfg.ref_lateral,
            (abs(obs.heading_error) / cfg.ref_heading) if obs.heading_valid else 0.0,
            (abs(obs.curvature) / cfg.ref_curvature) if obs.curvature_valid else 0.0,
            1.0 - obs.confidence,
        ]
        # QUEBRA DE RETA. O heading sobe em DOIS casos que pedem coisas
        # opostas: quebra chegando (frear esta certo) e robo apenas
        # TORTO numa reta (frear so atrasa a recuperacao). O residuo do
        # ajuste separa os dois -- ele e ~zero no segundo caso.
        if obs.residual_valid and cfg.ref_residual > 0.0:
            terms.append(abs(obs.fit_residual) / cfg.ref_residual)
        if preview is not None and preview.curvature_valid:
            terms.append(
                cfg.preview_speed_gain * abs(preview.curvature) / cfg.ref_curvature
            )
        if preview is not None and preview.heading_valid:
            # Mesmo quando a curvatura do preview e rejeitada (quina, nao
            # curva -- ver comentario em preview_heading_gain), o heading
            # sozinho ja e um aviso valido de que algo exigente esta
            # chegando, e antecipa a reducao de velocidade antes mesmo de
            # a camera inferior comecar a ver a quina.
            terms.append(
                cfg.preview_speed_gain
                * abs(preview.heading_error)
                / cfg.ref_heading
            )
        if servo_bearing_active:
            # A cabeca esta virada N graus para manter a linha centrada:
            # e um aviso direto de que o corpo esta (ou vai precisar
            # ficar) desalinhado por essa ordem de grandeza.
            terms.append(
                cfg.preview_speed_gain
                * abs(math.radians(servo_angle_deg))
                / cfg.ref_heading
            )
        return min(1.0, max(0.0, max(terms)))

    def _ganho_de_heading(self, vx: float, lookahead: float = 0.0) -> float:
        """k_heading que mantem o amortecimento constante com a velocidade.

        Ver o comentario de heading_gain_scheduling em ControllerConfig
        para a derivacao.

        DESCONTA o que a montagem ja da de graca: medindo o erro a L
        metros A FRENTE do robo, e_medido = e_centro + L*theta, logo
        k_lateral*L ja age como ganho de orientacao. Sem descontar, o
        agendamento empilha por cima e o robo fica sobreamortecido --
        com L=0.11m e k_lateral=9.0 isso sao 0.99 de ganho escondido,
        mais da metade do k_heading nominal.
        """
        cfg = self.config
        if not cfg.heading_gain_scheduling:
            return cfg.k_heading
        v = max(abs(vx), 1e-3)
        wn = math.sqrt(max(v * cfg.k_lateral, 1e-12))
        ja_tem = cfg.k_lateral * max(0.0, lookahead)
        agendado = (
            2.0 * cfg.target_damping * wn - cfg.k_damping * v - ja_tem
        )
        # Piso em heading_gain_min, nao em zero: ver a derivacao la. Zero
        # deixava o robo estacionar torto em baixa velocidade.
        return max(cfg.heading_gain_min, agendado)

    def _atrasa_curvatura(self, now: float, vx: float) -> float:
        """Curvatura que estava sob o robo agora, nao a que esta a frente.

        Ver curvature_eval_distance_m em ControllerConfig: sem isto o
        feedforward vira antes de o robo chegar na curva.
        """
        cfg = self.config
        self._curvature_hist.append((now, self._curvature_filtered))
        if cfg.curvature_eval_distance_m <= 0.0:
            self._curvature_hist = self._curvature_hist[-1:]
            return self._curvature_filtered
        # Velocidade de referencia com piso: parado, o atraso satura no
        # teto em vez de virar divisao por zero.
        v = max(abs(vx), 1e-3)
        atraso = min(
            cfg.curvature_delay_max_s, cfg.curvature_eval_distance_m / v
        )
        alvo = now - atraso
        while len(self._curvature_hist) > 1 and self._curvature_hist[1][0] <= alvo:
            self._curvature_hist.pop(0)
        # Guarda de memoria: o historico nunca precisa passar do teto.
        if len(self._curvature_hist) > 500:
            del self._curvature_hist[:-500]
        return self._curvature_hist[0][1]

    def _alinhamento_terminou(self, obs: Observation) -> bool:
        """Fim do giro de alinhamento, por qualquer um dos criterios.

        A visao so decide enquanto o erro ainda e pequeno o bastante
        para a banda mais proxima aguentar (align_closed_loop_deg);
        fora disso quem manda e a rotacao ja cumprida, estimada
        integrando o wz comandado, mais o timeout como ultimo recurso.
        """
        cfg = self.config
        if self._elapsed() > cfg.align_timeout:
            return True
        # Progresso COM SINAL, no sentido do alvo: girar para o lado
        # errado (odom com sinal trocado, roda patinando ao contrario)
        # nao pode contar como alinhamento cumprido -- sai pelo timeout.
        alvo = abs(self._align_alvo)
        progresso = math.copysign(1.0, self._align_alvo) * self._align_girado
        # Teto de rotacao ANTES da visao: e justamente com a visao ainda
        # dizendo "torto" (linha errada, reflexo) que o robo giraria alem
        # da conta. Antes este teste ficava depois do retorno da visao e
        # depois de um teste mais fraco, e nunca tinha efeito.
        if progresso >= cfg.align_overshoot_guard * alvo:
            return True
        # A VISAO decide sempre que estiver disponivel. Foi um erro
        # anterior tratar a estimativa de rotacao como criterio
        # primario: com atrito estatico o robo nao girava, a estimativa
        # dizia que sim, e o alinhamento terminava com o robo ainda
        # torto (medido: saiu do ALIGNING ainda a -23.7 graus).
        if obs.valid and obs.heading_valid:
            return abs(obs.heading_error) <= math.radians(cfg.align_exit_deg)
        # Sem visao, resta a rotacao MEDIDA pelos encoders.
        return progresso >= alvo

    def _wz_limit(self, vx: float) -> float:
        """Teto de giro que o chassi aguenta a esta velocidade linear."""
        cfg = self.config
        k = max(cfg.wheel_base_k, 1e-6)
        saturation = max(0.0, (cfg.wheel_max_mps - abs(vx)) / k)
        no_reversal = cfg.arc_ratio * abs(vx) / k
        return min(cfg.wz_max, saturation, no_reversal)

    def _steering(
        self,
        obs: Observation,
        preview: Observation | None,
        vx: float,
        dt: float,
        servo_bearing_active: bool = False,
        servo_angle_deg: float = 0.0,
    ) -> tuple[float, bool, float]:
        cfg = self.config

        error = obs.lateral_error
        if self._error_prev is None:
            derivative_raw = 0.0
        else:
            derivative_raw = (error - self._error_prev) / max(dt, 1e-3)
        self._error_prev = error
        alpha = dt / max(cfg.derivative_tau + dt, 1e-6)
        self._derivative += alpha * (derivative_raw - self._derivative)

        heading = obs.heading_error if obs.heading_valid else 0.0

        # Feedback: sinal de menos converte "linha a direita" (+x) em
        # "girar para a direita" (wz negativo na convencao ROS).
        wz = -(
            cfg.k_lateral * error
            + self._ganho_de_heading(vx, obs.lookahead_distance) * heading
            + cfg.k_damping * self._derivative
        )

        # Feedforward geometrico da propria camera inferior: para seguir
        # um arco de curvatura kappa a velocidade v, omega = v * kappa.
        # Curvatura ATRASADA: a que esta sob o robo agora. A crua, que
        # olha a frente, e para a CABECA antecipar -- nao para o corpo
        # virar cedo.
        #
        # SEM porta em obs.curvature_valid: a validade ja viaja no
        # historico (o filtro decai a zero quando a leitura e invalida, e
        # e esse zero que e atrasado). A porta usava a validade do frame
        # ATUAL, ~0,4 s a frente: na saida de uma curva, a quebra seguinte
        # invalidava o frame e zerava o feedforward da curva que o robo
        # ainda estava fazendo.
        wz += -cfg.k_feedforward * self._curvature_delayed * vx

        # Feedforward de preview: entra saturado e so quando a cabeca esta
        # centrada. Nunca vira a referencia de direcao. Dois termos,
        # curvatura (curva suave) e heading (quina/reta angulada), somados
        # e SO ENTAO saturados juntos -- assim o teto vale para a
        # contribuicao total do preview, nao para cada termo separado.
        preview_raw = 0.0
        preview_active = False
        if preview is not None and preview.curvature_valid:
            preview_raw += -cfg.preview_ff_gain * preview.curvature * vx
            preview_active = True
        if preview is not None and preview.heading_valid:
            preview_raw += -cfg.preview_heading_gain * preview.heading_error
            preview_active = True
        if servo_bearing_active:
            # Leitura direta e geometricamente limpa: o angulo do servo
            # ja E o bearing do corpo em relacao a linha, sem precisar
            # inferir nada da imagem em movimento.
            preview_raw += -cfg.servo_bearing_gain * math.radians(servo_angle_deg)
            preview_active = True

        preview_ff = 0.0
        if preview_active:
            # Mesmo teto para os tres termos somados -- curvatura,
            # heading de imagem e bearing do servo -- garante que a
            # camera/cabeca de cima nunca dominam a referencia primaria,
            # nao importa qual caminho estiver contribuindo.
            ceiling = cfg.preview_ff_max_ratio * cfg.wz_max
            preview_ff = max(-ceiling, min(ceiling, preview_raw))
            wz += preview_ff

        return wz, preview_active, preview_ff

    def _ramp(self, current: float, target: float, rate: float, dt: float) -> float:
        step = rate * dt
        return max(current - step, min(current + step, target))

    # ------------------------------------------------------------------
    def update(
        self,
        now: float,
        enabled: bool,
        obs: Observation,
        preview: Observation | None = None,
        odom_speed: float | None = None,
        servo_angle_deg: float = 0.0,
        odom_yaw_rate: float | None = None,
    ) -> Command:
        cfg = self.config
        if self._now == 0.0:
            # Primeiro ciclo apos reset: nao ha dt medido e o relogio ROS
            # e um numero grande, entao ancorar _state_since aqui evita
            # que _elapsed() dispare um timeout imediato.
            dt = 0.02
            self._state_since = now
        else:
            dt = max(1e-3, min(0.2, now - self._now))
        self._now = now

        self._odom_yaw_rate = odom_yaw_rate
        if odom_speed is not None:
            self.distance += abs(odom_speed) * dt

        if not enabled:
            self.state = State.IDLE
            self._vx = 0.0
            self._wz = 0.0
            self._error_prev = None
            self._derivative = 0.0
            return Command(state=State.IDLE, head_mode=HeadMode.CENTER)

        # Uma deteccao so conta como fresca se e valida, recente e com
        # confianca acima do piso. Idade vem do stamp do frame, entao uma
        # camera travada (publicando o mesmo frame ou nada) envelhece.
        fresh = (
            obs.valid
            and obs.age <= cfg.vision_timeout
            and obs.confidence >= cfg.confidence_min
        )
        preview_usable = (
            preview is not None
            and cfg.use_preview
            and preview.valid
            and preview.age <= cfg.preview_timeout
            and preview.confidence >= cfg.preview_confidence_min
        )
        # Assim que o preview vale, memoriza o lado. O calculo do lado e o
        # mesmo usado na recuperacao: com a cabeca girada quem manda e o
        # angulo do pan, com ela centrada e o erro lateral do preview.
        if preview_usable:
            if abs(servo_angle_deg) > cfg.preview_center_tol_deg:
                self._preview_side = 1.0 if servo_angle_deg >= 0.0 else -1.0
            else:
                # Mesma projecao usada para _last_side, e pela mesma razao:
                # na aproximacao da quebra o erro lateral da superior ainda
                # e ruido (medido: +5,8mm e -0,5mm) enquanto o heading dela
                # ja marca +49,6deg e +66,9deg. Projetando 50mm isso da
                # +64mm e +117mm -- o lado da quebra, sem ambiguidade.
                projetado = preview.lateral_error
                if preview.heading_valid:
                    projetado += cfg.recovery_projection_m * math.tan(
                        preview.heading_error
                    )
                self._preview_side = 1.0 if projetado >= 0.0 else -1.0
            self._preview_side_time = self._now

        # O preview so entra no calculo de direcao com a cabeca centrada.
        # Com o pan girado, o deslocamento horizontal na imagem mistura
        # "a linha desviou" com "a camera girou", e nao existe aqui uma
        # calibracao camera-servo para separar os dois. Fora da tolerancia
        # ele deixa de valer como feedforward -- mas continua valendo na
        # recuperacao, onde so o LADO importa e o proprio angulo do servo
        # ja diz para onde a cabeca estava olhando quando achou a linha.
        head_centered = abs(servo_angle_deg) <= cfg.preview_center_tol_deg
        # Papel DIRECAO e papel VELOCIDADE usam caminhos separados. Os
        # dois exigem cabeca centrada (ambiguidade de projecao), mas o
        # de direcao esta desligado por padrao -- ver os comentarios das
        # chaves em ControllerConfig.
        preview_ready = preview_usable and head_centered
        preview_steer = (
            preview if (preview_ready and cfg.preview_steering_enabled)
            else None
        )
        preview_speed = (
            preview if (preview_ready and cfg.preview_speed_enabled)
            else None
        )
        # Bearing do servo: NAO exige cabeca centrada -- e o oposto do
        # preview acima, porque so faz sentido quando a cabeca esta de
        # fato girada rastreando a linha (head_mode=TRACK).
        servo_bearing_active = cfg.use_servo_bearing and preview_usable

        # Filtro da curvatura usada no feedforward. Roda sempre que ha
        # leitura fresca e valida; quando a curvatura fica invalida
        # (heading grande, poucas bandas), o filtro DECAI para zero em
        # vez de segurar o ultimo valor -- segurar uma curvatura velha
        # manteria o robo curvando depois que a curva ja acabou.
        alpha_k = dt / max(cfg.curvature_filter_tau + dt, 1e-6)
        alvo_k = obs.curvature if (fresh and obs.curvature_valid) else 0.0
        self._curvature_filtered += alpha_k * (alvo_k - self._curvature_filtered)
        # A cabeca usa a curvatura CRUA (antecipa); o corpo usa a
        # ATRASADA (a que esta sob ele agora).
        self._curvature_delayed = self._atrasa_curvatura(now, self._vx)

        if fresh:
            self._seen_line = True
            # MEDIDO NO ROBO (pista do laboratorio, curva real): exatamente
            # na transicao para a perda, a leitura "fresca" (que so exige
            # confidence_min=0.15) as vezes vem com o sinal invertido em
            # relacao as leituras confiaveis anteriores -- a curva aperta,
            # a camera perde a linha por um instante, e o ultimo frame
            # antes de cair pra COASTING/RECOVERING pode ser justamente
            # esse frame ruim. Como last_side decide o LADO do giro de
            # busca em RECOVERING, confiar nele sem filtro fazia o robo
            # buscar para o lado ERRADO logo depois de uma curva bem
            # negociada (dois casos reais: conf 0.52->0.15 e 0.53->0.24,
            # sinal invertendo nos dois exatamente quando a confianca
            # cruzou para abaixo de confidence_ok). Exigir confidence_ok
            # (nao so confidence_min) para atualizar last_side ancora a
            # busca na ultima leitura em que a linha realmente estava
            # bem vista, nao no ultimo pixel ruidoso antes de perder.
            if obs.confidence >= cfg.confidence_ok:
                # NAO usa o erro lateral cru. MEDIDO NA PISTA
                # (10/09/2026): entrando em curva, as bandas morrem por
                # HEADING enquanto o erro lateral ainda esta em +3 a
                # +5mm -- ou seja, ruido. Decidir o lado da busca por
                # esse sinal e cara-ou-coroa, e o robo girava para o
                # lado errado ("perde, vira pro lado perdido, e ai
                # acha"). A linha PROJETADA a frente responde a pergunta
                # certa -- para onde a linha estava indo:
                #     x(D) = lateral + D * tan(heading)
                # Com lateral +3mm e heading +14deg, projetar 50mm ja da
                # +15mm: o lado da curva, sem ambiguidade.
                projetado = obs.lateral_error
                if obs.heading_valid:
                    limite = math.radians(60.0)
                    heading = max(-limite, min(limite, obs.heading_error))
                    projetado += cfg.recovery_projection_m * math.tan(heading)
                self._last_side = 1.0 if projetado >= 0.0 else -1.0
            self.error_sum += abs(obs.lateral_error)
            self.error_count += 1

        # --- transicoes de estado -----------------------------------
        if not self._seen_line:
            self._enter(State.IDLE)
        elif fresh:
            if self.state == State.ALIGNING:
                if self._alinhamento_terminou(obs):
                    self._enter(
                        State.FOLLOWING
                        if obs.confidence >= cfg.confidence_ok
                        else State.DEGRADED
                    )
            elif self.state in (State.RECOVERING, State.SAFE_STOP):
                self._relock_count += 1
                if self._relock_count >= cfg.relock_frames:
                    self._enter(
                        State.FOLLOWING
                        if obs.confidence >= cfg.confidence_ok
                        else State.DEGRADED
                    )
            elif (
                self.state == State.IDLE
                and cfg.align_on_start
                and obs.heading_valid
                and obs.confidence >= cfg.confidence_ok
                and abs(obs.heading_error) > math.radians(cfg.align_heading_deg)
            ):
                # Largado torto: alinha parado antes de andar. Ver o
                # comentario de align_on_start em ControllerConfig.
                self._relock_count = 0
                # Trava o alvo AGORA, com a leitura boa que acabou de
                # autorizar a manobra. Depois disso a visao pode se
                # perder no meio do giro sem estragar o alinhamento.
                self._align_alvo = obs.heading_error
                self._align_girado = 0.0
                self._enter(State.ALIGNING)
            else:
                self._relock_count = 0
                self._enter(
                    State.FOLLOWING
                    if obs.confidence >= cfg.confidence_ok
                    else State.DEGRADED
                )
        else:
            self._relock_count = 0
            if self.state == State.ALIGNING:
                # Perder a linha no meio do giro e ESPERADO (a banda mais
                # proxima so aguenta ~9 graus). O robo esta parado, entao
                # nao ha o que abortar: segue ate cumprir o alvo travado
                # ou estourar o timeout.
                if self._alinhamento_terminou(obs):
                    self._enter(State.COASTING)
            elif self.state in (State.FOLLOWING, State.DEGRADED):
                self._enter(State.COASTING)
            elif self.state == State.COASTING:
                if self._elapsed() > cfg.coast_time:
                    self._enter(State.RECOVERING)
            elif self.state == State.RECOVERING:
                if self._elapsed() > cfg.recovery_time:
                    self._enter(State.SAFE_STOP)

        command = Command(state=self.state)

        # --- acao por estado ----------------------------------------
        if self.state in (State.IDLE, State.SAFE_STOP):
            self._vx = self._ramp(self._vx, 0.0, cfg.decel, dt)
            self._wz = self._ramp(self._wz, 0.0, cfg.wz_accel, dt)
            self._error_prev = None
            self._derivative = 0.0
            if (
                self.state == State.SAFE_STOP
                and cfg.scan_in_safe_stop
                and cfg.head_tracking_enabled
            ):
                # O corpo esta parado (vx e wz vao a zero acima), entao
                # varrer a cabeca aqui e ganho puro de percepcao, sem
                # risco de atuacao. MEDIDO NA PISTA (09/09/2026): num
                # episodio de perda de 9.9s, a cabeca varreu so os 3s de
                # RECOVERING e depois ficou parada no centro por ~7s --
                # o robo se cegava exatamente enquanto esperava a linha
                # voltar. Continuar varrendo aumenta a chance de
                # reencontrar sozinho (ou assim que alguem reposicionar).
                command.head_mode = HeadMode.SCAN
            else:
                command.head_mode = HeadMode.CENTER

        elif self.state == State.ALIGNING:
            # Gira PARADO: sem avanco nao ha desvio lateral nenhum, que e
            # exatamente o que impedia o alinhamento andando para frente.
            # Nao usa _wz_limit aqui de proposito -- aquele teto existe
            # para nao inverter roda EM MOVIMENTO, e com vx=0 ele daria
            # zero. Mesma logica do giro de busca em RECOVERING.
            self._vx = self._ramp(self._vx, 0.0, cfg.decel, dt)
            # O sentido vem do alvo TRAVADO, nao da leitura instantanea:
            # no meio do giro a leitura pode sumir ou ficar ruidosa.
            restante = self._align_alvo - self._align_girado
            alvo = -math.copysign(cfg.align_wz, restante)
            self._wz = self._ramp(self._wz, alvo, cfg.wz_accel, dt)
            # Progresso pelo giro MEDIDO nos encoders. Integrar o
            # comandado era o erro anterior: com atrito estatico o robo
            # ficava parado e a conta seguia correndo.
            medido = (
                self._odom_yaw_rate
                if self._odom_yaw_rate is not None
                else self._wz
            )
            self._align_girado += -medido * dt
            self._error_prev = None
            self._derivative = 0.0
            command.severity = 1.0
            command.wz_limit = cfg.align_wz
            command.head_mode = HeadMode.CENTER

        elif self.state in (State.FOLLOWING, State.DEGRADED):
            severity = self._severity(
                obs, preview_speed, servo_bearing_active, servo_angle_deg
            )
            if self.state == State.DEGRADED:
                # Evidencia fraca: anda no minimo e ignora o preview
                # baseado em imagem, que nao tem como ser mais confiavel
                # que a camera primaria. O bearing do servo e uma leitura
                # geometrica direta, independente dessa fraqueza -- por
                # isso continua valendo mesmo em DEGRADED.
                severity = 1.0
                preview_steer = None
            target_vx = cfg.v_min + (cfg.v_max - cfg.v_min) * (1.0 - severity)
            rate = cfg.accel if target_vx > self._vx else cfg.decel
            self._vx = self._ramp(self._vx, target_vx, rate, dt)

            wz_raw, preview_active, preview_ff = self._steering(
                obs, preview_steer, self._vx, dt,
                servo_bearing_active, servo_angle_deg,
            )
            limit = self._wz_limit(self._vx)
            wz_target = max(-limit, min(limit, wz_raw))
            self._wz = self._ramp(self._wz, wz_target, cfg.wz_accel, dt)
            self._wz = max(-limit, min(limit, self._wz))

            command.severity = severity
            command.wz_limit = limit
            command.preview_active = preview_active
            command.preview_ff = preview_ff
            command.head_mode = (
                HeadMode.TRACK if cfg.head_tracking_enabled
                else HeadMode.CENTER
            )
            if cfg.head_tracking_enabled:
                command.head_angle = self._mira_da_cabeca(obs)

        elif self.state == State.COASTING:
            # Perda de 1 a 3 frames (sombra, reflexo, vibracao). Parar de
            # vez aqui joga o robo para fora da linha por inercia e por
            # tranco mecanico; seguir na velocidade normal e avancar cego.
            # O meio termo: mantem a curva que ja estava sendo feita e
            # desacelera para uma fracao de v_min. Em 0.25 s a 0.035 m/s
            # o robo anda menos de 1 cm.
            target_vx = cfg.v_min * cfg.coast_speed_ratio
            self._vx = self._ramp(self._vx, target_vx, cfg.decel, dt)
            limit = self._wz_limit(self._vx)
            self._wz = max(-limit, min(limit, self._wz))
            command.wz_limit = limit
            command.severity = 1.0
            command.head_mode = HeadMode.CENTER

        elif self.state == State.RECOVERING:
            # Perda confirmada: nao avanca mais. Gira devagar para o lado
            # por onde a linha saiu e usa a camera superior para varrer.
            self._vx = self._ramp(self._vx, 0.0, cfg.decel, dt)
            command.severity = 1.0
            command.wz_limit = cfg.recovery_wz

            if cfg.scan_in_recovery and cfg.head_tracking_enabled:
                command.head_mode = HeadMode.SCAN
            else:
                command.head_mode = HeadMode.CENTER

            # O LADO e decidido ANTES do giro deste tick (antes o giro
            # usava o lado do tick anterior).
            if preview_usable and cfg.preview_recovery_enabled:
                # A camera superior achou a linha. Ela decide o LADO do
                # giro, nunca a magnitude. _preview_side acabou de ser
                # calculado neste update: angulo do pan com a cabeca
                # girada, lateral PROJETADO pelo heading com ela centrada.
                # Antes este ramo usava o lateral CRU, que na quebra e
                # ruido (-0,5 mm com heading +49,6 graus) e mandava girar
                # para o lado errado.
                self._last_side = self._preview_side
                # Para de varrer e segura onde encontrou, para o robo girar
                # em direcao a linha em vez de perseguir a cabeca.
                command.head_mode = HeadMode.HOLD
                command.head_angle = servo_angle_deg
            elif (
                cfg.preview_recovery_enabled
                and self._preview_side is not None
                and self._now - self._preview_side_time
                <= cfg.preview_memory_time
            ):
                # A superior nao ve nada AGORA, mas viu a curva ha pouco.
                # Essa leitura e melhor que _last_side da inferior, que
                # na quebra vem de 1-2 bandas. So o lado; a magnitude
                # continua sendo recovery_wz.
                self._last_side = self._preview_side

            target_wz = -self._last_side * cfg.recovery_wz
            self._wz = self._ramp(self._wz, target_wz, cfg.wz_accel, dt)

        command.vx = self._vx
        command.wz = self._wz
        return command

    # ------------------------------------------------------------------
    @property
    def mean_abs_error(self) -> float:
        if self.error_count == 0:
            return 0.0
        return self.error_sum / self.error_count

    @property
    def state_name(self) -> str:
        return STATE_NAMES[self.state]
