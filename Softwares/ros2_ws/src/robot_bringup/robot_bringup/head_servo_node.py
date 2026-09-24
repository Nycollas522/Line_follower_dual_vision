#!/usr/bin/env python3
"""Politica de apontamento da camera superior.

UNICO publicador de /servo/command. Existe para que a seguranca do servo
(limite de angulo, limite de taxa, retorno a centro, watchdog) fique num
lugar so, e para que o loop rapido de direcao nunca dispute esse comando
com outro no.

Quem quer mover a cabeca publica um line_msgs/HeadRequest em /head/request.
Este no decide se e possivel e com que velocidade -- um pedido de varredura
nunca vira um salto de servo, porque o salto sacode a imagem e derruba a
deteccao no frame seguinte.

Regras de seguranca aplicadas aqui, sempre, independentemente do pedido:
  - angulo saturado em +/- max_angle_deg;
  - taxa saturada em max_rate_deg_s (e mais devagar ainda no seguimento);
  - sem pedido por request_timeout -> volta a centro;
  - autonomia desligada -> volta a centro e ignora pedidos;
  - so envia comando quando o alvo mudou de verdade, para nao saturar a
    serial com trafego redundante a 20 Hz.

MODE_TRACK: a cabeca rastreia a linha na camera superior por conta
propria (assina /line_front/detection direto, nao passa pelo controle
do corpo). E um servovisual classico: a TAXA de giro do pan e
proporcional ao erro lateral da imagem, integrada a cada ciclo -- nao um
angulo calculado direto. Sem deteccao fresca e confiavel, cai para o
mesmo comportamento de centralizar do MODE_CENTER.

O erro rastreado NAO e o lateral_error cru. Esse campo vem da banda mais
proxima do detector (line_geometry: xs[nearest]), a ~24 cm do robo, e
zera-lo aponta a cabeca para a tangente LOCAL da linha: a camera fica
paralela a pista em vez de olhar para onde ela vai. Numa curva isso
trava a cabeca no trecho reto perto do robo enquanto o vertice sai do
quadro -- que e o modo de falha observado na pista. Entao o erro e
extrapolado ate head_preview_distance_m usando o heading e a curvatura
que o proprio detector ja publica e valida:

    x(D) = lateral + tan(heading)*(D - d0) + curvatura*(D - d0)^2 / 2

Se heading ou curvatura vierem invalidos, os termos correspondentes
somem e o comportamento degrada para o de antes -- nunca para algo pior.
"""

from __future__ import annotations

import math

from line_msgs.msg import HeadRequest, LineDetection

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


def control_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class HeadServoNode(Node):
    def __init__(self) -> None:
        super().__init__('head_servo_node')

        # Limite mecanico/util do pan. O firmware ja satura em +/-90, mas
        # varrer ate o limite mecanico bate no batente e nao ajuda: a
        # linha nunca esta a 90 graus do eixo do robo.
        self.declare_parameter('max_angle_deg', 45.0)
        self.declare_parameter('max_rate_deg_s', 60.0)
        # Taxa reduzida enquanto a cabeca so precisa voltar/ficar no centro,
        # para nao introduzir vibracao no seguimento normal.
        self.declare_parameter('center_rate_deg_s', 25.0)
        self.declare_parameter('scan_amplitude_deg', 35.0)
        self.declare_parameter('scan_rate_deg_s', 40.0)
        self.declare_parameter('center_deadband_deg', 0.6)
        self.declare_parameter('request_timeout', 0.5)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('autonomy_state_topic', '/autonomy/state')
        # MODE_TRACK: servovisual pan-only. A TAXA de giro (graus/s) e
        # proporcional ao erro lateral (metros) da camera superior --
        # converge sozinho, sem precisar calcular um angulo alvo direto.
        self.declare_parameter('track_gain_deg_s_per_m', 300.0)
        self.declare_parameter('track_confidence_min', 0.40)
        self.declare_parameter('track_timeout', 0.40)
        # Filtro passa-baixa (exponencial) no erro antes de virar taxa.
        # Sem isso, o ruido de frame a frame do ajuste de bandas vira
        # movimento em passos no servo (medido no robo: perceptivelmente
        # nao-liso). alpha=1.0 desliga o filtro (raw, sem suavizacao).
        self.declare_parameter('track_filter_alpha', 0.35)
        # Distancia (m, a frente do robo) para onde a cabeca mira. A
        # frontal ve de depth_near=0.205 ate depth_far=0.545, mas com
        # roi_top=0.45 so a parte de baixo entra: 0.205..0.358. Miramos
        # na ponta distante do que ela REALMENTE mede. 0.0 desliga a
        # extrapolacao e volta a mirar na banda mais proxima.
        self.declare_parameter('head_preview_distance_m', 0.34)
        # Vazamento do integrador visual (s). _tracked_angle e integral
        # pura: so o clamp de max_angle_deg o limita, e 45 graus e muito
        # permissivo. Na SAIDA da curva isso trava a cabeca no angulo que
        # ela tinha DENTRO da curva, e como esse angulo realimenta a
        # direcao do corpo (servo_bearing_gain em line_controller), corpo
        # e cabeca se prendem num laco. O vazamento garante que a parte
        # visual precise ser sustentada por erro real para persistir.
        # Constante longa: nao atrapalha o rastreio normal, so impede que
        # um desvio velho fique de pe sozinho. 0.0 desliga o vazamento.
        self.declare_parameter('track_leak_tau', 2.0)
        self.declare_parameter('track_hold_tau', 1.5)
        # Guarda: tan() explode perto de 90 graus. Alem disso um heading
        # tao grande e quase sempre deteccao ruim, nao pista real.
        self.declare_parameter('head_preview_max_heading_deg', 50.0)

        self.max_angle = float(self.get_parameter('max_angle_deg').value)
        self.max_rate = float(self.get_parameter('max_rate_deg_s').value)
        self.center_rate = float(self.get_parameter('center_rate_deg_s').value)
        self.scan_amplitude = float(
            self.get_parameter('scan_amplitude_deg').value
        )
        self.scan_rate = float(self.get_parameter('scan_rate_deg_s').value)
        self.deadband = float(self.get_parameter('center_deadband_deg').value)
        self.request_timeout = float(self.get_parameter('request_timeout').value)
        self.track_gain = float(
            self.get_parameter('track_gain_deg_s_per_m').value
        )
        self.track_confidence_min = float(
            self.get_parameter('track_confidence_min').value
        )
        self.track_timeout = float(self.get_parameter('track_timeout').value)
        self.preview_distance = float(
            self.get_parameter('head_preview_distance_m').value
        )
        self.track_leak_tau = float(
            self.get_parameter('track_leak_tau').value
        )
        self.track_hold_tau = float(
            self.get_parameter('track_hold_tau').value
        )
        self.preview_max_heading = math.radians(float(
            self.get_parameter('head_preview_max_heading_deg').value
        ))
        self.track_filter_alpha = float(
            self.get_parameter('track_filter_alpha').value
        )

        self.command_pub = self.create_publisher(
            Float32, '/servo/command', control_qos()
        )
        self.create_subscription(
            HeadRequest, '/head/request', self._on_request, control_qos()
        )
        self.create_subscription(
            Float32, '/servo/state', self._on_state, control_qos()
        )
        self.create_subscription(
            LineDetection, '/line_front/detection', self._on_front,
            control_qos(),
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('autonomy_state_topic').value),
            self._on_autonomy,
            control_qos(),
        )
        self.create_subscription(
            Bool, '/controle/enable', self._on_autonomy, control_qos()
        )
        self.create_subscription(
            String, '/robot/mode', self._on_mode, control_qos()
        )

        self._mode = HeadRequest.MODE_CENTER
        self._requested_angle = 0.0
        self._request_time: float | None = None
        self._target = 0.0
        self._commanded = 0.0
        self._measured = 0.0
        self._scan_direction = 1.0
        self._enabled = False
        # Modo de operacao publicado pelo mode_manager. RC conta
        # como 'alguem no comando' tanto quanto a autonomia.
        self._modo = ''
        self._last_sent: float | None = None
        self._tracked_angle = 0.0   # so a parte vinda do loop visual
        self._front_lateral_error = 0.0
        self._front_error_filtered = 0.0
        self._front_confidence = 0.0
        self._front_time: float | None = None

        self._period = 1.0 / max(1.0, float(self.get_parameter('rate_hz').value))
        self.create_timer(self._period, self._tick)

        self.add_on_set_parameters_callback(self._on_parameters)

        # Garante que o servo comeca em posicao conhecida, mesmo que o
        # ESP32 tenha sido reiniciado sem o Pi saber.
        self._send(0.0, force=True)
        self.get_logger().info(
            f'Cabeca: +/-{self.max_angle:.0f} graus, '
            f'{self.max_rate:.0f} graus/s, varredura {self.scan_amplitude:.0f} graus'
        )

    # ------------------------------------------------------------------
    def _on_parameters(self, parameters) -> SetParametersResult:
        """Aplica ajuste ao vivo dos ganhos/limites de ajuste fino.

        rate_hz fica de fora: muda o periodo do timer, que so e criado
        uma vez no __init__ -- mudar isso ao vivo exigiria destruir e
        recriar o timer, e nao vale a complexidade extra so por isso.
        Mude rate_hz no YAML e reinicie o no quando precisar ajustar.
        """
        floats = {
            'max_angle_deg': 'max_angle',
            'max_rate_deg_s': 'max_rate',
            'center_rate_deg_s': 'center_rate',
            'scan_amplitude_deg': 'scan_amplitude',
            'scan_rate_deg_s': 'scan_rate',
            'center_deadband_deg': 'deadband',
            'request_timeout': 'request_timeout',
            'track_gain_deg_s_per_m': 'track_gain',
            'track_confidence_min': 'track_confidence_min',
            'track_timeout': 'track_timeout',
            'track_filter_alpha': 'track_filter_alpha',
            'head_preview_distance_m': 'preview_distance',
            'track_leak_tau': 'track_leak_tau',
            'track_hold_tau': 'track_hold_tau',
        }
        for parameter in parameters:
            if parameter.name not in floats:
                continue
            if parameter.value < 0:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} nao pode ser negativo',
                )
        for parameter in parameters:
            attr = floats.get(parameter.name)
            if attr is not None:
                setattr(self, attr, float(parameter.value))
        return SetParametersResult(successful=True)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_request(self, msg: HeadRequest) -> None:
        self._mode = msg.mode
        self._requested_angle = float(msg.angle_deg)
        self._request_time = self._now()

    def _on_state(self, msg: Float32) -> None:
        # Ja chega em angulo VERDADEIRO: o offset do suporte e a
        # inversao de sinal moram os dois no motor_serial_node, a unica
        # fronteira com o hardware.
        self._measured = float(msg.data)

    def _on_front(self, msg: LineDetection) -> None:
        if not msg.valid:
            return
        self._front_lateral_error = self._erro_de_mira(msg)
        self._front_error_filtered += self.track_filter_alpha * (
            self._front_lateral_error - self._front_error_filtered
        )
        self._front_confidence = float(msg.confidence)
        self._front_time = self._now()

    def _erro_de_mira(self, msg: LineDetection) -> float:
        """Onde a linha estara a head_preview_distance_m do robo.

        lateral_error e medido na banda mais proxima; mirar nele deixa a
        cabeca paralela a linha. Aqui a linha e projetada para a frente
        com o heading e a curvatura publicados pelo detector.
        """
        erro = float(msg.lateral_error)
        avanco = self.preview_distance - float(msg.lookahead_distance)
        if self.preview_distance <= 0.0 or avanco <= 0.0:
            return erro
        if msg.heading_valid:
            heading = max(
                -self.preview_max_heading,
                min(self.preview_max_heading, float(msg.heading_error)),
            )
            erro += math.tan(heading) * avanco
        if msg.curvature_valid:
            erro += 0.5 * float(msg.curvature) * avanco * avanco
        return erro

    def _operando(self) -> bool:
        """Ha alguem no comando do robo?

        Era so `self._enabled` (autonomia ligada), e isso tornava o servo
        INUTIL no modo RC: ali a autonomia fica DESLIGADA de proposito,
        porque e o que faz o motor_serial_node obedecer ao /cmd_vel do
        joystick em vez do comando autonomo. Resultado: o analogico
        direito nao mexia a cabeca e nao havia sintoma que explicasse.

        Relatado pelo operador em 21/09/2026, junto com os eixos
        invertidos do teleop.
        """
        return self._enabled or self._modo == 'RC'

    def _on_mode(self, msg: String) -> None:
        self._modo = msg.data

    def _on_autonomy(self, msg: Bool) -> None:
        if bool(msg.data) == self._enabled:
            return
        self._enabled = bool(msg.data)
        if not self._enabled:
            # Autonomia desligada: cabeca ao centro. Teleoperacao nao usa
            # a camera superior para dirigir, e uma cabeca torta atrapalha
            # a proxima habilitacao.
            self._mode = HeadRequest.MODE_CENTER
            self._requested_angle = 0.0
            self._request_time = None
            self._scan_direction = 1.0
            self._tracked_angle = 0.0

    # ------------------------------------------------------------------
    def _clamp(self, angle: float) -> float:
        return max(-self.max_angle, min(self.max_angle, angle))

    def _tick(self) -> None:
        now = self._now()
        mode = self._mode

        stale = (
            self._request_time is None
            or (now - self._request_time) > self.request_timeout
        )
        if stale or not self._operando():
            # Watchdog: quem pedia parou de pedir (ou ninguem esta no
            # comando). A cabeca nao fica travada de lado esperando
            # alguem lembrar dela.
            mode = HeadRequest.MODE_CENTER

        if mode == HeadRequest.MODE_SCAN:
            rate = self.scan_rate
            self._target += self._scan_direction * rate * self._period
            if self._target >= self.scan_amplitude:
                self._target = self.scan_amplitude
                self._scan_direction = -1.0
            elif self._target <= -self.scan_amplitude:
                self._target = -self.scan_amplitude
                self._scan_direction = 1.0
            limit = self.max_rate
        elif mode == HeadRequest.MODE_HOLD:
            self._target = self._clamp(self._requested_angle)
            limit = self.max_rate
        elif mode == HeadRequest.MODE_TRACK:
            front_fresh = (
                self._front_time is not None
                and (now - self._front_time) <= self.track_timeout
                and self._front_confidence >= self.track_confidence_min
            )
            # angle_deg em MODE_TRACK carrega a ANTECIPACAO GEOMETRICA
            # calculada pelo controlador a partir da curvatura ja medida
            # pela camera inferior (ver servo_lookahead_gain la). Ela faz
            # o movimento grosso -- sem ela a cabeca so reage ao erro que
            # ja apareceu e chega sempre atrasada na curva.
            lookahead = self._clamp(self._requested_angle)
            if front_fresh:
                # Servovisual sobre a antecipacao: taxa proporcional ao
                # erro, integrada apenas na parte VISUAL. O sinal do erro
                # (metros, +x = direita) e o de graus do servo
                # (+ = direita) ja sao a mesma convencao, entao o ganho e
                # positivo direto -- erro a direita gira a cabeca para a
                # direita, reduzindo o erro.
                rate = self.track_gain * self._front_error_filtered
                if self.track_leak_tau > 0.0:
                    # Vazamento antes de integrar: sem erro sustentado a
                    # parte visual volta sozinha para a antecipacao
                    # geometrica, em vez de segurar o angulo da curva.
                    self._tracked_angle *= max(
                        0.0, 1.0 - self._period / self.track_leak_tau
                    )
                self._tracked_angle = self._clamp(
                    self._tracked_angle + rate * self._period
                )
                self._target = self._clamp(lookahead + self._tracked_angle)
                limit = self.max_rate
            else:
                # Sem deteccao fresca/confiavel, SEGURA o ultimo angulo
                # visual em vez de zera-lo.
                #
                # POR QUE MUDOU (10/09/2026): zerar so fazia sentido
                # enquanto a antecipacao geometrica valia alguma coisa --
                # ela vem da CURVATURA da camera inferior, e a curvatura
                # foi DESLIGADA (ver curvature_enabled, desligada com
                # evidencia cruzada). Com ela em zero, "voltar para o
                # lookahead" virou "voltar para o CENTRO", e isso
                # acontecia exatamente na quebra.
                #
                # MEDIDO em 180s de pista: a superior fica invalida em
                # 13.8% dos quadros (24.7s), e 5 dos 6 episodios longos
                # (17.0s somados) sao seguidos de uma quebra em ate 3s.
                # A assinatura do problema esta na propria estatistica:
                # a superior e MAIS valida com a cabeca virada (94.9%
                # entre +8 e +20 graus) do que centrada (85.0%) -- e a
                # perda mandava a cabeca justamente para o centro, o que
                # realimenta a cegueira.
                #
                # A linha saiu para um lado: esse e o melhor palpite de
                # onde ela esta. Mesmo principio de track_memory no
                # detector e de _last_side na recuperacao do corpo. O
                # decaimento evita o outro extremo -- segurar para sempre
                # um angulo grande quando a linha sumiu de vez.
                if self.track_hold_tau > 0.0:
                    self._tracked_angle *= max(
                        0.0, 1.0 - self._period / self.track_hold_tau
                    )
                else:
                    self._tracked_angle = 0.0
                self._target = self._clamp(lookahead + self._tracked_angle)
                limit = self.max_rate
            self._scan_direction = 1.0
        else:
            self._target = 0.0
            limit = self.center_rate
            self._scan_direction = 1.0

        target = self._clamp(self._target)
        step = limit * self._period
        self._commanded = max(
            self._commanded - step, min(self._commanded + step, target)
        )
        self._send(self._commanded)

    def _send(self, angle: float, force: bool = False) -> None:
        angle = self._clamp(angle)
        if (
            not force
            and self._last_sent is not None
            and abs(angle - self._last_sent) < self.deadband
        ):
            return
        message = Float32()
        message.data = float(angle)
        self.command_pub.publish(message)
        self._last_sent = angle

    def shutdown(self) -> None:
        try:
            self._send(0.0, force=True)
        except Exception:  # noqa: BLE001 - contexto pode ja estar fechado
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
