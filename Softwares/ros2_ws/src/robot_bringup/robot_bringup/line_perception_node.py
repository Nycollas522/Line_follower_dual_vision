#!/usr/bin/env python3
"""No de percepcao de linha. Um processo por camera.

O mesmo executavel serve a camera inferior e a superior: elas fazem o
mesmo trabalho geometrico e mudam apenas de ROI, geometria de chao e
namespace de saida. Manter um codigo so evita que uma correcao aplicada
numa camera fique faltando na outra -- que era o caso na versao anterior,
onde os dois nos de visao divergiram em morfologia e validacao.

Saida principal: <ns>/detection (line_msgs/LineDetection), atomica.
Saidas de compatibilidade (mantidas para rqt_plot e para nao quebrar
nada que ja estava em uso): <ns>/error, <ns>/heading_error, <ns>/status,
<ns>/centroid, <ns>/mask, <ns>/debug_image.
"""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from line_msgs.msg import LineDetection
from robot_bringup.line_geometry import (
    CameraGeometry,
    DetectorConfig,
    LineDetector,
    draw_debug,
)


def sensor_qos(depth: int = 1) -> QoSProfile:
    """Best effort, profundidade 1: sempre o frame mais novo.

    Numa fila reliable com profundidade maior, um pico de CPU faz o no
    processar frames velhos enquanto a realidade ja mudou -- latencia que
    o controlador nao tem como compensar. Descartar e melhor que atrasar.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def control_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


class LinePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('line_perception_node')

        self.declare_parameter('image_topic', '/cam_bottom/image_raw')
        self.declare_parameter('output_namespace', '/line')
        self.declare_parameter('label', 'bottom')
        self.declare_parameter('publish_compat_topics', True)

        # Deteccao
        self.declare_parameter('roi_top', 0.30)
        self.declare_parameter('roi_bottom', 1.00)
        self.declare_parameter('rotate_180', True)
        self.declare_parameter('flank_weight', 0.7)
        self.declare_parameter('work_width', 160)
        self.declare_parameter('bands', 5)
        self.declare_parameter('band_height', 0.10)
        self.declare_parameter('line_width_m', 0.020)
        self.declare_parameter('line_width_tolerance', 0.75)
        self.declare_parameter('min_contrast', 25.0)
        self.declare_parameter('max_white_fraction', 0.45)
        self.declare_parameter('track_weight', 0.6)
        self.declare_parameter('track_sigma', 0.25)
        self.declare_parameter('track_memory', 6)
        self.declare_parameter('min_band_score', 0.25)
        self.declare_parameter('open_kernel', 3)
        # Liga/desliga o PROCESSAMENTO sem derrubar o no. No modo RC a
        # percepcao nao serve para nada e custa caro: medido em
        # 18/09/2026, os dois nos de percepcao somam ~72% de um nucleo.
        # A camera continua publicando, entao o rqt de um PC externo
        # continua enxergando a imagem -- que e o ponto de manter tudo
        # no ar em vez de usar launches separados por modo.
        self.declare_parameter('enabled', True)
        self.declare_parameter('background_kernel_ratio', 3.0)
        self.declare_parameter('curvature_enabled', True)
        self.declare_parameter('min_bands_for_curvature', 4)
        self.declare_parameter('max_curvature', 12.0)
        self.declare_parameter('max_heading_for_curvature', 0.30)

        # Geometria medida com regua
        self.declare_parameter('depth_near_m', 0.035)
        self.declare_parameter('depth_far_m', 0.135)
        self.declare_parameter('width_near_m', 0.090)
        self.declare_parameter('width_far_m', 0.160)
        self.declare_parameter('camera_x_offset_m', 0.0)

        self.label = str(self.get_parameter('label').value)
        namespace = str(self.get_parameter('output_namespace').value).rstrip('/')
        self.publish_compat = bool(
            self.get_parameter('publish_compat_topics').value
        )

        self.bridge = CvBridge()
        self.detector = LineDetector(self._build_config())
        self.add_on_set_parameters_callback(self._on_parameters)

        self.detection_pub = self.create_publisher(
            LineDetection, f'{namespace}/detection', control_qos()
        )
        self.debug_pub = self.create_publisher(
            Image, f'{namespace}/debug_image', sensor_qos()
        )
        self.mask_pub = self.create_publisher(
            Image, f'{namespace}/mask', sensor_qos()
        )
        if self.publish_compat:
            self.error_pub = self.create_publisher(
                Float32, f'{namespace}/error', control_qos(10)
            )
            self.heading_pub = self.create_publisher(
                Float32, f'{namespace}/heading_error', control_qos(10)
            )
            self.status_pub = self.create_publisher(
                Bool, f'{namespace}/status', control_qos(10)
            )
            self.centroid_pub = self.create_publisher(
                Point, f'{namespace}/centroid', control_qos(10)
            )

        # A inscricao e CRIADA E DESTRUIDA conforme 'enabled'.
        #
        # So pular o processamento nao basta: MEDIDO em 18/09/2026, com a
        # inscricao viva e o callback retornando de imediato, os dois nos
        # de percepcao caiam de 57.8% para 48.2% -- o grosso do custo e
        # RECEBER e desserializar a imagem, nao processa-la. Cancelando a
        # inscricao o no para de pagar esse transporte.
        #
        # A CAMERA continua publicando nos dois casos, de proposito: e o
        # que permite ver a imagem por rqt de um PC externo mesmo em modo
        # RC, sem manter a percepcao ligada.
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._image_sub = None
        self._aplica_enabled(bool(self.get_parameter('enabled').value))
        self.add_on_set_parameters_callback(self._on_params)

        self._frames = 0
        self._fps = 0.0
        self._last_stamp: float | None = None
        self._cpu_ms = 0.0
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f'Percepcao "{self.label}" em {self._image_topic} -> {namespace}/detection'
        )

    # ------------------------------------------------------------------
    def _build_config(self) -> DetectorConfig:
        get = self.get_parameter
        return DetectorConfig(
            roi_top=float(get('roi_top').value),
            roi_bottom=float(get('roi_bottom').value),
            rotate_180=bool(get('rotate_180').value),
            flank_weight=float(get('flank_weight').value),
            work_width=int(get('work_width').value),
            bands=int(get('bands').value),
            band_height=float(get('band_height').value),
            line_width_m=float(get('line_width_m').value),
            line_width_tolerance=float(get('line_width_tolerance').value),
            min_contrast=float(get('min_contrast').value),
            max_white_fraction=float(get('max_white_fraction').value),
            track_weight=float(get('track_weight').value),
            track_sigma=float(get('track_sigma').value),
            track_memory=int(get('track_memory').value),
            min_band_score=float(get('min_band_score').value),
            open_kernel=int(get('open_kernel').value),
            background_kernel_ratio=float(
                get('background_kernel_ratio').value
            ),
            curvature_enabled=bool(get('curvature_enabled').value),
            min_bands_for_curvature=int(get('min_bands_for_curvature').value),
            max_curvature=float(get('max_curvature').value),
            max_heading_for_curvature=float(get('max_heading_for_curvature').value),
            geometry=CameraGeometry(
                depth_near_m=float(get('depth_near_m').value),
                depth_far_m=float(get('depth_far_m').value),
                width_near_m=float(get('width_near_m').value),
                width_far_m=float(get('width_far_m').value),
                camera_x_offset_m=float(get('camera_x_offset_m').value),
            ),
        )

    def _on_parameters(self, parameters) -> SetParametersResult:
        """Valida antes de aplicar. Nenhum parametro e aplicado se um
        deles for invalido, para o detector nunca rodar meio configurado."""
        pending = {p.name: p.value for p in parameters}

        def value(name, default):
            return pending.get(name, default)

        roi_top = float(value('roi_top', self.detector.config.roi_top))
        roi_bottom = float(value('roi_bottom', self.detector.config.roi_bottom))
        if not 0.0 <= roi_top < roi_bottom <= 1.0:
            return SetParametersResult(
                successful=False,
                reason='exige 0 <= roi_top < roi_bottom <= 1',
            )
        if int(value('bands', self.detector.config.bands)) < 1:
            return SetParametersResult(
                successful=False, reason='bands deve ser >= 1'
            )
        if int(value('work_width', self.detector.config.work_width)) < 32:
            return SetParametersResult(
                successful=False, reason='work_width deve ser >= 32'
            )
        geometry = self.detector.config.geometry
        for name, current in (
            ('width_near_m', geometry.width_near_m),
            ('width_far_m', geometry.width_far_m),
            ('line_width_m', self.detector.config.line_width_m),
        ):
            if float(value(name, current)) <= 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{name} deve ser > 0'
                )

        for parameter in parameters:
            if self.has_parameter(parameter.name):
                self._set_local(parameter)
        return SetParametersResult(successful=True)

    def _set_local(self, parameter) -> None:
        """Aplica no dataclass. Chamado depois da validacao."""
        config = self.detector.config
        geometry_fields = {
            'depth_near_m', 'depth_far_m', 'width_near_m',
            'width_far_m', 'camera_x_offset_m',
        }
        name = parameter.name
        if name in geometry_fields:
            setattr(config.geometry, name, float(parameter.value))
        elif hasattr(config, name):
            current = getattr(config, name)
            setattr(config, name, type(current)(parameter.value))

    # ------------------------------------------------------------------
    def _aplica_enabled(self, ligado: bool) -> None:
        if ligado and self._image_sub is None:
            # O rastreio de antes do desligamento nao vale mais: em PARADO
            # o robo costuma ser reposicionado a mao, e a predicao velha
            # (com _miss_count parado em zero) puxaria a busca para onde a
            # linha ESTAVA, favorecendo um reflexo perto dali.
            self.detector.reset()
            self._image_sub = self.create_subscription(
                Image, self._image_topic, self._on_image, sensor_qos()
            )
            self.get_logger().info('Percepcao LIGADA (inscrita na camera).')
        elif not ligado and self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            self.get_logger().info(
                'Percepcao DESLIGADA (inscricao cancelada; a camera '
                'continua publicando para o rqt).'
            )

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'enabled':
                self._aplica_enabled(bool(p.value))
        return SetParametersResult(successful=True)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:  # noqa: BLE001 - cv_bridge lanca varios tipos
            self.get_logger().error(f'Falha ao converter imagem: {error}')
            return

        observation = self.detector.process(frame)

        now = self.get_clock().now()
        stamp_seconds = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        now_seconds = now.nanoseconds * 1e-9
        # Se a camera nao carimba o header, o stamp fica zerado e a
        # latencia calculada seria absurda: nesse caso usamos "agora",
        # e o controlador continua protegido pelo seu proprio watchdog.
        if stamp_seconds <= 0.0:
            header_stamp = now.to_msg()
            latency = observation.processing_time
        else:
            header_stamp = msg.header.stamp
            latency = max(0.0, now_seconds - stamp_seconds)

        detection = LineDetection()
        detection.header.stamp = header_stamp
        detection.header.frame_id = msg.header.frame_id or self.label
        detection.valid = bool(observation.valid)
        detection.confidence = float(observation.confidence)
        detection.lateral_error = float(observation.lateral_error)
        detection.heading_error = float(observation.heading_error)
        detection.curvature = float(observation.curvature)
        detection.lookahead_distance = float(observation.lookahead_distance)
        detection.line_width = float(observation.line_width)
        detection.bands_valid = int(observation.bands_valid)
        detection.bands_total = int(observation.bands_total)
        detection.heading_valid = bool(observation.heading_valid)
        detection.curvature_valid = bool(observation.curvature_valid)
        detection.fit_residual = float(observation.fit_residual)
        detection.residual_valid = bool(observation.residual_valid)
        detection.contrast = float(observation.contrast)
        detection.processing_time = float(observation.processing_time)
        detection.pipeline_latency = float(latency)
        self.detection_pub.publish(detection)

        if self.publish_compat:
            self._publish_compat(observation, msg)

        self._publish_debug(frame, observation, msg)

        self._frames += 1
        self._cpu_ms += observation.processing_time * 1000.0
        if self._last_stamp is not None:
            delta = now_seconds - self._last_stamp
            if delta > 1e-4:
                instant = 1.0 / delta
                self._fps = instant if self._fps == 0.0 else (
                    self._fps + 0.1 * (instant - self._fps)
                )
        self._last_stamp = now_seconds

    def _publish_compat(self, observation, msg: Image) -> None:
        error = Float32()
        error.data = float(observation.lateral_error) if observation.valid else 0.0
        self.error_pub.publish(error)

        heading = Float32()
        heading.data = (
            float(observation.heading_error)
            if observation.valid and observation.heading_valid
            else 0.0
        )
        self.heading_pub.publish(heading)

        status = Bool()
        status.data = bool(observation.valid)
        self.status_pub.publish(status)

        centroid = Point()
        if observation.samples:
            centroid.x = float(observation.samples[-1].x_norm)
            centroid.y = float(observation.samples[-1].depth_m)
        centroid.z = float(observation.confidence)
        self.centroid_pub.publish(centroid)

    def _publish_debug(self, frame, observation, msg: Image) -> None:
        if (
            self.mask_pub.get_subscription_count() > 0
            and observation.mask is not None
        ):
            mask_msg = self.bridge.cv2_to_imgmsg(
                np.ascontiguousarray(observation.mask), encoding='mono8'
            )
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

        if self.debug_pub.get_subscription_count() > 0:
            display = draw_debug(
                frame, observation, self.detector.config, self.label
            )
            debug_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def _report(self) -> None:
        if self._frames == 0:
            self.get_logger().warn(
                f'Percepcao "{self.label}": nenhum frame em 5 s. '
                'Verifique se a camera esta publicando.'
            )
            return
        self.get_logger().info(
            f'Percepcao "{self.label}": {self._fps:.1f} fps, '
            f'{self._cpu_ms / self._frames:.1f} ms/frame'
        )
        self._frames = 0
        self._cpu_ms = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
