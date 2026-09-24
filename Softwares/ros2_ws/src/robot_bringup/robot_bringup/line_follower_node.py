#!/usr/bin/env python3
"""Supervisor e controlador do seguidor de linha.

Autoridade unica de /cmd_vel_auto. Nao publica /servo/command: pede
apontamento em /head/request e deixa a politica de servo com o
head_servo_node. Assim nao existem dois nos disputando o mesmo comando.

Toda a logica de decisao vive em line_controller.py (sem ROS). Este
arquivo so faz a ponte: assina, converte para unidades do controlador,
chama update() num timer de taxa fixa e publica.

Por que a lei de controle roda no timer e nao no callback da camera:
  - o dt fica previsivel e nao depende do FPS instantaneo da camera;
  - se a camera parar, o timer continua rodando e a maquina de estados
    percebe a perda (na versao anterior o PID vivia dentro do callback,
    entao uma camera travada congelava o ultimo comando indefinidamente).
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from line_msgs.msg import FollowerStatus, HeadRequest, LineDetection
from nav_msgs.msg import Odometry

import rclpy
from rcl_interfaces.msg import SetParametersResult
from robot_bringup.line_controller import (
    Command,
    ControllerConfig,
    FollowerController,
    HeadMode,
    Observation,
    State,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32


def control_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class LineFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('line_follower_node')

        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_auto')
        self.declare_parameter('detection_topic', '/line/detection')
        self.declare_parameter('preview_topic', '/line_front/detection')
        self.declare_parameter('autonomy_state_topic', '/autonomy/state')
        # Cede o controle da cabeca a um diagnostico externo (ex.:
        # scripts/servo_panorama.py) sem precisar derrubar o seguidor.
        # Com False o seguidor para de publicar em /head/request; o
        # watchdog do head_servo_node continua valendo, entao se o
        # diagnostico tambem parar a cabeca volta sozinha ao centro.
        self.declare_parameter('head_control_enabled', True)
        # Simetrico ao de cima, para BANCADA: com False o seguidor para
        # de publicar em cmd_vel, entao da para exercitar a cabeca e a
        # percepcao com a autonomia ligada sem que o robo saia andando.
        # Nao substitui desligar a tensao dos motores -- e conveniencia
        # de teste, nao trava de seguranca. A ausencia de cmd_vel faz o
        # watchdog do ESP32 parar os motores, que e o lado seguro.
        # FALSE por padrao, de proposito. O robo sobe SEM autoridade
        # sobre as rodas e alguem precisa arma-lo explicitamente.
        #
        # POR QUE MUDOU (21/09/2026): com o autostart do systemd, o robo
        # passou a ligar sozinho no boot. Com o default True ele subia
        # com o corpo ARMADO, e bastava a autonomia ser ligada por
        # qualquer motivo para sair andando. Aconteceu: um script que
        # precisava mexer o SERVO ligou a autonomia (o head_servo_node
        # exige isso) e o robo andou, com o operador tendo que tira-lo
        # do chao.
        #
        # O mode_manager liga isto quando o modo SEGUIDOR e escolhido.
        # Se ele nao estiver no ar, o robo simplesmente nao anda -- que
        # e o lado certo para falhar.
        self.declare_parameter('body_control_enabled', False)

        for name, default in _CONTROLLER_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.controller = FollowerController(self._build_config())
        self._head_control = bool(
            self.get_parameter('head_control_enabled').value
        )
        self._body_control = bool(
            self.get_parameter('body_control_enabled').value
        )
        self.add_on_set_parameters_callback(self._on_parameters)

        self.enabled = False
        self._detection: Observation = Observation()
        self._detection_stamp: float | None = None
        self._preview: Observation = Observation()
        self._preview_stamp: float | None = None
        self._odom_speed = 0.0
        self._odom_yaw_rate = 0.0
        self._odom_stamp: float | None = None
        self._servo_angle = 0.0
        self._vision_fps = 0.0
        self._vision_latency = 0.0
        self._last_detection_arrival: float | None = None
        self._last_tick: float | None = None
        self._last_state = State.IDLE

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), control_qos(10)
        )
        self.head_pub = self.create_publisher(
            HeadRequest, '/head/request', control_qos(10)
        )
        self.status_pub = self.create_publisher(
            FollowerStatus, '/follower/status', control_qos(10)
        )

        self.create_subscription(
            LineDetection,
            str(self.get_parameter('detection_topic').value),
            self._on_detection,
            control_qos(),
        )
        self.create_subscription(
            LineDetection,
            str(self.get_parameter('preview_topic').value),
            self._on_preview,
            control_qos(),
        )
        self.create_subscription(
            Odometry, '/odom', self._on_odom, control_qos()
        )
        self.create_subscription(
            Float32, '/servo/state', self._on_servo_state, control_qos(10)
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('autonomy_state_topic').value),
            self._on_autonomy,
            control_qos(10),
        )
        # Compatibilidade: continua aceitando o comando direto que ja era
        # usado na bancada. A autoridade real e /autonomy/state, publicado
        # pelo motor_serial_node (que tambem ouve o menu fisico do ESP32).
        self.create_subscription(
            Bool, '/controle/enable', self._on_autonomy, control_qos(10)
        )

        period = 1.0 / max(1.0, float(self.get_parameter('control_rate').value))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            'Seguidor iniciado DESABILITADO. '
            'Habilite com: ros2 topic pub --once /controle/enable '
            'std_msgs/msg/Bool "{data: true}"'
        )

    # ------------------------------------------------------------------
    def _build_config(self) -> ControllerConfig:
        config = ControllerConfig()
        for name in _CONTROLLER_DEFAULTS:
            setattr(config, name, self._typed(config, name))
        return config

    def _typed(self, config: ControllerConfig, name: str):
        current = getattr(config, name)
        return type(current)(self.get_parameter(name).value)

    def _on_parameters(self, parameters) -> SetParametersResult:
        config = self.controller.config
        staged = {}
        head_control = self._head_control
        body_control = self._body_control
        for parameter in parameters:
            if parameter.name == 'head_control_enabled':
                head_control = bool(parameter.value)
                continue
            if parameter.name == 'body_control_enabled':
                body_control = bool(parameter.value)
                continue
            if not hasattr(config, parameter.name):
                continue
            current = getattr(config, parameter.name)
            try:
                staged[parameter.name] = type(current)(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name}: tipo incompativel',
                )

        merged = {
            name: staged.get(name, getattr(config, name))
            for name in _CONTROLLER_DEFAULTS
        }
        if merged['v_min'] > merged['v_max']:
            return SetParametersResult(
                successful=False, reason='v_min nao pode ser maior que v_max'
            )
        for name in (
            'v_max', 'accel', 'decel', 'wz_max', 'wz_accel',
            'wheel_base_k', 'wheel_max_mps', 'ref_lateral',
            'ref_heading', 'ref_curvature', 'derivative_tau',
        ):
            if merged[name] <= 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{name} deve ser > 0'
                )
        if not 0.0 < merged['arc_ratio'] <= 1.0:
            return SetParametersResult(
                successful=False, reason='arc_ratio deve estar em (0, 1]'
            )
        if not 0.0 <= merged['confidence_min'] <= merged['confidence_ok'] <= 1.0:
            return SetParametersResult(
                successful=False,
                reason='exige 0 <= confidence_min <= confidence_ok <= 1',
            )

        for name, value in staged.items():
            setattr(config, name, value)
        if head_control != self._head_control:
            self._head_control = head_control
            self.get_logger().warn(
                'Controle da cabeca '
                + ('DEVOLVIDO ao seguidor' if head_control
                   else 'CEDIDO a um diagnostico externo')
            )
        if body_control != self._body_control:
            self._body_control = body_control
            self.get_logger().warn(
                'Controle do corpo '
                + ('LIGADO -- o robo volta a andar' if body_control
                   else 'DESLIGADO -- modo bancada, cmd_vel suspenso')
            )
        if staged:
            self.get_logger().info(
                f'Parametros atualizados: {sorted(staged)}'
            )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _to_observation(msg: LineDetection) -> Observation:
        return Observation(
            valid=msg.valid,
            confidence=msg.confidence,
            lateral_error=msg.lateral_error,
            heading_error=msg.heading_error,
            curvature=msg.curvature,
            heading_valid=msg.heading_valid,
            curvature_valid=msg.curvature_valid,
            fit_residual=msg.fit_residual,
            residual_valid=msg.residual_valid,
            lookahead_distance=msg.lookahead_distance,
        )

    def _on_detection(self, msg: LineDetection) -> None:
        now = self._now()
        self._detection = self._to_observation(msg)
        self._detection_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._vision_latency += 0.2 * (msg.pipeline_latency - self._vision_latency)
        if self._last_detection_arrival is not None:
            delta = now - self._last_detection_arrival
            if delta > 1e-4:
                instant = 1.0 / delta
                self._vision_fps += 0.1 * (instant - self._vision_fps)
        self._last_detection_arrival = now

    def _on_preview(self, msg: LineDetection) -> None:
        self._preview = self._to_observation(msg)
        self._preview_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_speed = msg.twist.twist.linear.x
        # Giro MEDIDO pelos encoders (od.wz no firmware, com fusao de
        # IMU no yaw) -- nao o comandado. E o que permite ao alinhamento
        # saber que o robo de fato girou, em vez de supor que obedeceu.
        self._odom_yaw_rate = msg.twist.twist.angular.z
        self._odom_stamp = self._now()

    def _on_servo_state(self, msg: Float32) -> None:
        self._servo_angle = float(msg.data)

    def _on_autonomy(self, msg: Bool) -> None:
        if msg.data == self.enabled:
            return
        self.enabled = bool(msg.data)
        # Sempre reinicia estado e filtros na borda: nem o erro velho nem
        # a rampa de velocidade anterior podem sobreviver a um ciclo de
        # desligar/ligar.
        self.controller.reset()
        self._detection = Observation()
        self._detection_stamp = None
        self._preview = Observation()
        self._preview_stamp = None
        self.cmd_pub.publish(Twist())
        self._request_head(HeadMode.CENTER, 0.0)
        self.get_logger().warn(
            'AUTONOMIA HABILITADA' if self.enabled else 'AUTONOMIA DESABILITADA'
        )

    # ------------------------------------------------------------------
    def _aged(self, observation: Observation, stamp: float | None,
              now: float) -> Observation:
        """Preenche a idade a partir do stamp do frame, nao da chegada."""
        if stamp is None:
            observation.age = math.inf
        else:
            observation.age = max(0.0, now - stamp)
        return observation

    def _tick(self) -> None:
        now = self._now()
        loop_dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        detection = self._aged(self._detection, self._detection_stamp, now)
        preview = self._aged(self._preview, self._preview_stamp, now)

        odom_speed = None
        odom_yaw_rate = None
        if self._odom_stamp is not None and now - self._odom_stamp < 0.5:
            odom_speed = self._odom_speed
            odom_yaw_rate = self._odom_yaw_rate

        command = self.controller.update(
            now=now,
            enabled=self.enabled,
            obs=detection,
            preview=preview,
            odom_speed=odom_speed,
            odom_yaw_rate=odom_yaw_rate,
            servo_angle_deg=self._servo_angle,
        )

        if self._body_control:
            twist = Twist()
            twist.linear.x = float(command.vx)
            twist.linear.y = 0.0   # seguidor nunca usa deslocamento lateral
            twist.angular.z = float(command.wz)
            self.cmd_pub.publish(twist)

        self._request_head(command.head_mode, command.head_angle)
        self._publish_status(command, detection, preview, loop_dt)

        if command.state != self._last_state:
            self.get_logger().info(
                f'{State(self._last_state).name} -> {State(command.state).name} '
                f'(conf={detection.confidence:.2f} idade={detection.age:.3f}s)'
            )
            self._last_state = command.state

    def _request_head(self, mode: HeadMode, angle: float) -> None:
        if not self._head_control:
            return
        request = HeadRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.mode = int(mode)
        request.angle_deg = float(angle)
        self.head_pub.publish(request)

    def _publish_status(
        self,
        command: Command,
        detection: Observation,
        preview: Observation,
        loop_dt: float,
    ) -> None:
        status = FollowerStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
        status.state = int(command.state)
        status.state_name = State(command.state).name
        status.cmd_vx = float(command.vx)
        status.cmd_wz = float(command.wz)
        status.wz_limit = float(command.wz_limit)
        status.lateral_error = float(detection.lateral_error)
        status.heading_error = float(detection.heading_error)
        status.curvature = float(detection.curvature)
        status.confidence = float(detection.confidence)
        status.severity = float(command.severity)
        status.speed_scale = float(
            command.vx / max(self.controller.config.v_max, 1e-6)
        )
        status.vision_age = float(
            detection.age if math.isfinite(detection.age) else -1.0
        )
        status.vision_fps = float(self._vision_fps)
        status.vision_latency = float(self._vision_latency)
        status.loop_dt = float(loop_dt)
        status.preview_active = bool(command.preview_active)
        status.preview_curvature = float(preview.curvature)
        status.preview_ff = float(command.preview_ff)
        status.loss_events = int(self.controller.loss_events)
        status.last_recovery_time = float(self.controller.last_recovery_time)
        status.distance_travelled = float(self.controller.distance)
        status.mean_abs_error = float(self.controller.mean_abs_error)
        self.status_pub.publish(status)

    def shutdown(self) -> None:
        try:
            self.cmd_pub.publish(Twist())
            self._request_head(HeadMode.CENTER, 0.0)
        except Exception:  # noqa: BLE001 - contexto ja pode estar fechado
            pass


# Defaults declarados como parametros ROS. Mantidos em sincronia com
# ControllerConfig por construcao: qualquer campo aqui precisa existir la.
_CONTROLLER_DEFAULTS = {
    'v_max': 0.22,
    'v_min': 0.07,
    'accel': 0.35,
    'decel': 0.60,
    'k_lateral': 9.0,
    'k_heading': 1.6,
    'k_damping': 0.35,
    'k_feedforward': 0.85,
    'derivative_tau': 0.08,
    'wz_max': 2.5,
    'wz_accel': 6.0,
    'wheel_base_k': 0.165,
    'wheel_max_mps': 0.70,
    'arc_ratio': 0.80,
    'ref_lateral': 0.045,
    'ref_heading': 0.55,
    'ref_curvature': 6.0,
    'ref_residual': 0.0028,
    'confidence_ok': 0.45,
    'confidence_min': 0.15,
    'vision_timeout': 0.20,
    'coast_time': 0.25,
    'coast_speed_ratio': 0.5,
    'recovery_time': 3.0,
    'recovery_wz': 0.9,
    'recovery_projection_m': 0.05,
    'relock_frames': 3,
    'align_on_start': True,
    'align_heading_deg': 18.0,
    'align_exit_deg': 7.0,
    'align_wz': 0.9,
    'align_closed_loop_deg': 9.0,
    'align_overshoot_guard': 1.25,
    'align_timeout': 4.0,
    'use_preview': True,
    'preview_steering_enabled': False,
    'preview_speed_enabled': True,
    'preview_recovery_enabled': True,
    'preview_timeout': 0.40,
    'preview_confidence_min': 0.40,
    'preview_memory_time': 2.0,
    'preview_ff_gain': 0.35,
    'preview_heading_gain': 0.40,
    'preview_ff_max_ratio': 0.30,
    'preview_center_tol_deg': 8.0,
    'preview_speed_gain': 1.0,
    'use_servo_bearing': False,
    'servo_bearing_gain': 1.2,
    'head_tracking_enabled': True,
    'preview_lookahead_m': 0.375,
    'servo_lookahead_gain': 0.8,
    'curvature_filter_tau': 0.15,
    'curvature_eval_distance_m': 0.085,
    'curvature_delay_max_s': 1.5,
    'heading_gain_scheduling': True,
    'target_damping': 1.0,
    'heading_gain_min': 1.0,
    'scan_in_recovery': True,
    'scan_in_safe_stop': True,
}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollowerNode()
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
