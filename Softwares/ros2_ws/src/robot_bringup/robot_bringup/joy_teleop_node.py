#!/usr/bin/env python3
"""Controle por joystick (DualShock 4) via Bluetooth da Raspberry.

POR QUE NO PI E NAO NO ESP32: o ESP32-S3 tem Bluetooth 5 **LE apenas**,
sem Bluetooth Classic (BR/EDR). O DualShock 4 e HID sobre Bluetooth
Classic. O ESP32 original conseguia (Bluepad32), o S3 nao. Entao parear
no Pi nao e conveniencia, e o unico caminho possivel.

O firmware NAO muda nada para isto funcionar: este no publica em
/cmd_vel, que o motor_serial_node ja arbitra contra /cmd_vel_auto da
autonomia, e em /head/request, que o head_servo_node ja atende.

Mapeamento (todos os indices sao parametros -- driver e kernel mudam a
numeracao, e remapear nao deveria exigir editar codigo):

    analogico esquerdo   Y = frente/tras      X = giro
    L1 / R1              translacao lateral (strafe)
    analogico direito    X = servo da cabeca (pan)
    L2 (segurar)         HOMEM-MORTO: sem ele, nada se move

O homem-morto nao foi pedido, foi incluido de proposito: analogico com
deriva e um robo que sai andando sozinho, e o custo de segurar um gatilho
e menor que o de perseguir o robo pela sala.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from line_msgs.msg import HeadRequest
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def control_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class JoyTeleopNode(Node):
    """Traduz /joy em /cmd_vel e /head/request."""

    def __init__(self) -> None:
        super().__init__('joy_teleop_node')

        # --- eixos e botoes (indices do DualShock 4 no driver hid-playstation) ---
        self.declare_parameter('axis_vx', 1)        # analogico esq. Y
        self.declare_parameter('axis_wz', 0)        # analogico esq. X
        self.declare_parameter('axis_servo', 3)     # analogico dir. X
        self.declare_parameter('button_strafe_left', 4)   # L1
        self.declare_parameter('button_strafe_right', 5)  # R1
        # L2 vem como EIXO (-1 solto, +1 afundado) no driver moderno.
        self.declare_parameter('deadman_axis', 2)
        self.declare_parameter('deadman_button', -1)  # alternativa, se preferir
        self.declare_parameter('require_deadman', True)

        # SINAIS INVERTIVEIS POR PARAMETRO.
        #
        # RELATADO em 21/09/2026: frente ia para tras, esquerda para a
        # direita e R1 para a esquerda -- os TRES invertidos. A convencao
        # de eixo do driver de joystick nao e estavel entre versoes de
        # kernel e do pacote `joy`, entao fixar sinal no codigo garante
        # que isso volte a acontecer. Aqui e parametro: se inverter de
        # novo, troca no YAML sem recompilar nada.
        #
        # true = multiplica por -1.
        self.declare_parameter('invert_vx', True)
        self.declare_parameter('invert_wz', True)
        self.declare_parameter('invert_vy', True)
        self.declare_parameter('invert_servo', False)

        # TURBO. Pedido do operador: um botao que da mais velocidade sem
        # mudar o limite normal. Multiplica os limites enquanto segurado.
        # O teto real continua sendo maxWheelMps do firmware (0.70 m/s);
        # este fator so levanta o limite do teleop.
        self.declare_parameter('turbo_button', 7)    # R2 como botao
        self.declare_parameter('turbo_axis', -1)     # ou como eixo
        self.declare_parameter('turbo_scale', 1.6)

        # --- limites ---
        self.declare_parameter('max_vx', 0.20)
        self.declare_parameter('max_vy', 0.15)
        self.declare_parameter('max_wz', 1.20)
        self.declare_parameter('max_servo_deg', 45.0)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('rate_hz', 30.0)
        # Sem /joy por este tempo (controle desligou, bateria acabou, saiu
        # de alcance), zera o comando. O ESP32 ja tem watchdog proprio de
        # 200 ms, mas parar aqui tambem evita depender so dele.
        self.declare_parameter('joy_timeout', 0.5)
        # Cala o seguidor no /head/request. Sem isso os dois publicam e o
        # servo fica sendo disputado -- armadilha ja documentada no README.
        self.declare_parameter('take_head_authority', True)

        get = self.get_parameter
        self.axis_vx = int(get('axis_vx').value)
        self.axis_wz = int(get('axis_wz').value)
        self.axis_servo = int(get('axis_servo').value)
        self.btn_left = int(get('button_strafe_left').value)
        self.btn_right = int(get('button_strafe_right').value)
        self.deadman_axis = int(get('deadman_axis').value)
        self.deadman_button = int(get('deadman_button').value)
        self._le_ajustes()
        # Sem este callback os valores ficariam congelados do __init__ e
        # `ros2 param set` nao faria nada -- o que ANULA o motivo de eles
        # serem parametros. Descobrir o sinal certo e tentativa e erro
        # com o controle na mao; ter de reiniciar o no a cada tentativa
        # tornaria isso insuportavel.
        self.add_on_set_parameters_callback(self._on_params)
        self.deadzone = float(get('deadzone').value)
        self.joy_timeout = float(get('joy_timeout').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', control_qos())
        self.head_pub = self.create_publisher(
            HeadRequest, '/head/request', control_qos()
        )
        self.create_subscription(Joy, '/joy', self._on_joy, control_qos())
        # Obedece ao modo escolhido no menu do ESP32. Sem isto o
        # joystick dirigiria mesmo com o seguidor no comando, e os dois
        # brigariam pelo /cmd_vel do robo.
        self._modo = ''
        self.declare_parameter('exigir_modo_rc', True)
        self.exigir_modo_rc = bool(self.get_parameter('exigir_modo_rc').value)
        self.create_subscription(
            String, '/robot/mode',
            lambda m: setattr(self, '_modo', m.data), control_qos()
        )

        self._joy: Joy | None = None
        self._joy_time = 0.0
        self._ativo = False
        self._avisou = False

        periodo = 1.0 / max(1.0, float(get('rate_hz').value))
        self.create_timer(periodo, self._tick)

        if bool(get('take_head_authority').value):
            self._pede_autoridade_da_cabeca()

        self.get_logger().info(
            'Joy teleop pronto. '
            + ('Segure L2 para mover.' if self.require_deadman
               else 'HOMEM-MORTO DESLIGADO -- o robo anda sem gatilho.')
        )

    # ------------------------------------------------------------------
    def _pede_autoridade_da_cabeca(self) -> None:
        """Desliga head_control_enabled no seguidor, se ele estiver no ar.

        Assincrono e tolerante a ausencia: em modo RC puro o seguidor pode
        nem estar rodando, e isso nao e erro.
        """
        cli = self.create_client(
            SetParameters, '/line_follower_node/set_parameters'
        )
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                'line_follower_node ausente; assumindo a cabeca sem disputa.'
            )
            return
        pedido = SetParameters.Request()
        pedido.parameters = [
            Parameter(
                name='head_control_enabled',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=False
                ),
            )
        ]
        cli.call_async(pedido)
        self.get_logger().info(
            'head_control_enabled=false no seguidor (autoridade da cabeca).'
        )

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._joy_time = self.get_clock().now().nanoseconds * 1e-9

    def _eixo(self, msg: Joy, index: int) -> float:
        if index < 0 or index >= len(msg.axes):
            return 0.0
        v = float(msg.axes[index])
        # Zona morta aplicada e depois REESCALADA: sem reescalar, o
        # comando salta de 0 para o valor da zona morta assim que sai
        # dela, e o robo da um tranco.
        if abs(v) < self.deadzone:
            return 0.0
        sinal = 1.0 if v > 0 else -1.0
        return sinal * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _botao(self, msg: Joy, index: int) -> bool:
        if index < 0 or index >= len(msg.buttons):
            return False
        return bool(msg.buttons[index])

    def _le_ajustes(self) -> None:
        """(Re)le os ajustes que o operador troca ao vivo."""
        get = self.get_parameter
        sinal = lambda nome: -1.0 if bool(get(nome).value) else 1.0
        self.sig_vx = sinal('invert_vx')
        self.sig_wz = sinal('invert_wz')
        self.sig_vy = sinal('invert_vy')
        self.sig_servo = sinal('invert_servo')
        self.turbo_button = int(get('turbo_button').value)
        self.turbo_axis = int(get('turbo_axis').value)
        self.turbo_scale = max(1.0, float(get('turbo_scale').value))
        self.max_vx = float(get('max_vx').value)
        self.max_vy = float(get('max_vy').value)
        self.max_wz = float(get('max_wz').value)
        self.max_servo = float(get('max_servo_deg').value)
        self.require_deadman = bool(get('require_deadman').value)

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        nomes = {p.name for p in params}
        vivos = {
            'invert_vx', 'invert_wz', 'invert_vy', 'invert_servo',
            'turbo_button', 'turbo_axis', 'turbo_scale',
            'max_vx', 'max_vy', 'max_wz', 'max_servo_deg',
            'require_deadman',
        }
        if nomes & vivos:
            # O valor novo so aparece em get_parameter DEPOIS que este
            # callback aceita, entao a releitura vai no proximo ciclo.
            self.create_timer(0.0, self._releitura_unica)
        return SetParametersResult(successful=True)

    def _releitura_unica(self) -> None:
        for t in list(self.timers):
            if t.timer_period_ns == 0:
                self.destroy_timer(t)
        self._le_ajustes()
        self.get_logger().info(
            f'ajustes: vx{"-" if self.sig_vx < 0 else "+"} '
            f'wz{"-" if self.sig_wz < 0 else "+"} '
            f'vy{"-" if self.sig_vy < 0 else "+"} '
            f'servo{"-" if self.sig_servo < 0 else "+"}  '
            f'turbo x{self.turbo_scale:.2f}'
        )

    def _turbo(self, msg: Joy) -> bool:
        if self.turbo_button >= 0 and self._botao(msg, self.turbo_button):
            return True
        if 0 <= self.turbo_axis < len(msg.axes):
            return float(msg.axes[self.turbo_axis]) > 0.0
        return False

    def _liberado(self, msg: Joy) -> bool:
        if not self.require_deadman:
            return True
        if self.deadman_button >= 0:
            return self._botao(msg, self.deadman_button)
        if 0 <= self.deadman_axis < len(msg.axes):
            # Gatilho solto = -1, afundado = +1. Meio caminho ja libera.
            return float(msg.axes[self.deadman_axis]) > 0.0
        return False

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        agora = self.get_clock().now().nanoseconds * 1e-9
        msg = self._joy
        velho = msg is None or (agora - self._joy_time) > self.joy_timeout

        if velho:
            if self._ativo:
                self.cmd_pub.publish(Twist())
                self._ativo = False
                self.get_logger().warn(
                    'Sem /joy; comando zerado.', throttle_duration_sec=5.0
                )
            return

        # Fora do modo RC o joystick fica calado -- nem o homem-morto
        # libera. E deliberado: dois controladores publicando no mesmo
        # /cmd_vel e a receita do robo fazer coisa que ninguem pediu.
        if self.exigir_modo_rc and self._modo not in ('RC', ''):
            if self._ativo:
                self.cmd_pub.publish(Twist())
                self._ativo = False
            return

        if not self._liberado(msg):
            if self._ativo:
                self.cmd_pub.publish(Twist())
                self._ativo = False
            return

        # Turbo levanta os limites enquanto segurado.
        k = self.turbo_scale if self._turbo(msg) else 1.0

        twist = Twist()
        twist.linear.x = (self.sig_vx * self._eixo(msg, self.axis_vx)
                          * self.max_vx * k)
        twist.angular.z = (self.sig_wz * self._eixo(msg, self.axis_wz)
                           * self.max_wz * k)

        # Strafe pelos ombros.
        lateral = 0.0
        if self._botao(msg, self.btn_left):
            lateral += 1.0
        if self._botao(msg, self.btn_right):
            lateral -= 1.0
        twist.linear.y = self.sig_vy * lateral * self.max_vy * k

        self.cmd_pub.publish(twist)
        self._ativo = True

        pedido = HeadRequest()
        pedido.header.stamp = self.get_clock().now().to_msg()
        pedido.mode = HeadRequest.MODE_HOLD
        pedido.angle_deg = float(
            self.sig_servo * self._eixo(msg, self.axis_servo)
            * self.max_servo
        )
        self.head_pub.publish(pedido)

    def destroy_node(self) -> bool:
        # Parada explicita ao sair: nao deixar o ultimo comando de pe.
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
