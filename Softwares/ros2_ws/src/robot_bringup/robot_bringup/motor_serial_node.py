#!/usr/bin/env python3
"""Ponte serial ROS 2 <-> ESP32 e arbitro entre teleoperacao e autonomia.

O PROTOCOLO SERIAL NAO FOI ALTERADO. Este no fala exatamente o que
SerialProtocol.cpp entende e le exatamente o que main.cpp imprime:

  Pi -> ESP32   TWIST,vx,vy,wz | SERVO,ang | STOP | CONTROL_STATE,0|1
                SET_PID,kp,ki,kd | SAVE_SETTINGS | STATUS
  ESP32 -> Pi   ENC,4x | ODOM,6x | WHEEL,4x | TARGET,4x | PWM,4x
                IMU,6x | SERVO_STATE,1x | BATT,1x | READY,... |
                STATUS,... | MENU,CONTROL_TOGGLE

Mudancas em relacao a versao anterior, todas do lado do Pi:

1. Leitura por buffer de bytes em vez de readline() com timeout=0.
   Com timeout zero o readline() do pyserial retorna o que estiver no
   buffer do SO mesmo sem ter chegado o '\\n', o que partia linhas de
   telemetria ao meio -- era a origem dos avisos "Telemetria invalida".
   Agora os bytes sao acumulados e so linhas completas sao processadas.

2. Arbitragem explicita de autoridade. /controle/enable e COMANDO
   (qualquer um pode publicar, inclusive o teclado). /autonomy/state e
   ESTADO, publicado so por este no, que e quem tambem recebe o toggle do
   menu fisico do ESP32. Isso acaba com o no publicando e assinando o
   mesmo topico, que era o arranjo anterior.

3. Watchdog por fonte. Cada modo tem sua propria idade de comando; um
   /cmd_vel_auto velho nao mantem o robo andando so porque a teleoperacao
   esta ativa em outro topico (e vice-versa).

4. TARGET e PWM, que o firmware ja enviava e ninguem lia, viram topicos.
   Sao o que permite ver no rosbag se um PWM saturou ou se uma roda nao
   alcancou o alvo -- diagnostico que antes so existia no OLED.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32, Int32MultiArray, String
from tf2_ros import TransformBroadcaster

WHEEL_NAMES = ['wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']


def control_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class MotorSerialNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_crc', True)
        # Segundos apos o inicio do no em que um pedido de
        # desligamento e IGNORADO. Protege contra ciclo de boot.
        self.declare_parameter('shutdown_grace', 60.0)
        self.declare_parameter('manual_cmd_topic', '/cmd_vel')
        self.declare_parameter('auto_cmd_topic', '/cmd_vel_auto')
        self.declare_parameter('enable_command_topic', '/controle/enable')
        self.declare_parameter('autonomy_state_topic', '/autonomy/state')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_tf', True)
        # Limites finais, aplicados depois de qualquer controlador. O
        # firmware satura de novo por roda; esta e a primeira barreira.
        self.declare_parameter('max_vx', 0.45)
        self.declare_parameter('max_vy', 0.45)
        self.declare_parameter('max_wz', 2.5)
        # OFFSET DO SUPORTE. O suporte da camera superior nao aponta
        # exatamente ao longo do eixo do robo. Fica aqui, junto com a
        # inversao de sinal, porque esta e a UNICA fronteira entre a
        # convencao ROS e o hardware -- todo o resto do sistema
        # (HeadRequest, head_servo_node, line_controller) raciocina em
        # angulo verdadeiro, com 0 = reto a frente.
        # MEDIDO (10/09/2026) varrendo o pan e comparando a leitura da
        # camera superior com a da inferior: discordam -3.65mm por grau,
        # linear, cruzando zero em pan = -3.03 graus.
        self.declare_parameter('servo_offset_deg', -3.0)
        self.declare_parameter('min_servo_angle', -90.0)
        self.declare_parameter('max_servo_angle', 90.0)
        self.declare_parameter('cmd_timeout', 0.30)
        # Taxa de leitura da serial. O firmware manda telemetria a cada
        # 200ms (TELEMETRY_MS); checar o buffer bem mais rapido que isso
        # so gasta CPU -- medido no robo real: a 200Hz (5ms) este no
        # sozinho consumia ~40% de uma CPU do Pi e derrubava o FPS das
        # cameras por contencao de escalonamento. 50Hz (20ms) ainda da
        # folga de sobra (10x a taxa real de chegada de dados) por uma
        # fracao do custo.
        self.declare_parameter('serial_read_rate', 50.0)
        self.declare_parameter('twist_rate', 50.0)
        self.declare_parameter('state_rate', 5.0)
        self.declare_parameter('start_autonomy_enabled', False)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.servo_offset = float(
            self.get_parameter('servo_offset_deg').value
        )
        self.min_servo = float(self.get_parameter('min_servo_angle').value)
        self.max_servo = float(self.get_parameter('max_servo_angle').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)

        self._lock = threading.Lock()
        self._rx = bytearray()
        self._manual = (0.0, 0.0, 0.0)
        self._manual_time: float | None = None
        self._auto = (0.0, 0.0, 0.0)
        self._auto_time: float | None = None
        self._autonomy = bool(
            self.get_parameter('start_autonomy_enabled').value
        )
        self._last_servo = 0.0
        self._battery = 0.0
        self._last_telemetry: float | None = None
        self._bad_lines = 0

        # --- publicadores -------------------------------------------
        self.autonomy_pub = self.create_publisher(
            Bool, str(self.get_parameter('autonomy_state_topic').value),
            control_qos(),
        )
        self.encoder_pub = self.create_publisher(
            Int32MultiArray, '/wheel_encoder_ticks', control_qos()
        )
        self.wheel_pub = self.create_publisher(
            JointState, '/wheel_states', control_qos()
        )
        self.wheel_target_pub = self.create_publisher(
            JointState, '/wheel_targets', control_qos()
        )
        self.wheel_pwm_pub = self.create_publisher(
            Int32MultiArray, '/wheel_pwm', control_qos()
        )
        self.imu_pub = self.create_publisher(
            Imu, '/imu/data_raw', control_qos()
        )
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', control_qos()
        )
        # Patinagem MEDIDA: wz dos encoders menos wz do giroscopio. Num
        # mecanum os roletes patinam por projeto, e ate agora nao havia
        # como separar "patinou" de "o motor nao entregou" -- as duas
        # coisas aparecem igual na odometria de roda. Agora aparecem
        # separadas, e isso alimenta direto a analise de corrida.
        self.slip_pub = self.create_publisher(
            Float32, '/wheel_slip', control_qos()
        )
        # Ponte do menu do ESP32 para o gerente de modo. O firmware PEDE
        # (MENU,MODE,n) e o Pi CONFIRMA (MODE_STATE,n) -- assim o OLED
        # nunca anuncia um modo que o Pi nao esta executando de fato.
        self.mode_req_pub = self.create_publisher(
            String, '/robot/mode_request', control_qos()
        )
        # Perfil de velocidade escolhido no menu do ESP32 (vperfil na
        # linha SETTINGS). TRANSIENT_LOCAL: o mode_manager pode subir
        # depois deste no e ainda assim recebe o ultimo perfil.
        self.speed_req_pub = self.create_publisher(
            String, '/robot/speed_request',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.servo_state_pub = self.create_publisher(
            Float32, '/servo/state', control_qos()
        )
        self.battery_pub = self.create_publisher(
            Float32, '/battery/voltage', control_qos()
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if bool(self.get_parameter('publish_tf').value)
            else None
        )

        # --- assinaturas --------------------------------------------
        self.create_subscription(
            Twist, str(self.get_parameter('manual_cmd_topic').value),
            self._on_manual, control_qos(),
        )
        self.create_subscription(
            Twist, str(self.get_parameter('auto_cmd_topic').value),
            self._on_auto, control_qos(),
        )
        self.create_subscription(
            Bool, str(self.get_parameter('enable_command_topic').value),
            self._on_enable_command, control_qos(),
        )
        self.create_subscription(
            Float32, '/servo/command', self._on_servo_command, control_qos()
        )

        self._open_serial()
        # Registra no log as configuracoes que o ESP32 tem em vigor, sem
        # esperar um reboot dele.
        self._write('GET_SETTINGS\n')

        twist_period = 1.0 / max(
            1.0, float(self.get_parameter('twist_rate').value)
        )
        state_period = 1.0 / max(
            1.0, float(self.get_parameter('state_rate').value)
        )
        read_period = 1.0 / max(
            1.0, float(self.get_parameter('serial_read_rate').value)
        )
        self.create_timer(read_period, self._read_serial)
        self.create_timer(twist_period, self._send_twist)
        self.create_timer(state_period, self._publish_autonomy_state)
        self.create_timer(2.0, self._health_check)

    # ------------------------------------------------------------------
    def _open_serial(self) -> None:
        port = str(self.get_parameter('port').value)
        baudrate = int(self.get_parameter('baudrate').value)
        self.serial_crc = bool(self.get_parameter('serial_crc').value)
        # Inicio do no, para a carencia de desligamento. TEM de ser aqui:
        # criado so no primeiro pedido, a carencia contava a partir DELE
        # -- um pedido legitimo horas depois do boot era ignorado ("no
        # ar ha 0s"), e dois disparos espurios separados por mais que a
        # carencia desligavam o Pi.
        self._t0_node = time.monotonic()
        self.shutdown_grace = float(
            self.get_parameter('shutdown_grace').value
        )
        try:
            self.ser = serial.Serial(
                port, baudrate, timeout=0.0, write_timeout=0.2
            )
        except serial.SerialException as error:
            self.get_logger().fatal(f'Nao foi possivel abrir {port}: {error}')
            raise
        # O ESP32-S3 reinicia ao abrir a porta CDC; esperar antes de
        # limpar evita processar o lixo do boot como telemetria.
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.get_logger().info(f'ESP32 conectado em {port} @ {baudrate}')
        # Info do host no OLED. 5 s basta: IP e hostname mudam raramente,
        # e so escreve na serial o que de fato mudou.
        self._info_cache: dict = {}
        self.create_timer(5.0, self._envia_info)
        self._modo_atual = ''
        self.create_subscription(
            String, '/robot/mode', self._on_mode, control_qos()
        )
        # Alinha o firmware ao estado que este no considera verdadeiro.
        self._write(f'CONTROL_STATE,{int(self._autonomy)}\n')

    @staticmethod
    def _crc8(data: bytes) -> int:
        """CRC-8 poly 0x07, igual ao SerialProtocol::crc8 do firmware."""
        c = 0
        for byte in data:
            c ^= byte
            for _ in range(8):
                c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        return c

    def _write(self, line: str) -> bool:
        # CRC so no sentido Pi -> ESP32. Telemetria corrompida o parser
        # ja descarta (cabecalho desconhecido); COMANDO corrompido vira
        # velocidade errada num robo que anda -- ex.: um byte trocado em
        # "TWIST,0.15,0.00,0.30" da "3.30", giro 10x maior, aceito calado.
        # O firmware aceita linha sem sufixo, entao da para depurar na mao.
        corpo = line.rstrip('\n')
        if self.serial_crc and corpo:
            linha = f'{corpo}*{self._crc8(corpo.encode("ascii")):02X}\n'
        else:
            linha = line
        try:
            with self._lock:
                self.ser.write(linha.encode('ascii'))
            return True
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(
                f'Falha ao escrever na serial: {error}', throttle_duration_sec=2.0
            )
            return False

    @staticmethod
    def _ip_local() -> str:
        """IP da interface que sai para a rede.

        O truque do socket UDP nao envia pacote nenhum: so pergunta ao
        sistema qual origem ele usaria para aquele destino. Resolve o
        caso de varias interfaces (wlan + eth + docker) sem depender de
        parsear a saida de comando externo.
        """
        import socket
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.connect(('8.8.8.8', 1))
                return sock.getsockname()[0]
            finally:
                sock.close()
        except OSError:
            return 'sem rede'

    def _envia_info(self) -> None:
        """Empurra IP, hostname e estado para o OLED do ESP32.

        Serve para uma coisa bem concreta: descobrir o IP para abrir SSH
        sem monitor, sem teclado e sem caçar no roteador. O firmware nao
        interpreta o conteudo -- so guarda e desenha -- entao acrescentar
        informacao aqui nao exige firmware novo.
        """
        import socket
        ip = self._ip_local()
        if ip != self._info_cache.get(0):
            self._info_cache[0] = ip
            self._write(f'SET_INFO,0,{ip[:21]}\n')
        nome = socket.gethostname()
        if nome != self._info_cache.get(1):
            self._info_cache[1] = nome
            self._write(f'SET_INFO,1,{nome[:21]}\n')
        estado = 'autonomia ON' if self._autonomy else 'autonomia OFF'
        if estado != self._info_cache.get(2):
            self._info_cache[2] = estado
            self._write(f'SET_INFO,2,{estado}\n')

    def _desliga_o_pi(self) -> None:
        """Desliga a Raspberry de verdade, a pedido do menu do ESP32.

        Cortar a energia de um Pi ligado corrompe o cartao SD, e o
        operador nao tem como saber por fora quando o sistema parou de
        escrever. Aqui ele pede pelo menu e o Pi desliga direito.

        O ESP32 e alimentado PELO Pi, entao o OLED apagar e o sinal de
        que ja da para cortar a chave -- nao ha como o firmware avisar
        depois, porque ele cai junto.

        Requer /etc/sudoers.d/robo-poweroff. Sem a regra o comando falha
        e o operador descobre pelo log, em vez de esperar um
        desligamento que nunca vem.
        """
        # Para os motores ANTES: o desligamento leva alguns segundos e o
        # robo nao pode passa-los andando.
        for _ in range(5):
            self._write('TWIST,0.0000,0.0000,0.0000\n')
        self._write('STOP\n')
        self.get_logger().warn('Parando motores e desligando a Raspberry.')

        import subprocess
        try:
            r = subprocess.run(
                ['sudo', '-n', '/sbin/shutdown', '-h', 'now'],
                capture_output=True, text=True, timeout=10,
            )
            if r.returncode != 0:
                self.get_logger().error(
                    'FALHA ao desligar: '
                    f'{(r.stderr or r.stdout).strip()!r}. '
                    'Falta /etc/sudoers.d/robo-poweroff? '
                    'Veja "Desligar pelo menu" no README.'
                )
        except Exception as erro:
            self.get_logger().error(f'FALHA ao desligar: {erro}')

    def _repassa_perfil(self, linha: str) -> None:
        """Publica o perfil de velocidade (vperfil) para o mode_manager."""
        campos = dict(
            item.split('=', 1) for item in linha.split(',')[1:] if '=' in item
        )
        nome = {'0': 'SUAVE', '1': 'MEDIA', '2': 'RAPIDA'}.get(
            campos.get('vperfil', '')
        )
        if nome is None:
            return   # firmware sem perfil: fica o que o YAML definiu
        msg = String()
        msg.data = nome
        self.speed_req_pub.publish(msg)

    def _on_mode(self, msg: String) -> None:
        """Confirma no OLED o modo que o Pi esta de fato executando."""
        if msg.data == self._modo_atual:
            return
        self._modo_atual = msg.data
        codigo = {'PARADO': 0, 'SEGUIDOR': 1, 'RC': 2}.get(msg.data)
        if codigo is not None:
            self._write(f'MODE_STATE,{codigo}\n')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _clamp_twist(self, msg: Twist):
        # NaN/inf viram PARADA. max(-l, min(l, nan)) devolve +l -- um
        # comando NaN saia daqui como velocidade maxima.
        valores = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in valores):
            self.get_logger().error(
                f'Twist nao finito {valores}; enviando parada.',
                throttle_duration_sec=1.0,
            )
            return (0.0, 0.0, 0.0)
        return (
            max(-self.max_vx, min(self.max_vx, msg.linear.x)),
            max(-self.max_vy, min(self.max_vy, msg.linear.y)),
            max(-self.max_wz, min(self.max_wz, msg.angular.z)),
        )

    def _on_manual(self, msg: Twist) -> None:
        self._manual = self._clamp_twist(msg)
        self._manual_time = self._now()

    def _on_auto(self, msg: Twist) -> None:
        self._auto = self._clamp_twist(msg)
        self._auto_time = self._now()

    def _on_enable_command(self, msg: Bool) -> None:
        self._set_autonomy(bool(msg.data), source='topico')

    def _set_autonomy(self, enabled: bool, source: str) -> None:
        if enabled == self._autonomy:
            return
        self._autonomy = enabled
        # Ao trocar de modo, o comando da fonte que sai e descartado na
        # hora: nenhum valor antigo pode sobreviver a troca.
        self._auto = (0.0, 0.0, 0.0)
        self._auto_time = None
        self._manual = (0.0, 0.0, 0.0)
        self._manual_time = None
        self._write('TWIST,0.0000,0.0000,0.0000\n')
        self._write(f'CONTROL_STATE,{int(enabled)}\n')
        self._publish_autonomy_state()
        self.get_logger().warn(
            f'Autonomia {"HABILITADA" if enabled else "DESABILITADA"} ({source})'
        )

    def _publish_autonomy_state(self) -> None:
        message = Bool()
        message.data = self._autonomy
        self.autonomy_pub.publish(message)

    def _on_servo_command(self, msg: Float32) -> None:
        angle = max(self.min_servo, min(self.max_servo, float(msg.data)))
        self._last_servo = angle
        # MEDIDO NO ROBO: o servo fisico gira para a ESQUERDA quando se
        # manda +angulo, invertido em relacao a convencao ROS do resto
        # do sistema (+ = direita, documentada em HeadRequest.msg). O
        # ajuste fica isolado aqui, na unica fronteira entre ROS e o
        # protocolo fisico -- ninguem mais no sistema (HeadRequest,
        # head_servo_node, line_controller) precisa saber disso.
        # Offset do suporte somado no angulo VERDADEIRO, antes da
        # inversao -- assim os dois ajustes ficam na mesma fronteira.
        physical_angle = -(angle + self.servo_offset)
        self._write(f'SERVO,{physical_angle:.2f}\n')

    def _send_twist(self) -> None:
        if self._autonomy:
            twist, stamp = self._auto, self._auto_time
        else:
            twist, stamp = self._manual, self._manual_time

        if stamp is None or (self._now() - stamp) > self.cmd_timeout:
            # Watchdog do Pi. O firmware tem o seu (CMD_TIMEOUT_MS = 200ms),
            # mas ele so dispara se a serial parar por completo. Este aqui
            # pega o caso de o no controlador morrer com a serial viva.
            twist = (0.0, 0.0, 0.0)

        vx, vy, wz = twist
        self._write(f'TWIST,{vx:.4f},{vy:.4f},{wz:.4f}\n')

    # ------------------------------------------------------------------
    def _read_serial(self) -> None:
        try:
            with self._lock:
                waiting = self.ser.in_waiting
                if waiting:
                    self._rx.extend(self.ser.read(waiting))
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(
                f'Falha ao ler a serial: {error}', throttle_duration_sec=2.0
            )
            return

        # Só processa linhas completas. O resto fica no buffer para a
        # proxima chamada -- e isso que impede telemetria partida ao meio.
        while True:
            index = self._rx.find(b'\n')
            if index < 0:
                break
            raw = bytes(self._rx[:index])
            del self._rx[: index + 1]
            line = raw.decode('utf-8', errors='replace').strip()
            if line:
                self._process_line(line)

        if len(self._rx) > 4096:
            # Sem '\n' em 4 KB o fluxo esta corrompido; descartar e melhor
            # que crescer o buffer sem limite.
            self.get_logger().warn('Buffer serial sem terminador; descartando.')
            self._rx.clear()

    def _process_line(self, line: str) -> None:
        if line.startswith('READY'):
            self.get_logger().info(f'ESP32: {line}')
            # O ESP32 reiniciou: reenviar o estado, senao ele fica achando
            # que a autonomia esta no default dele.
            self._write(f'CONTROL_STATE,{int(self._autonomy)}\n')
            return
        if line.startswith('STATUS') or line.startswith('OK,'):
            self.get_logger().info(f'ESP32: {line}')
            return
        if line.startswith('SETTINGS,'):
            # Configuracoes em vigor no ESP32 (NVS). O firmware manda no
            # boot, a pedido e depois de cada SALVAR -- entao este log
            # registra, com hora, qualquer ajuste feito pelo menu do OLED.
            self.get_logger().info(f'ESP32: {line}')
            self._repassa_perfil(line)
            return
        if line.startswith('ERROR') or line.startswith('ERR,'):
            self.get_logger().error(f'ESP32: {line}')
            return

        fields = line.split(',')
        head = fields[0]

        if fields == ['MENU', 'SHUTDOWN_REQ']:
            # REGISTRA SEMPRE e so desliga depois da carencia de boot
            # (shutdown_grace). Requer /etc/sudoers.d/robo-poweroff.
            #
            # POR QUE O REGISTRO: a primeira versao desligava de verdade e o
            # Pi passou a desligar sozinho. O caminho de confirmacao que
            # eu escrevi no firmware era INALCANCAVEL (ficava numa cadeia
            # else-if depois do ramo que captura toda pressao longa),
            # entao a causa real nunca foi encontrada -- e implementar de
            # novo sem descobri-la seria repetir o erro.
            #
            # Este log e o instrumento: se o pedido aparecer sem ninguem
            # ter confirmado no menu, o culpado e o firmware e da para
            # ver a hora. Se o Pi desligar sem este log, a causa nunca
            # foi esta funcionalidade.
            # Tempo MONOTONICO desde o inicio do no. self._now() devolve
            # o relogio de epoca (1.79e9 s), inutil para correlacionar
            # com "ha quanto tempo o robo esta ligado" -- que e
            # exatamente o que este log existe para responder.
            agora = time.monotonic()
            inicio = self._t0_node
            anterior = getattr(self, '_ultimo_shutdown_req', None)
            self._ultimo_shutdown_req = agora
            self._n_shutdown_req = getattr(self, '_n_shutdown_req', 0) + 1
            desde = ('primeiro' if anterior is None
                     else f'anterior ha {agora - anterior:.1f}s')
            self.get_logger().warn(
                f'PEDIDO DE DESLIGAMENTO #{self._n_shutdown_req} do menu '
                f'do ESP32 (no ar ha {agora - inicio:.1f}s, {desde}).'
            )

            # CARENCIA DE BOOT. Um pedido espurio logo apos ligar criaria
            # um ciclo: desliga, o operador religa, desliga de novo. Foi
            # exatamente esse o sintoma relatado ("desligar sempre depois
            # de um tempo"), e a carencia quebra o pior caso -- da tempo
            # de chegar ao menu e desabilitar antes de perder a maquina.
            if agora - inicio < self.shutdown_grace:
                self.get_logger().error(
                    f'IGNORADO: o no esta no ar ha so {agora - inicio:.0f}s '
                    f'(carencia de {self.shutdown_grace:.0f}s). Se este '
                    'pedido nao foi seu, ha um disparo espurio -- e esta '
                    'linha e a prova, com o horario.'
                )
                return

            self._desliga_o_pi()
            return

        if len(fields) == 3 and fields[0] == 'MENU' and fields[1] == 'MODE':
            nome = {'0': 'PARADO', '1': 'SEGUIDOR', '2': 'RC'}.get(fields[2])
            if nome:
                msg = String()
                msg.data = nome
                self.mode_req_pub.publish(msg)
                self.get_logger().info(f'Menu do ESP32 pediu modo {nome}')
            return

        if fields == ['MENU', 'CONTROL_TOGGLE']:
            # O botao fisico do ESP32 e uma fonte legitima de comando.
            self._set_autonomy(not self._autonomy, source='menu do ESP32')
            return

        try:
            if head == 'BATT' and len(fields) == 2:
                self._battery = float(fields[1])
                self._publish_float(self.battery_pub, self._battery)
            elif head == 'SERVO_STATE' and len(fields) == 2:
                # Desfaz a mesma inversao no sentido contrario, para que
                # /servo/state volte a estar na convencao ROS (+ =
                # direita) igual ao que foi pedido em /servo/command.
                self._last_servo = -float(fields[1]) - self.servo_offset
                self._publish_float(self.servo_state_pub, self._last_servo)
            elif head == 'ENC' and len(fields) == 5:
                message = Int32MultiArray()
                message.data = [int(v) for v in fields[1:]]
                self.encoder_pub.publish(message)
            elif head == 'PWM' and len(fields) == 5:
                message = Int32MultiArray()
                message.data = [int(v) for v in fields[1:]]
                self.wheel_pwm_pub.publish(message)
            elif head == 'WHEEL' and len(fields) == 5:
                self._publish_wheels(
                    self.wheel_pub, [float(v) for v in fields[1:]]
                )
            elif head == 'TARGET' and len(fields) == 5:
                self._publish_wheels(
                    self.wheel_target_pub, [float(v) for v in fields[1:]]
                )
            elif head == 'SLIP' and len(fields) == 4:
                self._publish_float(self.slip_pub, float(fields[3]))
            elif head == 'IMU' and len(fields) in (7, 8):
                self._publish_imu([float(v) for v in fields[1:]])
            elif head == 'ODOM' and len(fields) == 7:
                self._publish_odom([float(v) for v in fields[1:]])
            else:
                return
            self._last_telemetry = self._now()
        except ValueError:
            self._bad_lines += 1
            self.get_logger().warn(
                f'Telemetria invalida: {line!r}', throttle_duration_sec=5.0
            )

    # ------------------------------------------------------------------
    def _publish_float(self, publisher, value: float) -> None:
        message = Float32()
        message.data = float(value)
        publisher.publish(message)

    def _publish_wheels(self, publisher, velocities) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = WHEEL_NAMES
        message.velocity = [float(v) for v in velocities]
        publisher.publish(message)

    def _publish_imu(self, values) -> None:
        """Telemetria do MPU6050 -> sensor_msgs/Imu no frame do ROBO.

        O firmware manda os seis eixos CRUS do chip mais, no setimo
        campo, o yawRate ja no eixo certo e sem vies. A placa deste robo
        esta em pe, com o chip apontando para a frente, entao o eixo
        vertical e o X do chip -- publicar gz cru em angular_velocity.z
        seria mentira: em ROS (REP-103) z e o eixo vertical e .z E a
        taxa de guinada. Por isso o yawRate vai para .z.

        Aceita 6 valores tambem, para nao quebrar com firmware antigo.
        """
        if len(values) >= 7:
            ax, ay, az, gx, gy, gz, yaw_rate = values[:7]
        else:
            ax, ay, az, gx, gy, gz = values[:6]
            yaw_rate = gz
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.imu_frame
        # O MPU6050 nao entrega orientacao absoluta aqui: -1 na primeira
        # covariancia e a forma padrao de dizer "sem orientacao".
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = gx
        message.angular_velocity.y = gy
        message.angular_velocity.z = yaw_rate
        message.linear_acceleration.x = ax
        message.linear_acceleration.y = ay
        message.linear_acceleration.z = az
        message.angular_velocity_covariance = [
            0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02,
        ]
        message.linear_acceleration_covariance = [
            0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.25,
        ]
        self.imu_pub.publish(message)

    def _publish_odom(self, values) -> None:
        x, y, yaw, vx, vy, wz = values
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz
        message.pose.covariance = [
            0.03, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.03, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
        ]
        message.twist.covariance = [
            0.10, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.20,
        ]
        self.odom_pub.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    # ------------------------------------------------------------------
    def _health_check(self) -> None:
        if self._last_telemetry is None:
            self.get_logger().warn(
                'Nenhuma telemetria do ESP32 ainda. Porta, baudrate ou firmware?',
                throttle_duration_sec=10.0,
            )
            return
        age = self._now() - self._last_telemetry
        if age > 1.0:
            self.get_logger().error(
                f'Telemetria do ESP32 parada ha {age:.1f}s. '
                'O firmware para os motores sozinho em 200 ms.',
                throttle_duration_sec=5.0,
            )

    def shutdown(self) -> None:
        try:
            self._write('TWIST,0.0000,0.0000,0.0000\n')
            self._write('STOP\n')
            time.sleep(0.05)
            with self._lock:
                if self.ser.is_open:
                    self.ser.close()
        except Exception:  # noqa: BLE001 - encerramento nao pode falhar
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorSerialNode()
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
