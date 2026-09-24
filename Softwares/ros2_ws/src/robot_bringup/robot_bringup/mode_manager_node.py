#!/usr/bin/env python3
"""Aplica o MODO DE OPERACAO escolhido no menu do ESP32.

POR QUE EXISTE: com o Pi subindo sozinho no boot, o ESP32 e a unica
interface disponivel sem um PC por perto. O menu dele pede um modo; este
no traduz o pedido em parametros nos outros nos.

POR QUE NAO SAO LAUNCHES SEPARADOS: trocar de modo derrubando e subindo
processos custa os ~25 s de subida da pilha, e derrubaria as cameras
junto -- e o operador quer ver a imagem por rqt nos DOIS modos. Aqui
todos os nos ficam no ar e apenas mudam de comportamento.

    PARADO    ninguem dirige. Cameras publicando, percepcao off.
    SEGUIDOR  autonomia ligada, percepcao ligada.
    RC        joystick dirige, percepcao DESLIGADA (~72% de um nucleo).

Em todos os modos as cameras continuam publicando, de proposito.
"""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

IDLE, FOLLOWER, RC = 0, 1, 2
NOMES = {IDLE: 'PARADO', FOLLOWER: 'SEGUIDOR', RC: 'RC'}


def control_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class ModeManagerNode(Node):
    """Traduz /robot/mode_request em parametros nos demais nos."""

    def __init__(self) -> None:
        super().__init__('mode_manager_node')
        self.declare_parameter('modo_inicial', IDLE)

        self.estado_pub = self.create_publisher(
            String, '/robot/mode', control_qos()
        )
        self.enable_pub = self.create_publisher(
            Bool, '/controle/enable', control_qos()
        )
        self.create_subscription(
            String, '/robot/mode_request', self._on_request, control_qos()
        )

        self._clientes: dict[str, object] = {}
        # Pedidos ainda NAO CONFIRMADOS pelo destino, por no+parametro.
        # Guarda sempre o valor mais recente pedido.
        self._pendentes: dict[tuple, bool] = {}
        # Pedidos enviados aguardando resposta: chave -> (valor, instante).
        self._em_voo: dict[tuple, tuple[bool, float]] = {}
        self._modo = -1
        # Republica o estado periodicamente: quem subir depois (e o
        # proprio ESP32, se reiniciar) precisa descobrir o modo atual
        # sem ter que perguntar.
        self.create_timer(1.0, self._publica_estado)
        self.create_timer(1.0, self._tenta_pendentes)
        self.create_timer(
            2.0, lambda: self._aplica_uma_vez(
                int(self.get_parameter('modo_inicial').value)
            )
        )

    # ------------------------------------------------------------------
    def _cliente(self, node_name: str):
        if node_name not in self._clientes:
            self._clientes[node_name] = self.create_client(
                SetParameters, f'/{node_name}/set_parameters'
            )
        return self._clientes[node_name]

    def _set_bool(self, node_name: str, param: str, valor: bool) -> None:
        """Ajusta um parametro booleano; REENFILEIRA se o no nao respondeu.

        POR QUE INSISTE (21/09/2026): a versao anterior pulava em
        silencio quando o servico de parametros ainda nao estava pronto.
        No boot pelo systemd o mode_manager sobe junto com os demais, e
        essa corrida deixava o seguidor no default do YAML. Com
        body_control_enabled=True naquele default, o robo ligava com o
        corpo ARMADO -- e ninguem ficava sabendo, porque o erro era
        silencioso.

        Agora o pedido fica pendente e e repetido pelo temporizador ate
        o no aparecer. Desligar nunca pode depender de sorte de timing.

        E so SAI da fila quando o destino responde successful=True
        (24/09/2026): antes saia logo depois do call_async, e uma resposta
        perdida -- no reiniciando, servico que sumiu entre o 'ready' e a
        chamada -- deixava body_control ligado sem aviso nenhum.
        """
        chave = (node_name, param)
        self._pendentes[chave] = valor
        cli = self._cliente(node_name)
        if not cli.service_is_ready():
            self.get_logger().warn(
                f'{node_name} ainda sem servico de parametros; '
                f'{param}={valor} fica pendente e sera repetido.',
                throttle_duration_sec=10.0,
            )
            return
        pedido = SetParameters.Request()
        pedido.parameters = [
            Parameter(
                name=param,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=valor
                ),
            )
        ]
        self._em_voo[chave] = (valor, self._agora())
        futuro = cli.call_async(pedido)
        futuro.add_done_callback(
            lambda f, chave=chave, valor=valor: self._confirmado(
                chave, valor, f
            )
        )

    def _confirmado(self, chave: tuple, valor: bool, futuro) -> None:
        """Resposta do set_parameters: so aqui o pedido sai da fila."""
        if self._em_voo.get(chave, (None,))[0] == valor:
            self._em_voo.pop(chave, None)
        try:
            resposta = futuro.result()
            ok = bool(resposta and resposta.results
                      and resposta.results[0].successful)
            motivo = '' if ok else (
                resposta.results[0].reason
                if resposta and resposta.results else 'sem resposta'
            )
        except Exception as erro:  # noqa: BLE001 - qualquer falha reenfileira
            ok, motivo = False, repr(erro)
        if ok:
            # Um pedido NOVO para a mesma chave (outro valor) continua na
            # fila: so remove se o confirmado e o mais recente.
            if self._pendentes.get(chave) == valor:
                self._pendentes.pop(chave, None)
            return
        self.get_logger().error(
            f'{chave[0]} recusou {chave[1]}={valor} ({motivo}); '
            'sera repetido.'
        )

    def _agora(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tenta_pendentes(self) -> None:
        """Repete o que nao foi confirmado. Roda pelo temporizador."""
        agora = self._agora()
        for chave, valor in list(self._pendentes.items()):
            em_voo = self._em_voo.get(chave)
            # Ainda esperando a resposta do mesmo valor: da 3 s antes de
            # considerar perdido e repetir.
            if em_voo and em_voo[0] == valor and agora - em_voo[1] < 3.0:
                continue
            node_name, param = chave
            if self._cliente(node_name).service_is_ready():
                self.get_logger().info(
                    f'{node_name}: repetindo {param}={valor} (sem '
                    'confirmacao ainda).'
                )
                self._set_bool(node_name, param, valor)

    def _on_request(self, msg: String) -> None:
        texto = msg.data.strip().upper()
        alvo = {'PARADO': IDLE, 'IDLE': IDLE,
                'SEGUIDOR': FOLLOWER, 'FOLLOWER': FOLLOWER,
                'RC': RC}.get(texto)
        if alvo is None:
            self.get_logger().warn(f'Modo desconhecido: {msg.data!r}')
            return
        self._aplica(alvo)

    def _aplica_uma_vez(self, modo: int) -> None:
        if self._modo < 0:
            self._aplica(modo)

    def _aplica(self, modo: int) -> None:
        if modo == self._modo:
            return
        self._modo = modo

        percepcao = modo == FOLLOWER
        self._set_bool('line_perception_bottom', 'enabled', percepcao)
        self._set_bool('line_perception_front', 'enabled', percepcao)
        self._set_bool('line_follower_node', 'body_control_enabled',
                       modo == FOLLOWER)
        # A cabeca e disputada: o seguidor publica /head/request a 50 Hz
        # e ganharia do joystick. Em RC a autoridade vai para o joystick.
        self._set_bool('line_follower_node', 'head_control_enabled',
                       modo != RC)

        # ARMAR E DESARMAR SAO ASSIMETRICOS, DE PROPOSITO.
        #
        # Escolher um modo NUNCA liga a autonomia. Com o Pi subindo
        # sozinho no boot, o menu do ESP32 vira a interface principal --
        # e um menu em que "escolher seguidor" faz o robo sair andando e
        # um menu perigoso. Modo diz QUEM tem autoridade; armar continua
        # sendo um ato separado e explicito (o toggle de autonomia, ou o
        # homem-morto do joystick).
        #
        # Sair do modo seguidor, por outro lado, DESARMA sempre: desligar
        # nunca precisa de confirmacao.
        if modo != FOLLOWER:
            enable = Bool()
            enable.data = False
            for _ in range(5):
                self.enable_pub.publish(enable)

        if modo == FOLLOWER:
            self.get_logger().info(
                'Modo SEGUIDOR: percepcao ligada. A autonomia continua '
                'DESARMADA -- arme separadamente para o robo andar.'
            )
        self.get_logger().info(f'Modo aplicado: {NOMES[modo]}')
        self._publica_estado()

    def _publica_estado(self) -> None:
        if self._modo < 0:
            return
        msg = String()
        msg.data = NOMES[self._modo]
        self.estado_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
