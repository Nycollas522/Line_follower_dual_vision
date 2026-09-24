# Line Follower — Visão Dupla (ROS 2 Jazzy + ESP32)

Robô seguidor de linha com chassi mecanum 4x4, câmera inferior (percepção
primária) + câmera superior em servo (preview/antecipação), controlado por
Raspberry Pi (ROS 2 Jazzy) com interface de baixo nível num ESP32-S3.

> 📘 **Para operar o robô, use o [OPERACAO.md](OPERACAO.md)**: parar,
> ligar/parar/reiniciar a pilha, trocar de modo pelo menu e pelo ROS,
> armar a autonomia, perfil de velocidade, menu do ESP32, resets, gravar
> firmware e calibrações — conferido contra o código em 24/09/2026.
> Este README guarda arquitetura, parâmetros, depuração e histórico; onde
> os dois divergirem, **vale o OPERACAO.md**.

> ⚠️ **Este robô anda de verdade.** Antes de habilitar autonomia, garanta
> espaço livre, esteja de olho no robô e saiba como pará-lo
> ([OPERACAO.md §1](OPERACAO.md#1-parar-o-robô-agora)). O **B1 longo** no
> ESP32 para e trava os motores (firmware de 24/09/2026 em diante).

---

## Sumário

- [**Próxima sessão — checklist**](#próxima-sessão--checklist)
- [Arquitetura em um parágrafo](#arquitetura-em-um-parágrafo)
- [Diagrama de nós e tópicos](#diagrama-de-nós-e-tópicos)
- [Estrutura do repositório](#estrutura-do-repositório)
- [Build](#build)
- [Subir o sistema](#subir-o-sistema)
- [Parar tudo, agora](#parar-tudo-agora)
- [Ligar/desligar autonomia (do jeito confiável)](#ligardesligar-autonomia-do-jeito-confiável)
- [Modo bancada — mexer na cabeça sem o robô sair andando](#modo-bancada--mexer-na-cabeça-sem-o-robô-sair-andando)
- [Modos de operação (escolhidos no menu do ESP32)](#modos-de-operação-escolhidos-no-menu-do-esp32)
- [Ligar sozinho no boot (systemd)](#ligar-sozinho-no-boot-systemd)
- [Modo RC (joystick DualShock 4)](#modo-rc-joystick-dualshock-4)
- [Teleoperação](#teleoperação)
- [Tabela de tópicos](#tabela-de-tópicos)
- [Protocolo serial (Pi ↔ ESP32)](#protocolo-serial-pi--esp32)
- [Parâmetros — referência completa](#parâmetros--referência-completa)
- [Comandos de depuração](#comandos-de-depuração)
- [Scripts de teste prontos](#scripts-de-teste-prontos)
- [Gravar e reproduzir dados (rosbag)](#gravar-e-reproduzir-dados-rosbag)
- [Plano de testes incremental](#plano-de-testes-incremental)
- [Tabela de diagnóstico (sintoma → causa → ajuste)](#tabela-de-diagnóstico-sintoma--causa--ajuste)
- [Armadilhas conhecidas desta stack](#armadilhas-conhecidas-desta-stack)
- [Testes automatizados (pytest)](#testes-automatizados-pytest)
- [Convenções de sinal](#convenções-de-sinal)
- [Geometria e calibração física](#geometria-e-calibração-física)
- [Histórico de correções relevantes](#histórico-de-correções-relevantes)

---

## Próxima sessão — checklist

Estado ao fechar 10/09/2026: **2 voltas completas na pista**, 160 testes
passando, pilha em modo bancada. Últimos dados em
`logs/corrida_20260910_203230.csv`.

### 1. Rodas — o único achado sem explicação (fazer PRIMEIRO)

O lado direito lê **parado 2.3× mais** que o esquerdo quando a cinemática
pede acima de 0.12 m/s:

```
FL   6.08%      FR  12.73%
RL   5.12%      RR  13.38%
```

Não é quantização de encoder (episódios de 200–400 ms, alguns de 1.9 s) e
não é zona morta (é *pior* em 0.12–0.18 m/s do que em 0.05–0.08). Está
**idêntico nas duas corridas de 3 min** (14.67% e 14.58%), então não veio
de nenhuma mudança recente. Este robô já escondeu mau contato em duas
rodas por um dia inteiro.

```bash
# RODAS SUSPENSAS -- isola tração de atrito com o chão. ~2 min.
python3 scripts/teste_rodas.py
```

Não tirar conclusão sem esse teste. O sintoma na pista ("guiada um pouco
mais ruidosa") é fraco demais para acusar hardware sozinho.

### 2. Decidir o limite de quebra

Modelo validado com **n=6 em duas corridas**: toda quebra acima de
**57.8°** travou, e nenhuma abaixo travou na primeira corrida.

```
limite = 2·acos( 1 / ((margem − erro_rastreio)/R + 1) )
R = wheel_base_k / arc_ratio = 0.165 m      erro_rastreio ≈ 8.4 mm (p50)
```

| margem | altura da inferior | limite |
|---|---|---|
| 31.9 mm | 87 mm (hoje) | **57.8°** |
| 45.0 mm | ~110 mm | **70.1°** |

Decisão do operador foi **não** subir a câmera e usar a quebra fechada
como teste de reencontro. Se mudar de ideia, é só remontagem +
recalibração (`depth_*`, `width_*`, `roi_bottom`) — nenhuma linha de
código.

### 3. A superior: aviso, não reencontro

Medido: no vértice a pista **já saiu do campo dela** (20.5 a 54.5 cm à
frente). Virar a cabeça não traz de volta o que não está lá — confirmado
com o robô parado na quebra, inferior lendo +21.4° com 5/5 bandas e a
superior em SEM LINHA com a cabeça em +15.4°.

Onde ela ganha o lugar dela: **avisou 17 de 23 quebras, 1.5 a 3.4 s
antes**. Planejar em cima disso, não de reencontro no vértice.

Se for investigar: o alvo é a ROI e o alcance dela, não ganho de
controlador.

### 4. Pista

Quebras acima de ~63° são fisicamente impossíveis para este chassi na
margem atual. Suavizá-las é a alternativa a subir a câmera.

### Antes de qualquer medição — as três armadilhas de 10/09/2026

1. **`/head/request` tem dois publicadores.** Cale o seguidor com
   `head_control_enabled: false` antes de comandar a cabeça, ou o servo
   vai ser disputado. Já produziu um diagnóstico de hardware falso.
2. **`/servo/state` é eco do comando**, não posição. A prova de que o
   servo girou é a IMAGEM (`logs/servo_prova.py`).
3. **Taxa derivada de log só vale se log e laço tiverem a mesma taxa.**
   O CSV grava a ~123 Hz e o controle roda a 50 Hz; derivar direto
   inflou a aceleração de 0.857 para 1.485 m/s². Reamostre a 20 ms.

E uma quarta, geral: **compare medições da mesma cena**. Duas leituras da
superior na "mesma" posição deram 287/287 e 0/14 porque o robô foi movido
entre elas. Grave o `heading` da inferior junto de tudo — foi ele que
denunciou.

---

## Arquitetura em um parágrafo

A câmera **inferior** é a referência primária e única fonte de direção no
modo normal — ela está a poucos centímetros da linha e enxerga só ~10 cm à
frente. A câmera **superior**, no servo, nunca comanda o corpo diretamente:
ela só antecipa (reduz velocidade antes de uma curva/quina, ou dá
feedforward saturado e limitado) e ajuda a buscar a linha quando ela se
perde. Cada tópico de comando físico (`/cmd_vel_auto`, `/servo/command`) tem
**um único publicador** no sistema — isso é verificado nos testes e é a
regra de segurança mais importante deste projeto.

## Diagrama de nós e tópicos

```
cam_bottom ──► line_perception_bottom ──► /line/detection ─────┐
                                                                 │
cam_front ───► line_perception_front ───► /line_front/detection┤
                                                                 ▼
                                          ┌──────────────────────────────┐
/odom ────────────────────────────────►  │      line_follower_node      │
/servo/state ─────────────────────────►  │  (ÚNICO publicador de        │
/autonomy/state, /controle/enable ────►  │   /cmd_vel_auto)             │
                                          └───────┬──────────┬───────────┘
                                   /cmd_vel_auto  │          │ /head/request
                                                  ▼          ▼
                                    ┌─────────────────┐  ┌──────────────────┐
                                    │ motor_serial_node│  │ head_servo_node  │
                                    │ (ponte serial +  │  │ (ÚNICO publicador│
                                    │  árbitro teleop) │  │  de /servo/      │
                                    └────────┬─────────┘  │  command)        │
                                             serial        └──────┬───────────┘
                                              ▼                   │
                                          ┌────────┐   /servo/command
                                          │ ESP32-S3│◄─────────────┘
                                          └────────┘
```

`/cmd_vel` (teleop) entra direto em `motor_serial_node`, que arbitra entre
teleop e autonomia via `/autonomy/state`. Veja a [tabela de
tópicos](#tabela-de-tópicos) completa mais abaixo.

## Estrutura do repositório

```
ros2_ws/
├── src/
│   ├── line_msgs/                  # mensagens custom (interfaces)
│   │   └── msg/
│   │       ├── LineDetection.msg    # saida de percepcao (por camera)
│   │       ├── FollowerStatus.msg   # estado do supervisor (telemetria/IC)
│   │       └── HeadRequest.msg      # pedido de apontamento do servo
│   │
│   └── robot_bringup/
│       ├── config/line_follower.yaml   # TODOS os parametros, comentados
│       ├── launch/line_follower.launch.py
│       ├── robot_bringup/
│       │   ├── line_geometry.py        # deteccao pura (sem ROS, testavel)
│       │   ├── line_controller.py      # lei de controle + FSM (sem ROS)
│       │   ├── line_perception_node.py # no ROS: 1 processo por camera
│       │   ├── line_follower_node.py   # supervisor: unico pub /cmd_vel_auto
│       │   ├── head_servo_node.py      # unico pub /servo/command
│       │   └── motor_serial_node.py    # ponte serial + arbitro teleop/auto
│       └── test/
│           ├── test_line_geometry.py    # pytest puro (sem robo/camera)
│           └── test_line_controller.py  # pytest puro (sem robo/camera)
│
├── src/camera_ros/, src/libcamera/     # dependencias de terceiros (vendored)
└── build/, install/, log/              # gerados pelo colcon (nao versionar)
```

Firmware do ESP32-S3 (protocolo serial, PID de roda, encoders, IMU, menu
físico) fica num repositório/pasta separado (`Firmware_Ippo`), compilado
via PlatformIO. Este README cobre o lado ROS 2; o protocolo entre os dois
está documentado na seção [Protocolo serial](#protocolo-serial-pi--esp32).

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select line_msgs robot_bringup --symlink-install
source install/setup.bash
```

`--symlink-install` é importante: os `.py` do pacote ficam **linkados** ao
código-fonte, então editar um nó e reiniciar o processo já basta — não
precisa rebuildar para toda mudança de lógica Python (só quando muda algo em
`line_msgs` ou `setup.py`/`package.xml`).

Depois de qualquer `git pull`/edição, sempre:

```bash
colcon build --packages-select line_msgs robot_bringup --symlink-install && source install/setup.bash
```

## Subir o sistema

> ⚠️ **A pilha já sobe sozinha no boot pelo `robo.service`.** Com o serviço
> no ar, **não rode `ros2 launch`**: duas instâncias disputam as câmeras e
> a serial. Ligar, parar e reiniciar: `sudo systemctl start|stop|restart
> robo` ([OPERACAO.md §2](OPERACAO.md#2-a-pilha-ros-ligar-parar-reiniciar)).
> Os comandos abaixo são para depuração, **com o serviço parado**.

```bash
# sistema completo (2 cameras + percepcao + controle + servo + serial)
ros2 launch robot_bringup line_follower.launch.py

# porta serial diferente
ros2 launch robot_bringup line_follower.launch.py port:=/dev/ttyUSB0

# validar visao SEM mover o robo (motores nao recebem nada)
ros2 launch robot_bringup line_follower.launch.py serial:=false

# sem a camera superior/servo (so seguimento basico com a inferior)
ros2 launch robot_bringup line_follower.launch.py preview:=false

# sem nenhuma camera (util pra tocar rosbag gravado)
ros2 launch robot_bringup line_follower.launch.py cameras:=false serial:=false
```

Argumentos do launch: `params_file`, `cameras` (`true`/`false`), `serial`,
`preview`, `port`. Todos com default sensato — normalmente só `port:=` muda.

Ao subir, os 10 nós esperados são: `camera_bottom`, `camera_front`,
`line_perception_bottom`, `line_perception_front`, `line_follower_node`,
`head_servo_node`, `motor_serial_node`, `mode_manager_node`, `joy_node`,
`joy_teleop_node`.

```bash
ros2 node list
```

## Parar tudo, agora

O sistema já tem 3 camadas de segurança independentes, mas se precisar
agir manualmente:

```bash
# 1) Zera o comando publicado (autonomia OU teleop, o que estiver ativo)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# 2) Desliga a autonomia (o robo passa a obedecer so /cmd_vel, que esta zerado)
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: false}"

# 3) Ultimo recurso: para a pilha. Sem TWIST, o watchdog do ESP32 zera
#    os motores em 200 ms. NAO use pkill: o robo.service (Restart=on-failure)
#    sobe tudo de novo 10 s depois.
sudo systemctl stop robo
```

Mais rápido, no próprio robô: menu **MODO → Parado → B2 curto** (o Pi
sempre desarma ao sair do SEGUIDOR). Numa corrida, **Ctrl-C** na
`corrida.py`.

Redundâncias já embutidas no sistema (não dependem de você digitar nada):
- **Firmware do ESP32**: para os motores se não chegar `TWIST` novo em 200 ms.
- **`motor_serial_node`**: zera comando se não chegar `/cmd_vel`/`/cmd_vel_auto`
  novo em `cmd_timeout` (0.30 s).
- **`line_follower_node`**: se a câmera inferior parar de publicar ou a
  confiança cair, o supervisor desacelera → recupera → para sozinho
  (`RECOVERING` → `SAFE_STOP`), nunca avança "as cegas". Em `SAFE_STOP` o
  corpo fica parado mas a **cabeça continua varrendo**, procurando a
  linha (`scan_in_safe_stop`).
- **B1 longo no ESP32**: para os motores e **trava** (ignora `TWIST`) até
  alguém rearmar ou entrar em SEGUIDOR/RC; o Pi recebe `MENU,STOP`,
  desarma e vai para PARADO. Funciona mesmo com o ROS travado (o robô
  fica parado). TESTES → CONTROLE ROS alterna a autonomia.

## Ligar/desligar autonomia (do jeito confiável)

⚠️ **Não use só `ros2 topic pub --once` para autonomia.** Em testes reais,
o `--once` às vezes dispara antes de todos os assinantes (`line_follower_
node`, `head_servo_node`, `motor_serial_node`) terminarem a descoberta DDS,
e a mensagem chega só em parte deles. Ver [Armadilhas
conhecidas](#armadilhas-conhecidas-desta-stack).

**Jeito robusto (recomendado)** — publica em burst e confirma:

```bash
ros2 topic pub -r 15 /controle/enable std_msgs/msg/Bool "{data: true}" &
PUBPID=$!
sleep 1.5
kill $PUBPID; wait $PUBPID 2>/dev/null

# confirma que pegou nos 3 nos
ros2 topic echo /autonomy/state --once
```

**Jeito simples**, aceitável se você reenviar e confirmar o resultado:

```bash
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: true}"
sleep 0.5
ros2 topic echo /autonomy/state --once   # confirme que virou "true"
```

Para desligar, o mesmo padrão com `{data: false}`.

**Nunca** deixe um `ros2 topic pub -r ...` correndo em background entre
comandos do terminal sem matá-lo explicitamente — um publisher esquecido
brigando com o próximo comando que você mandar é a causa mais provável de
"o robô não respeita o enable/disable que eu mandei".

## Modo bancada — mexer na cabeça sem o robô sair andando

O `head_servo_node` só aceita comando com a **autonomia ligada**, e ligar
a autonomia normalmente faz o `line_follower_node` publicar `cmd_vel` —
ou seja, o robô anda. Para testar servo/percepção com o robô no chão e
os motores energizados, suspenda só o corpo:

```bash
ros2 param set /line_follower_node body_control_enabled false
ros2 param get /line_follower_node body_control_enabled   # confirme: False
```

Com isso o seguidor para de publicar `cmd_vel` e a ausência de comando
faz o **watchdog do ESP32 parar os motores** — o lado seguro da falha.
A cabeça continua funcionando normalmente.

⚠️ **Isto é conveniência de teste, não trava de segurança.** Não
substitui cortar a tensão dos motores. Se o teste for arriscado, corte a
tensão.

As **paradas de segurança continuam passando** mesmo em modo bancada: na
troca de estado da autonomia e no shutdown o nó publica um `Twist()`
zerado de propósito, para o robô parar. Se você estiver monitorando
`cmd_vel_auto` para conferir que nada se move, ignore os twists zerados e
observe só os que têm velocidade real:

```bash
# so mostra comando que faria o robo andar
ros2 topic echo /cmd_vel_auto | grep -A1 'linear' | grep -v '0\.0'
```

Para devolver o corpo:

```bash
ros2 param set /line_follower_node body_control_enabled true
```

Existe o simétrico para a cabeça, quando um diagnóstico externo precisa
assumir `/head/request` sem brigar com o seguidor (que publica a ~50 Hz e
sempre vence a disputa):

```bash
ros2 param set /line_follower_node head_control_enabled false
```

O `scripts/servo_panorama.py` já faz esse handshake sozinho.

## Modos de operação (escolhidos no menu do ESP32)

O robô sobe em **PARADO** e o modo e escolhido no OLED:
**Menu -> Modo de operacao**. Com o Pi ligando sozinho no boot, o ESP32 e
a unica interface que existe sem um PC por perto.

| modo | quem dirige | percepcao | cameras |
|---|---|---|---|
| **PARADO** | ninguem | desligada | **publicando** |
| **SEGUIDOR** | seguidor de linha | ligada | publicando |
| **RC** | joystick | **desligada** | **publicando** |

**As cameras publicam nos tres modos, de proposito**: e o que permite
abrir `rqt` de um PC externo e ver a imagem mesmo dirigindo na mao.

**Trocar de modo nao reinicia nada.** Todos os nos ficam no ar e apenas
mudam de comportamento -- launches separados por modo custariam os ~25 s
de subida da pilha a cada troca, e derrubariam as cameras junto.

A percepcao nao apenas ignora os quadros: ela **cancela a inscricao** na
camera. Medido em 18/09/2026 com delta de `/proc` (e nao com
`ps -eo pcpu`, que da a media desde o boot do processo e mascarou o
efeito na primeira tentativa):

```
modo       CPU percepcao   CPU camera   imagem para o rqt
SEGUIDOR           63.4%        35.7%        18.7 Hz
RC                  0.0%        26.9%        27.2 Hz
```

### Escolher um modo NUNCA arma a autonomia

Modo diz **quem tem autoridade**; armar e um ato separado (o toggle de
autonomia no menu, ou o homem-morto do joystick). Um menu em que
"escolher seguidor" faz o robo sair andando e um menu perigoso -- ainda
mais num robo que liga sozinho.

Sair do modo seguidor, por outro lado, **desarma sempre**: desligar
nunca precisa de confirmacao.

### Por fora do OLED

```bash
ros2 topic pub --once /robot/mode_request std_msgs/String "{data: RC}"
ros2 topic echo /robot/mode      # o que esta ativo agora
```

---

## Ligar sozinho no boot (systemd)

```bash
sudo cp /tmp/robo.service /etc/systemd/system/robo.service
sudo systemctl daemon-reload
sudo systemctl enable --now robo.service
systemctl status robo.service
journalctl -u robo.service -f        # ou: tail -f ~/ros2_ws/logs/boot.log
```

O `scripts/iniciar_robo.sh` **espera ate 30 s pela porta serial** antes
de lancar: no boot o CH343 leva um tempo para enumerar no USB, e sem
esperar o `motor_serial_node` sobe, nao acha a porta, e o robo fica sem
atuacao ate alguem reiniciar o servico na mao.

O `KillMode=control-group` do servico existe pelo mesmo motivo do
`gravar.sh`: matar so o launch deixaria cameras orfas segurando o
pipeline do libcamera.

Para desligar o autostart: `sudo systemctl disable --now robo.service`.

---

## Modo RC (joystick DualShock 4)

**O ESP32-S3 NAO pode parear o controle.** Ele tem Bluetooth 5 **LE
apenas**, sem Bluetooth Classic (BR/EDR), e o DualShock 4 e HID sobre
Classic. O ESP32 original conseguia (via Bluepad32); o S3 nao. Por isso
o joystick vive na Raspberry e desce pela serial como `TWIST` comum --
**o firmware nao sabe que existe modo RC, e nao precisa saber**.

### Parear o controle (uma vez so)

Segure **PS + Share** ate a barra de luz piscar rapido, entao:

```bash
bluetoothctl
  scan on
  # anote o MAC do "Wireless Controller"
  pair    <MAC>
  trust   <MAC>     # trust faz reconectar sozinho nas proximas vezes
  connect <MAC>
  quit
ls /dev/input/js0   # tem de existir
```

### Dirigir

```bash
ros2 launch robot_bringup rc.launch.py
```

Sobe so ponte serial, cabeca e joystick -- sem cameras e sem seguidor,
que em modo RC apenas competiriam por CPU e pela autoridade da cabeca.

| controle | funcao |
|---|---|
| **L2 (segurar)** | **homem-morto: sem ele nada se move** |
| analogico esquerdo | Y = frente/tras, X = giro |
| L1 / R1 | translacao lateral (strafe) |
| **R2 (segurar)** | turbo: levanta os limites por `turbo_scale` |
| analogico direito | X = servo da cabeca (pan) |

### Sinais dos eixos: ajuste ao vivo, nao no codigo

A numeracao e a convencao de sinal de cada eixo saem do driver de
joystick, e a montagem fisica das rodas entra junto. Nao ha simetria a
procurar -- cada eixo e independente. Por isso sao parametros com
callback: `ros2 param set` vale NA HORA, sem reiniciar o no.

```bash
ros2 param set /joy_teleop_node invert_vx true      # frente/tras
ros2 param set /joy_teleop_node invert_wz true      # giro
ros2 param set /joy_teleop_node invert_vy true      # strafe L1/R1
ros2 param set /joy_teleop_node invert_servo true   # pan da cabeca
```

O no loga a combinacao a cada troca (`ajustes: vx+ wz+ vy- servo-`).
Achou a certa, grave no YAML.

> Armadilha que custou duas rodadas de teste em 22/09/2026: o codigo
> antigo ja tinha um sinal negativo embutido, entao `invert_*: true`
> reproduzia o comportamento antigo em vez de inverte-lo. E os valores
> eram lidos so no `__init__`, entao `ros2 param set` nao fazia nada --
> o que anulava o motivo de serem parametros. Ambos corrigidos.

O homem-morto nao foi pedido pelo operador, foi incluido de proposito:
analogico com deriva e um robo que sai andando sozinho, e segurar um
gatilho custa menos que perseguir o robo pela sala. Para desligar:
`-p require_deadman:=false`.

### Se os controles vierem trocados

Os indices de eixo e botao sao **parametros**, porque driver e kernel
mudam a numeracao. Veja o que o seu controle manda e ajuste no YAML:

```bash
ros2 topic echo /joy
```

Camadas de parada, da mais rapida para a mais lenta: soltar o L2 zera o
comando na hora; sem `/joy` por `joy_timeout` (0.5 s) o no zera; e sem
`TWIST` por 200 ms o **watchdog do proprio ESP32** corta os motores.

---

## Teleoperação

Sempre disponível, independente do estado da autonomia:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# publica em /cmd_vel — so e obedecido quando /autonomy/state == false
```

Para voltar da autonomia para teleop: desligue `/controle/enable` (ver
seção acima). O supervisor zera `/cmd_vel_auto` e centraliza o servo
automaticamente nessa transição.

---

## Tabela de tópicos

| Tópico | Tipo | Publicador (único, quando aplicável) | Assinantes |
|---|---|---|---|
| `/cam_bottom/image_raw` | `sensor_msgs/Image` | `camera_bottom` | `line_perception_bottom` |
| `/cam_front/image_raw` | `sensor_msgs/Image` | `camera_front` | `line_perception_front` |
| `/line/detection` | `line_msgs/LineDetection` | `line_perception_bottom` | `line_follower_node` |
| `/line_front/detection` | `line_msgs/LineDetection` | `line_perception_front` | `line_follower_node`, `head_servo_node` |
| `/line/error`, `/line/heading_error`, `/line/status`, `/line/centroid` | vários | `line_perception_bottom` | compatibilidade/rqt (opcional, `publish_compat_topics`) |
| `/line/debug_image`, `/line/mask` | `sensor_msgs/Image` | `line_perception_bottom` | só publica se alguém assinar (rqt_image_view) |
| `/line_front/debug_image`, `/line_front/mask` | `sensor_msgs/Image` | `line_perception_front` | idem |
| **`/cmd_vel_auto`** | `geometry_msgs/Twist` | **`line_follower_node`** | `motor_serial_node` |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop (usuário) | `motor_serial_node` |
| `/head/request` | `line_msgs/HeadRequest` | `line_follower_node` | `head_servo_node` |
| **`/servo/command`** | `std_msgs/Float32` | **`head_servo_node`** | `motor_serial_node` |
| `/servo/state` | `std_msgs/Float32` | `motor_serial_node` | `line_follower_node`, `head_servo_node` |
| `/controle/enable` | `std_msgs/Bool` | usuário / menu ESP32 (via `motor_serial_node`) | todos |
| `/autonomy/state` | `std_msgs/Bool` | `motor_serial_node` | `line_follower_node`, `head_servo_node` |
| `/follower/status` | `line_msgs/FollowerStatus` | `line_follower_node` | rosbag/depuração |
| `/odom` | `nav_msgs/Odometry` | `motor_serial_node` | `line_follower_node` |
| `/imu/data_raw` | `sensor_msgs/Imu` | `motor_serial_node` | — |
| `/wheel_states`, `/wheel_targets` | `sensor_msgs/JointState` | `motor_serial_node` | rosbag/depuração |
| `/wheel_encoder_ticks`, `/wheel_pwm` | `std_msgs/Int32MultiArray` | `motor_serial_node` | rosbag/depuração |
| `/battery/voltage` | `std_msgs/Float32` | `motor_serial_node` | — |

**Convenção de sinal (igual em todos os tópicos acima):** `+x` = direita do
robô. `lateral_error > 0` = linha à direita. `angular.z (wz) > 0` = giro
anti-horário = **esquerda** (padrão ROS). Servo: `angle_deg > 0` = **direita**
(fisicamente corrigido em `motor_serial_node`, ver [Histórico de
correções](#histórico-de-correções-relevantes)).

## Protocolo serial (Pi ↔ ESP32)

115200 baud, linhas terminadas em `\n`. **Não alterado nesta stack** — é o
contrato com o firmware, tratado como fronteira estável.

**Pi → ESP32:**

| Comando | Formato | Efeito |
|---|---|---|
| `TWIST` | `TWIST,vx,vy,wz\n` | velocidade desejada (m/s, m/s, rad/s) |
| `SERVO` | `SERVO,angulo\n` | ângulo do pan, graus, já com sinal físico corrigido |
| `STOP` | `STOP\n` | zera tudo e para |
| `CONTROL_STATE` | `CONTROL_STATE,0\|1\n` | espelha `/autonomy/state` no firmware (só afeta o OLED, não trava motor) |
| `SET_PID` | `SET_PID,kp,ki,kd\n` | ajusta PID de roda do firmware |
| `SAVE_SETTINGS` | `SAVE_SETTINGS\n` | grava config atual na EEPROM do ESP32 |
| `STATUS` | `STATUS\n` | pede status (IMU, OLED) |

**ESP32 → Pi** (telemetria, a cada `TELEMETRY_MS` = 200 ms):

| Linha | Formato | Vira tópico |
|---|---|---|
| `ENC` | `ENC,t0,t1,t2,t3` | `/wheel_encoder_ticks` |
| `ODOM` | `ODOM,x,y,yaw,vx,vy,wz` | `/odom` |
| `WHEEL` | `WHEEL,v0,v1,v2,v3` | `/wheel_states` |
| `TARGET` | `TARGET,v0,v1,v2,v3` | `/wheel_targets` |
| `PWM` | `PWM,p0,p1,p2,p3` | `/wheel_pwm` |
| `IMU` | `IMU,ax,ay,az,gx,gy,gz` | `/imu/data_raw` |
| `SERVO_STATE` | `SERVO_STATE,angulo` | `/servo/state` (sinal desfeito de volta) |
| `BATT` | `BATT,tensao` | `/battery/voltage` |
| `READY` | `READY,...` | log (reconexão do ESP32) |
| `MENU,CONTROL_TOGGLE` | — | toggle físico do botão → `_set_autonomy()` |

Ordem das rodas em todos os arrays: **FL, FR, RL, RR**.

---

## Parâmetros — referência completa

Todos vivem em `config/line_follower.yaml`, organizados por nó. Os **6
que resolvem 90% dos problemas de controle** estão marcados com ⭐.

### `line_perception_bottom` / `line_perception_front`

Mesmo executável (`line_perception_node`), instâncias diferentes.

| Parâmetro | Bottom | Front | O que faz |
|---|---|---|---|
| `image_topic` | `/cam_bottom/image_raw` | `/cam_front/image_raw` | tópico de entrada |
| `output_namespace` | `/line` | `/line_front` | prefixo dos tópicos de saída |
| `rotate_180` | `true` | `true` | **as duas câmeras estão montadas de cabeça para baixo** |
| `depth_near_m` / `depth_far_m` | `0.1104` / `0.1647` | `0.205` / `0.545` | distância física (m) da ROI, ver [Geometria](#geometria-e-calibração-física) |
| `width_near_m` / `width_far_m` | `0.087` / `0.087` | `0.220` / `0.420` | largura de chão coberta pela imagem, calibrado. **Iguais na inferior**: ela aponta reto para baixo, vista ortográfica |
| `curvature_enabled` | `false` | `true` | **desligada na inferior**: base de profundidade curta demais, ver a seção da montagem |
| `camera_x_offset_m` | `0.0` | `0.0` | deslocamento lateral da câmera vs. eixo do robô |
| `line_width_m` | `0.020` | `0.020` | largura real da fita (meça a sua!) |
| `roi_top` / `roi_bottom` | `0.0` / `0.831` | `0.45` / `1.00` | recorte vertical. Na inferior o corte é **por baixo**: o próprio robô aparece nas 81 linhas mais próximas |
| `work_width` | `160` | `160` | downscale antes de processar (CPU) |
| `bands` | `5` | `5` | nº de faixas horizontais amostradas |
| `min_contrast` | `25.0` | `18.0` | separação mínima linha/fundo |
| `max_white_fraction` | `0.80` | `0.85` | guarda contra quadro uniforme (não separa linha/piso — isso é o `flank_weight`) |
| `flank_weight` | `0.7` | `0.7` | exige candidato mais claro que os vizinhos (pista = fita clara + bordas escuras) |
| `track_weight` / `track_sigma` / `track_memory` | `0.6`/`0.25`/`6` | `0.5`/`0.30`/`4` | continuidade temporal entre frames |
| `max_curvature` | `12.0` | `8.0` | 1/m; acima disso, ajuste é ruído |
| `max_heading_for_curvature` | `0.30` rad | `0.30` rad | acima deste desalinhamento, curvatura é descartada (artefato geométrico, não curva real) |
| `min_bands_for_curvature` | `4` | `4` | mínimo de bandas válidas para tentar ajuste quadrático |
| `publish_compat_topics` | `true` | `true` | publica `/error`, `/status` etc. (rqt) |

### `line_follower_node` — supervisor + controlador

⭐ **Os 6 principais:**

| Parâmetro | Valor | Efeito |
|---|---|---|
| `v_max` | `0.22` | m/s em reta limpa. Suba de 0.02 em 0.02. |
| `v_min` | `0.07` | m/s no pior caso ainda seguindo. Se travar em curva, está baixo demais. |
| `k_lateral` | `9.0` | rad/s por metro de erro lateral. Oscila em reta → baixe. Corta curva por dentro → suba. |
| `k_heading` | `1.6` | ganho de orientação — faz entrar alinhado na curva, não só reagir ao deslocamento. |
| `ref_lateral` | `0.045` | erro (m) que já justifica `v_min`. |
| `ref_curvature` | `6.0` | curvatura (1/m) que já justifica `v_min` — freio de curva. |

**Direção (fino):**

| Parâmetro | Valor | Nota |
|---|---|---|
| `k_damping` | `0.35` | amortecimento derivativo |
| `k_feedforward` | `0.15` | ⚠️ **medido no robô**, não valor de projeto — curvatura da câmera inferior é ruidosa (baseline de 10 cm); valor alto cancelava a correção primária. Não subir sem filtrar curvatura primeiro. |
| `derivative_tau` | `0.08` | s, filtro passa-baixa da derivada |
| `ref_heading` | `0.55` | rad |
| `wz_max` | `2.5` | rad/s, teto absoluto |
| `wz_accel` | `6.0` | rad/s², rampa — casado com o firmware |

**Limite cinemático (chassi mecanum — não mexer sem trocar o chassi):**

| Parâmetro | Valor | Nota |
|---|---|---|
| `wheel_base_k` | `0.165` | m, = `halfL + halfW` do firmware |
| `wheel_max_mps` | `0.70` | m/s máximo por roda |
| `arc_ratio` | `0.80` | 1.0 permitiria roda interna parar; 0.80 dá margem contra inversão de roda. **Raio mínimo de curva = `wheel_base_k / arc_ratio` = 20.6 cm, e isso é CONSTANTE, não muda com a velocidade.** |

**Rampas:** `accel: 0.35`, `decel: 0.60` (m/s²).

**Perda de linha e recuperação:**

| Parâmetro | Valor | Nota |
|---|---|---|
| `confidence_ok` | `0.45` | acima → `FOLLOWING` |
| `confidence_min` | `0.15` | abaixo → detecção ignorada |
| `vision_timeout` | `0.35` | s; medido no robô: câmera tem jitter de até ~230 ms entre frames |
| `coast_time` | `0.20` | s antes de `COASTING` → `RECOVERING` |
| `coast_speed_ratio` | `0.5` | fração de `v_min` mantida no coast |
| `recovery_time` | `3.0` | s de busca antes de `SAFE_STOP` |
| `recovery_wz` | `0.9` | rad/s do giro de busca |
| `relock_frames` | `3` | detecções seguidas para voltar a `FOLLOWING` |
| `scan_in_safe_stop` | `true` | a cabeça **continua varrendo** em `SAFE_STOP`. O corpo já está parado, então varrer é ganho puro de percepção, sem risco de atuação. Medido: num episódio de perda de 9,9 s a cabeça varria só os 3 s de `RECOVERING` e depois esperava cega ~7 s |

**Alinhamento parado (`ALIGNING`) — largado torto:**

| Parâmetro | Valor | Nota |
|---|---|---|
| `align_on_start` | `true` | ao sair de `IDLE` com heading acima do limiar, gira **parado** antes de andar |
| `align_heading_deg` | `18.0` | acima disso, alinha antes de avançar |
| `align_exit_deg` | `7.0` | histerese de saída |
| `align_wz` | `0.9` | rad/s do giro parado. **Não use 0.6**: dá `wheel_base_k·wz = 0,099 m/s`, abaixo do atrito estático — medido, o robô ficou 1,2 s parado comandando giro |
| `align_timeout` | `4.0` | s; nunca fica preso alinhando |
| `align_closed_loop_deg` | `9.0` | até aqui a banda próxima aguenta o giro e a visão decide sozinha |
| `align_overshoot_guard` | `1.25` | ×alvo; teto do giro quando cego |

**Por que este estado existe.** Medido em 09/09/2026: largado a 25° da linha, o robô **não consegue se alinhar andando para frente**. O limite é geométrico, não de ganho: o teto de giro sem inverter roda é `arc_ratio·v/wheel_base_k`, logo a curvatura máxima do caminho é `arc_ratio/wheel_base_k = 4,85 1/m` — raio mínimo de **20,6 cm, constante**, independente da velocidade (desacelerar não aperta a curva). Girar 25° nesse raio custa `R·(1−cos25°) = 19 mm` de desvio lateral, que somado ao offset inicial passa dos ±19 mm da banda mais próxima. Na medição ele atravessou a linha, foi a −49,8 mm e a confiança caiu de 0,88 para 0,03.

⚠️ **O progresso do giro vem do `/odom` (encoders), nunca do `wz` comandado.** Uma versão anterior integrava o comando e, com atrito estático, concluiu que tinha girado enquanto o robô seguia a −23,7° — soltou o robô torto e ele saiu da linha. A visão é critério primário sempre que estiver disponível; a rotação medida só encerra a manobra quando a visão sumiu.

**Autoridade de publicação (bancada / diagnóstico):**

| Parâmetro | Valor | Nota |
|---|---|---|
| `body_control_enabled` | `true` | `false` suspende `cmd_vel` — testa cabeça e percepção com a autonomia ligada sem o robô andar. Ver [Modo bancada](#modo-bancada--mexer-na-cabeça-sem-o-robô-sair-andando). Conveniência de teste, **não** trava de segurança |
| `head_control_enabled` | `true` | `false` cede `/head/request` a um diagnóstico externo. Sem isso o seguidor publica a ~50 Hz e vence qualquer script |

**Câmera superior (preview):**

| Parâmetro | Valor | Nota |
|---|---|---|
| `use_preview` | `true` | **chave-mestra** da câmera superior: `false` desliga os três papéis de uma vez |
| `preview_steering_enabled` | `false` | papel **direção** (κ/heading → `wz`). **Desligado por medição**: com ele ligado o robô perdeu a linha 32% do tempo e o viés de curva chegou a −16 mm; desligado, 0% de perda e viés +0,46 mm |
| `preview_speed_enabled` | `true` | papel **velocidade**: só freia antes da curva. Errar custa tempo, não rastreio |
| `preview_recovery_enabled` | `true` | papel **recuperação**: escolhe o *lado* do giro em `RECOVERING`, nunca a magnitude. É o papel de menor risco e o que mais justifica a segunda câmera |
| `preview_timeout` | `0.40` | s |
| `preview_confidence_min` | `0.40` | |
| `preview_ff_gain` | `0.35` | feedforward de curvatura antecipada — só vale para curva **suave** |
| `preview_heading_gain` | `0.40` | feedforward de heading antecipado — vale para **quina** (reta→reta angulada), onde curvatura explode |
| `preview_ff_max_ratio` | `0.30` | teto do preview (curvatura+heading+bearing somados), fração de `wz_max` — garante que a câmera de cima nunca domina |
| `preview_center_tol_deg` | `8.0` | acima disso, pan não está centrado → preview de imagem não vale como direção |
| `preview_speed_gain` | `1.0` | peso do preview na redução de velocidade |
| `use_servo_bearing` | `false` | realimenta o ângulo do servo no giro do **corpo**. Reprovado na medição de 09/09/2026. Controla **só** a realimentação — quem move a cabeça é `head_tracking_enabled` |
| `servo_bearing_gain` | `1.2` | rad/s por rad de ângulo do servo (inerte enquanto `use_servo_bearing` for `false`) |
| `head_tracking_enabled` | `true` | a cabeça rastreia a linha (`HeadMode.TRACK` + antecipação geométrica). Vale manter ligada mesmo sem realimentação: a cabeça virada enquadra a curva, alimentando os papéis de velocidade e recuperação |
| `scan_in_recovery` | `true` | cabeça varre durante `RECOVERING` |

**Direção — atraso e amortecimento:**

| Parâmetro | Valor | Nota |
|---|---|---|
| `curvature_eval_distance_m` | `0.085` | **atraso de transporte** do feedforward. A curvatura é ajustada numa janela 3,5–13,5 cm **à frente**, mas `ω = v·κ` vale para a curvatura *sob* o robô. Sem isto ele vira antes de chegar na curva e entra por dentro |
| `curvature_delay_max_s` | `1.5` | teto do atraso (protege contra `v → 0`) |
| `heading_gain_scheduling` | `true` | agenda `k_heading` para manter o amortecimento constante com a velocidade |
| `target_damping` | `1.0` | ζ alvo; 1,0 = crítico, sem sobressinal |

**O sintoma que o atraso resolve.** Medido: em `t=1,00 s` a curvatura já marcava +0,45 e o `wz` já virava à direita, mas o heading da linha ainda era +0,6° — **o robô continuava na reta**. Ele saía por dentro (`lat` chegou a −34,5 mm) e só então o feedback puxava à esquerda. Na pista isso se vê como *"virou para a esquerda numa curva para a direita"* — o giro à esquerda é a reação, não a causa.

**Por que agendar `k_heading`.** A dinâmica de duas variáveis (`ė = v·θ`, `θ̇ = wz`) com esta lei dá:

```
ë + (k_heading + k_damping·v)·ė + v·k_lateral·e = 0
ωn = √(v·k_lateral)      ζ = (k_heading + k_damping·v) / (2·√(v·k_lateral))
```

Com `k_heading` fixo o amortecimento **cai conforme o robô acelera** — justamente na reta, que é onde ele acelera:

| v (m/s) | ζ sem agendamento | comportamento |
|---|---|---|
| 0,07 | 1,02 | crítico |
| 0,15 | 0,71 | sobressinal ~4% |
| 0,22 | 0,60 | sobressinal ~9,5% |

O agendamento nunca reduz `k_heading` abaixo do valor base, então baixa velocidade fica idêntica.

### `head_servo_node`

| Parâmetro | Valor | Nota |
|---|---|---|
| `max_angle_deg` | `45.0` | limite útil do pan (firmware satura em 90, mas não ajuda ir até lá) |
| `max_rate_deg_s` | `60.0` | |
| `center_rate_deg_s` | `25.0` | taxa reduzida ao voltar/ficar no centro |
| `scan_amplitude_deg` | `35.0` | |
| `scan_rate_deg_s` | `40.0` | |
| `center_deadband_deg` | `0.3` | não reenvia comando se mudou menos que isso |
| `request_timeout` | `0.5` | s; sem `/head/request` novo → volta ao centro |
| `rate_hz` | `40.0` | ⚠️ **precisa reiniciar o nó para mudar** (recria o timer). 40, não 20: o PWM físico do servo roda a 50 Hz — mandar alvo novo com menos frequência que isso é visível como movimento em passos. |
| `track_gain_deg_s_per_m` | `300.0` | `MODE_TRACK`: taxa de giro (°/s) proporcional ao erro lateral (m) da câmera superior |
| `track_confidence_min` | `0.40` | |
| `track_timeout` | `0.40` | s |
| `track_filter_alpha` | `0.45` | filtro passa-baixa no erro antes de virar taxa; menor = mais suave e mais lag, maior = mais responsivo e mais "passos" |

Todos acima **exceto `rate_hz`** têm callback de parâmetros — mudam ao vivo
com `ros2 param set`, sem reiniciar nada.

### `motor_serial_node`

| Parâmetro | Valor | Nota |
|---|---|---|
| `port` | `/dev/ttyACM0` | |
| `baudrate` | `115200` | |
| `max_vx` / `max_vy` / `max_wz` | `0.45` / `0.45` / `2.5` | barreira final, depois de qualquer controlador |
| `cmd_timeout` | `0.30` | s; watchdog do lado do Pi (o firmware tem o seu próprio, 200 ms) |
| `serial_read_rate` | `50.0` | Hz; ⚠️ **não subir**: a 200 Hz este nó sozinho consumia ~40% de uma CPU do Pi e derrubava o FPS das câmeras por contenção de escalonamento |
| `twist_rate` | `50.0` | Hz de envio de `TWIST` |
| `state_rate` | `5.0` | Hz de publicação de `/autonomy/state` |
| `start_autonomy_enabled` | `false` | |

---

## Comandos de depuração

### Ver tudo de uma vez

```bash
ros2 topic list                     # todos os topicos ativos
ros2 node list                      # todos os nos ativos (7 esperados)
ros2 node info /line_follower_node  # publishers/subscribers de um no
```

### Confirmar que um tópico crítico tem exatamente 1 publicador

Isso é a garantia de segurança mais importante do sistema — vale a pena
verificar sempre que algo parecer estranho:

```bash
python3 -c "
import rclpy
from rclpy.node import Node
rclpy.init()
n = Node('pubcheck')
import time
t0 = time.time()
while time.time() - t0 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.1)
for topic in ['/cmd_vel_auto', '/servo/command']:
    info = n.get_publishers_info_by_topic(topic)
    print(topic, '->', len(info), 'publisher(s):', [i.node_name for i in info])
n.destroy_node()
rclpy.shutdown()
"
```

Se aparecer mais de 1, **pare tudo** e investigue (`ros2 node list` +
`pkill` dos processos duplicados) antes de habilitar autonomia.

### Ver a percepção sem mover o robô

```bash
ros2 launch robot_bringup line_follower.launch.py serial:=false

# visual
rqt_image_view   # escolha /line/debug_image ou /line_front/debug_image

# numerico, uma leitura
ros2 topic echo /line/detection --once
ros2 topic echo /line_front/detection --once

# continuo
ros2 topic echo /line/detection
```

Campos-chave em `LineDetection`: `valid`, `confidence`, `lateral_error` (m),
`heading_error` (rad), `curvature` (1/m), `bands_valid`/`bands_total`,
`contrast`, `line_width` (compare com a fita real — é o teste da própria
calibração), `processing_time`, `pipeline_latency`.

### Acompanhar o supervisor em tempo real

```bash
ros2 topic echo /follower/status
```

Campos-chave: `state_name` (`IDLE`/`FOLLOWING`/`DEGRADED`/`COASTING`/
`RECOVERING`/`SAFE_STOP`), `cmd_vx`/`cmd_wz`, `wz_limit` (se `cmd_wz` está
saturado nesse valor, o robô quer virar mais do que o limite cinemático
permite), `severity` (0–1, quanto a velocidade foi reduzida), `vision_age`
(idade da última detecção válida), `loss_events`, `mean_abs_error`.

### FPS e latência das câmeras

`ros2 topic hz` é **pouco confiável nesta stack** (corrida de descoberta
DDS — ver [Armadilhas conhecidas](#armadilhas-conhecidas-desta-stack)).
Prefira medir direto com `rclpy`:

```bash
python3 -c "
import rclpy, time
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
rclpy.init()
n = Node('fps_check')
counts = {'bottom': 0, 'front': 0}
n.create_subscription(Image, '/cam_bottom/image_raw', lambda m: counts.__setitem__('bottom', counts['bottom']+1), QOS)
n.create_subscription(Image, '/cam_front/image_raw', lambda m: counts.__setitem__('front', counts['front']+1), QOS)
t0 = time.time()
while time.time() - t0 < 5.0:
    rclpy.spin_once(n, timeout_sec=0.05)
elapsed = time.time() - t0
print(f'bottom: {counts[\"bottom\"]/elapsed:.1f} fps')
print(f'front:  {counts[\"front\"]/elapsed:.1f} fps')
n.destroy_node(); rclpy.shutdown()
"
```

### CPU por processo (se o FPS cair)

```bash
ps -eo pid,pcpu,pmem,etimes,comm --sort=-pcpu | head -12
uptime   # load average; compare com "nproc" (numero de nucleos)
nproc
```

Se algo estiver consumindo CPU de forma desproporcional, o suspeito mais
comum é um timer rodando rápido demais sem necessidade — veja o histórico
de `serial_read_rate` acima.

### Servo — testar isoladamente

⚠️ O watchdog do `head_servo_node` exige `/autonomy/state == true` para
aceitar qualquer modo (`HOLD`/`SCAN`/`TRACK`) — com autonomia desligada ele
sempre força `MODE_CENTER`, mesmo se você mandar outra coisa. Isso é
proposital (cabeça não faz nada sozinha com o corpo desligado), mas
significa que **não dá para testar o servo com o corpo garantidamente
parado** — ligue a autonomia e observe, pronto para desligar rápido.

```bash
python3 -c "
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32
from line_msgs.msg import HeadRequest
QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
rclpy.init()
n = Node('servo_check')
head_pub = n.create_publisher(HeadRequest, '/head/request', QOS)
enable_pub = n.create_publisher(Bool, '/controle/enable', QOS)
angle = [None]
n.create_subscription(Float32, '/servo/state', lambda m: angle.__setitem__(0, m.data), QOS)
deadline = time.time() + 5.0
while time.time() < deadline and enable_pub.get_subscription_count() < 3:
    rclpy.spin_once(n, timeout_sec=0.05)
on = Bool(); on.data = True
for _ in range(10):
    enable_pub.publish(on); rclpy.spin_once(n, timeout_sec=0.05)
time.sleep(0.6)
req = HeadRequest(); req.mode = HeadRequest.MODE_HOLD; req.angle_deg = 25.0
t0 = time.time()
while time.time() - t0 < 3.0:
    head_pub.publish(req); rclpy.spin_once(n, timeout_sec=0.05)
print('servo_angle:', angle[0], '(esperado ~+22 a +25, positivo = direita)')
req.mode = HeadRequest.MODE_CENTER
off = Bool(); off.data = False
for _ in range(20):
    head_pub.publish(req); enable_pub.publish(off); rclpy.spin_once(n, timeout_sec=0.05)
n.destroy_node(); rclpy.shutdown()
"
```

### Testar rodas isoladamente (rodas suspensas)

```bash
# ver ticks acumulados (ordem FL, FR, RL, RR)
ros2 topic echo /wheel_encoder_ticks

# ver velocidade medida / alvo / PWM por roda
ros2 topic echo /wheel_states
ros2 topic echo /wheel_targets
ros2 topic echo /wheel_pwm
```

Gire cada roda à mão: só o índice correspondente deve mudar, e crescer se
girada "para frente". Depois, com o motor: `TWIST,0.08,0,0` (via
`/cmd_vel`) deve dar as 4 velocidades **positivas** em `/wheel_states` — se
alguma vier negativa, é sinal de fiação/`MOTOR_SIGN` invertido naquela
roda, não bug de software.

### Ajustar parâmetro ao vivo

```bash
ros2 param get /line_follower_node k_lateral
ros2 param set /line_follower_node k_lateral 7.0
ros2 param set /line_perception_bottom min_contrast 20.0
ros2 param set /head_servo_node track_filter_alpha 0.30
```

Funciona para praticamente tudo **exceto** `rate_hz` do `head_servo_node`
(precisa reiniciar o nó — o motivo está comentado no próprio código).

---

## Scripts de teste prontos

Scripts `rclpy` autocontidos são mais confiáveis que combinar `ros2 topic
pub`/`echo` na CLI (ver [Armadilhas conhecidas](#armadilhas-conhecidas-desta-stack)).
Salve estes em `scripts/` (crie a pasta) para reusar entre sessões.

### `scripts/live_test.py` — liga, monitora, desliga com garantia

```python
#!/usr/bin/env python3
"""Liga autonomia, monitora /follower/status e /cmd_vel_auto por N
segundos, desliga com confirmacao. Uso: python3 live_test.py [segundos]"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from line_msgs.msg import FollowerStatus

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST, depth=10)
RUN_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
EXPECTED_SUBS = 3  # line_follower_node, head_servo_node, motor_serial_node


class LiveTest(Node):
    def __init__(self):
        super().__init__('live_test')
        self.enable_pub = self.create_publisher(Bool, '/controle/enable', QOS)
        self.create_subscription(Twist, '/cmd_vel_auto', self._on_cmd, QOS)
        self.create_subscription(FollowerStatus, '/follower/status', self._on_status, QOS)
        self.create_subscription(Float32, '/servo/state', self._on_servo, QOS)
        self.last_cmd = None
        self.last_status = None
        self.last_servo = None

    def _on_cmd(self, msg):
        self.last_cmd = msg

    def _on_status(self, msg):
        self.last_status = msg

    def _on_servo(self, msg):
        self.last_servo = msg.data

    def wait_subs(self, minimum, timeout_s=5.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            n = self.enable_pub.get_subscription_count()
            if n >= minimum:
                return n
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.enable_pub.get_subscription_count()

    def set_enable(self, value, settle_s=0.5):
        self.wait_subs(EXPECTED_SUBS)
        msg = Bool(); msg.data = value
        for _ in range(5):
            self.enable_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        end = time.time() + settle_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = LiveTest()
    try:
        node.set_enable(True)
        print(f'>>> AUTONOMIA LIGADA por {RUN_SECONDS:.0f}s -- OBSERVE O ROBO <<<')
        started = time.time()
        while time.time() - started < RUN_SECONDS:
            end = time.time() + 0.5
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            s, c = node.last_status, node.last_cmd
            if s is not None:
                servo = f'{node.last_servo:+.1f}' if node.last_servo is not None else '?'
                sat = 'SAT' if abs(c.angular.z if c else 0) >= s.wz_limit - 0.01 else '   '
                print(f'  t={time.time()-started:4.1f}s  st={s.state_name:9s} '
                      f'vx={c.linear.x if c else 0:.3f} wz={c.angular.z if c else 0:+.3f} '
                      f'/{s.wz_limit:.3f}{sat}  e={s.lateral_error*1000:+6.1f}mm  '
                      f'servo={servo}deg  sev={s.severity:.2f} conf={s.confidence:.2f}')
    finally:
        print('>>> DESLIGANDO <<<')
        node.set_enable(False, settle_s=0.8)
        node_c = node.last_cmd
        if node_c is not None:
            ok = abs(node_c.linear.x) < 1e-6 and abs(node_c.angular.z) < 1e-6
            print('ROBO PARADO (confirmado)' if ok else 'AVISO: comando nao esta em zero!')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Uso:

```bash
python3 scripts/live_test.py 5.0   # roda 5s de autonomia com log detalhado
```

### Isolar deriva mecânica vs. controle (teleop puro, sem visão)

Use delta de ticks acumulados (imune a ruído de quantização em baixa
velocidade — **não** use `/wheel_states` instantâneo para isso, ele é
ruidoso a baixas velocidades e pode indicar assimetria que não existe):

```python
#!/usr/bin/env python3
"""python3 straight_isolation.py [duracao_s] [vx]"""
import sys, time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
VX = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10

rclpy.init()
node = Node('straight_isolation')
cmd_pub = node.create_publisher(Twist, '/cmd_vel', QOS)
ticks = [None]
node.create_subscription(Int32MultiArray, '/wheel_encoder_ticks',
                          lambda m: ticks.__setitem__(0, list(m.data)), QOS)

t0 = time.time()
while time.time() - t0 < 5.0 and cmd_pub.get_subscription_count() < 1:
    rclpy.spin_once(node, timeout_sec=0.05)
while ticks[0] is None:
    rclpy.spin_once(node, timeout_sec=0.05)
before = ticks[0][:]

twist = Twist(); twist.linear.x = VX
t_start = time.time()
while time.time() - t_start < DURATION:
    cmd_pub.publish(twist)
    rclpy.spin_once(node, timeout_sec=0.02)
elapsed = time.time() - t_start

stop = Twist()
for _ in range(15):
    cmd_pub.publish(stop); rclpy.spin_once(node, timeout_sec=0.02)
time.sleep(0.3); rclpy.spin_once(node, timeout_sec=0.2)
after = ticks[0]

ticks_per_m = 330.0 / (3.14159265 * 0.078)  # 330 ticks/rev, roda 7.8cm
names = ['FL', 'FR', 'RL', 'RR']
speeds = [(a - b) / ticks_per_m / elapsed for a, b in zip(after, before)]
for name, v in zip(names, speeds):
    print(f'{name}: {v:+.4f} m/s')
left, right = (speeds[0]+speeds[2])/2, (speeds[1]+speeds[3])/2
print(f'esquerda={left:+.4f}  direita={right:+.4f}  '
      f'diff={abs(right-left)/max(abs((left+right)/2),1e-6)*100:.1f}%')
node.destroy_node(); rclpy.shutdown()
```

Diferença acima de ~8-10% entre lados sugere assimetria mecânica real
(diâmetro de roda, atrito, fiação) — abaixo disso, é ruído normal.

---

## Gravar e reproduzir dados (rosbag)

Para coleta extensa (pista maior, relatório de IC) — melhor que ler em
tempo real, porque não exige confirmação a cada poucos segundos:

```bash
ros2 bag record -o sessao_$(date +%F_%H%M) \
  /follower/status /line/detection /line_front/detection \
  /cmd_vel_auto /odom /servo/state /imu/data_raw \
  /wheel_states /wheel_targets /wheel_pwm /battery/voltage
```

Evite incluir `/line/debug_image`/`/line_front/debug_image` — as imagens
enchem o cartão rápido. Grave essas separadamente e curtas, só quando
precisar depurar visualmente um trecho específico.

Reproduzir depois (sem robô/câmera):

```bash
ros2 launch robot_bringup line_follower.launch.py cameras:=false serial:=false
ros2 bag play sessao_2026-09-09_2200
```

Análise rápida de um bag:

```bash
ros2 bag info sessao_2026-09-09_2200
```

---

## Plano de testes incremental

Sempre nesta ordem — cada etapa assume que a anterior passou.

1. **Rodas suspensas.** `TWIST` manual por `/cmd_vel`, confirme sentido de
   giro nas 4 rodas (`/wheel_states` positivo pra frente).
2. **Validação visual sem mover.** `serial:=false`, `rqt_image_view`,
   confira `line_width` medido contra a fita real.
3. **Sinal do erro.** Mova a mão pela direita da linha:
   `lateral_error > 0` **e** `cmd_wz < 0` em `/follower/status`. Se
   inverteu, pare e investigue antes de qualquer teste no chão.
4. **Reta lenta.** `v_max` baixo (~0.10) primeiro. `mean_abs_error` deve
   ficar de dígito único de mm depois de convergir.
5. **Curva ampla**, depois **curva fechada** — observe `wz` vs `wz_limit`
   (saturado = está no teto cinemático) e `severity`.
6. **Iluminação diferente** — `contrast` deve continuar > `min_contrast`;
   `confidence` > `confidence_ok`.
7. **Perda de 1 frame** (tapar a câmera rapidamente) — deve entrar em
   `COASTING` e voltar a `FOLLOWING` sem parar de todo.
8. **Perda persistente** (tapar por vários segundos) —
   `COASTING → RECOVERING (cabeça varre) → SAFE_STOP`.
9. **Retorno à teleop** — desligar autonomia deve zerar `/cmd_vel_auto` e
   centralizar o servo automaticamente.

Suba a velocidade (`v_max`) só depois que o item anterior estiver limpo.

---

## Tabela de diagnóstico (sintoma → causa → ajuste)

| Sintoma | Observe | Causa provável | Ajuste único | Critério de sucesso |
|---|---|---|---|---|
| Oscila em reta | `cmd_wz` alternando sinal | `k_lateral` alto | `k_lateral` −20% | `mean_abs_error` cai |
| Corta curva por dentro | `heading_error` grande, `wz` pequeno | `k_heading` baixo | `k_heading` +30% | entra alinhado na curva |
| Entra rápido demais na curva | `severity` sobe tarde | `ref_curvature` alto | `ref_curvature` −30% | `cmd_vx` cai antes da curva |
| Trava/para em curva fechada | `cmd_wz == wz_limit` constante | `v_min` baixo, **ou** o raio da curva é menor que `wheel_base_k/arc_ratio` (fisicamente impossível nessa velocidade) | `v_min` +0.02, ou aumentar `arc_ratio` | conclui a curva |
| Gira no lugar em vez de curvar | `wz_limit` muito alto | `arc_ratio` alto | `arc_ratio` → 0.65 | descreve arco, não pirueta |
| Deriva progressiva mesmo em reta | `curvature` alto numa reta conhecida | desalinhamento inicial fazendo curvatura parecer real (artefato geométrico) | confirme `max_heading_for_curvature` ativo; se `heading_error` já estava grande, o problema é físico (posicionamento), não parâmetro | `curvature_valid=false` quando heading é grande |
| Perde em sombra | `contrast` < `min_contrast` | limiar rígido | `min_contrast` −10 | `confidence` > `confidence_ok` volta |
| Pega reflexo em vez da linha | `line_width` reportado ≠ real | continuidade fraca | `track_weight` → 0.8 | `line_width` estável e correto |
| Banda distante cai, mas o contraste está alto | `bands_valid` < total com `contrast` normal | **gradiente de iluminação** (reflexo): um Otsu único não serve os dois extremos da ROI | `background_kernel_ratio` → 3.0 na câmera afetada | `bands_valid` volta ao total, `line_width` não muda |
| `COASTING` sem motivo aparente | `vision_age` perto/acima de `vision_timeout` | jitter de FPS da câmera | `vision_timeout` +0.1 | `loss_events` para de subir |
| Anda "colado" num lado da linha | `lateral_error` com viés constante | câmera fisicamente descentrada | `camera_x_offset_m` | erro médio → perto de 0 |
| Robô para sozinho (`SAFE_STOP`) | `state_name` no bag | perda persistente real | veja `contrast`/`confidence` no trecho anterior | volta a seguir depois de reposicionar |
| FPS de câmera baixo (< 15) | `ps -eo pcpu` no processo da câmera/percepção | CPU do Pi saturada | `work_width` → 128; confira se algum outro nó está consumindo CPU desproporcional | volta a > 20 fps |
| Servo "em passos" | movimento visual | `rate_hz` baixo e/ou filtro fraco | `rate_hz` → 40 (precisa restart) e/ou `track_filter_alpha` −0.1 | movimento visualmente contínuo |
| Servo "lerdo"/atrasado | mesma faixa de `rate_hz` alta, mas resposta lenta | `track_filter_alpha` baixo demais (suaviza demais) | `track_filter_alpha` +0.1~0.15 | resposta mais imediata, ainda sem passos |
| Ao comandar ângulo positivo, servo vai para o lado errado | teste de `MODE_HOLD` com ângulo conhecido | convenção física do servo invertida vs. convenção ROS | conserto já aplicado em `motor_serial_node` — se reaparecer após trocar hardware, inverter de novo lá | `+angulo` físico gira para a **direita** |
| Cabeça não faz nada (fica em 0°) mesmo pedindo `TRACK`/`HOLD` | `/autonomy/state` | watchdog do servo exige autonomia ligada | ligar autonomia (ver seção específica) | servo responde ao pedido |
| Comando de habilitar/desabilitar "não pegou" | `ros2 topic echo /autonomy/state` logo depois | corrida de descoberta DDS num `--once` isolado | use o padrão de burst+confirmação | leitura confirma o valor esperado |
| Cabeça "vira" mas a imagem da superior não muda | compare quadros em ângulos opostos (`logs/servo_prova.py`) | **primeiro suspeite do seu script**: `line_follower_node` publica `/head/request` a 50 Hz e disputa o servo. Só depois, mecânica | `head_control_enabled: false` antes de comandar; `/servo/state` NÃO detecta o problema, é eco do comando | `-35` vs `+35` difere em dezenas de níveis de cinza, não em ~2 |

---

## Armadilhas conhecidas desta stack

Coisas que custaram tempo real de depuração nesta bancada — documentadas
para não repetir o mesmo ciclo.

1. **`ros2 topic pub --once` pode disparar antes de todos os assinantes
   serem descobertos.** Com 3 nós assinando `/controle/enable`, o `--once`
   às vezes entrega a mensagem só a 1 ou 2 deles. Sempre confirme com
   `ros2 topic echo` depois, ou use o padrão de burst (seção [Ligar/
   desligar autonomia](#ligardesligar-autonomia-do-jeito-confiável)).

2. **Processos em background sobrevivem entre comandos de terminais
   diferentes.** Um `ros2 topic pub -r 10 ... &` que você não matou
   explicitamente continua rodando e pode brigar com o próximo comando
   que você mandar minutos depois, causando comportamento aparentemente
   aleatório de enable/disable. Sempre `kill $PID; wait $PID` depois de
   um publisher em background, e cheque com
   `ps -eo pid,args | grep "ros2 topic pub"` se sobrou algo.

3. **`ros2 topic hz`/`ros2 node list` têm cache do daemon `ros2cli` que
   pode ficar visivelmente desatualizado** (nós "fantasma" duplicados
   logo após um restart, por exemplo). Se a saída parecer impossível
   (nó aparecendo 2x, hz vazio com o tópico claramente publicando),
   confirme com um script `rclpy` direto antes de assumir que é bug real.

4. **Parâmetros só mudam ao vivo se o nó tiver
   `add_on_set_parameters_callback` registrado.** Sem isso, `ros2 param
   set` atualiza o valor guardado no servidor de parâmetros mas **não**
   o comportamento do nó (que cacheou o valor antigo num atributo no
   `__init__`). Todos os nós desta stack têm o callback — mas se você
   adicionar um novo parâmetro num nó, lembre de expor no callback
   também, senão o ajuste "ao vivo" é só ilusão.

5. **Velocidade lateral do controlador de roda não muda o raio mínimo de
   curva.** Com o limite `wz_limit = arc_ratio * vx / wheel_base_k`, o
   raio mínimo alcançável é `wheel_base_k / arc_ratio` — **uma constante,
   independente de `vx`** (o `vx` se cancela na razão `vx/wz`). Reduzir a
   velocidade numa curva fechada não ajuda a fazer a curva mais fechada;
   só ajuda a controlar melhor uma curva que já é geometricamente
   possível.

6. **Curvatura estimada por ajuste de bandas explode em dois casos
   comuns:** (a) numa **quina real** (reta que vira reta angulada, sem
   arco entre elas — o ajuste quadrático não tem como representar uma
   dobra), e (b) quando o **próprio robô está desalinhado** (mesmo numa
   reta real, ver a linha de um ângulo faz ela parecer curvar). Os dois
   casos têm o guard `max_heading_for_curvature`; o primeiro também
   ganhou `preview_heading_gain` (usa o ângulo direto, não a curvatura).

7. **Watchdogs de segurança se sobrepõem.** O servo não aceita `TRACK`/
   `HOLD`/`SCAN` sem autonomia ligada; a câmera superior não vale como
   direção sem o pan centrado (a menos que seja `servo_bearing`, que é
   o oposto — só vale **com** o pan girado). Ao testar um comportamento
   isolado, confira se algum desses gates não está silenciosamente
   forçando o resultado que você observa.

---

## Testes automatizados (pytest)

`line_geometry.py` e `line_controller.py` não importam `rclpy` nem `cv2`
no core — são testáveis com Python puro, sem robô, sem câmera:

```bash
cd ~/ros2_ws
source install/setup.bash
python3 -m pytest src/robot_bringup/test/test_line_geometry.py src/robot_bringup/test/test_line_controller.py -v
```

Cobrem: convenção de sinal (erro/heading/curvatura → `wz`), limite
cinemático do mecanum, máquina de estados completa (perda/recuperação/
parada segura), rejeição de curvatura espúria (quina e desalinhamento),
continuidade contra reflexos, feedforward do preview (curvatura + heading
+ bearing do servo, individualmente e combinados, sempre respeitando o
mesmo teto).

Lint (estilo, não cobre nós ROS que exigem hardware):

```bash
python3 -m flake8 --max-line-length 99 --select=E,F,C --extend-ignore=W503 src/robot_bringup/robot_bringup/
```

---

## Convenções de sinal

Válidas em **todo** o sistema, sem exceção — se algo parecer invertido,
o bug está em quem violou a convenção, não na convenção:

| Grandeza | Positivo significa |
|---|---|
| `lateral_error` (m) | linha à **direita** do robô |
| `heading_error` (rad) | linha se afastando inclinada para a **direita** |
| `curvature` (1/m) | pista curvando para a **direita** |
| `angular.z` / `wz` (rad/s) | giro **anti-horário** = **esquerda** (padrão ROS) |
| `angle_deg` do servo (`HeadRequest`, `/servo/state`) | pan apontando para a **direita** |

A tradução `erro positivo (direita) → wz negativo (direita)` acontece
**uma única vez**, em `line_controller.py::_steering()`. Nenhum outro
arquivo deveria precisar inverter sinal de novo — se precisar, é sinal de
que a convenção está sendo violada em algum lugar.

## Geometria e calibração física

Medidas do chassi (do firmware, `Config::HALF_L`/`HALF_W`):

```
Distância entre eixos (frente-trás):     17.6 cm  →  halfL = 8.8 cm
Distância entre rodas (esquerda-direita): 15.4 cm  →  halfW = 7.7 cm
wheel_base_k = halfL + halfW = 16.5 cm  =  0.165 m
Diâmetro das rodas:    7.8 cm
Ticks do encoder:      330 por volta
```

Câmeras (medidas com régua, ver comentários em `line_follower.yaml`):

```
Camera inferior (MONTAGEM NOVA, 10/09/2026): aponta RETO PARA BAIXO,
                 ~87 mm do chao, centro da lente 44 mm a frente do eixo
                 dianteiro (= 132 mm do centro cinematico).
                 Campo: 87.1 x 65.3 mm de chao, escala 7.4 px/mm.
                 ROI (ja sem o robo): 110.4 a 164.7 mm a frente do centro.
Camera superior: 20.5 cm a frente do robo, alcance visivel ~34 cm,
                 no servo (movimento horizontal).
```

### A montagem apontada para baixo — o que muda

**Vista ortográfica.** A fita mede a mesma largura em todas as linhas
(147.0 px perto contra 146.1 px longe — 0.6%, menor que o resíduo de
11.6 px do ajuste). Por isso `width_near_m == width_far_m`. Com a
montagem inclinada antiga a largura crescia com a distância.

**Campo escala 1:1 com a altura** nesta lente (~53° horizontal):
`largura_do_campo_em_mm ≈ altura_da_câmera_em_mm`. Verificado em duas
alturas: 30 mm → campo 30.0 mm; 87 mm → campo 87.1 mm. Use isso para
dimensionar qualquer suporte novo.

**A margem lateral é o que importa.** O detector separa a linha do piso
pelos **flancos escuros dos dois lados**; sem um flanco, ele perde a
referência de uma vez — e como todas as bandas têm a mesma largura, não
há a degradação suave que a montagem inclinada dava (banda próxima
estreita, distantes largas).

| altura | campo | margem/lado (fita 20 mm) | base de profundidade |
|---|---|---|---|
| 30 mm | 30 mm | 5 mm — **perde a linha a qualquer desvio** | 22.5 mm |
| 60 mm | 60 mm | 20 mm | 45 mm |
| **87 mm (atual)** | **87 mm** | **33.6 mm** | **54.5 mm** |
| 100 mm | 100 mm | 40 mm | 75 mm |

**O robô aparece no quadro.** Ocupa as 81 linhas mais próximas (brilho
médio ~50 contra ~120 do piso). Depois do `rotate_180` isso cai na base,
então o corte é `roi_bottom`, não `roi_top`.

**Isso também determina o `rotate_180`.** A parte que mostra o próprio
robô tem de ser a mais *próxima* do campo. No frame cru ela está no topo;
girando 180° vai para a base, que é onde `depth_at(y_ratio=1)` espera a
distância mínima. Como a linha corre na vertical (o que descarta rotações
de 90°), só restavam 0° e 180° — e o robô decide. **Não precisa de teste
com motor para conferir isso.**

**A antecipação passou a vir da montagem.** O erro é medido ~116 mm à
frente do centro, então `e_medido = e_centro + L·θ` e `k_lateral·L = 1.04`
já age como ganho de orientação — dois terços do `k_heading` nominal, de
graça. O controlador desconta isso sozinho (`_ganho_de_heading` lê o
`lookahead_distance` que o detector publica), o que mantém ζ = 1.00 em
toda a faixa de velocidade. Sem descontar ficaria em 1.37–1.66:
sobreamortecido, lento para voltar à linha.

Para recalibrar `width_near_m`/`width_far_m` depois de mudar a montagem
física de uma câmera:

1. Posicione o robô sobre a fita, parado e bem alinhado.
2. Meça a fração da largura da imagem que a fita ocupa nas bandas mais
   próxima e mais distante (via `/line/debug_image` ou analisando um
   frame salvo). Numa montagem apontada para baixo as duas dão o mesmo
   valor — se derem diferente, a câmera não está vertical.
3. `largura_de_chão = largura_real_da_fita / fração_ocupada` — aplique
   essa conta nas bandas próxima e distante para obter
   `width_near_m`/`width_far_m`.
4. Confira: `line_width` em `/line/detection` deve bater com a largura
   real da fita quando o robô está bem posicionado — é o teste da própria
   calibração.

---

## Histórico de correções relevantes

Para contexto de quem for revisar o código ou continuar os testes —
achados que só apareceram com o robô físico rodando, não visíveis em
revisão estática:

- **Matriz mecanum**: corrigida para X convencional depois da troca das
  rodas (roletes deixaram de estar espelhados nas traseiras). Verificado
  algebricamente (`INV @ F == identidade`) e com giro puro dando
  velocidades simétricas nas quatro rodas.
- **`k_feedforward` alto (0.85) cancelava a correção primária** — a
  curvatura da câmera inferior, com baseline de só ~10 cm, é ruidosa
  demais para um feedforward forte. Reduzido para 0.15 com validação ao
  vivo (erro lateral contido em ±2mm vs. deriva de 13-18mm antes).
- **Quina real (não curva suave) faz a curvatura ajustada explodir**
  (chegou a +11 1/m numa dobra de ~30°) — resolvido com
  `preview_heading_gain` (usa o ângulo direto da câmera superior, que não
  sofre da mesma instabilidade) e o guard `max_heading_for_curvature`.
- **Desalinhamento do próprio robô também inflava a curvatura** mesmo em
  pista reta (23° de desvio → +7 a +10 1/m medido) — mesmo guard resolve.
- **Servo físico invertido**: comando de `+25°` girava para a esquerda.
  Corrigido isolando a inversão na fronteira ROS↔serial
  (`motor_serial_node`), nos dois sentidos (`/servo/command` de saída e
  `SERVO_STATE` de entrada), para que o resto do sistema nunca precise
  saber disso.
- **`serial_read_rate` a 200 Hz** consumia ~40% de uma CPU do Pi só de
  polling, derrubando o FPS das câmeras por contenção — reduzido para
  50 Hz (10x a taxa real de chegada de telemetria do firmware, que é
  200 ms).
- **`head_servo_node` sem `add_on_set_parameters_callback`**: ajustes via
  `ros2 param set` não tinham efeito nenhum até reiniciar o nó — corrigido
  adicionando o callback (exceto `rate_hz`, que recria o timer e por isso
  continua exigindo restart).

### Sessão de 09/09/2026 — câmera superior, curva e alinhamento

Todas as afirmações abaixo vêm de corridas instrumentadas com o robô
andando, não de estimativa.

- **A câmera superior estava desestabilizando a direção.** Teste A/B
  isolado: com a realimentação do ângulo da cabeça no giro do corpo, o
  robô perdia a linha 31–33% do tempo e o viés de curva chegava a
  −16 mm. Deixando a cabeça **livre para rastrear** mas **sem
  realimentar** (`head_tracking_enabled: true`, `use_servo_bearing:
  false`), fez 45 s contínuos sem perder a linha uma vez e o viés caiu
  para +0,46 mm. A cabeça se mover não é o problema — injetar a leitura
  dela no laço fechado é.
- **Uma flag conflava quatro papéis.** `use_preview` controlava direção,
  realimentação do servo, velocidade **e recuperação** de uma vez.
  Desligá-la para curar a direção matava silenciosamente a dica de
  recuperação. Agora cada papel tem a sua chave (`preview_*_enabled`).
- **`use_servo_bearing` congelava a cabeça junto.** Desligar a
  realimentação mandava `head_mode` para `CENTER`, misturando duas
  decisões numa flag só — um A/B que queria isolar a realimentação
  acabava mudando duas variáveis. Separado em `head_tracking_enabled`.
- **A recuperação pela superior quase nunca está disponível.** Medido:
  quando a inferior perde a linha, a superior perdeu junto — confiança
  0,015 e utilizável em 0,3% dos frames perdidos. O FOV dela (±16,4°) é
  estreito demais para servir de sensor de recuperação. Foi mantida
  ligada porque é barata e segura, mas **não conte com ela**.
- **Em curva a curvatura é geometricamente indisponível.** Ela exige 4
  das 5 bandas, e as bandas distantes saem do quadro justamente quando o
  heading cresce: a 13,5 cm com heading de 20° a linha está a
  `20,6 + 135·tan20° = 69 mm`, além dos ±64 mm que aquela banda cobre.
  Medido: 71% da invalidez de curvatura vem de bandas insuficientes.
- **O `atan()` da antecipação da cabeça satura fácil.** `bearing =
  atan(D·κ/2)·ganho` com ganho 0,8 levava a cabeça a 44,7° (batente 45°)
  com curvatura de só 7,9 1/m. Reduzido para 0,5.
- **`/servo/state` é eco, não medição.** O firmware guarda o valor
  recebido e devolve ([`main.cpp:249`, `:208`]); servo de hobby não tem
  realimentação de posição. Para provar que a cabeça girou, compare
  imagens da câmera, não esse tópico.
- **Duas stacks rodando ao mesmo tempo** (uma minha, uma manual)
  produziram leituras de parâmetro misturadas e disputa de tópicos. O
  sintoma é `ros2 param list` vindo **duplicado**. Confira sempre
  `pgrep -af line_follower_node` antes de confiar numa medição.

### Sessão de 10/09/2026 — o reflexo da câmera inferior

**Sintoma relatado:** "camera inferior pega reflexo luminoso na parte
superior, uma das bandas fica quebrada."

**Medido no quadro capturado.** A mediana do piso cai de **151 (longe)
para 106 (perto)** — 45 contagens — mais 19 de inclinação lateral,
contra apenas **78** de contraste entre a linha e o piso. Ou seja, o
gradiente de iluminação valia mais da metade do sinal.

O Otsu que cada banda pediria ia de 168 (longe) a 131 (perto); o global
saiu **155**. Na banda distante o piso ao lado da fita estava em 151 —
4 contagens abaixo do limiar — e se **fundia** com ela:

```
banda 0   seg x  34..110  (39.9 mm)   <== o dobro da largura real
banda 1   seg x  55.. 94  (20.5 mm)
banda 2   seg x  56.. 94  (19.9 mm)
banda 3   seg x  56.. 94  (19.9 mm)
banda 4   seg x  56.. 94  (19.9 mm)
```

Com `line_width_tolerance: 0.45`, uma banda de 39.9 mm no lugar de 20 mm
é rejeitada pelo termo de largura. Daí "uma das bandas fica quebrada".

**Por que isso era o gargalo.** Na corrida anterior `1 - confiança` era o
**maior** termo do freio de velocidade (0.54 médio, p90 0.94) — o robô
andava devagar porque a detecção não confiava em si mesma, não por
geometria. E `1 - obs.confidence` entra direto em `_severity()`.

**Correção:** achatamento de fundo por *top-hat* com kernel horizontal,
antes do limiar. Para cada linha da imagem o fundo é o mínimo local numa
janela mais larga que a fita; subtraí-lo remove qualquer iluminação
suave. O limiar passa a sair da imagem achatada, mas **o contraste e o
teste de flanco continuam lendo o `gray` original** — eles medem física
da cena (linha branca ladeada de fita preta), não artefato de limiar.

Medido ao vivo, mesma posição, antes → depois:

| | antes | depois |
|---|---|---|
| bandas | 4/5 | **5.00/5** (mínimo 5) |
| confiança | 0.757 | **0.872** |
| contraste | 78.0 | 81.2 |
| largura | 19.9 mm | 19.9 mm |
| erro lateral | −2.6 mm | −2.6 mm |
| heading | −0.44° | 0.00° |

Largura e erro lateral **não mudaram** — a correção não introduziu viés,
só devolveu a banda. O heading limpou porque a banda inchada torcia o
ajuste.

**Custo de CPU: +0.26 ms sobre 6.71** (4%), medido intercalado. A
primeira medição deu +18 ms e estava **errada**: as duas metades do A/B
rodaram em instantes diferentes com a pilha inteira competindo por CPU.
Intercalar A/B/A/B cancela a deriva de carga — vale para qualquer
medição de desempenho neste Pi.

**Por que o kernel é 3× a largura da linha.** A janela precisa superar a
linha mais larga que a varredura horizontal pode ver: uma linha inclinada
em θ mede `largura/cos(θ)`, o **dobro** a 60°. Janela estreita demais faz
o top-hat comer a linha justamente na curva. Coberto por
`test_achatamento_nao_come_a_linha_inclinada`.

**Desligado na câmera superior**, e isso foi medido, não presumido: lá o
Otsu local de cada banda deu 169/173/172/170/171 contra 171 do global —
não há gradiente em profundidade. Ligar custaria: a largura medida caiu
de 19.3 para 18.0 mm, porque essa câmera tem perspectiva e uma janela
única é sempre um compromisso. **Reconsiderar** se voltar a sombra
diagonal que já prendeu a confiança dela em 0.49–0.52 com contraste 74:
contraste alto com confiança baixa é exatamente esta assinatura.

### Sessão de 10/09/2026 — o limite de quebra é 66°, e agora está medido

Com a câmera superior religada em **velocidade e recuperação** (direção
ainda desligada) e o `corrida.py` gravando **as duas câmeras**, a corrida
de 45 s atravessou 7 quebras. A superior avisou 4 delas, com 1.55 a
3.38 s de antecedência.

| quebra medida | desvio geométrico | cabe na margem? | parou? |
|---|---|---|---|
| 66.3° | 32.1 mm | **NÃO** | 1.44 s |
| 51.8° | 18.4 mm | sim | 0.00 s |
| 37.2° | 9.1 mm | sim | 0.00 s |
| 34.9° | 8.0 mm | sim | 0.00 s |

O desvio é `R(1/cos(Δ/2) − 1)` com `R = wheel_base_k / arc_ratio =
0.165 / 1.00 = 165 mm`, contra a margem de **31.9 mm** (campo de 83.9 mm
menos a fita de 20, dividido por dois).

Resolvendo para a margem atual: **o limite é 66.1°**. A única quebra que
travou mediu **66.3°** — 0.2° acima. As três que cabiam passaram sem
parar nenhuma vez.

**Ressalva de honestidade:** são 4 quebras, e o pico de heading é um
*proxy* do ângulo da quebra, não uma medida direta dele. A concordância
de 0.2° tem sorte dentro. O que é sólido é a **ordenação**: toda quebra
abaixo do limite passou, a única acima travou.

**O que a superior faz e o que ela não faz.** Ela avisou 2.21 s antes da
quebra de 66.3° — e o robô travou assim mesmo. Aviso não conserta
geometria: se o arco mínimo não cabe no campo, antecipar só faz frear
mais cedo. Onde ela paga é nas quebras que *cabem*: as de 51.8°, 37.2° e
34.9° passaram em arco, sem parada, e na corrida só com a inferior a
quebra grande equivalente parava 0.6 s.

**Como subir o limite** (em ordem de custo):

1. **Levantar a câmera inferior de 87 para ~110 mm.** O campo escala 1:1
   com a altura nesta lente, então a margem vai de 31.9 para 45 mm e o
   limite de quebra sobe de 66° para **76.4°**. Só remontagem e
   recalibração — nenhuma linha de código.
2. **Suavizar as quebras da pista**, que já era o seu plano.
3. `arc_ratio` acima de 1.0 diminuiria `R`, mas 1.0 já é o limite de
   tração: acima disso o robô pirueta em vez de arquear.

**Métricas da corrida** (melhores de todas até aqui): 99.8% de quadros
válidos, `|lat|` p50 **10.0 mm**, viés **+0.30 mm**, 3 perdas somando
**0.08 s**, 4.72 m percorridos, entrega 88%.

**Limite da própria superior:** válida em 86% dos quadros, curvatura
válida em 80%. Quando ela perde, o freio antecipado sai do ar sem aviso.

### Sessão de 10/09/2026 — 3 min de dados, e duas hipóteses minhas mortas

Corrida de 180 s, 22.088 amostras, **21.1 m**, gravada em
`logs/corrida_20260910_193825.csv`.

```
válidas 99.77%   |lat| p50 8.4  p90 26.5  max 35.4 mm   viés +1.10
perdas  4 episódios, 0.41s total (0.23% do tempo), maior 0.16s
entrega 93%      conf 0.674   bandas 4.66
19 quebras, 2 travadas (62.9° e 65.6°); tudo até 53.0° passou em arco
bateria 11.90 -> 11.91 V (mín 11.85) — sem queda em 3 min
```

**O limite de quebra precisa descontar o erro de rastreio.** O desvio da
quebra SOMA com o desvio que já existe: o robô não chega centrado, chega
com os ~8.4 mm típicos. Corrigindo o modelo:

| | só geometria | com o rastreio real |
|---|---|---|
| hoje (87 mm) | 66.1° | **57.8°** |
| com 110 mm | 76.4° | **70.1°** |

O modelo corrigido cai entre 53.0° (passou) e 62.9° (travou). O anterior
previa 66.1° e teria dito que as duas travadas deveriam passar.

#### Duas hipóteses de ajuste que a medição matou

Valem documentadas porque são erros plausíveis de repetir.

**1. "wz serrilha na reta, `k_lateral` está alto."** Falso. A
autocorrelação do `wz` na reta decai de +0.975 (40 ms) a +0.004 (960 ms)
**sem lóbulo negativo** — não há oscilação nenhuma, é um sinal suave com
~1 s de tempo de correlação. As 2.4 reversões de sinal por segundo são
travessias naturais de um sinal que ronda o zero. E 100% delas ocorrem
com o erro lateral do mesmo lado: é o termo de heading recuando na
aproximação, ou seja, o amortecimento fazendo o trabalho dele.

**2. "solavanco de vx, p99 de 1.485 m/s²."** Artefato de medição. O CSV
amostra a ~123 Hz e o controle roda a 50 Hz, então linhas consecutivas
têm dt ~7 ms com um passo inteiro de controle entre elas, e a divisão
infla a taxa. **Reamostrando na taxa real de controle: p99 0.857, máx
1.065** — as rampas configuradas (`accel: 0.35`, `decel: 0.60`) mais o
jitter de dt. Já existe limite de taxa e ele está funcionando.

> Lição geral: taxa derivada de um log só vale se o log e o laço tiverem
> a mesma taxa. Reamostre antes de derivar.

#### O que sobrou de real: a superior fica cega na quebra

Ela está inválida em **13.8% dos quadros** (24.7 s de 180), em 37
episódios. Seis episódios longos concentram **17.0 s**, e **5 desses 6
são seguidos de uma quebra em até 3 s**.

Não é o servo — a validade é melhor com a cabeça virada (94.9% entre
+8° e +20°) do que centrada (85.0%). Ela degrada antes de cair:
no quadro da queda a confiança já vinha em 0.166 com 2.08 bandas.

Ou seja, **o preview apaga exatamente quando mais vale**. É geometria do
campo dela (20.5 a 54.5 cm à frente): na quebra a linha sai do campo
distante. Próximo alvo de investigação — ROI e parâmetros da superior,
não ganhos do controlador.

### Sessão de 10/09/2026 — duas armadilhas do apontamento da cabeça

#### 1. `/head/request` tem DOIS publicadores

`line_follower_node` publica `/head/request` a 50 Hz sempre que a
autonomia está ligada. Qualquer script que também publique **disputa o
servo com ele**, e o resultado é o servo sendo puxado para o alvo do
seguidor no meio do movimento pedido.

Custou uma conclusão de hardware **errada** nesta sessão. Um teste de pan
sem o handshake deu:

```
-35° vs   0° :  2.31 níveis de cinza     <== "a câmera não gira"
  0° vs +35° :  2.32
-35° vs +35° :  2.33
```

Reportei que o servo tinha soltado do suporte. O sintoma que desmentiu
isso veio do operador: *"vejo ele se mexendo sim, mas parece que tem
algo puxando ele pro centro ao mesmo tempo"* — assinatura exata de dois
publicadores. Com `head_control_enabled: false` calando o seguidor, o
MESMO teste no MESMO hardware:

```
-35° vs   0° : 55.94 níveis de cinza     <== gira perfeitamente
  0° vs +35° : 26.36
-35° vs +35° : 49.57
```

**Sempre cale o seguidor antes de comandar a cabeça:**

```bash
ros2 param set /line_follower_node head_control_enabled false
# ... seu teste ...
ros2 param set /line_follower_node head_control_enabled true
```

`logs/servo_prova.py` e `logs/varredura.py` já fazem isso sozinhos.

#### 2. `/servo/state` é eco, não posição

`SERVO_STATE` no firmware (`main.cpp` linha 208) espelha `servoAngle`
(linha 249). Ou seja, **nenhuma checagem por tópico distingue "servo
girou" de "servo recebeu o comando e não girou"** — durante o teste
contaminado acima, `/servo/state` reportava +16.1° com toda a
convicção enquanto a imagem não mudava.

A prova é sempre a IMAGEM: capture em ângulos opostos e compare. Um pan
de 70° numa lente de ~53° desloca a cena por mais de uma largura de
quadro; abaixo de ~3 níveis de cinza é ruído de sensor.

#### 3. Comparar medições exige a MESMA cena

Duas medições da superior na "mesma" posição deram 287/287 quadros
válidos e 0/14. Não era ruído nem bug: o robô tinha sido movido entre
elas enquanto o suporte era inspecionado. Registre o `heading` da
inferior junto de qualquer leitura da superior — foi ele que denunciou
a troca de cena (+24° na quebra contra ~0° na reta).

### Sessão de 10/09/2026 — fechamento: a mira da cabeça, medida

Corrida de 180 s com a mira nova (`_mira_da_cabeca` + `track_hold_tau`),
em `logs/corrida_20260910_203230.csv`. Bateria 11.47 -> 11.42 V
(mín 11.41) — ficou acima do limiar, não contaminou.

#### O ganho está na disponibilidade da superior

| | antes | agora |
|---|---|---|
| superior válida | 86.0% | **94.9%** |
| tempo cega | 24.7 s | **9.0 s** |
| maior apagão | 5.70 s | **2.13 s** |
| quedas | 37 | 32 |
| `\|servo\|` médio | 4.4° | **8.6°** (máx 45°) |

E a validade cresce monotonicamente com o quanto a cabeça aponta —
o mecanismo confirmado de dentro dos dados:

```
servo  -8..+8   n=15239   válida 94.7%
servo  +8..+20  n= 3555   válida 96.4%
servo +20..+45  n= 2631   válida 97.3%
```

Antes desta mudança a mira comandava **0.0° em qualquer situação**: ela
vinha da curvatura da inferior, que fora desligada com evidência.

#### O limite geométrico: 6 de 6 entre duas corridas

Esta corrida caiu numa distribuição de quebras mais dura (p90 55.0 ->
60.2°, máx 65.6 -> 71.8°). Separando pelo limite de 57.8°:

| | corrida anterior | corrida de fechamento |
|---|---|---|
| quebras acima de 57.8° | 2 | 4 |
| delas travaram | **2/2** | **4/4** |
| abaixo de 57.8°, travaram | 0/17 | 2/19 |
| **parada média por travada** | 1.91 s | **0.76 s** |

Toda quebra acima do limite travou, nas duas corridas. O modelo
`R(1/cos(Δ/2) − 1) + erro de rastreio ≤ margem` está validado com n=6.
O que a mira melhorou não foi *evitar* a travada — geometria não se
negocia — e sim **sair dela 60% mais rápido**.

#### Resumo do corpo (praticamente inalterado, como esperado)

```
válidas 99.6%   |lat| p50 9.2  p90 26.0 mm   viés -0.91
perdas  4 episódios, 0.69 s        entrega 92%
20.2 m em 3 min, com quebras mais fechadas que na corrida anterior
```

#### Pendências para a semana que vem

1. **A superior não vê a linha no vértice** — e isso não é ajuste de
   detector: no vértice a pista já saiu do campo dela (20.5 a 54.5 cm à
   frente). Ela é sensor de AVISO (17/23 quebras, 1.5 a 3.4 s de
   antecedência), não de reencontro. Planejar em cima disso.
2. **Levantar a inferior para ~110 mm** sobe o limite de 57.8° para
   70.1°, cobrindo as quatro quebras que travaram nesta corrida.
   Decisão do operador foi adiar; a quebra fechada vira o teste.
3. **Suavizar as quebras da pista** — as acima de 63° são
   fisicamente impossíveis para este chassi na margem atual.

### Sessão de 10/09/2026 — "a guiada ficou mais ruidosa": o que a medição disse

Relato do operador após 2 voltas: *"o servo tá se alinhando bem com o
futuro da pista e curvas, mas a guiada em si achei um pouco mais ruidosa
do que naquele teste perfeito, mas ainda boa."*

**O comando não ficou mais ruidoso.** Comparando as duas corridas de
3 min, reamostradas a 50 Hz e restritas a reta em movimento:

| | corrida anterior | fechamento |
|---|---|---|
| `\|dwz/dt\|` p99 | 4.053 | **3.560** |
| `\|dvx/dt\|` p99 | 0.801 | **0.682** |
| erro de seguimento vx | 0.0289 | **0.0265** |
| erro de seguimento wz | 0.1542 | **0.1493** |
| escorregamento `\|vy\|` | 0.0093 | **0.0083** |
| bateria média | 11.88 V | 11.44 V |

Tudo igual ou melhor, com bateria pior. E a cabeça não contamina a
direção: correlação de **+0.044** entre `|dservo/dt|` e `|dwz/dt|`.

**O que o operador viu:** três vezes mais hesitações. A corrida caiu numa
distribuição de quebras mais dura (máx 65.6° -> 71.8°) e travou **6 vezes
contra 2** — cada travada 60% mais curta, mas o triplo delas. Some a isso
a cabeça se mexendo **2.5× mais** (5.48 -> 13.68 °/s).

#### Dois alarmes falsos evitados no caminho

Registrados porque o reflexo de acusar hardware é forte e estava errado
nas duas vezes:

**1. "As rodas da direita entregam só 82% das da esquerda."** Falso. A
pista é um **circuito fechado**: a odometria acumulou −12.5 e −11.1 rad,
ou seja 1.98 e 1.77 voltas. Pela cinemática mecanum, `dir − esq =
2·K·wz_médio`. Previsto −0.0238 e −0.0217 m/s; medido −0.0238 e −0.0217.
**Resto inexplicado: 0.0000.** Sempre desconte o giro líquido antes de
comparar lados.

**2. "21% dos quadros têm roda parada com comando alto."** Também falso
como enunciado: numa curva `FL = vx − K·wz` fica perto de zero sem falha
nenhuma. Filtrando pela previsão cinemática, sobram 14.6% — e ESSES são
reais (ver checklist da próxima sessão).

### Sessão de 18/09/2026 — firmware 2.0

Reescrita das fundações do firmware. Tudo medido antes e depois; os
números abaixo saem de `logs/` e do `DIAG` da propria telemetria.

| | antes | depois |
|---|---|---|
| quantizacao da velocidade de roda | 0.0743 m/s | **0.0186** (x4) |
| idem, na telemetria (janela 100 ms) | 0.0743 m/s | **~0.002** |
| `/odom` | 5 Hz | **50 Hz** |
| baud / ocupacao do link | 115200 / 9.1% | **921600 / 4.7%** |
| eixo do yaw da IMU | `gz` (morto) | `gx`, detectado pela gravidade |
| atrasos do laco de controle | 2.07% | **0.00%** |
| linhas seriais malformadas | 2 em 3283 | **0 em 4699** |

#### O que cada coisa consertou

**Encoders x1 -> x4.** Com x1 e janela de 10 ms a velocidade de roda so
podia valer multiplos de 0.0743 m/s, e o robo anda entre 0.07 e 0.20.
Uma roda a 0.106 m/s da 1.43 ticks por janela; abaixo de 1 ela le
EXATAMENTE ZERO girando perfeitamente. **Era isso, e nao falha
mecanica, que produzia o "roda parada" nos diagnosticos de pista** --
o achado que estava no topo do checklist da semana anterior e que agora
esta explicado e riscado.

**Migracao de Settings.** O NVS so tinha um flag "ok" ("ja gravei
alguma vez"), sem versao: mudar um default nunca chegava a um robo com
NVS gravada. Agora ha `SETTINGS_VERSION` com migracao -- a v1->v2
multiplica `ticksRev` por 4 em vez de forcar o default, preservando
calibracao fina.

**IMU no eixo errado.** A placa esta em pe, com o chip apontando para a
frente. Medido com o robo parado: a gravidade aparece em `ax = +9.85`,
nao em `az`. O firmware lia `gz`, que nesse arranjo e horizontal e fica
em ~0.001 rad/s -- o yaw integrado andava a 15% da taxa real, e o
`calibrate()` removia vies de `gz` enquanto `gx` carregava -0.132 rad/s
sem correcao. Nao afetou o controle (o ROS le `twist.angular.z`, que
vinha so dos encoders), mas deixava a IMU inutil.

**IMU agora entra no controle.** `od.wz` passou a ser 85% giroscopio +
15% encoder. Num mecanum os roletes patinam por projeto, entao a wz dos
encoders SUPERESTIMA rotacao justamente na curva. Verificado antes de
ligar: ruido 0.032 graus/s, vies sem deriva (-0.13169 -> -0.13170 em
60 s), e o SINAL conferido girando o robo a mao para a direita (97 de 97
amostras negativas, como manda a REP-103). Novo topico `/wheel_slip` =
wz_encoder - wz_giroscopio: patinagem medida, nao inferida.

**Tela apagada durante o movimento.** `display.display()` empurra 1024
bytes por I2C a 400 kHz -- ~23 ms bloqueando o mesmo `loop()` que roda o
controle a 100 Hz. Ideia do operador, e ela venceu a minha proposta de
task dedicada no core 1: periodo maximo caiu de 29843 para 13611 us e os
atrasos de 2.07% para **zero**, sem nenhum risco de concorrencia. A
task separada foi **descartada por medicao**, nao implementada.

> Armadilha: a condicao inicial era `!ros`, e estava errada. O
> `motor_serial_node` publica TWIST a 50 Hz mesmo com o robo PARADO,
> entao a tela nunca mais acendia com a pilha no ar. O criterio certo e
> movimento comandado, com histerese de 1 s.

**CRC8 nos comandos, so no sentido Pi -> ESP32.** Telemetria corrompida
o parser ja descarta pelo cabecalho; comando corrompido vira velocidade
errada num robo que anda. Testado: sem CRC passa (compatibilidade), com
CRC certo passa, com CRC errado e recusado e contado. A telemetria segue
em texto puro, entao `cat /dev/ttyACM0` continua servindo.

**UI do OLED.** Barra superior fixa com icones (bateria com nivel e
piscando quando critica, link serial, IMU), STATUS com tensao grande e
barras bipolares de VX/WZ, listas com selecao invertida e barra de
rolagem, telas HOST (IP da Raspberry, via `SET_INFO`) e DIAG (jitter,
CRC recusado, motivo da parada, uptime). Menu abre no clique e volta
para STATUS apos 10 s sem toque.

> Armadilha: o `clearDisplay()` estava espalhado por cada funcao de
> desenho e o STATUS ficou sem -- ele desenhava POR CIMA do menu
> anterior. Agora e **um so**, no inicio de `draw()`, onde e impossivel
> uma tela nova nascer sem ele.

#### Ferramenta nova: `gravar.sh`

Grava o firmware derrubando a pilha ROS antes e subindo depois. Existe
porque o `motor_serial_node` segura `/dev/ttyACM0` e o upload falha com
erro confuso (o esptool reclama de sincronismo, nao de porta ocupada).

> Armadilha: a primeira versao matava so `robot_bringup/`, mas o
> `camera_node` vem do pacote `camera_ros` e sobrevivia segurando o
> pipeline do libcamera. Chegaram a rodar **tres instancias empilhadas**.
> O script agora casa a arvore inteira do launch e ABORTA se sobrar
> processo, em vez de empilhar.

### Sessão de 18/09/2026 — a câmera está perfeita; a percepção e que perde quadro

Investigacao aberta pelo operador: *"no rqt eu vejo mais frames do que
18"*. Ele estava certo, e a correcao derrubou uma conclusao minha.

**A camera entrega 30 Hz exatos, sem jitter.** Medido contando
`camera_info` (mensagem minuscula) em vez de `image_raw` (1 MB/quadro):

```
901 quadros em 30 s = 30.0 Hz, nas duas cameras
carimbo de captura do driver:  p50 33.30   p90 33.31   max 33.35 ms
FrameDurationLimits = [33333, 33333] us  -> 30 fps travados
```

**RETRATADO:** eu havia reportado a camera com `p90 98 ms` e picos de
`503 ms`. Era **o meu proprio no de medicao engasgando** ao receber
imagens de 1 MB em Python -- medi o meu gargalo e atribui a camera.

> Licao: para medir a CADENCIA de um topico de imagem, conte um topico
> leve publicado junto (`camera_info`) ou o carimbo do driver. Um
> assinante Python de `image_raw` mede a si mesmo.

**O que de fato perde quadro e a percepcao**: recebe 16-28 dos 30,
processando em 5.5-11 ms. Como processar leva ~9 ms e ainda assim ela
perde, o custo esta em RECEBER e converter, nao em processar.

#### Vale reescrever visao e controle em C++?

| | medido | veredito |
|---|---|---|
| CPU da pilha ROS | 181% de 400% | nao ha escassez |
| jitter do controle (alvo 20 ms) | p50 19.8, p99 26.2, zero estouros | **C++ nao ajuda** |
| quadros perdidos na percepcao | ~25% | **so aqui C++ ajudaria** |

O grosso do processamento ja E C++: `resize`, Otsu, morfologia e top-hat
sao OpenCV compilado. O Python so orquestra. C++ atacaria o custo de
RECEBER a imagem -- e seria reescrever a parte mais sutil e mais testada
do projeto (160 testes na geometria) por ~35% de um nucleo que sobra.

#### Caminho barato ainda ABERTO: resolucao

O detector reduz para `work_width` (160 px) na PRIMEIRA operacao: de
307200 px ele usa ~19200. Os outros 16x sao serializados, transportados
e convertidos para serem jogados fora.

A 320x240 a CPU caiu forte -- percepcao 68.5% -> 45.8%, camera 38.0% ->
20.5%. Isso e solido (CPU nao depende da cena).

**Mas falta provar que o campo de visao e PRESERVADO.** Se o libcamera
escolher um modo que CORTA em vez de escalar, a margem lateral encolhe
-- e margem e exatamente o que limita a quebra (57.8 graus). Perder
campo por CPU que sobra seria pessimo negocio.

A primeira tentativa de medir isso **nao vale**: usei a largura
reportada da fita como sonda, e o robo nao estava sobre a linha (imagem
crua escura e desfocada, confianca 0.11). As leituras de 20.73, 28.62 e
38.16 mm foram todas sobre cena invalida.

Para fechar, com o robo NA PISTA:

```bash
python3 logs/fov.py 640x480          # com a pilha em 640x480
# relancar com cam_width:=320 cam_height:=240
python3 logs/fov.py 320x240
```

`logs/fov.py` mede a **fracao da largura do quadro** ocupada pela fita,
sem depender do detector nem da calibracao. Fracao igual = so escala, e
320x240 sai de graca. Fracao maior = esta cortando, e a ideia morre.

#### Ponta solta fechada

O modo parecia voltar sozinho de SEGUIDOR para PARADO. Testado: 3 min
observando `/robot/mode`, **zero mudancas espontaneas**. Era script meu
publicando PARADO sem eu rastrear. Nao ha bug.

> E de novo a armadilha do DDS: publicar em `/robot/mode_request` sem
> esperar `get_subscription_count() > 0` faz o pedido cair no vazio.
> `logs/modo.py` ja espera a descoberta -- use ele.
