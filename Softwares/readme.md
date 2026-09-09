# Line Follower — Visão Dupla (ROS 2 Jazzy + ESP32)

Robô seguidor de linha com chassi mecanum 4x4, câmera inferior (percepção
primária) + câmera superior em servo (preview/antecipação), controlado por
Raspberry Pi (ROS 2 Jazzy) com interface de baixo nível num ESP32-S3.

Este README é um guia de **operação e depuração** — comandos prontos para
copiar/colar durante testes na bancada e na pista. Não é o design doc (esse
fica nos comentários do próprio código e do `config/line_follower.yaml`).

> ⚠️ **Este robô anda de verdade.** Antes de habilitar autonomia, garanta
> espaço livre, esteja de olho no robô e saiba o comando de STOP (seção
> [Parar tudo, agora](#parar-tudo-agora)).

---

## Sumário

- [Arquitetura em um parágrafo](#arquitetura-em-um-parágrafo)
- [Diagrama de nós e tópicos](#diagrama-de-nós-e-tópicos)
- [Estrutura do repositório](#estrutura-do-repositório)
- [Build](#build)
- [Subir o sistema](#subir-o-sistema)
- [Parar tudo, agora](#parar-tudo-agora)
- [Ligar/desligar autonomia (do jeito confiável)](#ligardesligar-autonomia-do-jeito-confiável)
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

Ao subir, os 7 processos esperados são: `camera_bottom`, `camera_front`,
`line_perception_bottom`, `line_perception_front`, `line_follower_node`,
`head_servo_node`, `motor_serial_node`.

```bash
ros2 node list
# deve listar exatamente esses 7 (mais o proprio "ros2 launch" no ps aux)
```

## Parar tudo, agora

O sistema já tem 3 camadas de segurança independentes, mas se precisar
agir manualmente:

```bash
# 1) Zera o comando publicado (autonomia OU teleop, o que estiver ativo)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# 2) Desliga a autonomia (o robo passa a obedecer so /cmd_vel, que esta zerado)
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: false}"

# 3) Ultimo recurso: mata a stack toda
pkill -9 -f "ros2 launch robot_bringup|camera_node|line_perception_node|line_follower_node|head_servo_node|motor_serial_node"
```

Redundâncias já embutidas no sistema (não dependem de você digitar nada):
- **Firmware do ESP32**: para os motores se não chegar `TWIST` novo em 200 ms.
- **`motor_serial_node`**: zera comando se não chegar `/cmd_vel`/`/cmd_vel_auto`
  novo em `cmd_timeout` (0.30 s).
- **`line_follower_node`**: se a câmera inferior parar de publicar ou a
  confiança cair, o supervisor desacelera → recupera → para sozinho
  (`RECOVERING` → `SAFE_STOP`), nunca avança "as cegas".
- **Botão físico no ESP32**: alterna autonomia via menu local (`MENU,
  CONTROL_TOGGLE`), independente do que o Pi estiver fazendo — sempre
  funciona mesmo se o ROS travar.

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
| `depth_near_m` / `depth_far_m` | `0.035` / `0.135` | `0.205` / `0.545` | distância física (m) da ROI, ver [Geometria](#geometria-e-calibração-física) |
| `width_near_m` / `width_far_m` | `0.038` / `0.128` | `0.220` / `0.420` | largura de chão coberta pela imagem, calibrado |
| `camera_x_offset_m` | `0.0` | `0.0` | deslocamento lateral da câmera vs. eixo do robô |
| `line_width_m` | `0.020` | `0.020` | largura real da fita (meça a sua!) |
| `roi_top` / `roi_bottom` | `0.12` / `1.00` | `0.45` / `1.00` | recorte vertical analisado |
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

**Câmera superior (preview):**

| Parâmetro | Valor | Nota |
|---|---|---|
| `use_preview` | `true` | |
| `preview_timeout` | `0.40` | s |
| `preview_confidence_min` | `0.40` | |
| `preview_ff_gain` | `0.35` | feedforward de curvatura antecipada — só vale para curva **suave** |
| `preview_heading_gain` | `0.40` | feedforward de heading antecipado — vale para **quina** (reta→reta angulada), onde curvatura explode |
| `preview_ff_max_ratio` | `0.30` | teto do preview (curvatura+heading+bearing somados), fração de `wz_max` — garante que a câmera de cima nunca domina |
| `preview_center_tol_deg` | `8.0` | acima disso, pan não está centrado → preview de imagem não vale como direção |
| `preview_speed_gain` | `1.0` | peso do preview na redução de velocidade |
| `use_servo_bearing` | `true` | **experimental**: cabeça rastreia a linha ativamente (`HeadMode.TRACK`); o próprio ângulo do servo (lido de `/servo/state`) vira sinal de bearing do corpo — não exige pan centrado, ao contrário do preview de imagem |
| `servo_bearing_gain` | `1.2` | rad/s por rad de ângulo do servo |
| `scan_in_recovery` | `true` | cabeça varre durante `RECOVERING` |

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
| `COASTING` sem motivo aparente | `vision_age` perto/acima de `vision_timeout` | jitter de FPS da câmera | `vision_timeout` +0.1 | `loss_events` para de subir |
| Anda "colado" num lado da linha | `lateral_error` com viés constante | câmera fisicamente descentrada | `camera_x_offset_m` | erro médio → perto de 0 |
| Robô para sozinho (`SAFE_STOP`) | `state_name` no bag | perda persistente real | veja `contrast`/`confidence` no trecho anterior | volta a seguir depois de reposicionar |
| FPS de câmera baixo (< 15) | `ps -eo pcpu` no processo da câmera/percepção | CPU do Pi saturada | `work_width` → 128; confira se algum outro nó está consumindo CPU desproporcional | volta a > 20 fps |
| Servo "em passos" | movimento visual | `rate_hz` baixo e/ou filtro fraco | `rate_hz` → 40 (precisa restart) e/ou `track_filter_alpha` −0.1 | movimento visualmente contínuo |
| Servo "lerdo"/atrasado | mesma faixa de `rate_hz` alta, mas resposta lenta | `track_filter_alpha` baixo demais (suaviza demais) | `track_filter_alpha` +0.1~0.15 | resposta mais imediata, ainda sem passos |
| Ao comandar ângulo positivo, servo vai para o lado errado | teste de `MODE_HOLD` com ângulo conhecido | convenção física do servo invertida vs. convenção ROS | conserto já aplicado em `motor_serial_node` — se reaparecer após trocar hardware, inverter de novo lá | `+angulo` físico gira para a **direita** |
| Cabeça não faz nada (fica em 0°) mesmo pedindo `TRACK`/`HOLD` | `/autonomy/state` | watchdog do servo exige autonomia ligada | ligar autonomia (ver seção específica) | servo responde ao pedido |
| Comando de habilitar/desabilitar "não pegou" | `ros2 topic echo /autonomy/state` logo depois | corrida de descoberta DDS num `--once` isolado | use o padrão de burst+confirmação | leitura confirma o valor esperado |

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
Câmera inferior: 4.2 cm do chão, comeca a enxergar ~3.5cm a frente do
                 robo, alcance visivel ~10 cm. Angulo do suporte: 60 graus.
Câmera superior: 20.5 cm a frente do robo, alcance visivel ~34 cm,
                 no servo (movimento horizontal).
```

Para recalibrar `width_near_m`/`width_far_m` depois de mudar a montagem
física de uma câmera:

1. Posicione o robô sobre a fita, parado e bem alinhado.
2. Meça a fração da largura da imagem que a fita ocupa nas bandas mais
   próxima e mais distante (via `/line/debug_image` ou analisando um
   frame salvo).
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
