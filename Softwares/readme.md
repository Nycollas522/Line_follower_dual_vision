# Robo Seguidor de Linha - ROS 2 Jazzy

Robo omnidirecional com Raspberry Pi 5, duas cameras CSI e ESP32-S3. A Raspberry executa visao e controle; o ESP32 executa cinematica, PID das rodas, encoders, IMU, bateria, servo e OLED.

## Arquitetura

```text
camera inferior (/cam_bottom/image_raw)
  -> visao_linha_node -> /line/error, /line/status
  -> controle_node -> /cmd_vel
  -> motor_serial_node -> serial TWIST,vx,vy,wz
  -> ESP32 -> cinematica mecanum + PID das rodas

camera superior frontal (/cam_front/image_raw, servo pan esquerda/direita)
  -> visao_linha_frente_node -> /line_front/error, /line_front/status
  -> controle_node: preview de curva e busca de linha

ESP32 -> ENC, WHEEL, IMU, ODOM, BATT, SERVO_STATE -> motor_serial_node
```

A camera inferior e a referencia principal do seguidor. A camera superior e complementar: ela enxerga a pista a frente para antecipar curvas e procurar a linha quando a inferior a perde.

## Seguranca

- `controle_node` inicia desabilitado.
- Teste com rodas suspensas ou em area livre apos toda mudanca.
- B1 longo no OLED (`>= 0.7 s`) e parada de emergencia do firmware.
- Desabilitar `/controle/enable` tambem para o seguimento e zera os comandos.
- O bridge serial para o robo se `/cmd_vel` ficar sem atualizar por mais de 0.5 s.

## Build e execucao ROS 2

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
ros2 launch robot_bringup bringup_dual_camera.launch.py
```

O launch sobe as duas cameras, as duas visoes, o controle e o bridge serial. Mesmo assim, o robo so se move apos habilitar:

```bash
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: true}"
```

Para parar:

```bash
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: false}"
```

Executaveis esperados:

```text
robot_bringup visao_linha_node
robot_bringup visao_linha_frente_node
robot_bringup controle_node
robot_bringup motor_serial_node
```

## Firmware ESP32

Projeto: `~/Documents/PlatformIO/Projects/Firmware_Ippo`.

Use o PlatformIO do VS Code ou:

```bash
cd ~/Documents/PlatformIO/Projects/Firmware_Ippo
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run --target upload
```

Use `~/.platformio/penv/bin/pio`; o `pio` em `/usr/bin/pio` pode ser uma versao antiga incompatível.

### Protocolo serial

O protocolo segue `geometry_msgs/Twist`:

```text
TWIST,vx,vy,wz
vx = frente/tras (m/s)
vy = lateral (m/s)
wz = giro/yaw (rad/s)
```

Nao altere a ordem dos campos em apenas um lado. Firmware e ROS devem usar a mesma convencao.

## Geometria do chassi

Valores medidos e usados no firmware:

| Medida | Valor | Parametro |
|---|---:|---|
| Distancia entre centros dianteiro/traseiro | 17.6 cm | `halfL = 0.088 m` |
| Distancia entre centros esquerdo/direito | 15.4 cm | `halfW = 0.077 m` |
| Diametro das rodas | 7.8 cm | `wheelDiameter = 0.078 m` |
| Encoder | 330 ticks/volta | `ticksRev = 330` |
| $K = halfL + halfW$ | 0.165 m | firmware e `wheel_turn_k` no ROS |

No seguimento normal, o ROS limita a rotacao para evitar que uma roda precise inverter o sentido e o robo pivote em vez de descrever um arco:

$$
|wz| \leq \frac{|vx|}{wheel\_turn\_k}
$$

`wheel_turn_k` deve continuar igual a `halfL + halfW` configurado no ESP32. Nao e ganho de PID; so mude se a geometria medida mudar.

## Cameras e topicos

```text
/cam_bottom/image_raw       camera inferior fixa
/cam_front/image_raw        camera superior no servo
/line/debug_image           debug da camera inferior
/line/mask                  mascara da camera inferior
/line_front/debug_image     debug da camera superior
/line_front/mask            mascara da camera superior
```

Para visualizar e calibrar:

```bash
ros2 run rqt_image_view rqt_image_view
ros2 topic hz /cam_bottom/image_raw
ros2 topic hz /cam_front/image_raw
```

As imagens de debug/mascara so sao publicadas se alguem estiver inscrito. Abra o `rqt_image_view` para diagnostico; sem ele o sistema economiza CPU.

## Camera inferior: referencia principal

`visao_linha_node` publica:

| Topico | Conteudo |
|---|---|
| `/line/error` | erro lateral em pixels; positivo significa linha a direita |
| `/line/status` | `true` somente com linha detectada |
| `/line/centroid` | centroide detectado |

Quando a linha nao e achada, `/line/error` ainda sai como `0.0`. Portanto, nunca interprete `0.0` como deteccao central sem consultar `/line/status`. O controle ja usa o status corretamente.

Parametros da visao inferior:

| Parametro | Padrao | Ajuste quando... |
|---|---:|---|
| `white_s_max` | 80 | Aumente se a linha branca estiver sendo rejeitada; diminua se piso/reflexos entrarem. |
| `white_v_min` | 180 | Diminua em ambiente escuro; aumente se reflexos claros forem detectados. |
| `roi_start_ratio` | 0.5 | Diminua se a linha estiver acima da ROI; aumente para ignorar partes distantes. |
| `kernel_size` | 5 | Aumente para ruído; diminua se a linha for fina e sumir. Use impar. |
| `min_area` | 300 | Diminua se a linha e descartada; aumente contra falsos positivos. |

## Camera superior: linha fina e preview

`visao_linha_frente_node` publica `/line_front/error`, `/line_front/status`, `/line_front/centroid`, debug e mascara. Ela usa ROI central e uma morfologia diferente porque a linha distante ocupa poucos pixels:

1. HSV;
2. blur pequeno;
3. `MORPH_CLOSE` para unir intervalos da linha;
4. `MORPH_OPEN` pequeno para remover pontos isolados.

Fechar antes de abrir e importante: abrir com kernel grande primeiro pode erodir uma linha fina ate ela desaparecer.

Estes parametros aceitam `ros2 param set` ao vivo, sem reiniciar o no:

```bash
ros2 param set /visao_linha_frente_node white_v_min 160
ros2 param set /visao_linha_frente_node white_s_max 100
ros2 param set /visao_linha_frente_node roi_start_ratio 0.25
ros2 param set /visao_linha_frente_node roi_end_ratio 0.80
ros2 param set /visao_linha_frente_node open_kernel_size 1
ros2 param set /visao_linha_frente_node min_area 25.0
```

| Parametro | Padrao | Funcao |
|---|---:|---|
| `white_s_max` | 80 | Saturacao maxima do branco/cinza. |
| `white_v_min` | 180 | Brilho minimo. Reduza primeiro para `160` se nao enxerga linha. |
| `roi_start_ratio` / `roi_end_ratio` | 0.35 / 0.75 | Faixa vertical analisada. Deve obedecer $0 \le start < end \le 1$. |
| `blur_kernel_size` | 3 | Suavizacao; `<= 1` desliga. Nao aumente primeiro para linha fina. |
| `close_kernel_size` | 9 | Une fragmentos. Aumente se a linha aparecer interrompida. |
| `open_kernel_size` | 3 | Remove ruido. Use `1` para desligar se a linha estiver sumindo. |
| `min_area` | 40 | Area minima. Reduza para linha distante; aumente contra falsos positivos. |

### Calibrar a superior

1. Mantenha `use_front_camera=false` no controle.
2. Centralize o servo:

   ```bash
   ros2 topic pub --once /servo/command std_msgs/msg/Float32 "{data: 0.0}"
   ```

3. Abra `/line_front/debug_image` e `/line_front/mask`.
4. Ajuste a ROI para a faixa amarela cruzar a linha.
5. Ajuste HSV para a linha ficar branca na mascara.
6. Se fragmentar, aumente `close_kernel_size`; se sumir, use `open_kernel_size=1` e reduza `min_area`.
7. So entao habilite preview e varredura.

## Controle PID da linha

O `controle_node` usa erro normalizado da camera inferior, filtro, PID, reducao de velocidade em curva e limitacao de aceleracao angular.

Ordem recomendada de ajuste:

1. Comece com `Ki=0`, `Kd=0` e `v_linear=0.10` a `0.15`.
2. Aumente `Kp` ate corrigir o desvio sem oscilar.
3. Se oscilar, reduza `Kp` ou aumente `error_filter_alpha` levemente.
4. Use `Kd` pequeno apenas para sobrecorrecao rapida persistente.
5. Use `Ki` apenas para erro constante em reta apos estabilizar o resto.
6. Depois aumente `v_linear` e ajuste curva.

| Parametro | Padrao | Aumentar | Diminuir |
|---|---:|---|---|
| `Kp` | 0.8 | resposta lenta | oscilacao lateral |
| `Ki` | 0.0 | erro constante em reta | oscilacao lenta/acumulo |
| `Kd` | 0.0 | amortecer sobrecorrecao | ruido no comando |
| `v_linear` | 0.15 m/s | so apos estabilidade | curvas/pista estreita |
| `v_angular_max` | 1.0 rad/s | curva nao acompanha | tenta girar demais |
| `max_angular_accel` | 3.5 rad/s² | giro reage tarde | giro muda bruscamente |
| `error_deadband` | 0.08 | treme perto do centro | deixa pequenos desvios passar |
| `error_filter_alpha` | 0.15 | imagem ruidosa | resposta atrasada |
| `error_max` | 320 px | deve ser metade da largura para imagem 640px | nao reduza sem motivo |
| `curve_threshold` | 0.20 | ganho extra deve entrar mais cedo | entra cedo demais |
| `curve_boost` | 3.0 | curva fechada nao acompanha | giro excessivo |

Exemplo de tuning ao vivo:

```bash
ros2 param set /controle_node Kp 0.60
ros2 param set /controle_node v_linear 0.12
ros2 param set /controle_node curve_boost 2.0
ros2 param set /controle_node v_angular_max 0.70
```

## Preview e varredura com camera superior

Ative somente depois de calibrar a visao superior:

```bash
ros2 param set /controle_node use_front_camera true
ros2 param set /controle_node front_scan_enabled true
```

### Seguimento normal

Com o servo perto do centro (`front_max_servo_angle_deg=12`), a camera superior adiciona uma correcao antecipada e reduz velocidade antes de curvas. Ela nunca substitui a inferior como referencia final sobre a linha.

| Parametro | Padrao | Uso |
|---|---:|---|
| `front_feedforward_gain` | 0.5 | Aumente se curva chega tarde; diminua se antecipa demais. |
| `front_speed_preview_gain` | 0.6 | Aumente para frear antes; diminua se fica lento em reta. |
| `front_max_servo_angle_deg` | 12° | Faixa de pan confiavel para preview. Mantenha pequena. |

### Recuperacao de linha perdida

Depois de perder a inferior por `line_lost_timeout` (0.25 s), o robo para de avancar. Se `front_scan_enabled=true`, a camera superior varre esquerda/direita. Quando encontra a linha, o servo para naquela direcao e a rotacao do robo usa o bearing estimado:

$$
bearing = pan\ do\ servo + \frac{erro\ horizontal\ normalizado \cdot FOV\ horizontal}{2}
$$

Assim, uma linha centralizada numa camera que esta virada 30° para a direita ainda e corretamente interpretada como linha a direita do robo. Quando a inferior recupera a linha, o servo volta para 0°.

| Parametro | Padrao | Uso |
|---|---:|---|
| `front_scan_enabled` | `false` | Habilita a varredura na recuperacao. |
| `front_scan_amplitude_deg` | 45° | Limite para cada lado. Comece conservador para proteger cabos. |
| `front_scan_rate_deg_s` | 45°/s | Velocidade do servo. Rapido demais pode borrar a imagem. |
| `front_scan_recovery_time` | 4 s | Tempo maximo de busca com a superior ativa. |
| `front_detection_timeout` | 0.25 s | Tempo maximo para uma deteccao superior ser considerada atual. |
| `front_horizontal_fov_deg` | 70° | FOV aproximado da lente usado no bearing. Substitua pelo FOV real quando conhecido. |
| `front_bearing_deadband_deg` | 3° | Evita escolher lado por ruido no centro. |
| `front_servo_reversed` | `false` | `true` se servo positivo faz a camera olhar fisicamente para esquerda. |
| `front_image_reversed` | `false` | `true` se erro de imagem positivo gera giro para o lado errado. |

Teste os sentidos com rodas suspensas e corrija sem editar codigo:

```bash
ros2 param set /controle_node front_servo_reversed true
ros2 param set /controle_node front_image_reversed true
```

## ESP32, OLED e botoes

O firmware tem menu local persistente em flash. A tela inicial mostra tensao da bateria grande, estado da bateria, conexao ROS, velocidade e giro.

| Entrada | Acao |
|---|---|
| B1 curto | proximo item / aumenta valor |
| B1 medio (0.25 a 0.7 s) | item anterior / diminui valor |
| B1 longo (>= 0.7 s) | parada de emergencia em qualquer tela |
| B2 curto | entrar, selecionar ou confirmar |
| B2 longo editando | sair da edicao |
| B2 longo fora de edicao | salvar flash e voltar para Home |

Configuracoes editaveis:

```text
KP, KI, KD
PWM MIN, PWM LIMITE
TICKS/VOLTA
CAL BATERIA
MAX VELOCIDADE
PESO YAW
ACEL GIRO
EIXO L, EIXO W, DIAM RODA
SALVAR
```

Testes locais:

```text
BATERIA
SERVO
4 MOTORES
ENCODERS
IMU (B1 recalibra)
RESET ENCODERS
RESET ODOM
CONTROLE ROS
```

Em `TESTES -> CONTROLE ROS -> B2`, o ESP32 pede ao bridge que alterne `/controle/enable`; o OLED recebe a confirmacao `ROS ON/OFF`. Isso habilita/desabilita o seguimento e nao inicia/mata processos Linux.

Durante `TESTE IMU`, mantenha o robo parado antes de usar B1 para recalibrar. A IMU e usada no firmware para combinar yaw do giroscopio (`gz`) e yaw de encoder na odometria. O PID de linha ROS continua visual.

## Bateria

O firmware usa media de 32 amostras ADC e filtro exponencial. Isso remove ruido rapido; a calibracao corrige erro sistematico.

O valor padrao e `battCal=0.987`, calculado da medicao:

$$
battCal = \frac{11.78\ V\ (multimetro)}{11.935\ V\ (ADC)} \approx 0.987
$$

Configuracoes ja salvas preservam o valor antigo. Se o OLED ainda mostra aproximadamente 11.93-11.94 V com 11.78 V no multimetro, entre em `CONFIGURACOES -> CAL BATERIA`, ajuste para `0.987` em passos de `0.001` e faca B2 longo para salvar.

Para recalibrar depois:

$$
novo\ battCal = battCal\ atual \cdot \frac{tensao\ do\ multimetro}{tensao\ exibida\ no\ OLED}
$$

Meça com o robo parado e em condicao de carga parecida com uso real.

## Diagnostico

```bash
ros2 node list
ros2 node info /controle_node
ros2 node info /motor_serial_omni
ros2 topic echo /line/status
ros2 topic echo /line_front/status
ros2 topic echo /line/error
ros2 topic echo /line_front/error
ros2 topic echo /battery/voltage
ros2 topic echo /servo/state
ros2 topic hz /cmd_vel
ros2 topic hz /odom
rqt_graph
```

Monitor serial:

```bash
~/.platformio/penv/bin/platformio device monitor --port /dev/ttyACM0 --baud 115200
```

Mensagens esperadas: `READY`, `IMU,READY`, `ENC`, `WHEEL`, `ODOM`, `BATT` e `SERVO_STATE`. Mensagens `IMU,NOT_FOUND`, `OLED,NOT_FOUND` ou `ERR,...` indicam falha ou configuracao de hardware.

## Sequencia de validacao

1. Confirme OLED, bateria e IMU com rodas suspensas.
2. Confirme as duas imagens e ajuste mascara/ROI da inferior.
3. Ajuste a mascara/ROI da superior ate `/line_front/status` ficar confiavel.
4. Teste somente camera inferior, em baixa velocidade e com `use_front_camera=false`.
5. Ajuste PID e velocidade da inferior.
6. Ative `use_front_camera=true` e valide preview de curva.
7. Ative `front_scan_enabled=true`; esconda a linha da inferior e valide a varredura e o sentido de recuperacao.
8. Parametros alterados por `ros2 param set` voltam ao padrao depois do restart, a menos que sejam incluidos no launch/YAML. Configuracoes do OLED/firmware sao persistidas com SALVAR.
