# Guia de operação — seguidor de linha

Como ligar, parar, trocar de modo, resetar e diagnosticar o robô. Todos os
comandos rodam **no Raspberry Pi** (SSH ou terminal local), com o ROS já
carregado:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

Conferido contra o código em 24/09/2026 (firmware 2.0.0, protocolo 2). O
[README](README.md) guarda arquitetura, parâmetros e histórico; **para
operar, vale este arquivo**.

---

## Sumário

1. [Parar o robô agora](#1-parar-o-robô-agora)
2. [A pilha ROS: ligar, parar, reiniciar](#2-a-pilha-ros-ligar-parar-reiniciar)
3. [Modos: PARADO, SEGUIDOR, RC](#3-modos-parado-seguidor-rc)
4. [Autonomia: armar e desarmar](#4-autonomia-armar-e-desarmar)
5. [Perfil de velocidade](#5-perfil-de-velocidade)
6. [Rodar uma corrida](#6-rodar-uma-corrida)
7. [Menu do ESP32 (OLED)](#7-menu-do-esp32-oled)
8. [Resets](#8-resets)
9. [Gravar o firmware](#9-gravar-o-firmware)
10. [Calibrações](#10-calibrações)
11. [Joystick (modo RC)](#11-joystick-modo-rc)
12. [Ferramentas](#12-ferramentas)
13. [Problemas comuns](#13-problemas-comuns)
14. [Referência rápida](#14-referência-rápida)

---

## 1. Parar o robô agora

Em ordem do mais rápido ao mais drástico:

| Situação | O que fazer | Efeito |
|---|---|---|
| Qualquer momento, no robô | **B1 longo** (> 0,7 s, de qualquer tela) | Para os motores **e trava**; o Pi desarma e vai para PARADO (ver abaixo) |
| Rodando `corrida.py` | **Ctrl-C** no terminal dela | Desliga a autonomia, volta o modo para PARADO e ainda grava o CSV |
| Qualquer momento, no robô | Menu **MODO → Parado → B2 curto** | O Pi sai do SEGUIDOR e **sempre desarma** a autonomia |
| Qualquer momento, no robô | Menu **TESTES → CONTROLE ROS → B2 curto** | Alterna a autonomia (liga ↔ desliga) |
| Pelo terminal | `python3 ~/ros2_ws/logs/modo.py PARADO` | Igual ao menu MODO → Parado |
| Nada responde | `sudo systemctl stop robo` | A serial para de mandar comando e o **watchdog do ESP32 zera os motores em 200 ms** |
| Último recurso | Cortar a energia dos motores | — |

### Como o B1 longo funciona

1. O ESP32 para os motores, centra a cabeça, apita e **passa a ignorar
   todo comando de movimento do Pi** (LED de parado).
2. Avisa o Pi (`MENU,STOP`); o Pi desarma a autonomia e põe o modo em
   **PARADO** (o joystick também para). Aparece no log:
   `PARADA DE EMERGENCIA pelo menu do ESP32`.
3. **A trava só sai com uma ação explícita de voltar a andar**: armar a
   autonomia de novo, ou entrar em SEGUIDOR ou RC (pelo menu ou pelo ROS).
   A `corrida.py` faz isso sozinha na próxima corrida.
4. Se o Pi não responder (ROS travado), o robô **continua parado**.

> Até 24/09/2026 o B1 longo só zerava o comando e o `TWIST` do Pi (50 Hz)
> fazia o robô voltar a andar em ~20 ms. Firmware anterior a essa data
> não tem a trava.

Proteções que funcionam sozinhas, sem ninguém fazer nada:

- **Watchdog do ESP32:** sem `TWIST` novo por 200 ms, zera os motores.
- **Watchdog do `motor_serial_node`:** sem `/cmd_vel_auto` (ou `/cmd_vel`
  no RC) por 0,3 s, manda zero.
- **Seguidor:** perdeu a linha → desacelera → gira procurando
  (`RECOVERING`) → para (`SAFE_STOP`). Nunca avança às cegas.
- **Bateria crítica:** o ESP32 **só avisa** (LED e OLED piscando, motivo
  no DIAG); **não para os motores**. Pare você.

---

## 2. A pilha ROS: ligar, parar, reiniciar

A pilha (câmeras, percepção, controle, servo, serial, modo, joystick)
**sobe sozinha no boot** pelo serviço `robo.service`. Ela sobe sempre em
**PARADO**, com a autonomia desligada: ligar o Pi nunca faz o robô andar.

| Quero… | Comando |
|---|---|
| Ver se está no ar | `systemctl status robo` |
| Parar | `sudo systemctl stop robo` |
| Subir | `sudo systemctl start robo` |
| Reiniciar (depois de mudar código ou YAML) | `sudo systemctl restart robo` |
| Acompanhar o log | `tail -f ~/ros2_ws/logs/boot.log` |
| Não subir mais no boot | `sudo systemctl disable robo` |
| Voltar a subir no boot | `sudo systemctl enable robo` |

A subida leva ~25 s. Para conferir se está tudo no ar:

```bash
ros2 node list
```

Devem aparecer os 10 nós: `camera_bottom`, `camera_front`,
`line_perception_bottom`, `line_perception_front`, `line_follower_node`,
`head_servo_node`, `motor_serial_node`, `mode_manager_node`, `joy_node`,
`joy_teleop_node`. Um `rqt_gui_...` a mais é de algum `rqt` aberto em outro
computador.

**Regras:**

- **Não rode `ros2 launch` com o serviço no ar.** Duas instâncias disputam
  as câmeras (libcamera) e a serial, e nenhuma funciona direito. Para
  lançar na mão (depuração), pare o serviço antes.
- **Não pare a pilha com `pkill`.** O serviço tem `Restart=on-failure`: o
  systemd sobe tudo de novo 10 s depois. Use `systemctl stop`.
- **Mudou código Python ou o YAML?** Com o build em `--symlink-install`,
  basta `sudo systemctl restart robo`. Arquivo novo ou `setup.py` mudado
  pedem build antes:

  ```bash
  cd ~/ros2_ws && colcon build --packages-select robot_bringup --symlink-install
  ```

**Mensagens normais no log** que não são defeito:

- `Unable to open camera calibration file` — aparece a cada boot; a
  percepção usa a geometria do YAML, não esse arquivo.
- `nenhum frame em 5 s` com o robô em **PARADO** — o PARADO desliga a
  percepção de propósito (economia de CPU).

---

## 3. Modos: PARADO, SEGUIDOR, RC

| Modo | Quem dirige | Percepção | Câmeras |
|---|---|---|---|
| **PARADO** | ninguém | desligada | publicando |
| **SEGUIDOR** | seguidor de linha (quando armado) | ligada | publicando |
| **RC** | joystick | desligada | publicando |

Trocar de modo **não reinicia nada**: os nós só mudam de comportamento.
As câmeras publicam nos três modos, para dar para ver a imagem no `rqt` de
um PC externo.

**Escolher um modo nunca arma a autonomia.** SEGUIDOR liga a percepção e
dá ao seguidor autoridade sobre as rodas, mas o robô só anda depois de
armar (seção 4). Sair do SEGUIDOR **sempre desarma**.

### Pelo menu do ESP32

MODO → B1 curto até o modo desejado → **B2 curto** confirma. Embaixo da
lista aparece o modo ativo de fato: ele só muda quando o Pi confirma.

### Pelo ROS

O jeito confiável, que espera a descoberta DDS e confirma:

```bash
python3 ~/ros2_ws/logs/modo.py SEGUIDOR     # ou PARADO, RC
```

Na mão, repetindo a publicação para não perder a mensagem:

```bash
ros2 topic pub -t 5 -r 5 /robot/mode_request std_msgs/msg/String "{data: SEGUIDOR}"
ros2 topic echo --once /robot/mode          # confirma o modo ativo
```

> `ros2 topic pub --once` costuma sair antes de o `mode_manager_node` ser
> descoberto, e a mensagem se perde em silêncio. Use `-t 5 -r 5` ou o
> `modo.py`.

---

## 4. Autonomia: armar e desarmar

Armar = o seguidor passa a comandar as rodas (só no modo SEGUIDOR).

| Onde | Armar | Desarmar |
|---|---|---|
| `corrida.py` | automático | automático no fim ou no Ctrl-C |
| Menu | TESTES → CONTROLE ROS → B2 (alterna) | idem |
| Terminal | ver abaixo | ver abaixo |

```bash
# armar (so anda se o modo for SEGUIDOR)
ros2 topic pub -t 15 -r 15 /controle/enable std_msgs/msg/Bool "{data: true}"

# desarmar
ros2 topic pub -t 15 -r 15 /controle/enable std_msgs/msg/Bool "{data: false}"

# confirmar -- quem manda de fato e o motor_serial_node
ros2 topic echo --once /autonomy/state
```

`/controle/enable` tem **3 assinantes** (`line_follower_node`,
`head_servo_node`, `motor_serial_node`). Publicar uma vez só pode chegar
só em parte deles; por isso as 15 repetições e a confirmação pelo
`/autonomy/state`.

### Mexer na cabeça sem o robô andar (modo bancada)

O servo só aceita comando com a autonomia armada. Para testar servo ou
câmeras com o robô no chão, **tire a autoridade do corpo antes**:

```bash
ros2 param set /line_follower_node body_control_enabled false
ros2 param get /line_follower_node body_control_enabled     # confirme: False
```

Scripts Python devem usar `logs/servo_seguro.py`, que faz isso e confere:

```python
from servo_seguro import trava_motores, liga_autonomia, devolve
trava_motores()            # ou trava_motores(percepcao=True) se precisar de deteccao
liga_autonomia(node, True) # levanta erro se o motor_serial nao confirmar
...
liga_autonomia(node, False)
devolve()
```

---

## 5. Perfil de velocidade

Escolhido **antes da corrida**, no menu do ESP32:
**CONFIG → VELOCIDADE → B2 curto (editar) → B1/B2 curto (trocar) →
B2 longo (sair da edição) → SALVAR → B2 curto**.

Vale na hora do SALVAR e continua valendo depois de reiniciar.

| Perfil | Curvas (`v_min`) | Retas (`v_max`) | Medido em 24/09 (3 min) |
|---|---|---|---|
| SUAVE | 0,08 m/s | 0,25 m/s | — |
| MEDIA | 0,10 m/s | 0,32 m/s | — |
| RAPIDA | 0,12 m/s | 0,44 m/s | volta de 58 s, 8 recuperações curtas |

Os números ficam no YAML (`mode_manager_node`: `perfil_v_min`,
`perfil_v_max`); o ESP32 só guarda qual perfil foi escolhido.

```bash
ros2 topic echo --once /robot/speed_profile                 # perfil em vigor
grep "Perfil de velocidade" ~/ros2_ws/logs/boot.log | tail -1
```

Para testar um valor sem mexer no YAML (vale até reiniciar a pilha):

```bash
ros2 param set /line_follower_node v_max 0.30
```

---

## 6. Rodar uma corrida

> A corrida move o robô. Rode **no seu terminal**; o assistente
> (Claude Code) é bloqueado de propósito para isso.

1. Robô sobre a linha, bateria conferida.
2. Perfil escolhido no menu (seção 5).
3. Rode:

   ```bash
   python3 ~/ros2_ws/scripts/corrida.py 180      # segundos; padrao 45
   ```

   Ela põe o modo em SEGUIDOR, espera as duas câmeras, arma, grava e no
   fim desarma e volta a PARADO. **Ctrl-C** a qualquer momento faz o
   mesmo e ainda salva o CSV.
4. Resultado: `~/ros2_ws/logs/corrida_AAAAMMDD_HHMMSS.csv`, com um resumo
   no terminal (percepção, atuação por roda, bateria).
5. Mapa do percurso:

   ```bash
   python3 ~/ros2_ws/logs/mapa.py ~/ros2_ws/logs/corrida_AAAAMMDD_HHMMSS.csv
   ```

   Gera um PNG ao lado do CSV. Com mais de uma volta, mostra o erro de
   fechamento (posição e giro); nas corridas de 24/09 ficou entre 1,5 e
   5,6 cm por volta de 9,8 m.

---

## 7. Menu do ESP32 (OLED)

### Botões

| Botão | Toque curto (< 0,25 s) | Toque médio (0,25–0,7 s) | Longo (> 0,7 s) |
|---|---|---|---|
| **B1** | avança / `+` na edição | volta | **parada de emergência travada** (seção 1) |
| **B2** | entra / confirma / `−` na edição | — | sai da edição; fora dela, **salva e volta ao início** |

### Abas do início (B1 curto troca, B2 curto entra)

| Aba | Para quê |
|---|---|
| **MODO** | Parado / Seguidor de linha / RC (seção 3) |
| **CONFIG** | Configurações gravadas no ESP32 (lista abaixo) |
| **TESTES** | Testes de hardware e resets (lista abaixo) |
| **HOST** | IP, nome e estado do Pi |
| **DIAG** | Jitter do controle, comandos com CRC recusado, motivo da última parada |
| **DESLIGAR PI** | Pede o desligamento da Raspberry (seção 8) |

### CONFIG

KP, KI, KD, PWM MIN, PWM LIMITE, TICKS/VOLTA, CAL BATERIA, MAX VELOCIDADE,
PESO YAW, ACEL GIRO, EIXO L, EIXO W, DIAM RODA, **SERVO TRIM**,
**VELOCIDADE**, **SALVAR**.

Editar: B2 curto entra na edição; B1 curto soma, B2 curto subtrai;
**B2 longo sai da edição**. As mudanças valem na hora, mas só ficam
gravadas depois de **SALVAR**.

**Todo SALVAR aparece no log do Pi**, com todos os valores e o horário.
É assim que se confere o que ficou gravado de verdade:

```bash
grep "ESP32: SETTINGS" ~/ros2_ws/logs/boot.log | tail -1
```

Valores de fábrica para conferência: `halfl=0.0880`, `halfw=0.0770`,
`wheeld=0.0780`, `maxmps=0.70`, `kp=120`, `ki=100`, `kd=5`,
`ticks=1320`. Em 24/09: `strim=-8.50` (medido), `bcal=1.082` (ainda
não conferido com multímetro).

### TESTES

| Item | O que faz |
|---|---|
| BATERIA | Mostra a tensão |
| SERVO | B1 curto/médio gira de 10 em 10 graus (90 = centro calibrado) |
| 4 MOTORES | **Move as rodas.** B1 curto/médio muda o PWM de 50 em 50 |
| ENCODERS | Mostra as contagens |
| IMU | B1 curto **calibra a IMU** (robô parado!) |
| RESET ENCODERS | Zera as contagens |
| RESET ODOM | Zera a posição (x, y, yaw) |
| CONTROLE ROS | Alterna a autonomia (arma/desarma) |

---

## 8. Resets

| Quero zerar… | Como | Observação |
|---|---|---|
| **Posição (odometria)** | Menu TESTES → RESET ODOM | O `mapa.py` já usa a posição relativa ao início da corrida; não é obrigatório |
| **Encoders** | Menu TESTES → RESET ENCODERS | — |
| **IMU** | Menu TESTES → IMU → B1 curto | Robô **parado** e apoiado; recalibra o viés do giroscópio |
| **ESP32** | Botão **RST** da placa | Motores param; ao voltar, o Pi reenvia o estado e registra o `SETTINGS` no log |
| **Pilha ROS** | `sudo systemctl restart robo` | Volta em PARADO, desarmado |
| **Um parâmetro mexido na mão** | `sudo systemctl restart robo` | Volta ao YAML |
| **Descoberta DDS travada** | `sudo systemctl stop robo`, depois `rm -f /dev/shm/fastrtps_*`, depois `start` | Só com a pilha parada |
| **Raspberry** | `sudo reboot` | A pilha sobe sozinha no boot |

### Desligar a Raspberry

Pelo terminal:

```bash
sudo shutdown -h now
```

Espere o OLED apagar antes de cortar a chave (o ESP32 é alimentado pelo
Pi). Cortar a energia com o Pi ligado pode corromper o cartão SD.

**Pelo menu (DESLIGAR PI → B2 curto) hoje só registra o pedido**: a regra
de sudoers que autoriza o desligamento **não está instalada**. O pedido
aparece no log (`PEDIDO DE DESLIGAMENTO`), e pedidos nos primeiros 60 s
depois da subida são ignorados de propósito. Para habilitar:

```bash
sudo install -m 440 ~/ros2_ws/scripts/robo-poweroff /etc/sudoers.d/robo-poweroff
sudo visudo -c        # tem que responder "parsed OK"
```

Para desabilitar de novo: `sudo rm /etc/sudoers.d/robo-poweroff`.
Histórico: a primeira versão dessa função desligava o Pi sozinha e a causa
nunca foi achada — por isso ela ficou só registrando.

---

## 9. Gravar o firmware

O `motor_serial_node` segura a porta serial, então a pilha precisa estar
parada. O script **recusa gravar** com o serviço ativo.

```bash
sudo systemctl stop robo
cd ~/Documents/PlatformIO/Projects/Firmware_Ippo && ./gravar.sh --off
sudo systemctl start robo
```

Conferir depois:

```bash
grep -E "ESP32: (READY|SETTINGS)" ~/ros2_ws/logs/boot.log | tail -2
```

As configurações gravadas (trim, perfil, PID…) **sobrevivem** à
gravação: ficam na memória não volátil do ESP32. Quando uma versão nova
muda o significado de um campo, o firmware migra o valor sozinho.

---

## 10. Calibrações

### Trim do servo (alinhamento da câmera superior)

O centro da cabeça fica no firmware (CONFIG → SERVO TRIM, passo de 0,5°).
Recalibre sempre que o suporte da câmera for mexido:

1. Robô parado **no meio de uma reta**, alinhado com a linha. Não toque
   nele durante a medida.
2. Varra o pan nos dois sentidos, comparando as duas câmeras (com os
   motores travados). O ângulo em que o heading da superior iguala o da
   inferior é o centro; a média entre subir e descer desconta a folga do
   servo (~2°).
3. Some a diferença ao SERVO TRIM (sinal: pan que alinha em `+x` → trim
   `−x`), SALVAR, e confira o `strim` no log.

Referência: em 24/09 o centro ficou reproduzível ao centésimo de grau
depois de apertar o braço do servo. Antes disso a montagem escorregava.
Um risco de caneta atravessando braço e eixo mostra se escorregou.

### Bateria

CONFIG → CAL BATERIA. Meça a tensão real com multímetro e ajuste até o
robô mostrar o mesmo valor. **Pendente:** o valor gravado (1,082) difere
do padrão (0,987) e nunca foi conferido; as leituras podem estar ~10%
altas.

### Largura da linha nas câmeras

`python3 ~/ros2_ws/logs/calibra_camera.py` calcula `width_near_m` /
`width_far_m` medindo a fita na imagem.

---

## 11. Joystick (modo RC)

DualShock 4 pareado no Bluetooth do **Pi** (o ESP32-S3 não tem Bluetooth
clássico). Pareamento: ver a seção "Modo RC" do [README](README.md).

1. Modo **RC** (menu ou `modo.py RC`).
2. Segure **L2** (homem-morto): solto, o robô para.

| Comando | Controle |
|---|---|
| Frente / trás | analógico esquerdo, vertical |
| Girar | analógico esquerdo, horizontal |
| Andar de lado | L1 / R1 |
| Cabeça (pan) | analógico direito, horizontal |
| Turbo (×1,6 nos limites) | R2 |

Limites: 0,20 m/s à frente, 0,15 m/s de lado, 1,2 rad/s de giro. Sentido
invertido: `invert_vx`, `invert_wz`, `invert_vy`, `invert_servo` no YAML
ou ao vivo com `ros2 param set /joy_teleop_node invert_vx true`.

---

## 12. Ferramentas

Em `~/ros2_ws/logs/` (no repositório: `Softwares/ros2_ws/tools/`) e
`~/ros2_ws/scripts/`. **⚠️ = move o robô** — rode só com o robô
suspenso ou com espaço livre.

| Script | Para quê |
|---|---|
| `scripts/corrida.py` ⚠️ | Corrida instrumentada (seção 6) |
| `scripts/teste_rodas.py` ⚠️ | Confere se as quatro rodas respondem |
| `logs/prova_mecanum.py` ⚠️ | Mede se as rodas estão montadas no X |
| `logs/diag_roda.py` ⚠️ | Onde a roda deixa de alcançar o alvo: PID, saturação ou atrito |
| `logs/imu_vs_encoder_motor.py` ⚠️ | Giroscópio × encoders girando por comando |
| `logs/modo.py` | Troca o modo com confirmação |
| `logs/mapa.py` | Mapa e fechamento de volta a partir de um CSV |
| `logs/captura.py` | Salva imagens cruas e de debug das duas câmeras |
| `logs/amostra.py` | Leitura rápida das duas câmeras, parado |
| `logs/servo_seguro.py` | Trava dos motores para scripts de servo (seção 4) |
| `logs/varre_mira.py` | Varre o servo e mede a detecção da superior |
| `logs/olha_angulo.py` | Fotografa a superior em ângulos escolhidos |
| `logs/servo_prova.py` | Prova que o servo gira a câmera |
| `logs/varredura.py` | Varre o servo para inspeção visual do eixo |
| `logs/quebra_teste.py` | Detecções, servo e imagens no mesmo instante |
| `scripts/servo_panorama.py` | Panorama da câmera superior em vários ângulos |
| `logs/calibra_camera.py` | Largura da fita → `width_near_m` / `width_far_m` |
| `logs/fov.py` | Mede o campo de visão sem o detector |
| `logs/imu_vs_encoder.py` | Giroscópio × encoders girando à mão |
| `logs/sinal_imu.py` | Sinal do eixo de guinada da IMU, sem motor |

Todo script de servo usa a trava do `servo_seguro.py` antes de armar.

---

## 13. Problemas comuns

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| `corrida.py`: "mode_manager_node não apareceu" | Pilha fora do ar ou incompleta | `ros2 node list`; se faltar nó, `sudo systemctl restart robo` |
| `corrida.py`: "sem detecção em …" | Percepção desligada ou câmera travada | Ver o log; reiniciar a pilha |
| "nenhum frame em 5 s" no log | Modo PARADO (normal) | Nada, se o modo for PARADO |
| Robô não obedece ao enable/disable | Mensagem perdida na descoberta, ou um `ros2 topic pub -r` esquecido rodando | Usar `-t 15 -r 15`, conferir `/autonomy/state`, matar publishers esquecidos |
| Nós não se enxergam depois de muitos restarts | Segmentos DDS antigos em `/dev/shm` | Seção 8, "Descoberta DDS travada" |
| Cabeça torta no boot | Trim errado ou suporte escorregou | Seção 10 |
| Mudança no menu "não pegou" | Faltou SALVAR, ou editou outro item | Conferir o `SETTINGS` no log |
| Lento nas retas | Perfil baixo | Seção 5 |
| Perde a linha sempre na mesma quina fechada | Cabeça no limite de 45°; conhecido | Recupera sozinho em < 0,8 s; melhoria pendente |
| Robô fraco, entrega menos que o pedido | Bateria baixa | Resumo da `corrida.py` mostra tensão e % entregue |
| `gravar.sh` recusa | Serviço ativo | Seção 9 |

---

## 14. Referência rápida

```bash
# estado
systemctl status robo
ros2 node list
ros2 topic echo --once /robot/mode
ros2 topic echo --once /autonomy/state
ros2 topic echo --once /robot/speed_profile
ros2 topic echo --once /battery/voltage
grep "ESP32: SETTINGS" ~/ros2_ws/logs/boot.log | tail -1

# modo e autonomia
python3 ~/ros2_ws/logs/modo.py PARADO | SEGUIDOR | RC
ros2 topic pub -t 15 -r 15 /controle/enable std_msgs/msg/Bool "{data: false}"

# pilha
sudo systemctl restart robo
tail -f ~/ros2_ws/logs/boot.log

# corrida e mapa
python3 ~/ros2_ws/scripts/corrida.py 180
python3 ~/ros2_ws/logs/mapa.py ~/ros2_ws/logs/corrida_*.csv

# testes de logica
cd ~/ros2_ws && python3 -m pytest src/robot_bringup/test -q
```

| Tópico | Tipo | Para quê |
|---|---|---|
| `/robot/mode_request` | `String` | Pedir modo (PARADO/SEGUIDOR/RC) |
| `/robot/mode` | `String` | Modo ativo |
| `/controle/enable` | `Bool` | Armar/desarmar a autonomia |
| `/autonomy/state` | `Bool` | Estado real da autonomia (do `motor_serial_node`) |
| `/robot/speed_profile` | `String` | Perfil de velocidade em vigor |
| `/cmd_vel_auto` | `Twist` | Comando do seguidor |
| `/cmd_vel` | `Twist` | Comando do joystick / manual |
| `/line/detection`, `/line_front/detection` | `LineDetection` | Detecção das câmeras inferior e superior |
| `/servo/state` | `Float32` | Ângulo **comandado** da cabeça (eco, não leitura) |
| `/odom` | `Odometry` | Posição e velocidade (encoders + giroscópio) |
| `/battery/voltage` | `Float32` | Tensão da bateria |
