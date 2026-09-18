# Line Follower — Visão Dupla

Robô seguidor de linha com **chassi mecanum 4x4** e **duas câmeras**:
uma inferior apontada para o chão, que é a percepção primária do laço
rápido, e uma superior num servo, que serve de aviso antecipado de
curva.

O processamento é dividido em dois: a **Raspberry Pi 5** roda visão e
controle em ROS 2 Jazzy; o **ESP32-S3** fecha o laço de velocidade de
cada roda a 100 Hz e é quem corta os motores se a comunicação cair.

---

## Organização

| pasta | conteúdo |
|---|---|
| **[`Softwares/`](Softwares/)** | Firmware do ESP32-S3 e a pilha ROS 2. Comece pelo [guia de operação](Softwares/ros2_ws/README.md). |
| [`Eletronica/`](Eletronica/) | Placas e esquemáticos |
| [`Mecanica/`](Mecanica/) | Chassi e modelos CAD |

---

## Especificações

### Chassi

| grandeza | valor |
|---|---|
| Entre-eixos | 176 mm |
| Bitola | 154 mm |
| Diâmetro da roda mecanum | 78 mm |
| Raio mínimo de arco | 165 mm |

### Eletrônica

- **Raspberry Pi 5** — visão e controle (ROS 2 Jazzy)
- **ESP32-S3** — controle de baixo nível, PID por roda a 100 Hz
- 2× câmera CSI (OV5647)
- Servo de pan para a câmera superior
- IMU MPU6050 e display OLED SSD1306
- Bateria LiPo 3S

### Percepção

A **câmera inferior** aponta reto para baixo, 132 mm à frente do centro
cinemático. A vista é ortográfica: a fita mede a mesma largura em toda a
profundidade do quadro, o que simplifica a calibração e torna o erro
lateral uma medida direta em milímetros.

A **câmera superior** enxerga de 20,5 a 54,5 cm à frente. Ela **não** é
referência de direção — é sensor de aviso: nas corridas instrumentadas
antecipou 17 de 23 quebras, com 1,5 a 3,4 s de folga.

---

## Modos de operação

Escolhidos no **menu do OLED do ESP32**. Como a Pi sobe sozinha no boot,
essa é a única interface que existe sem um PC por perto.

| modo | quem dirige | percepção | câmeras |
|---|---|---|---|
| **Parado** | ninguém | desligada | publicando |
| **Seguidor** | seguidor de linha | ligada | publicando |
| **RC** | joystick (DualShock 4) | desligada | publicando |

As câmeras publicam nos três modos de propósito: permite acompanhar a
imagem por `rqt` de um PC externo mesmo dirigindo na mão.

> Escolher um modo **nunca arma a autonomia**. Modo diz quem tem
> autoridade; armar é um ato separado.

---

## Resultados medidos

Corrida de 3 minutos na pista, com telemetria completa
([dados](Softwares/ros2_ws/dados/)):

```
21,1 m percorridos          99,77% de quadros válidos
erro lateral mediano 8,4 mm       perdas: 0,41 s (0,23% do tempo)
19 quebras atravessadas, 2 travadas
```

### O limite físico das quebras

Uma quebra de ângulo Δ exige um desvio lateral de `R·(1/cos(Δ/2) − 1)`,
onde `R` é o raio mínimo de arco. Somando o erro de rastreio típico e
comparando com a margem lateral da câmera, sai um **limite de 57,8°**.

Validado em duas corridas independentes: **toda** quebra acima desse
ângulo travou, e nenhuma abaixo dele travou na primeira corrida.
Levantar a câmera inferior de 87 para 110 mm elevaria o limite para
70,1° — é margem lateral, não potência, que limita a curva.

---

## Rodando

```bash
cd Softwares/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup line_follower.launch.py serial:=true
```

> ⚠️ **Este robô anda de verdade.** Antes de armar a autonomia, garanta
> espaço livre e saiba onde está o STOP.

O [guia completo](Softwares/ros2_ws/README.md) tem calibração geométrica,
tabela de diagnóstico sintoma → causa → ajuste, as armadilhas já
descobertas nesta bancada e o histórico de correções com os números que
as motivaram.
