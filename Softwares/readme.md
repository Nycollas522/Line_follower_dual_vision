# Softwares

Todo o software do robô: firmware do ESP32-S3 e a pilha ROS 2 da
Raspberry Pi.

| pasta | o que é |
|---|---|
| **[`Firmware_Ippo/`](Firmware_Ippo/)** | Firmware do ESP32-S3 (PlatformIO/Arduino). Controle das rodas a 100 Hz, PID por roda, odometria com IMU, menu no OLED, protocolo serial. |
| **[`ros2_ws/`](ros2_ws/)** | Pilha ROS 2 Jazzy. Percepção das duas câmeras, controlador do seguidor, ponte serial, servo da cabeça, modo RC por joystick. |
| **[`ros2_ws/README.md`](ros2_ws/README.md)** | **Guia completo de operação e depuração.** Comece por aqui. |
| [`ros2_ws/dados/`](ros2_ws/dados/) | CSVs de corrida que sustentam os números citados na documentação. |
| [`Codigos_antigos/`](Codigos_antigos/) | Versões anteriores, só como registro. Nada aqui roda no robô atual. |

> ⚠️ **Este robô anda de verdade.** Antes de habilitar a autonomia,
> garanta espaço livre e saiba onde está o STOP. Veja a seção
> "Parar tudo, agora" no guia.

## Em um parágrafo

Chassi mecanum 4x4. A **câmera inferior** aponta reto para baixo e é a
percepção primária do laço rápido; a **superior** fica num servo e serve
de aviso antecipado de curva, não de referência de direção. A Raspberry
Pi roda a visão e o controle; o **ESP32-S3** fecha o laço de velocidade
de cada roda a 100 Hz e é quem corta os motores se a comunicação cair.

O modo de operação (**parado / seguidor / RC**) é escolhido no **menu do
OLED do ESP32** — com o Pi subindo sozinho no boot, essa é a única
interface que existe sem um PC por perto.

## Estrutura mínima para rodar

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup line_follower.launch.py serial:=true
```

O guia em [`ros2_ws/README.md`](ros2_ws/README.md) tem o resto: calibração
geométrica, tabela de diagnóstico sintoma → causa → ajuste, armadilhas
conhecidas desta bancada e o histórico de correções com os números que
as motivaram.
