# Código antigo

Versões anteriores do software, guardadas como registro da evolução do
projeto. **Nada aqui é usado pelo robô atual** — servem para consulta
histórica.

| pasta | o que era | substituído por |
|---|---|---|
| `IC_seguidor_S3/` | primeiro firmware do ESP32-S3 | `Softwares/Firmware_Ippo/` |
| `Firmware_Ippo.zip` | firmware empacotado em zip | `Softwares/Firmware_Ippo/` (agora versionado como fonte) |
| `line_control/` | nó de controle da primeira arquitetura | `robot_bringup/line_controller.py` + `line_follower_node.py` |
| `line_vision/` | nó de visão da primeira arquitetura | `robot_bringup/line_geometry.py` + `line_perception_node.py` |

## O que mudou da arquitetura antiga para a atual

A primeira versão tinha um nó de visão e um de controle conversando por
tópico, com a lógica misturada ao ROS. A atual separa a **geometria pura**
(`line_geometry.py`, `line_controller.py` — sem ROS, testáveis com
`pytest`) dos **nós** que só fazem a ponte com o ROS. São 160 testes
automatizados em cima dessa parte pura.

Do lado do ESP32, o firmware passou de um laço único para módulos
(`MotorDriver`, `EncoderManager`, `ImuManager`, `LocalMenu`,
`SerialProtocol`, `Settings`), com telemetria em dois grupos de taxa,
CRC nos comandos e menu de modo no OLED.
