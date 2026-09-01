# Robô Seguidor de Linha — ROS 2 Jazzy

Guia de operação e diagnóstico do robô baseado em Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy e ESP32.

O sistema é composto por:

- Duas câmeras CSI usando `camera_ros`.
- Visão computacional para detecção da linha.
- Controle PID habilitado manualmente por segurança.
- Comunicação serial entre Raspberry Pi e ESP32.
- Controle de motores, servo, encoders, IMU, bateria e odometria.
- Display OLED conectado ao ESP32.

---

## 1. Preparar o ambiente

Execute em cada novo terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Para carregar automaticamente:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

O workspace utilizado é:

```text
~/ros2_ws
```

O pacote principal deste projeto é:

```text
robot_bringup
```

---

## 2. Compilar o projeto

Use `--symlink-install` para que alterações nos arquivos Python sejam refletidas mais facilmente:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

Se o launch não for encontrado ou a instalação estiver desatualizada, faça uma compilação limpa:

```bash
cd ~/ros2_ws
rm -rf build/robot_bringup install/robot_bringup log/robot_bringup
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

Verifique os executáveis instalados:

```bash
ros2 pkg executables robot_bringup
```

O resultado deve incluir:

```text
robot_bringup visao_linha_node
robot_bringup controle_node
robot_bringup motor_serial_node
```

---

## 3. Câmeras CSI

### 3.1. Listar câmeras físicas

```bash
rpicam-hello --list-cameras
```

Em algumas instalações, o comando equivalente é:

```bash
libcamera-hello --list-cameras
```

A opção `camera:=0` seleciona a primeira câmera detectada e `camera:=1` seleciona a segunda.

### 3.2. Rodar a câmera inferior manualmente

```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p width:=640 \
  -p height:=480 \
  -r image_raw:=/camera_bottom/image_raw
```

### 3.3. Rodar a segunda câmera manualmente

```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:=1 \
  -p width:=640 \
  -p height:=480 \
  -r image_raw:=/camera_front/image_raw
```

### 3.4. Verificar as imagens

```bash
ros2 topic list | grep -E 'image|camera'
ros2 topic hz /camera_bottom/image_raw
ros2 topic hz /camera_front/image_raw
```

Para abrir o visualizador:

```bash
ros2 run rqt_image_view rqt_image_view
```

Tópicos disponíveis:

```text
/camera_bottom/image_raw
/camera_front/image_raw
/line/debug_image
/line/mask
```

> Os vários dispositivos `/dev/videoN` que aparecem no Raspberry Pi 5 não representam necessariamente câmeras físicas independentes. Para selecionar câmeras CSI, prefira os índices mostrados por `rpicam-hello --list-cameras`.

---

## 4. Visão computacional da linha

### 4.1. Executar separadamente

```bash
ros2 run robot_bringup visao_linha_node
```

O nó deve receber a imagem da câmera inferior:

```text
/camera_bottom/image_raw
```

O parâmetro pode ser conferido com:

```bash
ros2 param get /visao_linha_node image_topic
```

O resultado esperado é:

```text
String value is: /camera_bottom/image_raw
```

### 4.2. Tópicos publicados

```text
/line/centroid
/line/error
/line/status
/line/debug_image
/line/mask
```

### 4.3. Monitorar a visão

```bash
ros2 topic hz /camera_bottom/image_raw
ros2 topic hz /line/debug_image
ros2 topic echo /line/error
ros2 topic echo /line/centroid
ros2 topic echo /line/status
```

No `rqt_image_view`, selecione:

```text
/line/debug_image
```

A imagem de debug apresenta a região de interesse, a linha de referência, o centroide detectado, o erro lateral e a área do contorno.

---

## 5. Controle PID

### 5.1. Executar separadamente

```bash
ros2 run robot_bringup controle_node
```

O controle inicia desabilitado. Portanto, executar o nó ou o launch não deve fazer o robô andar imediatamente.

O nó:

- Assina `/line/error`.
- Publica `/cmd_vel`.
- Publica comandos zerados enquanto estiver desabilitado.

### 5.2. Habilitar o robô

Antes de habilitar, mantenha as rodas suspensas durante o primeiro teste:

```bash
ros2 topic pub --once /controle/enable \
  std_msgs/msg/Bool "{data: true}"
```

### 5.3. Desabilitar o robô

```bash
ros2 topic pub --once /controle/enable \
  std_msgs/msg/Bool "{data: false}"
```

Ao desabilitar, o nó zera o comando `/cmd_vel` e também reinicia o estado integral do PID.

### 5.4. Ajustar parâmetros

```bash
ros2 param list /controle_node

ros2 param set /controle_node Kp 0.5
ros2 param set /controle_node Ki 0.0
ros2 param set /controle_node Kd 0.1
ros2 param set /controle_node v_linear 0.2
ros2 param set /controle_node v_angular_max 0.4
ros2 param set /controle_node error_max 200.0
```

Monitorar o comando:

```bash
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

---

## 6. Motores e comunicação serial

### 6.1. Identificar a porta do ESP32

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Exemplos possíveis:

```text
/dev/ttyUSB0
/dev/ttyACM0
```

### 6.2. Executar o nó serial

```bash
ros2 run robot_bringup motor_serial_node --ros-args \
  -p port:=/dev/ttyUSB0
```

Troque `/dev/ttyUSB0` pela porta real do ESP32.

O nó serial:

- Assina `/cmd_vel` para controlar os motores.
- Assina `/servo/command` para controlar o servo.
- Publica `/servo/state`.
- Publica `/wheel_encoder_ticks`.
- Publica `/wheel_states`.
- Publica `/imu/data_raw`.
- Publica `/odom`.
- Publica `/battery/voltage`.
- Publica transformações TF entre `odom` e `base_link` quando habilitado.

### 6.3. Monitorar a comunicação serial

```bash
~/.platformio/penv/bin/platformio device monitor \
  --port /dev/ttyUSB0 \
  --baud 115200
```

Mensagens importantes do ESP32 incluem:

```text
READY
IMU,READY
IMU,NOT_FOUND
OLED,NOT_FOUND
ERR,...
```

### 6.4. Compilar depois de modificar o código

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

---

## 7. Controle do servo

O nó ROS utiliza um ângulo lógico entre `-90` e `+90` graus:

```text
-90° = extremo de um lado
  0° = centro
+90° = extremo do outro lado
```

O ESP32 converte esse valor para a faixa física aproximada de `0` a `180` graus.

### 7.1. Centralizar o servo

```bash
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: 0.0}"
```

### 7.2. Testar os extremos

```bash
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: -90.0}"
```

```bash
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: 90.0}"
```

### 7.3. Testar posições intermediárias

```bash
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: -45.0}"
```

```bash
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: 45.0}"
```

### 7.4. Monitorar o servo

```bash
ros2 topic echo /servo/state
ros2 topic hz /servo/state
```

Se o servo se mover no sentido contrário ao desejado, a inversão deve ser feita no firmware do ESP32, na conversão do ângulo lógico para o ângulo físico.

---

## 8. Display OLED

O display OLED é controlado pelo firmware do ESP32. O nó serial apenas informa mensagens recebidas do microcontrolador.

Para verificar se o OLED foi encontrado, observe o terminal do nó serial ou o monitor da PlatformIO:

```text
OLED,NOT_FOUND
```

Se essa mensagem aparecer, verifique:

- Alimentação do display.
- GND comum entre OLED e ESP32.
- Linhas SDA e SCL.
- Endereço I2C configurado no firmware, normalmente `0x3C` ou `0x3D`.
- Biblioteca e inicialização do display no código do ESP32.

Para procurar dispositivos I2C conectados:

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
```

O resultado deve mostrar o endereço do display na tabela, por exemplo `3c`.

O nó serial trata mensagens OLED desta forma:

```text
OLED,NOT_FOUND
```

Essa mensagem gera um aviso no ROS, mas não impede necessariamente o funcionamento dos motores e sensores.

---

## 9. Sensores e estado do robô

### 9.1. Encoders

```bash
ros2 topic echo /wheel_encoder_ticks
ros2 topic hz /wheel_encoder_ticks
```

### 9.2. Estado das rodas

```bash
ros2 topic echo /wheel_states
ros2 topic hz /wheel_states
```

### 9.3. IMU

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

### 9.4. Bateria

```bash
ros2 topic echo /battery/voltage
ros2 topic hz /battery/voltage
```

### 9.5. Odometria

```bash
ros2 topic echo /odom
ros2 topic hz /odom
ros2 topic info /odom
```

### 9.6. TF

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 10. Launch completo

O launch inicia:

- Câmera inferior.
- Segunda câmera CSI.
- Visão da linha.
- Controle PID desabilitado.
- Nó serial dos motores.

Para iniciar tudo:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_bringup bringup_dual_camera.launch.py
```

Depois de iniciar, confirme:

```bash
ros2 node list
ros2 topic list
rqt_graph
```

Verifique as câmeras:

```bash
ros2 topic hz /camera_bottom/image_raw
ros2 topic hz /camera_front/image_raw
```

Verifique a visão:

```bash
ros2 topic hz /line/debug_image
ros2 topic echo /line/status
```

Verifique os sensores:

```bash
ros2 topic echo /battery/voltage
ros2 topic echo /servo/state
```

O controle continua parado até receber:

```bash
ros2 topic pub --once /controle/enable \
  std_msgs/msg/Bool "{data: true}"
```

Para parar:

```bash
ros2 topic pub --once /controle/enable \
  std_msgs/msg/Bool "{data: false}"
```

---

## 11. Teleop e comandos manuais

### 11.1. Teleop com teclado

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> Atenção: o teleop publica diretamente em `/cmd_vel`. Use as rodas suspensas e tenha cuidado, pois ele pode comandar o robô independentemente do controle PID.

### 11.2. Publicar comando manual

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 11.3. Parar manualmente

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Sempre que possível, desabilite também o controle:

```bash
ros2 topic pub --once /controle/enable \
  std_msgs/msg/Bool "{data: false}"
```

---

## 12. Diagnóstico

### 12.1. Nós ativos

```bash
ros2 node list
ros2 node info /visao_linha_node
ros2 node info /controle_node
ros2 node info /motor_serial_omni
```

### 12.2. Tópicos ativos

```bash
ros2 topic list
```

### 12.3. Informações detalhadas dos tópicos

```bash
ros2 topic info /camera_bottom/image_raw --verbose
ros2 topic info /camera_front/image_raw --verbose
ros2 topic info /line/error --verbose
ros2 topic info /line/debug_image --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /servo/command --verbose
ros2 topic info /servo/state --verbose
ros2 topic info /battery/voltage --verbose
```

### 12.4. Frequência dos tópicos

```bash
ros2 topic hz /camera_bottom/image_raw
ros2 topic hz /camera_front/image_raw
ros2 topic hz /line/debug_image
ros2 topic hz /line/error
ros2 topic hz /cmd_vel
ros2 topic hz /odom
```

### 12.5. Grafo ROS

```bash
rqt_graph
```

Se `/line/debug_image` não receber mensagens, confirme:

```bash
ros2 topic hz /camera_bottom/image_raw
ros2 param get /visao_linha_node image_topic
```

O parâmetro deve apontar para:

```text
/camera_bottom/image_raw
```

Se o nó da câmera estiver publicando com outro nome, ajuste o parâmetro `image_topic` no launch ou no arquivo da visão.

---

## 13. Acesso remoto

SSH simples:

```bash
ssh bolt@192.168.0.XXX
```

Com encaminhamento gráfico:

```bash
ssh -Y bolt@192.168.0.XXX
```

Depois, confira o display:

```bash
echo $DISPLAY
```

E abra o visualizador:

```bash
ros2 run rqt_image_view rqt_image_view
```

---

## 14. Fluxo rápido recomendado

```bash
# 1. Preparar o ambiente
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. Compilar
cd ~/ros2_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash

# 3. Iniciar todos os nós
ros2 launch robot_bringup bringup_dual_camera.launch.py

# 4. Em outro terminal, abrir imagens
ros2 run rqt_image_view rqt_image_view

# 5. Conferir visão
ros2 topic hz /line/debug_image
ros2 topic echo /line/status

# 6. Conferir sensores
ros2 topic echo /battery/voltage
ros2 topic echo /servo/state

# 7. Somente depois liberar o movimento
ros2 topic pub --once /controle/enable \
  std_msgs/msg/Bool "{data: true}"

# 8. Parar o robô
ros2 topic pub --once /controle/enable \
  std_msgs/msg/Bool "{data: false}"
```

Use este arquivo como referência principal para iniciar, testar e diagnosticar o robô.
