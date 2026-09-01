# Comandos – Robô Seguidor de Linha (ROS 2 Jazzy)

Este arquivo reúne os comandos principais para operar e debugar o robô no Raspberry Pi com Ubuntu 24.04 e ROS 2 Jazzy.

---

## 1. Configurar ambiente ROS

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 2. Câmeras

### 2.1. Câmera CSI inferior

A câmera CSI usa o `camera_ros` baseado em libcamera. Para listar as câmeras libcamera disponíveis:

```bash
rpicam-hello --list-cameras
```

Em algumas instalações, o comando equivalente é:

```bash
libcamera-hello --list-cameras
```

Para testar a CSI inferior:

```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p width:=640 \
  -p height:=480 \
  -r image_raw:=/camera_bottom/image_raw
```

> Os vários `/dev/video0` até `/dev/video7` exibidos por `v4l2-ctl` não representam necessariamente oito câmeras. No Raspberry Pi 5, eles são nós internos do pipeline CSI/PiSP. Para selecionar uma câmera CSI, use o índice listado por `rpicam-hello --list-cameras`, e não o número de `/dev/videoN`.

### 2.2. Segunda câmera USB

Depois de conectar a webcam USB, identifique o dispositivo:

```bash
v4l2-ctl --list-devices
```

Também é possível verificar os nós disponíveis:

```bash
ls -l /dev/video*
```

A câmera USB aparecerá com um nome de fabricante e um ou mais dispositivos `/dev/videoN`. O parâmetro `camera` do nó precisa ser testado de acordo com o fork instalado. Primeiro tente o índice usado pelo nó:

```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p width:=640 \
  -p height:=480 \
  -r image_raw:=/camera_top/image_raw
```

Se o fork aceitar caminho de dispositivo, prefira o caminho explícito:

```bash
ros2 run camera_ros camera_node --ros-args \
  -p device:=/dev/videoN \
  -p width:=640 \
  -p height:=480 \
  -r image_raw:=/camera_top/image_raw
```

Substitua `/dev/videoN` pelo dispositivo real da webcam USB.

### 2.3. Confirmar os tópicos das câmeras

```bash
ros2 topic list | grep -E 'image|camera'
ros2 topic hz /camera_bottom/image_raw
ros2 topic hz /camera_top/image_raw
```

### 2.4. Visualizar imagens

```bash
ros2 run rqt_image_view rqt_image_view
```

No `rqt_image_view`, selecione:

```text
/camera_bottom/image_raw
/camera_top/image_raw
/line/debug_image
/line/mask
```

---

## 3. Visão computacional da linha

### 3.1. Executar separadamente

```bash
ros2 run robot_bringup visao_linha_node
```

O nó de visão deve assinar:

```text
/camera_bottom/image_raw
```

E publicar:

```text
/line/centroid
/line/error
/line/status
/line/debug_image
/line/mask
```

### 3.2. Visualizar a imagem processada

```bash
ros2 run rqt_image_view rqt_image_view
```

Selecione:

```text
/line/debug_image
```

### 3.3. Monitorar a visão

```bash
ros2 topic echo /line/error
ros2 topic echo /line/centroid
ros2 topic echo /line/status
ros2 topic hz /line/debug_image
```

---

## 4. Controle PID

### 4.1. Executar separadamente

```bash
ros2 run robot_bringup controle_node
```

O controle inicia desabilitado para impedir que o robô comece a andar automaticamente.

### 4.2. Habilitar e desabilitar

```bash
# Habilitar
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: true}"

# Desabilitar
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: false}"
```

### 4.3. Ajustar parâmetros

```bash
ros2 param list /controle_node
ros2 param set /controle_node Kp 0.5
ros2 param set /controle_node Ki 0.0
ros2 param set /controle_node Kd 0.1
ros2 param set /controle_node v_linear 0.2
ros2 param set /controle_node v_angular_max 0.4
ros2 param set /controle_node error_max 200.0
```

### 4.4. Monitorar `/cmd_vel`

```bash
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

---

## 5. Motores, servo e comunicação serial

### 5.1. Descobrir a porta do ESP32

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Exemplo de execução do nó serial:

```bash
ros2 run robot_bringup motor_serial_node --ros-args \
  -p port:=/dev/ttyACM0
```

Se o executável ou pacote tiver outro nome, confira com:

```bash
ros2 pkg executables robot_bringup
```

O nó serial assina `/cmd_vel` e `/servo/command`, e publica dados dos motores, encoders, IMU, bateria, servo e odometria.

### 5.2. Compilar após alterações

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

### 5.3. Monitorar a serial diretamente

```bash
~/.platformio/penv/bin/platformio device monitor \
  --port /dev/ttyACM0 \
  --baud 115200
```

---

## 6. Controle manual do servo

O comando do servo usa ângulo lógico de `-90` a `+90` graus:

```text
-90° = extremo de um lado
  0° = centro
+90° = extremo do outro lado
```

O ESP32 converte esse intervalo para o servo físico de `0` a `180` graus.

### 6.1. Colocar no centro

```bash
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: 0.0}"
```

### 6.2. Testar os extremos

```bash
# Um lado
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: -90.0}"

# Outro lado
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: 90.0}"
```

### 6.3. Testar ângulos intermediários

```bash
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: -45.0}"

ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: 45.0}"
```

### 6.4. Publicar continuamente

```bash
ros2 topic pub -r 10 /servo/command \
  std_msgs/msg/Float32 "{data: 0.0}"
```

Finalize com `Ctrl+C`.

### 6.5. Monitorar o retorno do servo

```bash
ros2 topic echo /servo/state
ros2 topic hz /servo/state
```

Se o sentido físico estiver invertido, altere a conversão no firmware antes de testar novamente:

```cpp
lroundf(bounded + 90.0f)
```

para:

```cpp
lroundf(90.0f - bounded)
```

---

## 7. Tópicos de sensores e estado

```bash
ros2 node list
ros2 topic list
```

### Encoders e rodas

```bash
ros2 topic echo /wheel_encoder_ticks
ros2 topic echo /wheel_states
```

### Odometria

```bash
ros2 topic info /odom
ros2 topic echo /odom
ros2 topic hz /odom
```

### IMU

```bash
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

### Bateria

```bash
ros2 topic echo /battery/voltage
ros2 topic hz /battery/voltage
```

A tensão é medida pelo divisor resistivo de `100 kΩ` e `33 kΩ` conectado ao ADC do ESP32.

---

## 8. Teleop e comandos manuais

### 8.1. Teleop com teclado

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 8.2. Publicar comando manual

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 8.3. Parar os motores

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Ou desabilite o controle:

```bash
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: false}"
```

---

## 9. Fluxo recomendado para teste

### 9.1. Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

### 9.2. Iniciar a câmera inferior

```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p width:=640 \
  -p height:=480 \
  -r image_raw:=/camera_bottom/image_raw
```

### 9.3. Iniciar a câmera superior USB

Use o dispositivo `/dev/videoN` identificado com `v4l2-ctl`:

```bash
ros2 run camera_ros camera_node --ros-args \
  -p device:=/dev/videoN \
  -p width:=640 \
  -p height:=480 \
  -r image_raw:=/camera_top/image_raw
```

### 9.4. Iniciar visão, controle e serial

```bash
ros2 run robot_bringup visao_linha_node
ros2 run robot_bringup controle_node
ros2 run robot_bringup motor_serial_node --ros-args \
  -p port:=/dev/ttyACM0
```

### 9.5. Conferir antes de liberar as rodas

```bash
ros2 node list
ros2 topic list
ros2 topic hz /camera_bottom/image_raw
ros2 topic hz /camera_top/image_raw
ros2 topic hz /line/debug_image
ros2 topic echo /battery/voltage
ros2 topic echo /servo/state
```

Mantenha as rodas suspensas durante o primeiro teste. Só depois habilite:

```bash
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: true}"
```

---

## 10. Launch completo

Depois de criar e compilar o pacote `robot_bringup`:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
ros2 launch robot_bringup bringup_dual_camera.launch.py
```

O launch deve publicar, idealmente:

```text
/camera_bottom/image_raw
/camera_top/image_raw
```

Confira com:

```bash
ros2 topic list | grep -E 'image|camera'
ros2 node list
```

---

## 11. Diagnóstico

### Ver nós e conexões

```bash
ros2 node list
ros2 node info /visao_linha_node
ros2 node info /controle_node
ros2 node info /motor_serial_omni
rqt_graph
```

### Ver parâmetros do nó serial

```bash
ros2 param list /motor_serial_omni
ros2 param get /motor_serial_omni port
ros2 param get /motor_serial_omni servo_command_topic
ros2 param get /motor_serial_omni battery_topic
```

### Ver informações dos tópicos

```bash
ros2 topic info /camera_bottom/image_raw --verbose
ros2 topic info /camera_top/image_raw --verbose
ros2 topic info /line/error --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /servo/command --verbose
ros2 topic info /battery/voltage --verbose
```

### Verificar a visão

```bash
ros2 topic hz /camera_bottom/image_raw
ros2 topic hz /line/debug_image
ros2 topic echo /line/status
```

Se a câmera tiver frequência, mas `/line/debug_image` não tiver, confira se o parâmetro de imagem do nó de visão está apontando para:

```text
/camera_bottom/image_raw
```

### Verificar câmeras V4L2 USB

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/videoN
```

Substitua `N` pelo número da webcam USB. Os vários `/dev/videoN` da CSI não devem ser tratados automaticamente como câmeras físicas separadas.

---

## 12. Acesso remoto

```bash
ssh bolt@192.168.0.XXX
```

Para encaminhamento gráfico:

```bash
ssh -Y bolt@192.168.0.XXX
```

Depois:

```bash
echo $DISPLAY
ros2 run rqt_image_view rqt_image_view
```

---

## 13. Resumo rápido

```bash
# Ambiente
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Câmeras
rpicam-hello --list-cameras
v4l2-ctl --list-devices

# Iniciar câmera CSI inferior
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 -p width:=640 -p height:=480 \
  -r image_raw:=/camera_bottom/image_raw

# Iniciar câmera USB superior
ros2 run camera_ros camera_node --ros-args \
  -p device:=/dev/videoN -p width:=640 -p height:=480 \
  -r image_raw:=/camera_top/image_raw

# Serial
ros2 run robot_bringup motor_serial_node --ros-args \
  -p port:=/dev/ttyACM0

# Servo no centro
ros2 topic pub --once /servo/command \
  std_msgs/msg/Float32 "{data: 0.0}"

# Ler servo e bateria
ros2 topic echo /servo/state
ros2 topic echo /battery/voltage

# Habilitar robô
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: true}"

# Parar robô
ros2 topic pub --once /controle/enable std_msgs/msg/Bool "{data: false}"
```
