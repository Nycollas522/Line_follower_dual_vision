# Comandos – Robô···Seguidor de Linha (ROS 2 Jazzy)

Este arquivo reúne os comandos mais importantes para operar e debugar o sistema no Raspberry Pi, incluindo:

- Ambiente ROS  
- Câmeras (CSI e USB)  
- Visã··o computacional (linha, centroide, erro)  
- Controle (PID)  
- Motores / serial  
- Odometria / encoders  
- IMU  
- Teleop  
- Comandos úteis de diagnóstico  

---

## 1. Configurar ambiente ROS

### 1.1. Carregar ROS e workspace

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Workspace do projeto (ajuste o caminho se necessário)
source ~/ros2_ws/install/setup.bash
```

> Dica: coloque esses dois `source` no `~/.bashrc` para não precisar digitar sempre.

---

## 2. Câmeras

### 2.1. Câmera CSI (OV5647 / OV9281)

#### 2.1.1. Listar câmeras detectadas

```bash
rpicam-hello --list-cameras
```

#### 2.1.2. Rodar nó da câmera CSI

```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p width:=640 \
  -p height:=480
```

Se houver mais de uma câmera, ajuste o índice:

```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:=1 \
  -p width:=640 \
  -p height:=480
```

### 2.2. Câmera USB (webcam)

#### 2.2.1. Listar dispositivos de vídeo

```bash
v4l2-ctl --list-devices
```

#### 2.2.2. Rodar câmera USB

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/videoX \
  -p image_size:="[640, 480]" \
  -p output_encoding:=bgr8
```

Substitua `/dev/videoX` pelo dispositivo correto (ex.: `/dev/video0`).

### 2.3. Listar tó···picos de imagem

```bash
ros2 topic list | grep -E 'image|camera'
```

Tó···picos típicos:

- `/camera/image_raw` (CSI com `camera_ros`)  
- `/image_raw` (USB com `v4l2_camera`)  

### 2.4. Visualizar imagem

```bash
ros2 run rqt_image_view rqt_image_view
```

Selecione o tópico da câmera, por exemplo:

- `/camera/image_raw`  
- `/camera/image_raw/compressed`  
- `/line/debug_image` (saí··da do nó de visão com centroide desenhado)  

---

## 3. Visã··o computacional (linha e centroide)

### 3.1. Rodar nó de visã··o da linha

```bash
ros2 run line_vision visao_linha_node
```

Este nó:

- Assina `/camera/image_raw` (ou `/image_raw`, conforme configurado);  
- Publica:
  - `/line/centroid` (geometry_msgs/msg/Point)  
  - `/line/error` (std_msgs/msg/Float32)  
  - `/line/status` (std_msgs/msg/Bool)  
  - `/line/debug_image` (sensor_msgs/msg/Image)  

### 3.2. Visualizar imagem de debug

```bash
ros2 run rqt_image_view rqt_image_view
```

Selecione:

```text
/line/debug_image
```

Você·· verá a imagem com:

- Cí··rculo vermelho no centroide da linha;  
- Linha azul de referência no centro da imagem;  
- Texto com o valor do erro lateral.  

### 3.3. Monitorar erro e centroide

```bash
ros2 topic echo /line/error
ros2 topic echo /line/centroid
ros2 topic echo /line/status
```

---

## 4. Controle (PID)

### 4.1. Rodar nó de controle

```bash
ros2 run line_control controle_node
```

Este nó:

- Assina `/line/error`;  
- Publica `/cmd_vel` (geometry_msgs/msg/Twist).  

### 4.2. Ajustar parâ··metros do PID

Listar parâ··metros:

```bash
ros2 param list | grep controle_node
```

Ajustar ganhos e velocidades:

```bash
ros2 param set /controle_node Kp 0.5
ros2 param set /controle_node Ki 0.0
ros2 param set /controle_node Kd 0.1

ros2 param set /controle_node v_linear 0.2
ros2 param set /controle_node v_angular_max 0.4
ros2 param set /controle_node error_max 200.0
```

### 4.3. Monitorar `/cmd_vel`

```bash
ros2 topic echo /cmd_vel
```

---

## 5. Motores / Serial / Encoders

### 5.1. Construir pacote da serial (apenas quando modificar o código)

```bash
cd ~/ros2_ws
colcon build --packages-select motor_serial --symlink-install
```

### 5.2. Rodar nó da serial (motores + encoders)

```bash
ros2 run motor_serial motor_serial_node --ros-args \
  -p port:=/dev/ttyACM0
```

> Ajuste `port` caso seu Arduino apareç··a em outra porta (ex.: `/dev/ttyACM1`, `/dev/ttyUSB0`).

### 5.3. Monitorar comunicação com Arduino (PlatformIO)

```bash
~/.platformio/penv/bin/platformio device monitor \
  --port /dev/ttyACM0 \
  --baud 115200
```

---

## 6. Tó···picos importantes do sistema

### 6.1. Listar nó···s e tó···picos

```bash
ros2 node list
ros2 topic list
```

### 6.2. Odometria / encoders

```bash
# Estados das rodas (velocidade, posição, etc.)
ros2 topic echo /wheel_states

# Contagem de ticks dos encoders
ros2 topic echo /wheel_encoder_ticks

# Odometria estimada
ros2 topic info /odom
ros2 topic echo /odom
```

### 6.3. IMU

```bash
ros2 topic echo /imu/data_raw
```

---

## 7. Controle / Teleop

### 7.1. Teleop com teclado

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 7.2. Publicar comando de velocidade manualmente

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.00, y: 0.0}, angular: {z: 0.0}}"
```

Ajuste `linear.x` e `angular.z` para mover o robô··· (ex.: `x: 0.2`, `z: 0.3`).

---

## 8. Fluxo completo – seguimento de linha

Para fazer o robô··· seguir a linha, rode na ordem:

```bash
# Terminal 1 – câmera
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p width:=640 \
  -p height:=480

# Terminal 2 – visão
ros2 run line_vision visao_linha_node

# Terminal 3 – controle
ros2 run line_control controle_node

# Terminal 4 – motores
ros2 run motor_serial motor_serial_node --ros-args \
  -p port:=/dev/ttyACM0
```

Monitore:

```bash
ros2 topic echo /line/error
ros2 topic echo /cmd_vel
```

---

## 9. Debug e diagnóstico

### 9.1. Ver nó···s ativos

```bash
ros2 node list
ros2 node info /visao_linha_node
ros2 node info /controle_node
ros2 node info /motor_serial_node
```

### 9.2. Ver tó···picos

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /line/error
ros2 topic hz /cmd_vel
```

### 9.3. Informações de um tó···pico

```bash
ros2 topic info /camera/image_raw --verbose
ros2 topic info /line/error --verbose
ros2 topic info /cmd_vel --verbose
```

### 9.4. Echo de tó···picos específicos

```bash
ros2 topic echo /wheel_encoder_ticks
ros2 topic echo /wheel_states
ros2 topic echo /imu/data_raw
ros2 topic echo /odom
ros2 topic echo /line/error
ros2 topic echo /cmd_vel
```

---

## 10. Acesso remoto ao Raspberry

### 10.1. SSH simples

```bash
ssh bolt@192.168.0.XXX
```

Substitua `192.168.0.XXX` pelo IP real do Raspberry.

### 10.2. SSH com encaminhamento gráfico (para abrir janelas no PC)

```bash
ssh -Y bolt@192.168.0.XXX
```

Depois, no Raspberry:

```bash
echo $DISPLAY          # deve mostrar algo como localhost:10.0
ros2 run rqt_image_view rqt_image_view
```

---

## 11. Resumo rápido – comandos "chave"

- **Ambiente:**

  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ```

- **Câ··mera CSI:**

  ```bash
  rpicam-hello --list-cameras
  ros2 run camera_ros camera_node --ros-args \
    -p camera:=0 \
    -p width:=640 \
    -p height:=480
  ```

- **Câ··mera USB:**

  ```bash
  v4l2-ctl --list-devices
  ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0 \
    -p image_size:="[640, 480]" \
    -p output_encoding:=bgr8
  ```

- **Visualizar imagem:**

  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```

- **Visã··o da linha:**

  ```bash
  ros2 run line_vision visao_linha_node
  ros2 topic echo /line/error
  ros2 topic echo /line/centroid
  ```

- **Controle (PID):**

  ```bash
  ros2 run line_control controle_node
  ros2 param set /controle_node Kp 0.5
  ros2 param set /controle_node Kd 0.1
  ros2 topic echo /cmd_vel
  ```

- **Motores / serial:**

  ```bash
  colcon build --packages-select motor_serial --symlink-install
  ros2 run motor_serial motor_serial_node --ros-args \
    -p port:=/dev/ttyACM0
  ```

- **Odometria / encoders / IMU:**

  ```bash
  ros2 topic echo /wheel_encoder_ticks
  ros2 topic echo /wheel_states
  ros2 topic echo /odom
  ros2 topic echo /imu/data_raw
  ```

- **Teleop:**

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"
  ```

- **Diagnó···stico:**

  ```bash
  ros2 node list
  ros2 topic list
  ros2 topic info /cmd_vel --verbose
  ```

- **Acesso remoto:**

  ```bash
  ssh -Y bolt@192.168.0.XXX
  ```

---

Use este arquivo como referência principal para operar e debugar o robô··· no dia a dia.
