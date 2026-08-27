# Comandos – Robô···Seguidor de Linha (ROS 2 Jazzy)

Este arquivo reúne os comandos mais importantes para operar e debugar o sistema no Raspberry Pi 4, incluindo:

- Ambiente ROS  
- Câmeras  
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

### 2.1. Listar dispositivos de vídeo

```bash
v4l2-ctl --list-devices
```

### 2.2. Rodar câmera (ex.: câmera inferior)

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[640, 480]" \
  -p output_encoding:=bgr8
```

### 2.3. Listar tó···picos de imagem

```bash
ros2 topic list | grep image
```

### 2.4. Visualizar imagem

```bash
ros2 run rqt_image_view rqt_image_view
```

Selecione o tópico da câmera, por exemplo:

- `/camera/bottom/image_raw`  
- `/camera/top/image_raw`  

(ajuste conforme o namespace usado no seu sistema)

---

## 3. Motores / Serial / Encoders

### 3.1. Construir pacote da serial (apenas quando modificar o código)

```bash
cd ~/ros2_ws
colcon build --packages-select motor_serial --symlink-install
```

### 3.2. Rodar nó da serial (motores + encoders)

```bash
ros2 run motor_serial motor_serial_node --ros-args \
  -p port:=/dev/ttyACM0
```

> Ajuste `port` caso seu Arduino aparecê·· em outra porta (ex.: `/dev/ttyACM1`, `/dev/ttyUSB0`).

### 3.3. Monitorar comunicação com Arduino (PlatformIO)

```bash
~/.platformio/penv/bin/platformio device monitor \
  --port /dev/ttyACM0 \
  --baud 115200
```

---

## 4. Tópicos importantes do sistema

### 4.1. Listar tó···picos e nodes

```bash
ros2 topic list
ros2 node list
```

### 4.2. Odometria / encoders

```bash
# Estados das rodas (velocidade, posição, etc.)
ros2 topic echo /wheel_states

# Contagem de ticks dos encoders
ros2 topic echo /wheel_encoder_ticks

# Odometria estimada
ros2 topic info /odom
ros2 topic echo /odom
```

### 4.3. IMU

```bash
ros2 topic echo /imu/data_raw
```

---

## 5. Controle / Teleop

### 5.1. Teleop com teclado

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5.2. Publicar comando de velocidade manualmente

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.00, y: 0.0}, angular: {z: 0.0}}"
```

Ajuste `linear.x` e `angular.z` para mover o robô··· (ex.: `x: 0.2`, `z: 0.3`).

---

## 6. Debug e diagnóstico

### 6.1. Ver nodes ativos

```bash
ros2 node list
```

### 6.2. Ver tópicos

```bash
ros2 topic list
```

### 6.3. Informações de um tópico

```bash
ros2 topic info /odom
ros2 topic info /wheel_states
ros2 topic info /cmd_vel
```

### 6.4. Echo de tópicos específicos

```bash
ros2 topic echo /wheel_encoder_ticks
ros2 topic echo /wheel_states
ros2 topic echo /imu/data_raw
ros2 topic echo /odom
```

---

## 7. Acesso remoto ao Raspberry

### 7.1. SSH simples

```bash
ssh bolt@192.168.0.XXX
```

Substitua `192.168.0.XXX` pelo IP real do Raspberry.

### 7.2. SSH com encaminhamento gráfico (para abrir janelas no PC)

```bash
ssh -Y bolt@192.168.0.XXX
```

Depois, no Raspberry:

```bash
echo $DISPLAY          # deve mostrar algo como localhost:10.0
ros2 run rqt_image_view rqt_image_view
```

---

## 8. Resumo rápido – comandos “chave”

- **Ambiente:**

  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ```

- **Câmera:**

  ```bash
  v4l2-ctl --list-devices
  ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0 \
    -p image_size:="[640, 480]" \
    -p output_encoding:=bgr8
  ros2 run rqt_image_view rqt_image_view
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

- **Diagnóstico:**

  ```bash
  ros2 node list
  ros2 topic list
  ros2 topic info /odom
  ```

- **Acesso remoto:**

  ```bash
  ssh -Y bolt@192.168.0.XXX
  ```

---

Use este arquivo como referência principal para operar e debugar o robô··· no dia a dia.
