# Comandos Gerais – Robô···Seguidor de Linha (ROS 2 Jazzy)

Este arquivo reúne os comandos principais para operar toda a arquitetura do projeto no Raspberry Pi 4, incluindo:

- Câmeras (inferior e superior)  d
- Visã··o da linha e marcadores  
- Filtro de Kalman  
- Controle (PID) e cinemá··tica  
- Odometria (encoders)  
- Atuaç··o (motores via Arduino)  
- Node principal / launch do sistema  

> Ajuste os nomes de pacotes e nodes conforme a estrutura real do seu workspace (`ros2 pkg list`, `ros2 node list`).

---

## 1. Configuraç··o básica do ambiente

Carregar ROS 2 Jazzy (se ainda nã··o estiver no `.bashrc`):

```bash
source /opt/ros/jazzy/setup.bash
```

Colocar ROS no `.bashrc` (opcional, para nã··o precisar dar `source` sempre):

```bash
nano ~/.bashrc
```

No final do arquivo:

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
```

Salvar (`Ctrl+O` → `Enter` → `Ctrl+X`) e aplicar:

```bash
source ~/.bashrc
```

---

## 2. Instalar pacotes básicos (visã··o e utilitá··rios)

```bash
sudo apt update
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-rqt-image-view ros-jazzy-image-transport
```

Confirmar instalaç··o:

```bash
ros2 pkg list | grep -E 'v4l2|rqt_image|image_transport'
```

---

## 3. Comandos por subsistema

### 3.1. Câmeras

#### Câmera inferior (linha)

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[640, 480]" \
  -p output_encoding:=bgr8 \
  --ros-args -r /image_raw:=/camera/bottom/image_raw
```

#### Câmera superior (contexto / antecipaç··o)

Ajuste o dispositivo (ex.: `/dev/video1`):

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video1 \
  -p image_size:="[640, 480]" \
  -p output_encoding:=bgr8 \
  --ros-args -r /image_raw:=/camera/top/image_raw
```

> Em muitos casos, cada câmera cria seu próprio nó (`v4l2_camera_node_1`, `v4l2_camera_node_2`, etc.) ou usa namespaces. Ajuste conforme seu launch.

---

### 3.2. Visualizaç··o de imagem

```bash
ros2 run rqt_image_view rqt_image_view
```

Tó···picos esperados:

- `/camera/bottom/image_raw`  
- `/camera/top/image_raw`  

---

### 3.3. Visã··o da linha e marcadores

Nomes típicos (ajuste conforme seu pacote):

```bash
# Processamento da linha (centroide, erro lateral)
ros2 run ic_vision visao_linha_node

# Detecç··o de marcadores visuais
ros2 run ic_vision marcadores_node
```

Tó···picos típicos de saída:

- `/line/error`  
- `/line/centroid`  
- `/line/status`  
- `/markers/event`  

---

### 3.4. Filtro de Kalman (erro lateral)

```bash
ros2 run ic_control kalman_node
```

Entrada/saí··da típicas:

- Entrada: `/line/error`  
- Saí··da: `/line/error_filtered`  

---

### 3.5. Controle (PID) e cinemá··tica

```bash
# Controle PID + geraç··o de cmd_vel
ros2 run ic_control controle_node

# Cinemá··tica: cmd_vel → velocidades das rodas
ros2 run ic_control cinematica_node
```

Tó···picos típicos:

- Entrada: `/line/error_filtered`, `/nav/state`  
- Saí··da: `/cmd_vel` (controle)  
- Saí··da: `/wheel_cmd` (cinemá··tica)  

---

### 3.6. Odometria (encoders)

Node que lê os encoders (via serial com Arduino) e publica odometria:

```bash
ros2 run ic_control encoder_node
```

Tó···picos típicos:

- Entrada: `/serial/rx` (dados dos encoders)  
- Saí··da: `/wheel_states`, `/odom`  

---

### 3.7. Atuaç··o (motores via Arduino)

Bridge serial + atuaç··o:

```bash
# Ponte serial ROS ↔ Arduino
ros2 run ic_control serial_bridge_node

# Nó de atuaç··o (converte wheel_cmd em comandos para o Arduino)
ros2 run ic_control atuacao_node
```

Tó···picos típicos:

- Entrada: `/wheel_cmd`  
- Saí··da: `/serial/tx` (comandos para o Arduino)  

---

### 3.8. Gerenciador de estados e logger

```bash
# Máquinade estados (navegaç··o)
ros2 run ic_control gerenciador_estados_node

# Logger / gravaç··o de dados experimentais
ros2 run ic_control logger_node
```

Tó···picos relacionados:

- `/nav/state`  
- `/line/status`  
- `/markers/event`  

---

## 4. Node principal / Launch do sistema

Idealmente, vocí·· terá um launch que sobe todos os nodes de uma vez.

Exemplo de comando (ajuste o nome do pacote):

```bash
ros2 launch ic_bringup seguidor.launch.py
```

Ou, se houver um node “principal” que coordena tudo:

```bash
ros2 run ic_bringup principal_node
```

Dentro do launch, normalmente sã··o incluí··dos:

- `camera_inferior_node`  
- `camera_superior_node`  
- `visao_linha_node`  
- `marcadores_node`  
- `kalman_node`  
- `controle_node`  
- `cinematica_node`  
- `encoder_node`  
- `serial_bridge_node`  
- `atuacao_node`  
- `gerenciador_estados_node`  
- `logger_node`  

Ajuste os nomes de pacotes (`ic_vision`, `ic_control`, `ic_bringup`, etc.) conforme a estrutura real do seu workspace.

---

## 5. Comandos de diagnóstico

Ver nodes ativos:

```bash
ros2 node list
```

Ver tó···picos:

```bash
ros2 topic list
```

Ver informações de um tó···pico:

```bash
ros2 topic info /line/error --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /odom --verbose
```

Ver mensagens de exemplo:

```bash
ros2 topic echo /line/error --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once
```

Ver taxas de publicaç··o:

```bash
ros2 topic hz /camera/bottom/image_raw
ros2 topic hz /line/error
ros2 topic hz /cmd_vel
```

---

## 6. Resumo dos comandos “chave”

- **Carregar ROS (se nã··o estiver no `.bashrc`):**

  ```bash
  source /opt/ros/jazzy/setup.bash
  ```

- **Câ··meras:**

  ```bash
  ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0 \
    -p image_size:="[640, 480]" \
    -p output_encoding:=bgr8
  ```

- **Visualizar imagem:**

  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```

- **Nodes principais do projeto (exemplo):**

  ```bash
  ros2 run ic_vision visao_linha_node
  ros2 run ic_control kalman_node
  ros2 run ic_control controle_node
  ros2 run ic_control cinematica_node
  ros2 run ic_control encoder_node
  ros2 run ic_control serial_bridge_node
  ros2 run ic_control atuacao_node
  ```

- **Launch principal (ajustar nome):**

  ```bash
  ros2 launch ic_bringup seguidor.launch.py
  ```

---

Use este arquivo como referência rápida para subir o sistema completo e para testar cada subsistema de forma isolada.
