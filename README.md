# Comandos ROS 2 – Câmera no Raspberry (Jazzy)

Este documento reúne os comandos essenciais para configurar e usar a câmera USB no Raspberry com ROS 2 Jazzy, incluindo instalação, visual da imagem e acesso remoto.

---

## 1. Instalar ROS 2 Jazzy (resumo)

```bash
# No Raspberry
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Adicionar repositorio ROS 2 (se ainda nao tiver)
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop
```

Carregar ROS no terminal atual (se ainda nao colocou no `.bashrc`):

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2. Instalar pacote da câmera e visualizador

```bash
sudo apt update
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-rqt-image-view
```

Confirmar instalação:

```bash
ros2 pkg list | grep -E 'v4l2|rqt_image'
```

Deve aparecer, entre outros:

```text
v4l2_camera
rqt_image_view
```

---

## 3. Colocar ROS no `.bashrc` (para não precisar dar `source` sempre)

```bash
nano ~/.bashrc
```

No final do arquivo, adicionar:

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
```

Salvar (`Ctrl+O` → `Enter` → `Ctrl+X`) e aplicar:

```bash
source ~/.bashrc
```

A partir de agora, todo novo terminal já carrega o ROS automaticamente.

---

## 4. Rodar o node da câmera

Identificar o dispositivo de vídeo (geralmente `/dev/video0`):

```bash
ls /dev/video*
```

Rodar o node da câmera:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[640, 480]" \
  -p output_encoding:=bgr8
```

Deixe esse terminal aberto enquanto for usar a imagem.

---

## 5. Visualizar a imagem com `rqt_image_view`

### Opcao A – SSH com X11 forwarding (recomendado se usa Ubuntu no PC)

No PC (Ubuntu):

```bash
ssh -Y bolt@IP_DO_RASPBERRY
```

No Raspberry (via SSH):

```bash
echo $DISPLAY          # deve mostrar algo como localhost:10.0
ros2 run rqt_image_view rqt_image_view
```

Na janela do RQt, selecionar o tópico:

```text
/image_raw
```

### Opcao B – VNC (desktop remoto completo)

No Raspberry:

```bash
sudo apt install realvnc-vnc-server realvnc-vnc-viewer
sudo raspi-config
# Interface Options → VNC → Yes
```

No PC:

- Instalar VNC Viewer (`realvnc-vnc-viewer` ou similar).
- Conectar em `IP_DO_RASPBERRY:1`.
- Dentro da sessão gráfica, abrir terminal e rodar:

  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```

---

## 6. Comandos úteis de diagnóstico

Ver nodes ativos:

```bash
ros2 node list
```

Ver tó¬¬¬picos de imagem:

```bash
ros2 topic list | grep image
```

Ver informações do tópico:

```bash
ros2 topic info /image_raw --verbose
```

Ver mensagens de imagem (teste rápido):

```bash
ros2 topic echo /image_raw --once
```

---

## 7. Resumo dos comandos “chave”

- **Carregar ROS (se nao estiver no `.bashrc`):**

  ```bash
  source /opt/ros/jazzy/setup.bash
  ```

- **Instalar câmera + visualizador:**

  ```bash
  sudo apt install ros-jazzy-v4l2-camera ros-jazzy-rqt-image-view
  ```

- **Rodar a câmera:**

  ```bash
  ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0 \
    -p image_size:="[640, 480]" \
    -p output_encoding:=bgr8
  ```

- **Abrir visualizador (com SSH `-Y` ou local em GUI):**

  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```

- **Conectar via SSH com gráfico:**

  ```bash
  ssh -Y bolt@IP_DO_RASPBERRY
  ```

---

Use este arquivo como referência rápida para subir o ambiente de visão no Raspberry sempre que necessário.
