#!/usr/bin/env bash
# Sobe a pilha do robo. Chamado pelo systemd no boot, ou a mao.
#
# Nao recebe modo: o robo sobe em PARADO e quem escolhe o que fazer e o
# MENU DO ESP32 (Modo de operacao). Com o Pi ligando sozinho, o ESP32 e
# a unica interface que existe sem um PC por perto.
set -uo pipefail

WS=/home/bolt/ros2_ws
PORTA=${PORTA:-/dev/ttyACM0}

# 'set -u' quebra no setup.bash do ROS, que referencia COLCON_TRACE sem
# definir. Desliga so em volta dos sources.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# A serial do ESP32 demora a aparecer no boot: o CH343 precisa enumerar
# no USB. Sem esperar, o motor_serial_node sobe, nao acha a porta, e o
# robo fica sem atuacao ate alguem reiniciar o servico na mao.
for _ in $(seq 30); do
  [ -e "$PORTA" ] && break
  sleep 1
done
if [ ! -e "$PORTA" ]; then
  echo "AVISO: $PORTA nao apareceu em 30 s; subindo sem serial."
fi

exec ros2 launch robot_bringup line_follower.launch.py \
  serial:=true port:="$PORTA"
