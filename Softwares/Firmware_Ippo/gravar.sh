#!/usr/bin/env bash
# Grava o firmware derrubando a pilha ROS antes e subindo depois.
#
# POR QUE EXISTE: motor_serial_node mantem /dev/ttyACM0 aberto. Com a
# pilha no ar o upload falha por porta ocupada -- e o modo de falha e
# confuso, porque o esptool reclama de sincronismo e nao de permissao.
#
# Uso:  ./gravar.sh          grava e devolve a pilha como estava
#       ./gravar.sh --off    grava e deixa a pilha parada
set -euo pipefail
cd "$(dirname "$0")"

# ATENCAO ao padrao: o launch sobe camera_node, que vem do pacote
# camera_ros e NAO casa com 'robot_bringup/'. Matar so os nos do
# robot_bringup deixa as cameras vivas segurando o pipeline do libcamera,
# e a proxima subida falha com "Pipeline handler in use by another
# process" -- que foi exatamente o que aconteceu em 18/09/2026, com tres
# instancias empilhadas.
PADRAO='line_follower.launch|robot_bringup/lib|camera_ros/lib'

# A pilha agora sobe pelo robo.service (systemd, Restart=on-failure,
# RestartSec=10). Matar os processos por baixo dele faz o systemd subir
# a pilha de novo NO MEIO da gravacao, e o fim deste script ainda lanca
# uma segunda instancia. Com o servico ativo, recusa e diz o que fazer.
if systemctl is-active --quiet robo 2>/dev/null; then
  echo '!!! robo.service esta ativo. Pare antes e suba depois:'
  echo '      sudo systemctl stop robo'
  echo '      ./gravar.sh --off'
  echo '      sudo systemctl start robo'
  exit 1
fi

ESTAVA_NO_AR=0
if pgrep -f "$PADRAO" > /dev/null 2>&1; then
  ESTAVA_NO_AR=1
  echo '>>> derrubando a pilha ROS (ela segura a porta serial)'
  pkill -f "$PADRAO" 2>/dev/null || true
  for _ in $(seq 20); do
    pgrep -f "$PADRAO" > /dev/null 2>&1 || break
    sleep 0.5
  done
  pkill -9 -f "$PADRAO" 2>/dev/null || true
  sleep 2
  if pgrep -f "$PADRAO" > /dev/null 2>&1; then
    echo '!!! sobrou processo da pilha; abortando para nao empilhar'
    pgrep -af "$PADRAO" | cut -c1-80
    exit 1
  fi
fi

echo '>>> gravando'
~/.platformio/penv/bin/pio run -t upload

if [ "${1:-}" = "--off" ]; then
  echo '>>> pilha deixada PARADA (--off)'
  exit 0
fi

if [ "$ESTAVA_NO_AR" = "1" ]; then
  echo '>>> subindo a pilha de volta'
  cd /home/bolt/ros2_ws
  # 'set -u' quebra no setup.bash do ROS, que referencia COLCON_TRACE sem
  # definir. Desliga so em volta do source.
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u
  setsid nohup ros2 launch robot_bringup line_follower.launch.py \
    serial:=true > /home/bolt/ros2_ws/logs/stack.log 2>&1 < /dev/null &
  disown
  echo '>>> aguardando o motor_serial_node'
  for _ in $(seq 40); do
    if ros2 node list 2>/dev/null | grep -q motor_serial_node; then
      # Modo bancada por padrao: o robo nao sai andando ao voltar.
      ros2 param set /line_follower_node body_control_enabled false \
        > /dev/null 2>&1 || true
      echo '>>> pilha no ar, modo bancada ligado'
      exit 0
    fi
    sleep 2
  done
  echo '!!! a pilha nao subiu a tempo -- confira logs/stack.log'
fi
