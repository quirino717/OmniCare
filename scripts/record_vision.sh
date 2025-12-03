#!/bin/bash
#
# ROS 2 Bag Recorder Script for OmniCare Navigation
# Autor: Lucas Lagoeiro
# Descrição: grava automaticamente todos os tópicos relevantes da Visão e OmniCare,
# e para automaticamente se o arquivo ultrapassar 30 GB.
#

# =============== CONFIGURAÇÕES ===============
BAG_NAME="omnicare_vision_bag_$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$HOME/rosbags"
mkdir -p "$OUTPUT_DIR"

MAX_DURATION=0
MAX_SIZE_GB=30                       # Limite máximo em GB
MAX_SIZE_BYTES=$((MAX_SIZE_GB * 1024 * 1024 * 1024))

# =============== LISTA DE TÓPICOS ===============
TOPICS=(
  /camera1/image_raw
  /camera1/image_raw/theora
  /omnicare/floor_detector/debug
  /omnicare/floor_detector/elevator_display
  /omnicare/floor_detector/info
)

# =============== INÍCIO DO SCRIPT ===============
echo -e "\033[1;36m=============================\033[0m"
echo -e "\033[1;36m Iniciando gravação ROS2 Bag \033[0m"
echo -e "\033[1;36m=============================\033[0m"
echo ""
echo -e "\033[1;33mNome do bag:\033[0m $BAG_NAME"
echo -e "\033[1;33mSaída em:\033[0m $OUTPUT_DIR"
echo -e "\033[1;33mLimite de tamanho:\033[0m ${MAX_SIZE_GB} GB"
echo ""

echo -e "\033[1;32mTópicos a serem gravados:\033[0m"
for topic in "${TOPICS[@]}"; do
  echo "  - $topic"
done
echo ""

# Monta o comando
CMD="ros2 bag record -o ${OUTPUT_DIR}/${BAG_NAME}"
if [ "$MAX_DURATION" -gt 0 ]; then
  CMD+=" --duration $MAX_DURATION"
fi
for topic in "${TOPICS[@]}"; do
  CMD+=" $topic"
done

echo -e "\033[1;34mComando executado:\033[0m"
echo "$CMD"
echo ""

# Executa em background
eval "$CMD" &
BAG_PID=$!
echo "PID do ros2 bag record: $BAG_PID"
echo ""

# Caminho do diretório de gravação
BAG_PATH="${OUTPUT_DIR}/${BAG_NAME}"

# =============== MONITORAMENTO DE TAMANHO ===============
echo -e "\033[1;35mMonitorando tamanho da bag...\033[0m"
while kill -0 "$BAG_PID" 2>/dev/null; do
  if [ -d "$BAG_PATH" ]; then
    SIZE_BYTES=$(du -sb "$BAG_PATH" | awk '{print $1}')
    SIZE_GB=$(echo "scale=2; $SIZE_BYTES / (1024*1024*1024)" | bc)
    echo "→ Tamanho atual: ${SIZE_GB} GB"
    if [ "$SIZE_BYTES" -ge "$MAX_SIZE_BYTES" ]; then
      echo -e "\033[1;31mLimite de ${MAX_SIZE_GB} GB atingido! Encerrando gravação...\033[0m"
      kill "$BAG_PID"
      break
    fi
  fi
  sleep 10  # intervalo entre checagens (em segundos)
done

echo -e "\033[1;32mGravação finalizada.\033[0m"
