#!/bin/bash
#
# ROS 2 Bag Recorder Script for OmniCare Navigation
# Autor: Lucas Lagoeiro
# Descrição: grava automaticamente todos os tópicos relevantes do NAV2 e OmniCare
#

# =============== CONFIGURAÇÕES ===============
# Nome base do bag (será adicionado data/hora automaticamente)
BAG_NAME="omnicare_nav_bag_$(date +%Y%m%d_%H%M%S)"

# Diretório de saída
OUTPUT_DIR="$HOME/rosbags"
mkdir -p "$OUTPUT_DIR"

# Duração máxima (em segundos) - 0 = ilimitado
MAX_DURATION=0

# =============== LISTA DE TÓPICOS ===============
TOPICS=(
  # Pose e TF
  /amcl_pose
  /particle_cloud
  /odom
  /robot_description
  /tf
  /tf_static

  # Global Costmap
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /plan
  /global_costmap/voxel_marked_cloud

  # Controller
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /local_plan
  /marker

  # Percepção e mapas
  /scan
  /scan_filtered
  /map
)

# =============== INÍCIO DO SCRIPT ===============
echo -e "\033[1;36m=============================\033[0m"
echo -e "\033[1;36m Iniciando gravação ROS2 Bag \033[0m"
echo -e "\033[1;36m=============================\033[0m"
echo ""
echo -e "\033[1;33mNome do bag:\033[0m $BAG_NAME"
echo -e "\033[1;33mSaída em:\033[0m $OUTPUT_DIR"
echo ""

# Mostra os tópicos que serão gravados
echo -e "\033[1;32mTópicos a serem gravados:\033[0m"
for topic in "${TOPICS[@]}"; do
  echo "  - $topic"
done
echo ""

# Monta o comando ros2 bag record
CMD="ros2 bag record -o ${OUTPUT_DIR}/${BAG_NAME}"

# Adiciona tempo limite se configurado
if [ "$MAX_DURATION" -gt 0 ]; then
  CMD+=" --duration $MAX_DURATION"
fi

# Adiciona os tópicos
for topic in "${TOPICS[@]}"; do
  CMD+=" $topic"
done

# Mostra o comando final
echo -e "\033[1;34mComando executado:\033[0m"
echo "$CMD"
echo ""

# Executa o comando
eval "$CMD"
