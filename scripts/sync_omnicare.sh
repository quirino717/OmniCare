#!/usr/bin/env bash

# ================= CONFIGURAÇÕES =================
# Usuário e host do robô
ROBOT_USER="robot"
ROBOT_HOST="robot.local"    # ou IP da Jetson, ex: 192.168.0.50

# Diretório do workspace no robô (ajuste conforme seu setup)
REMOTE_WS="/home/robot/TCC/software/src"

# Pacotes disponíveis
ALL_PACKAGES=(
  "omnicare_behavior"
  "omnicare_bringup"
  "omnicare_control"
  "omnicare_control/omnidirecional_controllers"
  "omnicare_control/omnidrive_stm32"
  "omnicare_control/serial_interface_pkg"
  "omnicare_description"
  "omnicare_hri"
  "omnicare_hri/omnicare_assistant"
  "omnicare_hri/omnicare_expression"
  "omnicare_hri/omnicontroller_interface"
  "omnicare_msgs"
  "omnicare_navigation"
  "omnicare_navigation/bno055"
  "omnicare_navigation/navigation_pkg"
  "omnicare_navigation/rplidar_ros"
  "omnicare_simulation"
  "omnicare_vision"
  "omnicare_vision/dataset_generator"
  "omnicare_vision/floor_detector"
  "omnicare_vision/usb_cam"
)

# Exclusões padrão
RSYNC_EXCLUDES=(
  "--exclude=.git"
  "--exclude=.vscode"
  "--exclude=build"
  "--exclude=install"
  "--exclude=log"
  "--exclude=__pycache__"
  "--exclude=.pytest_cache"
)

# ================= FUNÇÕES =================

usage() {
  echo "Uso: $0 <pacote|all> [--dry-run]"
  echo
  echo "Exemplos:"
  echo "  $0 omnicare_behavior"
  echo "  $0 omnicare_navigation"
  echo "  $0 all"
  echo "  $0 omnicare_vision --dry-run   # só mostrar o que seria copiado"
  exit 1
}

# Verifica se pacote é válido
is_valid_package() {
  local pkg="$1"
  for p in "${ALL_PACKAGES[@]}"; do
    if [[ "$p" == "$pkg" ]]; then
      return 0
    fi
  done
  return 1
}

# ================= LÓGICA PRINCIPAL =================

[[ -z "$1" ]] && usage

TARGET="$1"
DRY_RUN=0

if [[ "$2" == "--dry-run" ]]; then
  DRY_RUN=1
fi

# Descobre diretório do projeto (onde o script está)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

# Monta flags de exclusão
EXCLUDE_FLAGS=("${RSYNC_EXCLUDES[@]}")

if [[ "$DRY_RUN" -eq 1 ]]; then
  EXCLUDE_FLAGS+=("-n" "--progress")
  echo "[INFO] Rodando em modo dry-run (nenhum arquivo será realmente copiado)"
fi

# Define lista de pacotes a sincronizar
PACKAGES_TO_SYNC=()

if [[ "$TARGET" == "all" ]]; then
  PACKAGES_TO_SYNC=("${ALL_PACKAGES[@]}")
else
  if ! is_valid_package "$TARGET"; then
    echo "[ERRO] Pacote '$TARGET' não é conhecido."
    echo "Pacotes disponíveis:"
    printf '  - %s\n' "${ALL_PACKAGES[@]}"
    exit 1
  fi
  PACKAGES_TO_SYNC=("$TARGET")
fi

echo "[INFO] Sincronizando para $ROBOT_USER@$ROBOT_HOST:$REMOTE_WS"
echo "[INFO] Pacotes: ${PACKAGES_TO_SYNC[*]}"
echo

for PKG in "${PACKAGES_TO_SYNC[@]}"; do
  LOCAL_DIR="$PROJECT_ROOT/src/$PKG"
  REMOTE_DIR="$REMOTE_WS/$PKG"

  if [[ ! -d "$LOCAL_DIR" ]]; then
    echo "[WARN] Diretório local não encontrado: $LOCAL_DIR (pulando...)"
    continue
  fi

  echo "=== rsync -> $PKG ==="
  rsync -avh --delete "${EXCLUDE_FLAGS[@]}" \
    "$LOCAL_DIR/" \
    "$ROBOT_USER@$ROBOT_HOST:$REMOTE_DIR/"
  echo
done

echo "[OK] Sincronização finalizada."
