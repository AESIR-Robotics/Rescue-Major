#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# setup_mapping.sh — Instalador de dependencias para el paquete mapping
# Equipo Aesir / RoboCup Rescue 2026
# ═══════════════════════════════════════════════════════════════════════════════
#
# Este script instala TODAS las dependencias necesarias para:
#   • object_detection_node.py  (YOLO + ROS 2)
#   • export_map_ply.py         (Open3D + rtabmap-export)
#   • depthai_test.launch.py    (OAK-D + RTAB-Map + IMU)
#
# Uso:
#   chmod +x setup_mapping.sh
#   ./setup_mapping.sh
#
# Probado en:
#   • Ubuntu 22.04 / ROS 2 Humble
#   • Jetson Orin Nano 8GB (JetPack 6.0)
# ═══════════════════════════════════════════════════════════════════════════════

set -e  # Salir si hay error

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}"
echo "═══════════════════════════════════════════════════════════════════════"
echo "  SETUP MAPPING — Equipo Aesir / RoboCup Rescue 2026"
echo "═══════════════════════════════════════════════════════════════════════"
echo -e "${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ─────────────────────────────────────────────────────────────────────────────
# 1. Verificar ROS 2 Humble
# ─────────────────────────────────────────────────────────────────────────────
echo -e "${YELLOW}[1/5] Verificando ROS 2 Humble...${NC}"

if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}[ERROR] ROS 2 no está sourced. Ejecuta:${NC}"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo -e "${YELLOW}[AVISO] ROS_DISTRO=$ROS_DISTRO (esperado: humble)${NC}"
fi

echo -e "${GREEN}  ✓ ROS 2 $ROS_DISTRO detectado${NC}"

# ─────────────────────────────────────────────────────────────────────────────
# 2. Dependencias APT (ROS 2 + sistema)
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[2/5] Instalando dependencias APT...${NC}"

sudo apt update

# Paquetes ROS 2 requeridos
ROS_PACKAGES=(
    ros-humble-cv-bridge
    ros-humble-message-filters
    ros-humble-tf2-ros
    ros-humble-tf2-geometry-msgs
    ros-humble-vision-msgs
    ros-humble-image-transport
    ros-humble-rtabmap-ros
    ros-humble-depthai-ros
    ros-humble-imu-filter-madgwick
)

# Paquetes del sistema
SYSTEM_PACKAGES=(
    python3-pip
    python3-opencv
    libopencv-dev
)

echo "  Instalando paquetes ROS 2..."
sudo apt install -y "${ROS_PACKAGES[@]}" || {
    echo -e "${YELLOW}  [AVISO] Algunos paquetes ROS 2 no se encontraron.${NC}"
    echo "  Intentando instalar los disponibles uno por uno..."
    for pkg in "${ROS_PACKAGES[@]}"; do
        sudo apt install -y "$pkg" 2>/dev/null || echo "    - $pkg no disponible"
    done
}

echo "  Instalando paquetes del sistema..."
sudo apt install -y "${SYSTEM_PACKAGES[@]}"

echo -e "${GREEN}  ✓ Dependencias APT instaladas${NC}"

# ─────────────────────────────────────────────────────────────────────────────
# 3. Dependencias Python (pip)
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[3/5] Instalando dependencias Python...${NC}"

# Detectar si estamos en Jetson
IS_JETSON=false
if [ -f /etc/nv_tegra_release ]; then
    IS_JETSON=true
    echo "  Jetson detectado - usando configuración optimizada"
fi

PIP_FLAGS="--break-system-packages"

# NumPy
echo "  Instalando numpy..."
pip3 install numpy $PIP_FLAGS

# OpenCV headless (sin GUI)
echo "  Instalando opencv-python-headless..."
pip3 install opencv-python-headless $PIP_FLAGS

# Open3D para export_map_ply.py
echo "  Instalando open3d..."
if [ "$IS_JETSON" = true ]; then
    # En Jetson puede necesitar compilación o wheel específico
    pip3 install open3d $PIP_FLAGS || {
        echo -e "${YELLOW}  [AVISO] open3d puede requerir compilación en Jetson.${NC}"
        echo "  Intentando instalar versión compatible..."
        pip3 install open3d==0.17.0 $PIP_FLAGS || true
    }
else
    pip3 install open3d $PIP_FLAGS
fi

# Ultralytics (YOLOv8)
echo "  Instalando ultralytics (YOLOv8)..."
pip3 install ultralytics $PIP_FLAGS

echo -e "${GREEN}  ✓ Dependencias Python instaladas${NC}"

# ─────────────────────────────────────────────────────────────────────────────
# 4. Verificar rtabmap-export
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[4/5] Verificando rtabmap-export...${NC}"

if command -v rtabmap-export &> /dev/null; then
    echo -e "${GREEN}  ✓ rtabmap-export disponible${NC}"
else
    echo -e "${YELLOW}  [AVISO] rtabmap-export no encontrado en PATH${NC}"
    echo "  Instalando rtabmap completo..."
    sudo apt install -y rtabmap || echo "  rtabmap ya debería estar instalado via ros-humble-rtabmap-ros"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 5. Crear directorios necesarios
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[5/5] Creando directorios...${NC}"

mkdir -p ~/maps
mkdir -p ~/models/yolo
mkdir -p ~/.ros

echo -e "${GREEN}  ✓ Directorios creados:${NC}"
echo "    ~/maps          - Archivos PLY exportados"
echo "    ~/models/yolo   - Modelos YOLO (coloca best.pt aquí)"
echo "    ~/.ros          - Base de datos RTAB-Map y objetos detectados"

# ─────────────────────────────────────────────────────────────────────────────
# Resumen final
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  ✓ INSTALACIÓN COMPLETADA${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════════════${NC}"
echo ""
echo "Próximos pasos:"
echo ""
echo "  1. Compilar el workspace:"
echo "     cd ~/Rescue-Major/workspace"
echo "     colcon build --packages-select mapping"
echo "     source install/setup.bash"
echo ""
echo "  2. Colocar modelo YOLO en ~/models/yolo/best.pt"
echo ""
echo "  3. Ejecutar el sistema:"
echo "     ros2 launch mapping depthai_test.launch.py"
echo ""
echo "  4. Exportar mapa PLY:"
echo "     python3 $SCRIPT_DIR/scripts/export_map_ply.py"
echo ""
