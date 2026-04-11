#!/bin/bash
# =============================================================================
# diagnostico_rtabmap.sh
# Script de diagnóstico para el pipeline LiDAR + OAK-D + RTAB-Map
# Uso: bash diagnostico_rtabmap.sh
# =============================================================================

BOLD='\033[1m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

DURATION=5  # segundos de muestreo por tópico

echo -e "${BOLD}${CYAN}"
echo "============================================================"
echo "   DIAGNÓSTICO RTAB-Map — LiDAR + OAK-D"
echo "============================================================"
echo -e "${NC}"

# ------------------------------------------------------------
# 1. FRECUENCIAS DE TÓPICOS CRÍTICOS
# ------------------------------------------------------------
echo -e "${BOLD}[1/5] Frecuencias de tópicos críticos (muestreo ${DURATION}s cada uno)${NC}"
echo "--------------------------------------------------------------"

check_hz() {
    local topic=$1
    local label=$2
    echo -ne "  ${label} (${topic}): "
    result=$(timeout $DURATION ros2 topic hz "$topic" 2>&1 | grep "average rate" | tail -1)
    if [ -z "$result" ]; then
        echo -e "${RED}NO PUBLICADO ✗${NC}"
    else
        rate=$(echo "$result" | awk '{print $3}' | tr -d ':')
        echo -e "${GREEN}${rate} Hz ✓${NC}"
    fi
}

check_hz "/unilidar/cloud"         "LiDAR cloud"
check_hz "/odom"                   "Odometría ICP"
check_hz "/color/video/image"      "RGB imagen"
check_hz "/color/video/camera_info" "RGB camera_info"
check_hz "/stereo/depth"           "Stereo depth"
check_hz "/stereo/camera_info"     "Stereo camera_info"

# ------------------------------------------------------------
# 2. DESFASE DE TIMESTAMPS ENTRE DEPTH Y RGB
# ------------------------------------------------------------
echo ""
echo -e "${BOLD}[2/5] Comparación de timestamps RGB vs Depth${NC}"
echo "--------------------------------------------------------------"
echo "  Capturando 1 mensaje de /color/video/image..."
ts_rgb=$(ros2 topic echo --once /color/video/image 2>/dev/null | grep -A2 "header:" | grep "stamp:" | head -1 | awk '{print $2, $3}')

echo "  Capturando 1 mensaje de /stereo/depth..."
ts_depth=$(ros2 topic echo --once /stereo/depth 2>/dev/null | grep -A2 "header:" | grep "stamp:" | head -1 | awk '{print $2, $3}')

echo -e "  RGB stamp:   ${CYAN}${ts_rgb}${NC}"
echo -e "  Depth stamp: ${CYAN}${ts_depth}${NC}"
echo -e "  ${YELLOW}▶ Si los segundos difieren en más de 0.1s, el approx_sync no matcheará.${NC}"

# ------------------------------------------------------------
# 3. ÁRBOL DE TRANSFORMACIONES TF
# ------------------------------------------------------------
echo ""
echo -e "${BOLD}[3/5] Estado del árbol TF${NC}"
echo "--------------------------------------------------------------"
echo "  Frames activos:"
ros2 run tf2_ros tf2_monitor 2>/dev/null &
TF_PID=$!
sleep 3
kill $TF_PID 2>/dev/null

echo ""
echo "  Verificando cadenas clave:"

check_tf() {
    local from=$1
    local to=$2
    result=$(timeout 3 ros2 run tf2_ros tf2_echo "$from" "$to" 2>&1 | head -3)
    if echo "$result" | grep -q "Waiting\|Invalid\|does not"; then
        echo -e "  ${RED}✗ ${from} → ${to}: NO DISPONIBLE${NC}"
    else
        echo -e "  ${GREEN}✓ ${from} → ${to}: OK${NC}"
    fi
}

check_tf "odom"           "unilidar_lidar"
check_tf "unilidar_lidar" "oak_rgb_camera_optical_frame"
check_tf "odom"           "oak_rgb_camera_optical_frame"

# ------------------------------------------------------------
# 4. DIAGNÓSTICO DE SYNC DEL RTABMAP
# ------------------------------------------------------------
echo ""
echo -e "${BOLD}[4/5] Diagnóstico de sincronización (escuchar /info de rtabmap por 10s)${NC}"
echo "--------------------------------------------------------------"
echo "  Si ves mensajes de /info → rtabmap está procesando frames correctamente."
echo "  Si no aparece nada → el sync nunca completa."
echo ""
timeout 10 ros2 topic echo /info 2>/dev/null | grep -E "stamp|loop_closure|land" | head -20
if [ $? -eq 124 ]; then
    echo -e "  ${RED}✗ No se recibió ningún mensaje en /info durante 10 segundos.${NC}"
    echo -e "  ${YELLOW}  Causa probable: sync approx nunca matchea los tópicos.${NC}"
fi

# ------------------------------------------------------------
# 5. RESUMEN DE COMANDOS ÚTILES
# ------------------------------------------------------------
echo ""
echo -e "${BOLD}[5/5] Comandos de diagnóstico manual recomendados${NC}"
echo "--------------------------------------------------------------"
echo ""
echo -e "  ${CYAN}# Ver frecuencia de cualquier tópico:${NC}"
echo "  ros2 topic hz /stereo/depth"
echo "  ros2 topic hz /color/video/image"
echo ""
echo -e "  ${CYAN}# Ver si los timestamps están alineados (ejecutar en paralelo):${NC}"
echo "  ros2 topic echo /stereo/depth --field header.stamp"
echo "  ros2 topic echo /color/video/image --field header.stamp"
echo ""
echo -e "  ${CYAN}# Ver el frame_id del depth (debe ser oak_right o similar, NO unilidar_lidar):${NC}"
echo "  ros2 topic echo /stereo/depth --field header.frame_id --once"
echo "  ros2 topic echo /color/video/image --field header.frame_id --once"
echo ""
echo -e "  ${CYAN}# Confirmar que rtabmap está recibiendo datos:${NC}"
echo "  ros2 topic hz /info"
echo "  ros2 topic hz /mapData"
echo "  ros2 topic hz /map"
echo ""
echo -e "  ${CYAN}# Ver el árbol TF completo en imagen:${NC}"
echo "  ros2 run tf2_tools view_frames"
echo ""
echo -e "  ${CYAN}# Verificar delay entre tópicos (busca 'delay' en los logs del icp_odometry):${NC}"
echo "  ros2 launch mapeo_opcionB_rgbd.py 2>&1 | grep -E 'delay|WARN|Did not'"
echo ""
echo -e "  ${CYAN}# Echo del frame_id del depth para confirmar que coincide con camera_info:${NC}"
echo "  ros2 topic echo /stereo/depth --once | grep frame_id"
echo "  ros2 topic echo /stereo/camera_info --once | grep frame_id"
echo ""
echo "============================================================"
echo -e "  ${BOLD}Diagnóstico completo.${NC}"
echo "============================================================"
