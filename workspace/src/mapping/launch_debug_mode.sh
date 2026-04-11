#!/bin/bash
# launch_debug_mode.sh — Lanza YOLO debugging en múltiples terminales
# ────────────────────────────────────────────────────────────────────
# Abre automáticamente:
#  1. Terminal 1: Backend (OAK-D + YOLO + SLAM)
#  2. Terminal 2: Viewer (detecciones en tiempo real)
#  3. Terminal 3: Diagnostics (estadísticas)
#  4. Terminal 4: RViz (opcional)
#
# Requisitos: tmux o gnome-terminal
# Uso: ./launch_debug_mode.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}YOLO Debugging Mode - Multi-Terminal Launcher${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo

# Detectar terminal disponible
if command -v tmux &> /dev/null; then
    TERM_APP="tmux"
    echo -e "${GREEN}✓ Usando: tmux${NC}"
elif command -v gnome-terminal &> /dev/null; then
    TERM_APP="gnome"
    echo -e "${GREEN}✓ Usando: gnome-terminal${NC}"
elif command -v xterm &> /dev/null; then
    TERM_APP="xterm"
    echo -e "${GREEN}✓ Usando: xterm${NC}"
else
    echo -e "${RED}✗ No se encontró terminal multiplexer${NC}"
    echo "  Instala alguno de: tmux, gnome-terminal, xterm"
    exit 1
fi

echo -e "${YELLOW}Workspace: $WORKSPACE_DIR${NC}"
echo

# Comando de setup
SETUP_CMD="cd $WORKSPACE_DIR && source install/setup.bash"

# ─────────────────────────────────────────────────────────────────────────────
# Opción 1: TMUX (recomendado - mejor control)
# ─────────────────────────────────────────────────────────────────────────────

if [ "$TERM_APP" = "tmux" ]; then
    echo -e "${BLUE}Configurando tmux session: yolo-debug${NC}"
    
    # Crear sesión
    tmux new-session -d -s yolo-debug -x 200 -y 50
    
    # Window 1: Backend
    tmux rename-window -t yolo-debug '1-backend'
    tmux send-keys -t yolo-debug "clear && echo '=== BACKEND ===' && $SETUP_CMD && ros2 launch mapping depthai_yolo_backend.launch.py" Enter
    sleep 3
    
    # Window 2: Viewer
    tmux new-window -t yolo-debug -n '2-viewer'
    tmux send-keys -t yolo-debug:2 "clear && echo '=== VIEWER ===' && $SETUP_CMD && python3 src/mapping/scripts/yolo_detection_viewer.py" Enter
    
    # Window 3: Diagnostics
    tmux new-window -t yolo-debug -n '3-diag'
    tmux send-keys -t yolo-debug:3 "clear && echo '=== DIAGNOSTICS ===' && $SETUP_CMD && python3 src/mapping/scripts/yolo_diagnostics.py" Enter
    
    # Window 4: RViz (opcional)
    tmux new-window -t yolo-debug -n '4-rviz'
    tmux send-keys -t yolo-debug:4 "clear && echo '=== RVIZ ===' && $SETUP_CMD && sleep 2 && rviz2" Enter
    
    # Ir a la primera ventana
    tmux select-window -t yolo-debug:0
    
    echo -e "${GREEN}✓ Sesión tmux creada: yolo-debug${NC}"
    echo
    echo -e "${YELLOW}Comandos útiles:${NC}"
    echo "  tmux attach -t yolo-debug     # Conectarse a la sesión"
    echo "  tmux kill-session -t yolo-debug  # Cerrar todo"
    echo "  Ctrl+B + o                    # Cambiar entre ventanas"
    echo "  Ctrl+B + n/p                  # Siguiente/anterior ventana"
    echo "  Ctrl+B + d                    # Desconectar (pero sigue corriendo)"
    echo
    
    # Conectarse automáticamente
    echo -e "${YELLOW}Conectando a tmux...${NC}"
    sleep 2
    tmux attach -t yolo-debug

# ─────────────────────────────────────────────────────────────────────────────
# Opción 2: GNOME-TERMINAL
# ─────────────────────────────────────────────────────────────────────────────

elif [ "$TERM_APP" = "gnome" ]; then
    echo -e "${BLUE}Abriendo gnome-terminal (4 ventanas)...${NC}"
    
    # Backend
    gnome-terminal --title="Backend (OAK-D + YOLO + SLAM)" -- bash -c "$SETUP_CMD && ros2 launch mapping depthai_yolo_backend.launch.py; read" &
    sleep 2
    
    # Viewer
    gnome-terminal --title="YOLO Viewer (Detecciones)" -- bash -c "$SETUP_CMD && python3 src/mapping/scripts/yolo_detection_viewer.py; read" &
    
    # Diagnostics
    gnome-terminal --title="Diagnostics (Estadísticas)" -- bash -c "$SETUP_CMD && python3 src/mapping/scripts/yolo_diagnostics.py; read" &
    
    # RViz
    gnome-terminal --title="RViz (Visualización)" -- bash -c "$SETUP_CMD && sleep 2 && rviz2; read" &
    
    echo -e "${GREEN}✓ Terminales abiertas${NC}"
    echo
    echo -e "${YELLOW}Tips:${NC}"
    echo "  • Cambia entre ventanas con Alt+Tab"
    echo "  • Cierra cada ventana con Ctrl+C o cerrando la pestaña"
    echo

# ─────────────────────────────────────────────────────────────────────────────
# Opción 3: XTERM (fallback)
# ─────────────────────────────────────────────────────────────────────────────

elif [ "$TERM_APP" = "xterm" ]; then
    echo -e "${BLUE}Abriendo xterm (4 ventanas)...${NC}"
    
    xterm -T "Backend" -e "bash -c '$SETUP_CMD && ros2 launch mapping depthai_yolo_backend.launch.py'" &
    sleep 2
    
    xterm -T "Viewer" -e "bash -c '$SETUP_CMD && python3 src/mapping/scripts/yolo_detection_viewer.py'" &
    
    xterm -T "Diagnostics" -e "bash -c '$SETUP_CMD && python3 src/mapping/scripts/yolo_diagnostics.py'" &
    
    xterm -T "RViz" -e "bash -c '$SETUP_CMD && sleep 2 && rviz2'" &
    
    echo -e "${GREEN}✓ Ventanas xterm abiertas${NC}"
fi

echo
echo -e "${GREEN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}¡Sistema YOLO Debug lanzado!${NC}"
echo -e "${GREEN}════════════════════════════════════════════════════════════════${NC}"
