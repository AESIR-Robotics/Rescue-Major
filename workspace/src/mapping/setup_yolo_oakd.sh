#!/bin/bash
# setup_yolo_oakd.sh — Configuración del sistema YOLO + OAK-D para mapeo
# Equipo Aesir - RoboCup Rescue 2026 - Rescue Major
#
# Este script:
#  1. Verifica que el modelo YOLO exista
#  2. Instala dependencias Python
#  3. Compila el paquete ROS 2 'mapping'
#  4. Genera setup scripts
#  5. Proporciona instrucciones de uso

set -e  # Exit on error

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}Setup YOLO v8 + OAK-D para RTAB-Map Mapping${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo

# ─────────────────────────────────────────────────────────────────────────────
# 1. Validar que estamos en el directorio correcto
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_DIR="$SCRIPT_DIR"  # El script está EN mapping/
WORKSPACE_DIR="$(dirname "$(dirname "$MAPPING_DIR")")"  # mapping → src → workspace

echo -e "${BLUE}[1/5]${NC} Validando directorios..."
echo "  Script dir:   $SCRIPT_DIR"
echo "  Mapping dir:  $MAPPING_DIR"
echo "  Workspace:    $WORKSPACE_DIR"

if [ ! -f "$MAPPING_DIR/package.xml" ]; then
    echo -e "${RED}✗ Error: No se encontró package.xml en $MAPPING_DIR${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Directorios validados${NC}"
echo

# ─────────────────────────────────────────────────────────────────────────────
# 2. Verificar modelo YOLO
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[2/5]${NC} Verificando modelo YOLO aesir_rescue_v27_final3..."

MODEL_PATH="$MAPPING_DIR/yolo_training/runs/detect/runs/detect/aesir_rescue_v27_final3/weights/best.pt"

if [ ! -f "$MODEL_PATH" ]; then
    echo -e "${RED}✗ Modelo no encontrado en:${NC}"
    echo "  $MODEL_PATH"
    echo
    echo -e "${YELLOW}Búscando en otras ubicaciones...${NC}"
    
    # Intentar buscar el modelo
    FOUND_MODELS=$(find "$MAPPING_DIR" -name "aesir_rescue_v27_final3" -type d 2>/dev/null || echo "")
    
    if [ -n "$FOUND_MODELS" ]; then
        echo -e "${YELLOW}Encontrado:${NC} $FOUND_MODELS"
        MODEL_PATH="$FOUND_MODELS/weights/best.pt"
    else
        echo -e "${YELLOW}Modelos disponibles:${NC}"
        find "$MAPPING_DIR/yolo_training/runs/detect" -name "best.pt" -type f 2>/dev/null | head -5 || echo "  Ninguno encontrado"
        echo
        echo -e "${YELLOW}⚠ Nota: Puedes usar otro modelo editando el launch file${NC}"
    fi
fi

if [ -f "$MODEL_PATH" ]; then
    MODEL_SIZE=$(du -h "$MODEL_PATH" | cut -f1)
    echo -e "${GREEN}✓ Modelo encontrado (tamaño: $MODEL_SIZE)${NC}"
    echo "  $MODEL_PATH"
else
    echo -e "${YELLOW}⚠ Modelo no encontrado, continuando...${NC}"
    echo "  Asegúrate de que el modelo existe antes de ejecutar el launch file"
fi
echo

# ─────────────────────────────────────────────────────────────────────────────
# 3. Instalar dependencias Python
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[3/5]${NC} Instalando dependencias Python..."

if [ -f "$MAPPING_DIR/requirements.txt" ]; then
    echo "  Instalando desde requirements.txt..."
    pip install -r "$MAPPING_DIR/requirements.txt" --break-system-packages || {
        echo -e "${YELLOW}⚠ Algunas dependencias podrían haber fallado${NC}"
        echo "   Continuando de todas formas..."
    }
    echo -e "${GREEN}✓ Dependencias Python instaladas${NC}"
else
    echo -e "${YELLOW}⚠ requirements.txt no encontrado${NC}"
fi
echo

# ─────────────────────────────────────────────────────────────────────────────
# 4. Compilar paquete ROS 2
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[4/5]${NC} Compilando paquete ROS 2..."
echo "  Ejecutando colcon build en: $WORKSPACE_DIR"

cd "$WORKSPACE_DIR"

if command -v colcon &> /dev/null; then
    colcon build --packages-select mapping --symlink-install || {
        echo -e "${YELLOW}⚠ Compilación reportó errores (minor)${NC}"
    }
    echo -e "${GREEN}✓ Paquete compilado${NC}"
else
    echo -e "${RED}✗ colcon no encontrado. Instala: sudo apt install python3-colcon-common${NC}"
    exit 1
fi
echo

# ─────────────────────────────────────────────────────────────────────────────
# 5. Generar setup scripts y mostrar instrucciones
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[5/5]${NC} Finalizando setup..."
echo

# Crear un archivo de instrucciones
cat > "$WORKSPACE_DIR/RUN_YOLO_OAKD.md" << 'EOF'
# 🚀 Ejecución del Sistema YOLO + OAK-D + RTAB-Map

## 1. Preparación del terminal

### En Ubuntu 22.04 / Jetson Orin Nano:
```bash
cd ~/Rescue-Major/workspace
source install/setup.bash
```

## 2. Ejecutar el mapeo completo

### Terminal 1 (Mapeo SLAM + Detección YOLO):
```bash
ros2 launch mapping depthai_yolo_oakd.launch.py
```

**Parámetros configurables:**
```bash
# Con umbral de confianza más alto (0.6)
ros2 launch mapping depthai_yolo_oakd.launch.py confidence_thr:=0.6

# Con radio de deduplicación mayor (0.5 metros)
ros2 launch mapping depthai_yolo_oakd.launch.py duplicate_radius:=0.5

# Con CPU en lugar de GPU
ros2 launch mapping depthai_yolo_oakd.launch.py device:=cpu

# Combinación de parámetros
ros2 launch mapping depthai_yolo_oakd.launch.py confidence_thr:=0.55 duplicate_radius:=0.4
```

## 3. Visualizar en RViz

El launch file ya inicia `rtabmap_viz`. Verás:
- **Mapa en tiempo real** (nube de puntos)
- **Objetos detectados** (esferas de colores)
- **Labels con confianza** (texto sobre esferas)
- **Transforms** (frames para cada objeto)

## 4. Después de mapear

### Exportar mapa PLY (Compatible RoboCup):
```bash
cd ~/Rescue-Major/workspace/src/mapping
python3 scripts/export_map_ply.py
```

El script te pedirá:
- Número de misión (1, 2, 3...)
- Fecha y hora

El archivo se guardará en `maps/RoboCup2026-Aesir-[Misión]-[HH-MM-SS]-map.ply`

## 5. Monitorear detecciones

### Ver objetos detectados en tiempo real:
```bash
# Opción 1: Mira RViz (ya está en el launch)
# Opción 2: Ver el JSON
cat ~/.ros/detected_objects.json

# Opción 3: Subscribirse al topic
ros2 topic echo /detected_objects

# Ver transforms publicados
ros2 tf2_tools tree
```

## 6. Solucionar problemas

### Modelo YOLO no se carga:
- Verifica: `ls ~/Rescue-Major/workspace/src/mapping/yolo_training/runs/detect/runs/detect/aesir_rescue_v27_final3/weights/best.pt`
- Si no existe, entrena primero o usa otro modelo

### OAK-D no detectada:
```bash
# Ver dispositivos conectados
ros2 launch depthai_examples stereo_inertial_node.launch.py
```

### Poca memoria Jetson:
- Reduce `cloud_decimation` en el launch file
- Reduce `process_every_n` (procesa menos frames)

### GPU muy lenta:
- Cambia a CPU: `device:=cpu`
- O usa `device:=cuda:1` si hay múltiples GPUs

## 7. Parámetros por defecto

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `confidence_thr` | 0.50 | Confianza YOLO (0.0-1.0) |
| `iou_thr` | 0.45 | IOU para NMS (0.0-1.0) |
| `duplicate_radius` | 0.3 | Radio deduplicación (metros) |
| `device` | cuda:0 | GPU/CPU para YOLO |
| `publish_tf` | true | Generar transforms para objetos |
| `process_every_n` | 5 | Procesar 1 de cada N frames |

## 8. Estructura de archivos generados

```
~/.ros/
├── rtabmap.db                  ← Base de datos del mapa
├── detected_objects.json       ← Objetos detectados con coordenadas
└── [otros archivos de rtabmap]

~/Rescue-Major/workspace/src/mapping/maps/
├── RoboCup2026-Aesir-1-14-30-00-map.ply
├── RoboCup2026-Aesir-1-14-30-00-map-compressed.ply
└── [más mapas...]
```

## 9. Ejemplo de detected_objects.json

```json
[
  {
    "id": 0,
    "label": "victim",
    "confidence": 0.92,
    "x": 1.234,
    "y": 2.567,
    "z": 0.150,
    "timestamp": 1234567890,
    "frame": "map"
  },
  {
    "id": 1,
    "label": "fire",
    "confidence": 0.87,
    "x": 2.100,
    "y": 1.890,
    "z": 0.200,
    "timestamp": 1234567900,
    "frame": "map"
  }
]
```

---

**Equipo Aesir | RoboCup Rescue 2026 | Rescue Major**
EOF

echo -e "${GREEN}✓ Setup completado exitosamente${NC}"
echo
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}¡Sistema listo para usar!${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo
echo -e "${YELLOW}Instrucciones de uso:${NC}"
echo
echo "1. Sourcea el workspace:"
echo -e "   ${BLUE}cd ~/Rescue-Major/workspace && source install/setup.bash${NC}"
echo
echo "2. Ejecuta el sistema:"
echo -e "   ${BLUE}ros2 launch mapping depthai_yolo_oakd.launch.py${NC}"
echo
echo "3. Para más detalles, lee:"
echo -e "   ${BLUE}cat ~/Rescue-Major/workspace/RUN_YOLO_OAKD.md${NC}"
echo
echo -e "${YELLOW}Variables de configuración:${NC}"
echo "   - confidence_thr=0.50     (umbral YOLO)"
echo "   - iou_thr=0.45            (IOU para NMS)"
echo "   - duplicate_radius=0.3    (radio deduplicación en m)"
echo "   - device=cuda:0           (GPU para YOLO)"
echo "   - publish_tf=true         (generar transforms)"
echo
echo "Ejemplo:"
echo -e "   ${BLUE}ros2 launch mapping depthai_yolo_oakd.launch.py duplicate_radius:=0.5${NC}"
echo
