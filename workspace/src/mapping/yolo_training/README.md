# YOLOv8 Training Setup para RoboCup Rescue 2026

## 🎯 Objetivo
Entrenar un detector de objetos YOLOv8 custom para identificar víctimas, peligros y opciones de rescate usando cámara OAK-D.

## 📁 Estructura
```
yolo_training/
├── configs/               # Configuraciones
│   └── dataset.yaml      # Estructura del dataset
├── datasets/             # Datos
│   └── rescue_dataset/   # Tu dataset (crear esta carpeta)
│       ├── images/
│       │   ├── train/    # 70% imágenes
│       │   ├── val/      # 15% imágenes
│       │   └── test/     # 15% imágenes (opcional)
│       └── labels/       # Anotaciones YOLO .txt
│           ├── train/
│           ├── val/
│           └── test/
├── models/               # Modelos
│   └── best.pt          # Modelo entrenado
├── runs/                # Outputs
│   └── detect/
│       └── train_yolov8n/
├── train_yolo.py        # Script entrenamiento
├── inference_oakd.py    # Inferencia en tiempo real
├── prepare_dataset.py   # Preparador de dataset
└── README.md            # Este archivo
```

## ⚙️ Instalación inicial

### 1. Instalar todas las dependencias
```bash
cd ~/Rescue-Major/workspace/src/mapping
pip install -r requirements.txt --break-system-packages
```

### 2. Permisos de ejecución
```bash
chmod +x yolo_training/*.py
chmod +x setup_mapping.sh
```

---

## 📊 Preparar Dataset

### Opción A: Ya tienes imágenes + etiquetas YOLO
```bash
# 1. Coloca tus imágenes en: yolo_training/datasets/rescue_dataset/images/train/
# 2. Coloca tus etiquetas .txt en: yolo_training/datasets/rescue_dataset/labels/train/
# 3. Divide automáticamente:
python3 yolo_training/prepare_dataset.py \
  --input yolo_training/datasets/rescue_dataset \
  --format yolo \
  --split 0.7 0.15 0.15
```

### Opción B: Paso a paso manual
```bash
# Crear carpetas
mkdir -p yolo_training/datasets/rescue_dataset/{images/{train,val,test},labels/{train,val,test}}

# Copiar tus imágenes a train/ (luego el script divide en val/test)
cp /tu/dataset/images/* yolo_training/datasets/rescue_dataset/images/train/
cp /tu/dataset/labels/* yolo_training/datasets/rescue_dataset/labels/train/

# Dividir en train/val/test
python3 yolo_training/prepare_dataset.py \
  --input yolo_training/datasets/rescue_dataset \
  --format yolo
```

### ⚠️ Formato de anotaciones YOLO
Cada imagen `image.jpg` debe tener un archivo `image.txt` con una línea por objeto detectado:

```
<class_id> <x_center> <y_center> <width> <height>
```

**Ejemplo:**
```
0 0.5 0.5 0.3 0.4
1 0.2 0.7 0.15 0.2
```

Donde:
- `0` = clase "victim" (según configs/dataset.yaml)
- `0.5` = posición X normalizada (0-1)
- `0.5` = posición Y normalizada (0-1)
- `0.3` = ancho normalizado
- `0.4` = alto normalizado

**Herramientas para anotar:**
- **LabelImg**: `pip install labelimg`
- **Roboflow**: https://roboflow.com (exportar en formato YOLO)

---

## 🚀 Entrenar el Modelo

### Entrenamiento básico (recomendado para Jetson)
```bash
cd ~/Rescue-Major/workspace/src/mapping

python3 yolo_training/train_yolo.py \
  --config yolo_training/configs/dataset.yaml \
  --model n \
  --epochs 100 \
  --batch 16 \
  --device 0
```

### Parámetros importantes
```bash
# Tamaño modelo: n (nano) < s (small) < m (medium) < l (large) < x (xlarge)
# Para Jetson: --model n  (más rápido)

# Épocas: más = mejor pero más lento
# Batch: más = menos ruido pero más memoria

# Device:
#   --device 0     (primera GPU)
#   --device cpu   (CPU, lento)
#   --device '0,1' (múltiples GPUs)
```

### Monitorear entrenamiento
```bash
# TensorBoard
tensorboard --logdir runs/detect/train_yolov8n/
```

**Salida esperada:**
```
Epoch 1/100: 100%|██████████| 35/35 [00:12<00:00, 2.85it/s]
  Class all: 100% validation: mAP50=0.523 loss=0.412
```

---

## 🎥 Inferencia en Tiempo Real

### Con OAK-D
```bash
python3 yolo_training/inference_oakd.py \
  --model yolo_training/runs/detect/train_yolov8n/weights/best.pt \
  --conf 0.5
```

**Controles:**
- `+` : Aumentar umbral de confianza
- `-` : Disminuir umbral de confianza
- `q` : Salir

### Desde Python
```python
from ultralytics import YOLO

# Cargar modelo
model = YOLO('yolo_training/runs/detect/train_yolov8n/weights/best.pt')

# Predecir
results = model.predict(source='image.jpg', conf=0.5)

# Acceder detecciones
for r in results:
    boxes = r.boxes.xyxy     # [x1, y1, x2, y2]
    confs = r.boxes.conf     # Confianzas
    classes = r.boxes.cls    # Clases
```

---

## 📈 Clases disponibles

Editar `configs/dataset.yaml`:

```yaml
nc: 6  # Número de clases

names:
  0: victim
  1: hazmat
  2: fire
  3: smoke
  4: door
  5: rubble
```

---

## 🔧 Troubleshooting

### Error: "CUDA out of memory"
```bash
# Reducir batch size
python3 yolo_training/train_yolo.py --batch 8

# O usar CPU
python3 yolo_training/train_yolo.py --device cpu
```

### Error: "Dataset not found"
```bash
# Verificar estructura
ls yolo_training/datasets/rescue_dataset/images/train/
ls yolo_training/datasets/rescue_dataset/labels/train/

# Deben tener el mismo número de archivos .jpg/.png y .txt
```

### Modelo muy lento en Jetson
```bash
# Usar modelo nano (más pequeño)
python3 yolo_training/train_yolo.py --model n --epochs 50

# O exportar a TensorRT
python3 -c "from ultralytics import YOLO; m=YOLO('best.pt'); m.export(format='engine')"
```

---

## 📚 Referencias

- **YOLOv8 Docs**: https://docs.ultralytics.com
- **Dataset Format**: https://docs.ultralytics.com/datasets/detect/
- **Roboflow**: https://roboflow.com/ (herramienta web para anotar)
- **OAK-D**: https://oak-d.org/

---

## ✅ Checklist de entrenamiento

- [ ] Dataset descargado y colocado en `yolo_training/datasets/rescue_dataset/`
- [ ] Images y labels verificados (mismo número de archivos)
- [ ] `configs/dataset.yaml` actualizado con clases correctas
- [ ] `setup_mapping.sh` ejecutado (instalar dependencias)
- [ ] Primer entrenamiento con 10 épocas para probar
- [ ] OAK-D conectada y funcionando
- [ ] Inferencia en tiempo real probada
- [ ] Modelo guardado en `yolo_training/models/`

---

## 🎓 Ejemplo completo

```bash
# 1. Preparar ambiente
cd ~/Rescue-Major/workspace/src/mapping
pip install -r requirements.txt --break-system-packages

# 2. Preparar dataset (ejemplo)
mkdir -p yolo_training/datasets/rescue_dataset/{images/{train,val,test},labels/{train,val,test}}
# ... copiar tus imágenes y etiquetas ...

# 3. Entrenar (primero con pocas épocas para probar)
python3 yolo_training/train_yolo.py --epochs 10 --batch 16

# 4. Con OAK-D: probar inferencia
python3 yolo_training/inference_oakd.py \
  --model yolo_training/runs/detect/train_yolov8n/weights/best.pt \
  --conf 0.5

# 5. Entrenar definitivamente
python3 yolo_training/train_yolo.py --epochs 100 --batch 16

# 6. Copiar modelo final
cp yolo_training/runs/detect/train_yolov8n/weights/best.pt \
   yolo_training/models/best.pt
```

---

**Equipo Aesir - RoboCup Rescue 2026**
