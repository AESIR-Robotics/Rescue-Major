# depthai_test.launch.py — OAK-D + RTAB-Map + YOLO
# Optimizado para Jetson Orin Nano 8 GB / ROS 2 Humble / Ubuntu 22.04
#
# Arquitectura:
#   stereo_inertial_node  →  rgbd_sync  →  rgbd_odometry  →  rtabmap
#                                      ↘  imu_filter_madgwick  ↗
#   object_detection_node (YOLO) → publica marcadores 3D al mapa
#
# Usage:
#   ros2 launch rtabmap_examples depthai_test.launch.py camera_model:=OAK-D

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# ─────────────────────────────────────────────────────────────────────────────
# PARÁMETROS COMPARTIDOS  (odometría + SLAM)
# Calibrados para Jetson Orin Nano 8 GB:
#   • Orin Nano tiene 1 024 CUDA cores NVIDIA Ampere → se puede usar GPU
#   • 8 GB LPDDR5 memoria unificada CPU/GPU
#   • Throttle térmico ~25 W → limitar procesos paralelos pesados
# ─────────────────────────────────────────────────────────────────────────────

# Parámetros sólo para odometría (velocidad > calidad)
odom_parameters = [{
    'frame_id':             'oak-d-base-frame',
    'subscribe_rgbd':       True,
    'subscribe_odom_info':  True,
    'approx_sync':          False,
    'wait_imu_to_init':     True,

    # IMU – Madgwick estable, no predecir movimiento con IMU
    # (evita deriva en superficies texturizadas irregulares)
    'Odom/GuessMotion':     'false',

    # ── Features / Visual matching ──────────────────────────────────────
    # ORB es el más rápido en CPU; 200 features = ~12 ms en Orin Nano
    'Vis/FeatureType':      '6',        # ORB
    'Vis/MaxFeatures':      '250',      # Balance tracking/RAM (↓ de 500)
    'Vis/MinInliers':       '8',        # Mínimo decente para no perder odom
    'Vis/MinDepth':         '0.2',      # Mínimo OAK-D real
    'Vis/MaxDepth':         '5.0',      # Contenido a 5 m (arena interior)
    'GFTT/MinDistance':     '8',        # Features bien distribuidos

    # ── Decimación / Resolución ─────────────────────────────────────────
    # Decimation 2 → half-res para odom = ~4× menos píxeles a procesar
    'Odom/ImageDecimation': '2',
    'Odom/ResetCountdown':  '2',        # Reintentos antes de reset

    # ── Profundidad ────────────────────────────────────────────────────
    'Grid/RangeMin':        '0.3',
    'Grid/RangeMax':        '5.0',
    'Grid/DepthDecimation': '4',        # Submuestreo para grid 2-D interno

    # ── Memoria ────────────────────────────────────────────────────────
    'Mem/NotLinkedNodesKept': 'false',  # No guardar nodos sin loop closure
}]

# Parámetros sólo para SLAM (calidad > velocidad — puede ser más lento)
slam_parameters = [{
    'frame_id':             'oak-d-base-frame',
    'subscribe_rgbd':       True,
    'subscribe_odom_info':  True,
    'approx_sync':          True,       # SLAM puede tolerar aprox sync
    'wait_imu_to_init':     True,

    'Odom/GuessMotion':     'false',

    # ── Detección SLAM: 1 Hz para reducir carga (suficiente para interior) ──
    # La odometría corre a ~10 Hz internamente; SLAM sólo añade keyframes a 1 Hz
    'Rtabmap/DetectionRate':     '1.0',
    'Rtabmap/TimeThr':           '700',   # ms máx por iteración SLAM

    # ── Mapa 3-D (nube de puntos para PLY export) ───────────────────────
    # Calidad moderada-alta: voxel 5 cm para arena de rescate
    'cloud_decimation':     2,
    'cloud_max_depth':      5.0,
    'cloud_voxel_size':     0.05,       # 5 cm → buena resolución para score

    # ── Grid / Obstacle map ────────────────────────────────────────────
    'Grid/RangeMin':             '0.3',
    'Grid/RangeMax':             '5.0',
    'Grid/DepthDecimation':      '4',
    'Grid/MaxGroundHeight':      '0.1',
    'Grid/MaxObstacleHeight':    '2.0',
    'Grid/NormalsSegmentation':  'false',

    # ── Colores (bonus point multiplier en RoboCup Rescue) ──────────────
    # Activa nube coloreada usando canal derecho (escala de grises → RGB fake)
    # El nodo de YOLO añade labels con color semántico
    'Grid/3D':                   'true',
    'RGBD/ProximityBySpace':     'true',
    'RGBD/LinearUpdate':         '0.1',
    'RGBD/AngularUpdate':        '0.05',

    # ── Loop closure ──────────────────────────────────────────────────
    'Kp/MaxFeatures':       '400',
    'Kp/DetectorStrategy':  '6',        # ORB
    'Vis/MinInliers':       '10',
    'Vis/MinDepth':         '0.2',
    'Vis/MaxDepth':         '5.0',

    # ── Memoria ────────────────────────────────────────────────────────
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize':            '30',     # Nodos en memoria de corto plazo
    'Mem/ImageKept':          'false',  # No guardar imágenes en DB (ahorra RAM)

    # ── Base de datos ─────────────────────────────────────────────────
    # Se guarda en home del usuario; se usa para export PLY después
    'database_path':        '~/.ros/rtabmap.db',
}]

# Remapping IMU (igual para todos los nodos que lo necesitan)
imu_remap = [('imu', '/imu/data')]


# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription([

        # ── 1. Driver cámara OAK-D ────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py'
            ]),
            launch_arguments={
                'depth_aligned':    'false',    # Usar stereo nativo (más estable)
                'enableRviz':       'false',
                'monoResolution':   '400p',     # 400p: ~720×400, buen balance
                'fps':              '15',       # 15 fps suficiente, menos carga USB
            }.items(),
        ),

        # ── 2. Sincronizador RGBD ─────────────────────────────────────────
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{
                'approx_sync':          False,
                'approx_sync_max_interval': 0.0,
                'queue_size':           10,
            }],
            remappings=[
                ('rgb/image',       '/right/image_rect'),
                ('rgb/camera_info', '/right/camera_info'),
                ('depth/image',     '/stereo/depth'),
            ]
        ),

        # ── 3. Filtro Madgwick (IMU) ──────────────────────────────────────
        # gain bajo = suavizado agresivo, menos sensible a vibraciones del robot
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag':          False,
                'world_frame':      'enu',
                'publish_tf':       False,
                'gain':             0.01,
                'zeta':             0.01,
                'use_magnetic_field_msg': False,
            }],
            remappings=[('imu/data_raw', '/imu')]
        ),

        # ── 4. Odometría visual + IMU ─────────────────────────────────────
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=odom_parameters,
            remappings=imu_remap
        ),

        # ── 5. SLAM principal ─────────────────────────────────────────────
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=slam_parameters,
            remappings=imu_remap,
            arguments=['-d']            # Borrar DB anterior al iniciar
        ),

        # ── 6. Detección de objetos YOLO + coordenadas 3D ────────────────
        # Nodo personalizado (ver object_detection_node.py)
        # Suscribe: /right/image_rect, /stereo/depth, /right/camera_info
        # Publica:  /detected_objects (MarkerArray), /detected_objects_list
        Node(
            package='mapping',
            executable='object_detection_node',
            name='object_detection',
            output='screen',
            parameters=[{
                'model_path':       '/home/aesir/Rescue-Major/workspace/src/mapping/yolo_training/runs/detect/runs/detect/aesir_rescue_v27_final3/weights/best.pt',   # Modelo YOLO entrenado
                'confidence_thr':   0.50,
                'iou_thr':          0.45,
                'device':           'cuda:0',   # GPU Jetson
                'save_path':        '~/.ros/detected_objects.json',
                'map_frame':        'map',
                'camera_frame':     'oak-d-base-frame',
                'process_every_n':  5,          # Procesar 1 de cada N frames
            }]
        ),

        # ── 7. Visualizador ───────────────────────────────────────────────
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=slam_parameters,
            remappings=imu_remap
        ),
    ])
