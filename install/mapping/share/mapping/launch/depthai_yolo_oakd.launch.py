# depthai_yolo_oakd.launch.py — OAK-D + RTAB-Map + YOLOv8 (Aesir Rescue v27 final3)
# Optimizado para Jetson Orin Nano 8 GB / ROS 2 Humble / Ubuntu 22.04
#
# Arquitectura:
#   stereo_inertial_node  →  rgbd_sync  →  rgbd_odometry  →  rtabmap
#                                      ↘  imu_filter_madgwick  ↗
#   object_detection_node (YOLOv8)    → genera TFs + marcadores 3D + JSON
#
# Características:
#  ✓ Modelo YOLO: aesir_rescue_v27_final3
#  ✓ Deduplicación de objetos con radio configurable
#  ✓ Generación de transforms (TF) para cada objeto
#  ✓ Publicación de marcadores RViz con colores por clase
#  ✓ Guardado persistente en JSON
#
# Usage:
#   ros2 launch mapping depthai_yolo_oakd.launch.py
#
# Para cambiar parámetros:
#   ros2 launch mapping depthai_yolo_oakd.launch.py duplicate_radius:=0.5 confidence_thr:=0.6

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ─────────────────────────────────────────────────────────────────────────────
# PARÁMETROS COMPARTIDOS  (odometría + SLAM)
# Calibrados para Jetson Orin Nano 8 GB
# ─────────────────────────────────────────────────────────────────────────────

# Parámetros sólo para odometría (velocidad > calidad)
odom_parameters = [{
    'frame_id':             'oak-d-base-frame',
    'subscribe_rgbd':       True,
    'subscribe_odom_info':  True,
    'approx_sync':          False,
    'wait_imu_to_init':     True,

    'Odom/GuessMotion':     'false',

    # ── Features / Visual matching ──────────────────────────────────────
    'Vis/FeatureType':      '6',        # ORB
    'Vis/MaxFeatures':      '250',
    'Vis/MinInliers':       '8',
    'Vis/MinDepth':         '0.2',
    'Vis/MaxDepth':         '5.0',
    'GFTT/MinDistance':     '8',

    # ── Decimación / Resolución ─────────────────────────────────────────
    'Odom/ImageDecimation': '2',
    'Odom/ResetCountdown':  '2',

    # ── Profundidad ────────────────────────────────────────────────────
    'Grid/RangeMin':        '0.3',
    'Grid/RangeMax':        '5.0',
    'Grid/DepthDecimation': '4',

    # ── Memoria ────────────────────────────────────────────────────────
    'Mem/NotLinkedNodesKept': 'false',
}]

# Parámetros sólo para SLAM (calidad > velocidad)
slam_parameters = [{
    'frame_id':             'oak-d-base-frame',
    'subscribe_rgbd':       True,
    'subscribe_odom_info':  True,
    'approx_sync':          True,
    'wait_imu_to_init':     True,

    'Odom/GuessMotion':     'false',

    # ── Detección SLAM ──────────────────────────────────────────────────
    'Rtabmap/DetectionRate':     '1.0',
    'Rtabmap/TimeThr':           '700',

    # ── Mapa 3-D (nube de puntos para PLY export) ───────────────────────
    'cloud_decimation':     2,
    'cloud_max_depth':      5.0,
    'cloud_voxel_size':     0.05,

    # ── Grid / Obstacle map ────────────────────────────────────────────
    'Grid/RangeMin':             '0.3',
    'Grid/RangeMax':             '5.0',
    'Grid/DepthDecimation':      '4',
    'Grid/MaxGroundHeight':      '0.1',
    'Grid/MaxObstacleHeight':    '2.0',
    'Grid/NormalsSegmentation':  'false',

    # ── Colores (RGB Profundity) ──────────────────────────────────────
    'Grid/3D':                   'true',
    'RGBD/ProximityBySpace':     'true',
    'RGBD/LinearUpdate':         '0.1',
    'RGBD/AngularUpdate':        '0.05',

    # ── Loop closure ──────────────────────────────────────────────────
    'Kp/MaxFeatures':       '400',
    'Kp/DetectorStrategy':  '6',
    'Vis/MinInliers':       '10',
    'Vis/MinDepth':         '0.2',
    'Vis/MaxDepth':         '5.0',

    # ── Memoria ────────────────────────────────────────────────────────
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize':            '30',
    'Mem/ImageKept':          'false',

    # ── Base de datos ─────────────────────────────────────────────────
    'database_path':        '~/.ros/rtabmap.db',
}]

# Remapping IMU
imu_remap = [('imu', '/imu/data')]


# ─────────────────────────────────────────────────────────────────────────────
# RUTA AL MODELO YOLO (Aesir Rescue v27 final3)
# ─────────────────────────────────────────────────────────────────────────────
# Construir ruta desde la carpeta del launch file
LAUNCH_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
MAPPING_DIR = os.path.dirname(LAUNCH_FILE_DIR)
YOLO_MODEL_PATH = os.path.join(
    MAPPING_DIR,
    'yolo_training/runs/detect/runs/detect/aesir_rescue_v27_final3/weights/best.pt'
)


# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    # ── Argumentos configurables ──────────────────────────────────────────
    declare_confidence_thr = DeclareLaunchArgument(
        'confidence_thr', default_value='0.50',
        description='Umbral de confianza YOLO (0.0-1.0)'
    )
    declare_iou_thr = DeclareLaunchArgument(
        'iou_thr', default_value='0.45',
        description='Umbral IOU para NMS YOLO (0.0-1.0)'
    )
    declare_duplicate_radius = DeclareLaunchArgument(
        'duplicate_radius', default_value='0.3',
        description='Radio de deduplicación en metros'
    )
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Publicar transforms para objetos'
    )
    declare_device = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='Dispositivo: cuda:0, cpu, etc.'
    )

    return LaunchDescription([
        declare_confidence_thr,
        declare_iou_thr,
        declare_duplicate_radius,
        declare_publish_tf,
        declare_device,

        # ── 1. Driver cámara OAK-D ────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py'
            ]),
            launch_arguments={
                'depth_aligned':    'false',
                'enableRviz':       'false',
                'monoResolution':   '400p',
                'fps':              '15',
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
            arguments=['-d']
        ),

        # ── 6. Detección de objetos YOLOv8 + transforms ─────────────────
        # Modelo: aesir_rescue_v27_final3
        # Características:
        #  • Genera transforms (TF) en el frame 'map'
        #  • Deduplicación configurable
        #  • Markers para visualizar en RViz
        #  • JSON persistente para export PLY
        Node(
            package='mapping',
            executable='object_detection_node',
            name='object_detection',
            output='screen',
            parameters=[{
                'model_path':           YOLO_MODEL_PATH,
                'confidence_thr':       LaunchConfiguration('confidence_thr'),
                'iou_thr':              LaunchConfiguration('iou_thr'),
                'device':               LaunchConfiguration('device'),
                'save_path':            '~/.ros/detected_objects.json',
                'map_frame':            'map',
                'camera_frame':         'oak-d-base-frame',
                'process_every_n':      5,
                'duplicate_radius':     LaunchConfiguration('duplicate_radius'),
                'publish_tf':           LaunchConfiguration('publish_tf'),
            }]
        ),

        # ── 7. Visualizador RtabMap ──────────────────────────────────────
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=slam_parameters,
            remappings=imu_remap
        ),
    ])
