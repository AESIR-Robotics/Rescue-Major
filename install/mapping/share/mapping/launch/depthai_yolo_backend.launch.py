# depthai_yolo_backend.launch.py — OAK-D + YOLO + RTAB-Map (Sin visualizadores)
# Versión para desarrollo/debugging
#
# Este launch file inicia SOLO el backend:
#  • OAK-D (cámara + IMU)
#  • YOLO Object Detection (con TF generation)
#  • RTAB-Map SLAM
#
# NO inicia visualizadores. Abre en terminales separadas:
#  • Terminal 1: ros2 launch mapping depthai_yolo_backend.launch.py
#  • Terminal 2: python3 src/mapping/scripts/yolo_detection_viewer.py
#  • Terminal 3: python3 src/mapping/scripts/yolo_diagnostics.py
#  • Terminal 4: rviz2 -d src/mapping/rviz_config.rviz (opcional)
#
# Ventajas:
#  ✓ Mejor control sobre qué ventanas abrir
#  ✓ Fácil debuggear con múltiples terminales
#  ✓ Menos carga si no necesitas viz
#  ✓ Puedes reabrir viewers sin reiniciar SLAM

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ─────────────────────────────────────────────────────────────────────────────
# PARÁMETROS (igual que depthai_yolo_oakd.launch.py)
# ─────────────────────────────────────────────────────────────────────────────

odom_parameters = [{
    'frame_id':             'oak-d-base-frame',
    'subscribe_rgbd':       True,
    'subscribe_odom_info':  True,
    'approx_sync':          False,
    'wait_imu_to_init':     True,
    'Odom/GuessMotion':     'false',
    'Vis/FeatureType':      '6',        # ORB
    'Vis/MaxFeatures':      '250',
    'Vis/MinInliers':       '8',
    'Vis/MinDepth':         '0.2',
    'Vis/MaxDepth':         '5.0',
    'GFTT/MinDistance':     '8',
    'Odom/ImageDecimation': '2',
    'Odom/ResetCountdown':  '2',
    'Grid/RangeMin':        '0.3',
    'Grid/RangeMax':        '5.0',
    'Grid/DepthDecimation': '4',
    'Mem/NotLinkedNodesKept': 'false',
}]

slam_parameters = [{
    'frame_id':             'oak-d-base-frame',
    'subscribe_rgbd':       True,
    'subscribe_odom_info':  True,
    'approx_sync':          True,
    'wait_imu_to_init':     True,
    'Odom/GuessMotion':     'false',
    'Rtabmap/DetectionRate':     '1.0',
    'Rtabmap/TimeThr':           '700',
    'cloud_decimation':     2,
    'cloud_max_depth':      5.0,
    'cloud_voxel_size':     0.05,
    'Grid/RangeMin':             '0.3',
    'Grid/RangeMax':             '5.0',
    'Grid/DepthDecimation':      '4',
    'Grid/MaxGroundHeight':      '0.1',
    'Grid/MaxObstacleHeight':    '2.0',
    'Grid/NormalsSegmentation':  'false',
    'Grid/3D':                   'true',
    'RGBD/ProximityBySpace':     'true',
    'RGBD/LinearUpdate':         '0.1',
    'RGBD/AngularUpdate':        '0.05',
    'Kp/MaxFeatures':       '400',
    'Kp/DetectorStrategy':  '6',
    'Vis/MinInliers':       '10',
    'Vis/MinDepth':         '0.2',
    'Vis/MaxDepth':         '5.0',
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize':            '30',
    'Mem/ImageKept':          'false',
    'database_path':        '~/.ros/rtabmap.db',
}]

imu_remap = [('imu', '/imu/data')]

# Ruta al modelo YOLO
LAUNCH_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
MAPPING_DIR = os.path.dirname(LAUNCH_FILE_DIR)
YOLO_MODEL_PATH = os.path.join(
    MAPPING_DIR,
    'yolo_training/runs/detect/runs/detect/aesir_rescue_v27_final3/weights/best.pt'
)


# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    declare_confidence_thr = DeclareLaunchArgument(
        'confidence_thr', default_value='0.50',
        description='Umbral de confianza YOLO'
    )
    declare_iou_thr = DeclareLaunchArgument(
        'iou_thr', default_value='0.45',
        description='Umbral IOU para NMS'
    )
    declare_duplicate_radius = DeclareLaunchArgument(
        'duplicate_radius', default_value='0.3',
        description='Radio de deduplicación (metros)'
    )
    declare_device = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='Dispositivo: cuda:0, cpu, etc.'
    )

    return LaunchDescription([
        declare_confidence_thr,
        declare_iou_thr,
        declare_duplicate_radius,
        declare_device,

        # ── 1. Driver OAK-D ───────────────────────────────────────────────
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

        # ── 4. Odometría visual ───────────────────────────────────────────
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=odom_parameters,
            remappings=imu_remap
        ),

        # ── 5. SLAM ───────────────────────────────────────────────────────
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=slam_parameters,
            remappings=imu_remap,
            arguments=['-d']
        ),

        # ── 6. YOLO Object Detection ──────────────────────────────────────
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
                'publish_tf':           True,
            }]
        ),
        
        # ✓ SIN VISUALIZADORES (abre en terminales separadas)
    ])
