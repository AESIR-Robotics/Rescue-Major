import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    try:
        rviz_config_dir = os.path.join(
            get_package_share_directory('unitree_lidar_ros2'),
            'view.rviz'
        )
    except:
        rviz_config_dir = ""

    res_folder = '/home/aesir/Rescue-Major/workspace/src/mapping/yolo_training/runs/detect/runs/detect/aesir_rescue_v27_final3/weights'

    # ── NOTA DE DISEÑO ───────────────────────────────────────────────────────────
    # frame_id = 'unilidar_lidar' en icp_odometry y rtabmap.
    # Usar 'base_link' como frame_id requiere que el TF base_link→unilidar_lidar
    # esté disponible en el timestamp exacto de cada nube, lo cual causa el error
    # "Lookup would require extrapolation" porque la TF estática solo tiene un
    # timestamp (el del arranque). Con unilidar_lidar como frame_id nativo,
    # ICP opera directamente en el frame del sensor y ese problema desaparece.
    # base_link sigue existiendo en el árbol para RViz y navegación.
    # ─────────────────────────────────────────────────────────────────────────────

    icp_parameters = {
        'Reg/Strategy': '1',
        'Icp/VoxelSize': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Odom/GuessMotion': 'true',
        'OdomF2M/ScanMaxSize': '10000',
        'Icp/MaxTranslation': '0.5',
        'Icp/MaxRotation': '0.78',
        'Odom/ResetCountdown': '1',
        'Odom/ExpectedUpdateRate': '20.0',
    }

    return LaunchDescription([

        # ── 1. SENSORES ──────────────────────────────────────────────────────────

        # 1A. Unitree L2 LiDAR
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen'
        ),

        # 1B. OAK-D — YOLO Espacial
        # sync_nn=False: evita que el bloqueo USB del NN bloquee el resto del pipeline
        Node(
            package='depthai_examples',
            executable='yolov4_spatial_node',
            name='yolov4_spatial_node',
            output='screen',
            parameters=[{
                'camera_model':       'OAK-D',
                'tf_prefix':          'oak',
                'sync_nn':            False,
                'resourceBaseFolder': res_folder,
                'nnName':             'best.blob',
                'nnConfig':           os.path.join(res_folder, 'best.json'),
            }],
            remappings=[
                ('color/image',       '/color/video/image'),
                ('color/camera_info', '/color/video/camera_info')
            ]
        ),

        # ── 2. TF ESTÁTICAS (new-style args para Humble) ─────────────────────────

        # base_link → unilidar_lidar (para RViz y navegación)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'unilidar_lidar'
            ]
        ),

        # unilidar_lidar → oak_rgb_camera_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_to_camera',
            arguments=[
                '--x', '0.1', '--y', '0.0', '--z', '0.05',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'unilidar_lidar',
                '--child-frame-id', 'oak_rgb_camera_optical_frame'
            ]
        ),

        # ── 3. ODOMETRÍA ICP ─────────────────────────────────────────────────────
        # frame_id = unilidar_lidar → el sensor ES el frame base del robot aquí.
        # Evita el error de extrapolación TF que ocurría con base_link.
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id':             'unilidar_lidar',   # ← clave del fix
                'odom_frame_id':        'odom',
                'wait_for_transform':   0.2,
                'expected_update_rate': 20.0,
                **icp_parameters
            }],
            remappings=[
                ('scan_cloud', '/unilidar/cloud'),
                ('imu',        '/unilidar/imu')
            ]
        ),

        # ── 4. RTAB-Map SLAM ─────────────────────────────────────────────────────
        # Modo: LiDAR-only + landmarks de YOLO via /landmark_detections
        # Sin RGB/Depth para eliminar el problema de sync de 3 topics.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id':              'unilidar_lidar',  # ← consistente con icp_odometry
                'subscribe_scan_cloud':  True,
                'subscribe_odom':        True,
                'subscribe_rgb':         False,
                'subscribe_depth':       False,
                'approx_sync':           True,
                'sync_queue_size':       30,
                'topic_queue_size':      30,
                'Grid/Sensor':           '0',
                'Grid/MaxObstacleHeight':'2.0',
                'RGBD/OptimizeLandmarks':'true',  # optimizar posición de landmarks
                **icp_parameters
            }],
            arguments=['-d'],
            remappings=[
                ('scan_cloud',          '/unilidar/cloud'),
                ('odom',                '/odom'),
                ('landmark_detections', '/landmark_detections')
            ]
        ),

        # ── 5. LANDMARK TF PUBLISHER ─────────────────────────────────────────────
        # Escucha /color/yolov4_Spatial_detections y /unilidar/cloud.
        # Publica TF lm_<clase>_<id> y LandmarkDetections para RTAB-Map.
        Node(
            package='mapping',
            executable='landmark_tf_publisher.py',
            name='landmark_tf_publisher',
            output='screen'
        ),

        # ── 6. RTAB-Map Viz ──────────────────────────────────────────────────────
        # Solo scan_cloud + odom. Sin RGB/Depth para evitar el crash exit 255.
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id':             'unilidar_lidar',
                'subscribe_scan_cloud': True,
                'subscribe_odom':       True,
                'subscribe_rgb':        False,
                'subscribe_depth':      False,
                'approx_sync':          True,
                'sync_queue_size':      30,
                'topic_queue_size':     30,
            }],
            remappings=[
                ('scan_cloud', '/unilidar/cloud'),
                ('odom',       '/odom')
            ]
        ),

        # ── 7. RViz2 ─────────────────────────────────────────────────────────────
        # Agregar en tu config:
        #   MarkerArray → /yolo_landmark_markers  (landmarks YOLO)
        #   PointCloud2 → /unilidar/cloud
        #   OccupancyGrid → /map
        #   TF → activar todos (verás lm_<clase>_<id>)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir] if rviz_config_dir else [],
            output='screen'
        ),

        # ── 8. Visor de Cámara YOLO en tiempo real ───────────────────────────────
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='visor_yolo',
            arguments=['/color/video/image']
        )
    ])