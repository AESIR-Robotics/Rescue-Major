from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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
        'Icp/VoxelSize': '0.2',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Odom/GuessMotion': 'true',
        'OdomF2M/ScanMaxSize': '10000',
        'Icp/MaxTranslation': '0.5',
        'Icp/MaxRotation': '0.78',
        'Odom/ResetCountdown': '1',
        'Odom/ExpectedUpdateRate': '20.0',
        'Icp/RangeMin': '1.00',
    }

    return LaunchDescription([

        # ── 1. SENSORES ──────────────────────────────────────────────────────────

        # 1A. Unitree L2 LiDAR
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters=[
                {'lidar_ip': '192.168.50.4'}
            ],
            remappings=[
                ('/tf', '/tf_basura'),
                ('/tf_static', '/tf_static_basura')
            ]
        ),
        
        
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': 0.15,   # Ignora el suelo y los topes de 10cm
                'max_height': 1.0,    # Ignora el techo (ajusta según la altura de tu laberinto)
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'range_min': 0.70,    # <-- ¡AQUÍ ESTÁ LA MAGIA! Ignora 40cm alrededor (tu propio chasis/brazo)
                'range_max': 20.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/unilidar/cloud'),
                ('scan', '/scan')
            ]
        ),

        # ── 2. TF ESTÁTICAS (new-style args para Humble) ─────────────────────────
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_to_base', 
            arguments=[
                '--x', '0.19616', '--y', '0.0', '--z', '0.17716',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '1.5708',
                '--frame-id', 'base_link',
                '--child-frame-id', 'unilidar_lidar'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_to_imu',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'unilidar_lidar',
                '--child-frame-id', 'unilidar_imu'
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
                'frame_id':             'base_link', 
                'odom_frame_id':        'odom',
                'wait_for_transform':   0.5,
                'expected_update_rate': 20.0,
                **icp_parameters
            }],
            remappings=[
                ('scan_cloud', '/unilidar/cloud'),
                ('imu',        '/unilidar/imu')
            ]
        ),

        # ── 4. RTAB-Map SLAM ─────────────────────────────────────────────────────
        # Modo: LiDAR-only
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id':              'base_link',  # ← consistente con icp_odometry
                'subscribe_scan_cloud':  True,
                'subscribe_odom':        True,
                'subscribe_rgb':         False,
                'subscribe_depth':       False,
                'approx_sync':           True,
                'sync_queue_size':       30,
                'topic_queue_size':      30,
                
                'Grid/RayTracing': 'true',
                'GridGlobal/OccupancyThr': '0.65',
                'Grid/NoiseFilteringRadius': '0.2',
                'Grid/NoiseFilteringMinNeighbors': '5',
                
                'Grid/Sensor':           '0',
                'Grid/MaxObstacleHeight':'2.0',
                **icp_parameters
            }],
            arguments=['-d'],
            remappings=[
                ('scan_cloud',          '/unilidar/cloud'),
                ('odom',                '/odom')
            ]
        )
    ])