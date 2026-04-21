import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    res_folder = '/home/aesir/AESIR/mapping/workspace/src/mapping/yolo_training/runs/aesir_v23'

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

        # 1A. Unitree L2 LiDAR - VALORES DE FÁBRICA
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters=[
                {'lidar_ip': '192.168.50.4'} # <-- IP de fábrica corregida
            ]
        ),

        # 1B. OAK-D — YOLO Espacial
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

        # ── 2. TF ESTÁTICAS ──────────────────────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_to_base',
            arguments=[
                '--x', '-0.12', '--y', '0.0', '--z', '-0.12',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'unilidar_lidar',
                '--child-frame-id', 'base_link'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_to_camera',
            arguments=[
                '--x', '0.08', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'unilidar_lidar',
                '--child-frame-id', 'oak_rgb_camera_optical_frame'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'unilidar_lidar', 'unilidar_imu']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_to_imu_initial',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'unilidar_lidar', 'unilidar_imu_initial']
        ),

        # ── 3. ODOMETRÍA ICP ─────────────────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id':             'unilidar_lidar',
                'odom_frame_id':        'odom',
                'wait_for_transform':   0.2,
                'expected_update_rate': 20.0,
                'qos': 1,
                'qos_scan': 1,
                **icp_parameters
            }],
            remappings=[
                ('scan_cloud', '/unilidar/cloud'),
                ('imu',        '/unilidar/imu')
            ]
        ),

        # ── 4. RTAB-Map SLAM ─────────────────────────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id':              'unilidar_lidar',
                'subscribe_scan_cloud':  True,
                'subscribe_odom':        True,
                'subscribe_rgb':         False,
                'subscribe_depth':       False,
                'approx_sync':           True,
                'sync_queue_size':       30,
                'topic_queue_size':      30,
                'qos': 1,
                'qos_scan': 1,
                'Grid/Sensor':           '0',
                'Grid/MaxObstacleHeight':'2.0',
                'RGBD/OptimizeLandmarks':'true',
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
        Node(
            package='mapping',
            executable='landmark_tf_publisher.py',
            name='landmark_tf_publisher',
            output='screen'
        )
    ])