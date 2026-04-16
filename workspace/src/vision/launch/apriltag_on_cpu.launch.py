# launch/apriltag_cpu_detection.launch.py
import launch
from launch_ros.actions import Node

def generate_launch_description():
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='',
        remappings=[
            # The standard node subscribes to 'image_rect' rather than 'image'
            ('image_rect', '/oak/rgb/image_rect'),
            ('camera_info', '/oak/rgb/camera_info'),
        ],
        parameters=[{
            'image_transport': 'raw',
            'family': '36h11', # Standard tag family (note: no 'tag' prefix in this package)
            'size': 0.15,      # Size of the tag in meters
            'threads': 4       # Number of CPU threads to allocate (adjust based on your PC)
        }],
        output='screen'
    )

    return launch.LaunchDescription([apriltag_node])