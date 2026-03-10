import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    vision_dir = get_package_share_directory('vision')
    config_file_path = os.path.join(vision_dir, 'config', 'vision_cameras.yaml')

    video_node = Node(
        package='vision',               
        executable='video_stream_publisher',
        name='video_stream_publisher',
        output='screen',
        parameters=[config_file_path]
    )

    sensors_node = Node(
        package='vision',            
        executable='sensors_master.py', 
        name='sensors_master',
        output='screen'
    )

    return LaunchDescription([
        video_node,
        sensors_node
    ])
