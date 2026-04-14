import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    vision_dir = get_package_share_directory('hardware')
    config_file_path = os.path.join(vision_dir, 'config', 'hardware.yaml')

    hardware_node = Node(
        package='hardware',               
        executable='dc_motors',
        name='dc_motors',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        hardware_node,
    ])