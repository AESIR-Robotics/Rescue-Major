from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    video_node = Node(
        package='vision',               
        executable='video_stream_publisher',
        name='video_stream_publisher',
        output='screen'
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
