import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'aesir_robot_description' 
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'rescue_robot_foxglove.urdf.xacro')

    robot_description_content = xacro.process_file(urdf_file).toxml()
    robot_description = {
        'robot_description': robot_description_content
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Open RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        #node_rviz
    ])