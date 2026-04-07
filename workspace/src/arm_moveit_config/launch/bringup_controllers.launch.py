import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('arm_moveit_config')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'demo.launch.py'))
    )

    servo_launch = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'servo_teleop.launch.py'))
        )]
    )

    spawn_position = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--inactive"],
        output="screen",
    )

    spawn_effort = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--inactive"],
        output="screen",
    )
    
    spawn_velocity = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--inactive"],
        output="screen",
    )

    return LaunchDescription([
        base_launch,
        spawn_position, 
        #spawn_effort,   
        spawn_velocity,
        servo_launch
    ])