import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_moveit_config')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
        "use_rviz": "false",
        "planning_pipeline": "ompl",
        "ompl.jiggle_fraction": "0.20",
        "ompl.start_state_max_bounds_error": "0.25",
        "ompl.start_state_max_dt": "0.5",
        }.items()
    )

    servo_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_path, 'launch', 'servo_teleop.launch.py')
                )
            )
        ]
    )
    
    flipper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flipper_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        base_launch,
        servo_launch,
        flipper_controller_spawner
    ])