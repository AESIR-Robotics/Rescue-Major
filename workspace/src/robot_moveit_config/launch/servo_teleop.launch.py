import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("custom_arm", package_name="robot_moveit_config").to_moveit_configs()


    servo_yaml_file = PathJoinSubstitution([
        FindPackageShare("robot_moveit_config"), # Make sure this matches where the YAML lives
        "config",
        "servo_params.yaml"
    ])
    
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_yaml_file
        ]
    )
    
    return LaunchDescription([servo_node])