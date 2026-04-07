import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("custom_arm", package_name="robot_moveit_config").to_moveit_configs()

    commander_node = Node(
        package="robot_moveit_config",
        executable="arm_command_node", 
        name="arm_commander_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False}
        ]
    )
    
    return LaunchDescription([commander_node])