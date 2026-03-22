import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("custom_arm", package_name="aesir_moveit_config").to_moveit_configs()
    
    # We MUST keep the "moveit_servo" wrapper so the node can find these parameters!
    servo_params = {
        "moveit_servo": {
            "move_group_name": "arm",
            "planning_frame": "base_link",
            "ee_frame_name": "link_6",  # Correct end-effector
            "robot_link_command_frame": "base_link",
            
            # Vital unlocks to prevent silent freezing
            "use_gazebo": False,
            "joint_topic": "/joint_states",
            "status_topic": "status",
            "is_primary_planning_scene_monitor": False,

            "command_in_type": "unitless",
            "scale": {
                "linear": 0.4,
                "rotational": 0.8,
                "joint": 0.5,
            },

            "command_out_topic": "/arm_controller/joint_trajectory",
            "command_out_type": "trajectory_msgs/JointTrajectory",

            "publish_joint_positions": True,
            "publish_joint_velocities": True,
            "publish_joint_accelerations": False,
            "publish_period": 0.034,

            "low_pass_filter_coeff": 2.0,
            
            "check_collisions": True,
            "collision_check_rate": 10.0,
        }
    }
    
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_params
        ]
    )
    
    return LaunchDescription([servo_node])