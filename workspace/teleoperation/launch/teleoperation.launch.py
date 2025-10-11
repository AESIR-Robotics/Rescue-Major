#!/usr/bin/env python3
import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'teleoperation'
    current_dir = os.path.dirname(os.path.realpath(__file__))

    ld = launch.LaunchDescription()

    command_server = launch_ros.actions.LifecycleNode(
        name='command_server.py',
        namespace='',
        package='teleoperation',
        executable='command_server',
        parameters=[],
        output='screen')
    
    server_py = launch_ros.actions.LifecycleNode(
        name='server_py',
        namespace='',
        package='teleoperation',
        executable='server.py',
        output='screen')
    
    ld.add_action(command_server)
    ld.add_action(server_py)

    return ld
