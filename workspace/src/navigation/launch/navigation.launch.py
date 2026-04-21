# launch/navigation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.actions import SetRemap 

def generate_launch_description():
    # Get the path to your package's config file
    nav_pkg_dir = get_package_share_directory('navigation')
    nav2_params_path = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')

    # Get the path to the official Nav2 bringup package
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Include the official navigation launch file
    # We use 'navigation_launch.py' instead of 'bringup_launch.py' 
    # to skip AMCL and Map Server (since RTAB-Map handles that)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_path
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])