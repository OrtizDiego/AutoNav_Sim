import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_bot'
    
    # Path to your saved map (make sure my_map.yaml exists!)
    map_file = os.path.join(get_package_share_directory(pkg_name), 'maps', 'my_map.yaml')
    
    # Nav2 Bringup Params
    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    
    # We use the default params for now, but you can create a custom one later
    params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                # We will use the default params provided by nav2 for now to keep it simple
                'params_file': params_file 
            }.items()
        )
    ])