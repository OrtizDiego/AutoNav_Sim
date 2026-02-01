import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Locate the config file inside the slam_toolbox package
    slam_config_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # Define the SLAM node directly
    # This prevents parameter propagation issues
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,       # Load the default config
            {'use_sim_time': True}  # Force Sim Time
        ]
    )

    return LaunchDescription([
        slam_node
    ])