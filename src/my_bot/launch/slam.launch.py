import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Use custom config file from my_bot package
    slam_config_file = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # Define the SLAM node directly
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        slam_node
    ])