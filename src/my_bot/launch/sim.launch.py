import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_bot'

# 1. Start Robot State Publisher (No change)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Start Gazebo (MODIFIED)
    # Define the path to your world file
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'room.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': world_path}.items() # Load your custom world
        # launch_arguments={
        #     'world': world_path,
        #     'gui': 'false',       # <--- Headless mode (No Gazebo GUI)
        #     'server_required': 'true' 
        # }.items()
    )

    # 3. Spawn Entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    # 4. Launch RViz (New Step!)
    # We find the config file we just saved
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'navigation.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file], # -d means "load this description file"
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz_node, # Add the new node here
    ])