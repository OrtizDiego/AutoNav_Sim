# Copyright 2026 root
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Robot in the museum (room.world) with Gazebo and RViz: `make sim`.

Every scenario launch file includes this one and overrides ``world`` and
``rviz_config``. The default layout is navigation.rviz (map, costmaps,
plan), so `make sim` + `make nav` shows the map exactly as before.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Gazebo simulation."""
    pkg_name = 'my_bot'

    # 1. Start Robot State Publisher (No change)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Start Gazebo. The world and the RViz layout are launch arguments so
    # the scenario launch files (ball_sim, person_sim, ...) can override them.
    default_world = os.path.join(
        get_package_share_directory(pkg_name), 'worlds', 'room.world')
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Absolute path to the Gazebo world file')
    rviz_arg = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(
            get_package_share_directory(pkg_name), 'config', 'navigation.rviz'),
        description='Absolute path to the RViz config file')

    # gzserver.launch.py declares ~25 launch arguments, among them
    # params_file:=''. Launch configurations are global, so without a scope
    # that empty params_file leaks into anything a scenario includes later
    # (Nav2's bringup then fails with "No such file or directory: ''").
    gazebo = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()  # Load your custom world
    )])

    # 3. Spawn Entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    # 4. Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
    ])
