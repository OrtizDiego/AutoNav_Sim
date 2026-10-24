# Copyright 2026 AutoNav Team
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

"""A world + Nav2 in one go: `make nav-sim [WORLD=<name>]`.

``world_name`` picks the environment from config/worlds.yaml (default
museum). Then one of two things:

* The world has a map (maps/<map>.yaml): Nav2 with map_server + AMCL on
  it. AMCL's initial pose (the spawn point, the map origin) comes from
  nav2_params.yaml, so the map and costmaps appear in RViz without clicking
  "2D Pose Estimate". Send goals with RViz's "Nav2 Goal" tool.
* It has none yet: SLAM Toolbox builds /map and map -> odom, Nav2 runs
  without map_server and AMCL (navigation_launch.py), and the explorer
  drives to frontiers until the map is complete, saves it as
  maps/<map>.yaml and returns to the start. The next nav-sim of that world
  takes the first branch.

Maps are looked up in and saved to src/my_bot/maps (worlds.maps_source_dir:
the folder the symlink install points to), so a saved map is used without
a rebuild and survives the container.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_bot.worlds import get_world, load_worlds, maps_source_dir


def _setup(context):
    pkg = get_package_share_directory('my_bot')
    name = LaunchConfiguration('world_name').perform(context) or 'museum'
    world = get_world(load_worlds(os.path.join(pkg, 'config', 'worlds.yaml')), name)
    map_yaml = world.map_yaml(maps_source_dir(pkg))

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world_name': name,
            'rviz_config': os.path.join(pkg, 'config', 'navigation.rviz'),
        }.items(),
    )
    if os.path.exists(map_yaml):
        # Give Gazebo a head start so /clock, /scan and odom TF exist when
        # AMCL and the costmaps activate. Pass Nav2's files explicitly: a
        # DeclareLaunchArgument default is skipped when an earlier include
        # has already set a launch configuration of the same name.
        nav2 = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'true',
            }.items(),
        )])
        return [LogInfo(msg=f'world {name}: Nav2 on {map_yaml}'), sim, nav2]

    # No map: SLAM + Nav2 (no map_server, no AMCL) + the explorer
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[os.path.join(pkg, 'config', 'mapper_params_online_async.yaml'),
                    {'use_sim_time': True}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    explorer = Node(
        package='my_bot', executable='explorer', name='explorer', output='screen',
        parameters=[os.path.join(pkg, 'config', 'behavior_params.yaml'),
                    {'use_sim_time': True,
                     'map_save_path': os.path.splitext(map_yaml)[0]}])
    return [
        LogInfo(msg=f'world {name}: no map at {map_yaml}; exploring to build it'),
        sim,
        TimerAction(period=5.0, actions=[slam]),
        TimerAction(period=8.0, actions=[nav2]),
        TimerAction(period=12.0, actions=[explorer]),
    ]


def generate_launch_description():
    """Compose the nav-sim launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name', default_value='museum',
            description='Environment from config/worlds.yaml'),
        OpaqueFunction(function=_setup),
    ])
