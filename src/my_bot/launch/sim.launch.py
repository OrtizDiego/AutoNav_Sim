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

``gui:=false`` starts gzserver only (no gzclient window): with software
rendering the Gazebo window costs CPU the simulation needs, and RViz shows
the robot, scan, map and camera anyway. Every scenario passes it through
(`make person-sim GUI=false`).

The museum (``model://iscas_museum``) is vendored in models/ and put on
GAZEBO_MODEL_PATH here, so no world needs Gazebo's online model database.

``world_name:=<name>`` picks an environment from config/worlds.yaml instead
of a ``world`` file (`make sim WORLD=warehouse`): its world file, the robot's
spawn pose and, for an external world, its Gazebo model/resource paths and
a copy of the world with shadows off (~/.ros/autonav_worlds).
``with_person:=true`` (person_sim.launch.py) takes the world's ``person_world``
instead: its world file plus the pedestrian actor.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_bot.worlds import gazebo_env, get_world, load_worlds, without_shadows


def _world_file(world, pkg_share):
    """World file for ``world``: external ones get a copy with shadows off."""
    path = world.world_file(pkg_share)
    if not os.path.exists(path):
        hint = ' (cloned by the Dockerfile: run make image)' if world.external else ''
        raise RuntimeError(f'world {world.name!r}: {path} not found{hint}')
    if not world.external:
        return path  # ours already have <shadows>0
    with open(path) as f:
        sdf = without_shadows(f.read())
    out_dir = os.path.expanduser('~/.ros/autonav_worlds')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, world.name + '.world')
    with open(out, 'w') as f:
        f.write(sdf)
    return out


def _gazebo_and_robot(context):
    """Gazebo with the chosen world, and the robot at its spawn pose."""
    pkg_share = get_package_share_directory('my_bot')
    world_path = LaunchConfiguration('world').perform(context)
    spawn = (0.0, 0.0, 0.0)
    actions = []
    name = LaunchConfiguration('world_name').perform(context)
    if name:
        world = get_world(load_worlds(os.path.join(pkg_share, 'config', 'worlds.yaml')), name)
        if LaunchConfiguration('with_person').perform(context) == 'true':
            world_path = world.person_world_file(pkg_share)  # ours: shadows are off
        else:
            world_path = _world_file(world, pkg_share)
        spawn = world.spawn
        actions += [SetEnvironmentVariable(var, value)
                    for var, value in gazebo_env(world, context.environment).items()]

    # gzserver.launch.py declares ~25 launch arguments, among them
    # params_file:=''. Launch configurations are global, so without a scope
    # that empty params_file leaks into anything a scenario includes later
    # (Nav2's bringup then fails with "No such file or directory: ''").
    actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': world_path,
                          'gui': LaunchConfiguration('gui')}.items()
    )]))
    actions.append(Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot',
                   '-x', str(spawn[0]), '-y', str(spawn[1]), '-Y', str(spawn[2])],
        output='screen'))
    return actions


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
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start the Gazebo window (gzclient); false = headless')
    rviz_on_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz; false for unattended runs (make scenario)')
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='',
        description='Environment from config/worlds.yaml; overrides world')
    with_person_arg = DeclareLaunchArgument(
        'with_person', default_value='false',
        description="With world_name: the world's person_world (adds the pedestrian)")

    # model://iscas_museum and any other vendored model (models/README.md)
    model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(get_package_share_directory(pkg_name), 'models'))

    # 4. Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        gui_arg,
        rviz_on_arg,
        world_name_arg,
        with_person_arg,
        model_path,
        rsp,
        # 3. Gazebo + the robot (world and spawn pose depend on world_name)
        OpaqueFunction(function=_gazebo_and_robot),
        rviz_node,
    ])
