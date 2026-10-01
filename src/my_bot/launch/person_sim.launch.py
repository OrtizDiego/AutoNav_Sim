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

"""Launch person.world + tracker + controller + follower.

Starts the full pipeline:

  1. Gazebo with ``person.world`` (animated actor driven by
     ``libperson_actor_plugin.so`` from the ``person_actor_plugin`` package).
  2. ``person_tracker``: YOLO + OpenCV tracker + Kalman filter.
  3. ``person_controller``: WALK/RUN/EXHAUSTED behaviour publishing
     ``/person/cmd_vel``.
  4. ``person_follower``: keeps the robot at the stand-off distance.

All nodes use simulation time so the person's timing matches Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Compose the person-sim launch description."""
    pkg = get_package_share_directory('my_bot')
    # behavior_params.yaml is a standard params file; each node picks up its
    # own <node_name>: ros__parameters: section.
    node_params = [
        os.path.join(pkg, 'config', 'behavior_params.yaml'),
        {'use_sim_time': True},
    ]

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'person.world'),
        }.items(),
    )

    person_tracker = Node(
        package='my_bot',
        executable='person_tracker',
        name='person_tracker',
        output='screen',
        parameters=node_params,
    )

    person_controller = Node(
        package='my_bot',
        executable='person_controller',
        name='person_controller',
        output='screen',
        parameters=node_params,
    )

    person_follower = Node(
        package='my_bot',
        executable='person_follower',
        name='person_follower',
        output='screen',
        parameters=node_params,
    )

    return LaunchDescription([
        sim_launch,
        person_tracker,
        person_controller,
        person_follower,
    ])
