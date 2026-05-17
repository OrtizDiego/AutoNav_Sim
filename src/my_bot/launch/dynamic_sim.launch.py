# Copyright 2024 AutoNav Team
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

"""Launch dynamic_sim: Gazebo with dynamic.world + obstacle_controller + intruder_bot."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_bot')

    # Read existing sim.launch.py but override the world file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg, 'worlds', 'dynamic.world')}.items(),
    )

    obstacle_controller = Node(
        package='my_bot',
        executable='obstacle_controller',
        name='obstacle_controller',
        output='screen',
    )

    intruder_bot = Node(
        package='my_bot',
        executable='intruder_bot',
        name='intruder_bot',
        output='screen',
    )

    return LaunchDescription([
        sim_launch,
        obstacle_controller,
        intruder_bot,
    ])
