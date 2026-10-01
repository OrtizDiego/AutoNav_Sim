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

"""Museum sim + Nav2 in one go: `make nav-sim`.

`make sim` alone has no map: nothing publishes /map or the map -> odom
transform until Nav2 (map_server + AMCL) runs. This starts both. AMCL's
initial pose (the spawn point, the map origin) comes from nav2_params.yaml,
so the map and costmaps appear in RViz without clicking "2D Pose Estimate".
Send goals with RViz's "Nav2 Goal" tool.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Compose the nav-sim launch description."""
    pkg = get_package_share_directory('my_bot')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'rviz_config': os.path.join(pkg, 'config', 'navigation.rviz'),
        }.items(),
    )

    # Give Gazebo a head start so /clock, /scan and odom TF exist when
    # AMCL and the costmaps activate.
    nav2 = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'navigation.launch.py')))])

    return LaunchDescription([sim, nav2])
