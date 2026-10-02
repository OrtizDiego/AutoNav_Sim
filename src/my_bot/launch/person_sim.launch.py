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

"""The full security-guard demo: `make person-sim`.

  1. Gazebo with person.world (museum + walking/running pedestrian) and RViz.
  2. Nav2 (map_server + AMCL + planners) on maps/my_map, for the patrol.
  3. person_controller: WALK / RUN / EXHAUSTED pedestrian that steers on the
     museum map so it never walks into a wall, and sprints away once the
     robot's tracker locks on.
  4. person_tracker: YOLOv8n + OpenCV tracker + Kalman -> /person_bbox.
  5. sensor_fusion (mode person): bbox + lidar -> range and bearing.
  6. security_guard_bt: py_trees tree. Patrols Nav2 waypoints; when the
     intruder is seen it cancels the patrol and follows at 2.5 m; when the
     intruder is lost it searches toward the last-seen side, then resumes
     the patrol. Latched e-stop from system_monitor overrides everything.
  7. system_monitor: sensor watchdog, /trigger_estop and /clear_estop.

Watch /security_guard/state for PatrolProtocol / IntruderProtocol /
SearchProtocol / EmergencyStop.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Compose the person-sim launch description."""
    pkg = get_package_share_directory('my_bot')
    # behavior_params.yaml is a standard params file; each node picks up its
    # own <node_name>: ros__parameters: section.
    params = [os.path.join(pkg, 'config', 'behavior_params.yaml'),
              {'use_sim_time': True}]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'person.world'),
            'rviz_config': os.path.join(pkg, 'config', 'person.rviz'),
        }.items(),
    )
    # Give Gazebo a head start so /clock, /scan and odom TF exist when
    # AMCL and the costmaps activate. Pass Nav2's files explicitly: a
    # DeclareLaunchArgument default is skipped when an earlier include has
    # already set a launch configuration of the same name.
    nav2 = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': os.path.join(pkg, 'maps', 'my_map.yaml'),
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )])

    def node(executable, extra=None):
        return Node(package='my_bot', executable=executable, name=executable,
                    output='screen', parameters=params + ([extra] if extra else []))

    return LaunchDescription([
        sim,
        nav2,
        node('person_controller', {
            'map_yaml': os.path.join(pkg, 'maps', 'my_map.yaml')}),
        node('person_tracker'),
        node('sensor_fusion', {'mode': 'person'}),
        node('security_guard_bt'),
        node('system_monitor'),
    ])
