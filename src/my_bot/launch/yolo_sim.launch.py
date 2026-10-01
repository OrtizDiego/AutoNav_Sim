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

"""YOLO + sensor fusion proof: `make yolo-sim`.

  1. Gazebo with yolo.world: a person standing 3 m ahead of the robot, a
     crate and a barrel to the sides. RViz shows the annotated camera view.
  2. person_tracker: YOLOv8n detection + OpenCV tracker + Kalman filter
     -> /person_bbox, /person_detected, /person_tracker/image.
  3. sensor_fusion (mode person): bbox + lidar (+ monocular check)
     -> /target_range (~2.8-3.0 m here), /target_bearing, /target_position.

Nothing moves the robot; `make teleop` to drive around and watch the range
and bearing follow.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Compose the yolo-sim launch description."""
    pkg = get_package_share_directory('my_bot')
    params = [os.path.join(pkg, 'config', 'behavior_params.yaml'),
              {'use_sim_time': True}]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'yolo.world'),
            'rviz_config': os.path.join(pkg, 'config', 'perception.rviz'),
        }.items(),
    )

    person_tracker = Node(
        package='my_bot', executable='person_tracker', name='person_tracker',
        output='screen', parameters=params,
    )
    sensor_fusion = Node(
        package='my_bot', executable='sensor_fusion', name='sensor_fusion',
        output='screen', parameters=params + [{'mode': 'person'}],
    )

    return LaunchDescription([sim, person_tracker, sensor_fusion])
