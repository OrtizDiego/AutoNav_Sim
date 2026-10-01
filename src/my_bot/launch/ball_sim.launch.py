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

"""Ball chase in the museum: `make ball-sim`.

  1. Gazebo with ball.world (museum + red ball) and RViz.
  2. ball_controller: drives the ball around its figure-eight, running from
     the robot when it gets close and waiting when it falls behind.
     `make teleop-ball` takes over the ball by hand.
  3. sensor_fusion (mode hsv): red blob in the camera + lidar -> range and
     bearing of the ball (/target_range, /target_bearing).
  4. ball_chaser: keeps the robot 1 m from the ball.

Launch arguments: chase:=false leaves the robot still (drive it yourself
with `make teleop`); autopilot:=false parks the ball until you teleop it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Compose the ball-sim launch description."""
    pkg = get_package_share_directory('my_bot')
    params = [os.path.join(pkg, 'config', 'behavior_params.yaml'),
              {'use_sim_time': True}]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'ball.world'),
            'rviz_config': os.path.join(pkg, 'config', 'perception.rviz'),
        }.items(),
    )

    ball_controller = Node(
        package='my_bot', executable='ball_controller', name='ball_controller',
        output='screen',
        parameters=params + [{'autopilot': ParameterValue(
            LaunchConfiguration('autopilot'), value_type=bool)}],
    )
    sensor_fusion = Node(
        package='my_bot', executable='sensor_fusion', name='sensor_fusion',
        output='screen', parameters=params + [{'mode': 'hsv'}],
    )
    ball_chaser = Node(
        package='my_bot', executable='ball_chaser', name='ball_chaser',
        output='screen', parameters=params,
        condition=IfCondition(LaunchConfiguration('chase')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('chase', default_value='true',
                              description='Run ball_chaser on the robot'),
        DeclareLaunchArgument('autopilot', default_value='true',
                              description='Let the ball drive itself'),
        sim,
        ball_controller,
        sensor_fusion,
        ball_chaser,
    ])
