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

"""Launch the full security guard stack: sensor_fusion + security_guard + system_monitor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    """Generate launch description for the complete security guard system."""
    params_file = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'behavior_params.yaml',
    )

    sensor_fusion_node = Node(
        package='my_bot',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen',
        parameters=[params_file],
    )

    # LifecycleNode auto-configures and activates on launch
    security_guard_node = LifecycleNode(
        package='my_bot',
        executable='security_guard',
        name='security_guard',
        namespace='',
        output='screen',
        parameters=[params_file],
    )

    system_monitor_node = Node(
        package='my_bot',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        parameters=[{
            'scan_timeout': 2.0,
            'camera_timeout': 2.0,
            'health_publish_rate': 1.0,
        }],
    )

    return LaunchDescription([
        sensor_fusion_node,
        security_guard_node,
        system_monitor_node,
    ])
