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
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_action
from lifecycle_msgs.msg import Transition


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

    security_guard_node = LifecycleNode(
        package='my_bot',
        executable='security_guard',
        name='security_guard',
        namespace='',
        output='screen',
        parameters=[params_file],
    )

    # Drive security_guard through configure -> activate on launch.
    # Activation only follows configure (start_state), so a manual deactivate
    # is not undone. on_activate blocks on waitUntilNav2Active(): Nav2 must run.
    configure_security_guard = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(security_guard_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=security_guard_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(security_guard_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
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
        activate_on_inactive,
        configure_security_guard,
        system_monitor_node,
    ])
