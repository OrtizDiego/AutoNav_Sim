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

"""One scripted, scored security-guard run: `make scenario NAME=<name>`.

  1. person-sim (person_sim.launch.py) with the person walking scenario
     ``scenario`` from config/scenarios.yaml instead of wandering.
     Headless by default: gui:=false (no Gazebo window), rviz:=false.
     A scenario with a second_person runs in two_person.world with a
     second scripted person_controller.
  2. ros2 bag record of the topics needed to replay and re-score the run
     (no images), into ``out_dir``/bag.
  3. scenario_evaluator: scores the run against the person's ground truth
     and writes ``out_dir``/report.json. When it exits, the whole launch
     shuts down (which also closes the bag).

``out_dir`` defaults to ~/.ros/autonav_scenarios/<scenario>_<date-time>.
``timeout_secs`` (sim s) overrides the evaluator's, e.g. for the CI smoke
test (src/smoke_test.sh).
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_bot.scenario_script import load_scenarios

# Enough to replay the run and recompute every metric; images are left out
# (two 320x240 streams would dwarf the rest).
RECORDED_TOPICS = [
    '/clock', '/tf', '/tf_static', '/odom', '/scan', '/cmd_vel',
    '/person/odom', '/person_mode', '/person2/odom', '/person2/mode',
    '/person_bbox', '/person_detected',
    '/target', '/intruder/track', '/intruder/state',
    '/security_guard/state', '/security_guard/metrics', '/security_events',
    '/scenario/status', '/scenario/second_person_status', '/scenario/report',
    '/amcl_pose',
]


def _setup(context):
    pkg = get_package_share_directory('my_bot')
    scenario = LaunchConfiguration('scenario').perform(context)
    scenarios = load_scenarios(os.path.join(pkg, 'config', 'scenarios.yaml'))
    if scenario not in scenarios:
        raise RuntimeError(f'unknown scenario {scenario!r}: one of {sorted(scenarios)}')
    two = scenarios[scenario].has_second_person
    out_dir = os.path.expanduser(LaunchConfiguration('out_dir').perform(context)) or (
        os.path.join(os.path.expanduser('~/.ros/autonav_scenarios'),
                     f"{scenario}_{time.strftime('%Y%m%d-%H%M%S')}"))
    os.makedirs(out_dir, exist_ok=True)

    person_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'person_sim.launch.py')),
        launch_arguments={
            'scenario': scenario,
            'world': os.path.join(
                pkg, 'worlds', 'two_person.world' if two else 'person.world'),
            'second_person': 'true' if two else 'false',
        }.items())
    overrides = {'use_sim_time': True, 'scenario': scenario,
                 'map_yaml': os.path.join(pkg, 'maps', 'my_map.yaml'),
                 'report_path': os.path.join(out_dir, 'report.json')}
    timeout = LaunchConfiguration('timeout_secs').perform(context)
    if timeout:
        overrides['timeout_secs'] = float(timeout)
    evaluator = Node(
        package='my_bot', executable='scenario_evaluator',
        name='scenario_evaluator', output='screen',
        parameters=[os.path.join(pkg, 'config', 'behavior_params.yaml'), overrides])
    actions = [
        person_sim,
        evaluator,
        RegisterEventHandler(OnProcessExit(
            target_action=evaluator,
            on_exit=[EmitEvent(event=Shutdown(reason='scenario scored'))])),
    ]
    if LaunchConfiguration('record').perform(context).lower() == 'true':
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(out_dir, 'bag'),
                 *RECORDED_TOPICS],
            output='screen'))
    return actions


def generate_launch_description():
    """Compose the scenario launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='walk_in',
            description='Scenario name in config/scenarios.yaml'),
        DeclareLaunchArgument(
            'out_dir', default_value='',
            description='Report + bag directory; empty = '
                        '~/.ros/autonav_scenarios/<scenario>_<date-time>'),
        DeclareLaunchArgument(
            'record', default_value='true', description='Record a rosbag'),
        DeclareLaunchArgument(
            'timeout_secs', default_value='',
            description="Evaluator timeout (sim s); empty = behavior_params.yaml's"),
        # Declared before person_sim includes sim.launch.py, so these
        # defaults win over sim's (launch configurations are global).
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
