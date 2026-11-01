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
     world's map so it never walks into a wall, and sprints away once the
     robot's tracker locks on.
  4. person_tracker: YOLOv8n + OpenCV tracker + Kalman -> /person_bbox.
  5. sensor_fusion (mode person): bbox + lidar -> range and bearing.
  6. target_tracker: odom-frame EKF (position + velocity) on /target and
     person-sized lidar clusters -> /intruder/track once confirmed.
  7. security_guard_bt: py_trees tree. Patrols the zones of
     config/guard_zones.yaml on their schedule; when the intruder track is
     confirmed it cancels the patrol, raises an alert (and another if the
     intruder enters a restricted area) and follows at 2.5 m; when the
     intruder is lost it drives (Nav2) to where the track was heading,
     turns toward the last-seen side, sweeps the rooms around from a few
     viewpoints picked on the map, then reports the incident and resumes
     the patrol. Latched e-stop from system_monitor overrides everything.
  8. system_monitor: sensor watchdog, /trigger_estop and /clear_estop.

``world_name:=<name>`` (`make person-sim WORLD=warehouse`) takes the world
from config/worlds.yaml instead: its ``person_world`` (the world plus the
actor), its map (must exist: `make nav-sim WORLD=<name>` builds it) for Nav2,
the pedestrian and the sweep, and its ``zones`` file for the guard. Without
it: person.world, maps/my_map and guard_zones.yaml (the museum).

``scenario:=<name>`` makes the person walk that scenario from
config/scenarios.yaml instead of wandering (scenario.launch.py, which also
scores the run). ``second_person:=true`` with ``world:=<two_person.world>``
adds a second person_controller for the scenario's second person, remapped
to the second actor's /person2 topics.

Watch /security_guard/state for PatrolProtocol / IntruderProtocol /
SearchProtocol / SweepProtocol / ReportProtocol / EmergencyStop, and /security_events for
the alerts (also in ~/.ros/security_incidents/incidents.jsonl).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from my_bot.worlds import get_world, load_worlds, maps_source_dir


def _setup(context):
    """Everything that depends on the chosen world."""
    pkg = get_package_share_directory('my_bot')
    name = LaunchConfiguration('world_name').perform(context)
    scenario = LaunchConfiguration('scenario').perform(context)
    second = LaunchConfiguration('second_person').perform(context) == 'true'

    if name:
        # config/worlds.yaml: the world with the actor, the world's map and
        # the guard's zones on that map
        world = get_world(load_worlds(os.path.join(pkg, 'config', 'worlds.yaml')), name)
        map_yaml = world.map_yaml(maps_source_dir(pkg))
        if not os.path.exists(map_yaml):
            raise RuntimeError(
                f'world {name!r}: no map at {map_yaml}; `make nav-sim WORLD={name}` builds it')
        zones_file = world.zones_file(pkg)
        world_args = {'world_name': name, 'with_person': 'true'}
        if (scenario or second) and name != 'museum':
            raise RuntimeError('scripted scenarios are written for the museum')
    else:
        map_yaml = os.path.join(pkg, 'maps', 'my_map.yaml')
        zones_file = os.path.join(pkg, 'config', 'guard_zones.yaml')
        world_args = {'world': LaunchConfiguration('world')}

    # behavior_params.yaml is a standard params file; each node picks up its
    # own <node_name>: ros__parameters: section.
    params = [os.path.join(pkg, 'config', 'behavior_params.yaml'),
              {'use_sim_time': True}]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            **world_args,
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
            'map': map_yaml,
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )])

    def node(executable, extra=None):
        return Node(package='my_bot', executable=executable, name=executable,
                    output='screen', parameters=params + ([extra] if extra else []))

    person_args = {
        'map_yaml': map_yaml,
        # value_type=str: an empty scenario stays '' instead of YAML null
        'scenario': ParameterValue(LaunchConfiguration('scenario'), value_type=str),
        'scenarios_file': os.path.join(pkg, 'config', 'scenarios.yaml')}
    # The second controller has its own node name, so the params file's
    # person_controller section is handed to it as a dict.
    with open(params[0]) as f:
        person_params = yaml.safe_load(f)['person_controller']['ros__parameters']
    second_person = Node(
        package='my_bot', executable='person_controller', name='person2_controller',
        output='screen', condition=IfCondition(LaunchConfiguration('second_person')),
        parameters=[person_params, {'use_sim_time': True}, person_args,
                    {'scenario_person': 'second'}],
        remappings=[('/person/cmd_vel', '/person2/cmd_vel'),
                    ('/person/odom', '/person2/odom'),
                    ('/person_mode', '/person2/mode'),
                    ('/scenario/status', '/scenario/second_person_status')])

    return [
        LogInfo(msg=f'person-sim: map {map_yaml}, zones {zones_file}'),
        sim,
        nav2,
        node('person_controller', person_args),
        second_person,
        node('person_tracker'),
        node('sensor_fusion', {'mode': 'person'}),
        node('target_tracker'),
        node('security_guard_bt', {'zones_file': zones_file, 'map_yaml': map_yaml}),
        node('system_monitor'),
    ]


def generate_launch_description():
    """Compose the person-sim launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='Scripted person from config/scenarios.yaml; empty = wander'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(
                get_package_share_directory('my_bot'), 'worlds', 'person.world'),
            description='Gazebo world (two_person.world for second_person)'),
        DeclareLaunchArgument(
            'world_name', default_value='',
            description='Environment from config/worlds.yaml (museum, warehouse): its '
                        "person_world, map and zones; overrides world"),
        DeclareLaunchArgument(
            'second_person', default_value='false',
            description="Walk the scenario's second person (needs two_person.world)"),
        OpaqueFunction(function=_setup),
    ])
