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

"""Every node's ROS interface, compared with a committed snapshot.

A node's contract with the rest of the system (launch files, the params
file, RViz layouts, the other nodes, the web demo) is its name, its
parameters with their defaults, and the topics, services and timers it
creates. ``contracts/node_interfaces.json`` records all of that for each
node in each configuration that changes the interface. A refactoring must
leave the file untouched; a change that is meant to alter an interface
regenerates it, and the diff of the JSON shows the reviewer exactly what
moved:

    UPDATE_CONTRACTS=1 python3 -m pytest src/my_bot/test/test_node_contracts.py
"""

import json
import os

import py_trees
import pytest
from rclpy.node import Node

from conftest import BasicNavigator, PKG_DIR

SNAPSHOT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'contracts', 'node_interfaces.json')
MAP_YAML = os.path.join(PKG_DIR, 'maps', 'my_map.yaml')
CONFIG = os.path.join(PKG_DIR, 'config')

# contract name -> (module, class, parameter overrides, constructor kwargs).
# One entry per configuration that creates a different interface.
NODES = {
    'ball_chaser': ('ball_chaser', 'BallChaser', {}, {}),
    'ball_controller': ('ball_controller', 'BallControllerNode', {}, {}),
    'ball_teleop': ('ball_teleop', 'BallTeleop', {}, {}),
    'explorer': ('explorer', 'ExplorerNode', {}, {'navigator': BasicNavigator}),
    'perf_monitor': ('perf_monitor', 'PerfMonitorNode', {}, {}),
    'person_controller': ('person_controller', 'PersonControllerNode', {}, {}),
    'person_controller[scenario]': (
        'person_controller', 'PersonControllerNode',
        {'map_yaml': MAP_YAML, 'scenario': 'sprint',
         'scenarios_file': os.path.join(CONFIG, 'scenarios.yaml')}, {}),
    'person_tracker': (
        'person_tracker', 'PersonTrackerNode',
        {'model_path': '/nonexistent/yolov8n.onnx'}, {}),
    'scenario_evaluator': (
        'scenario_evaluator', 'ScenarioEvaluatorNode',
        {'map_yaml': MAP_YAML, 'report_path': '<tmp>/report.json'}, {}),
    'security_guard_bt': ('security_guard_bt', 'SecurityGuardBTNode', {}, {}),
    'security_guard_bt[person-sim]': (
        'security_guard_bt', 'SecurityGuardBTNode',
        {'map_yaml': MAP_YAML, 'incident_dir': '<tmp>',
         'zones_file': os.path.join(CONFIG, 'guard_zones.yaml')}, {}),
    'security_guard_bt[use_track=false]': (
        'security_guard_bt', 'SecurityGuardBTNode', {'use_track': False}, {}),
    'sensor_fusion': ('sensor_fusion', 'SensorFusionNode', {}, {}),
    'sensor_fusion[person]': (
        'sensor_fusion', 'SensorFusionNode', {'mode': 'person'}, {}),
    'system_monitor': ('system_monitor', 'SystemMonitorNode', {}, {}),
    'target_tracker': ('target_tracker', 'TargetTrackerNode', {}, {}),
}


def _qos(qos):
    """Stable text for what a node passed as QoS."""
    if qos is None:
        return 'sensor_data'  # conftest's qos_profile_sensor_data
    if isinstance(qos, int):
        return f'depth {qos}'
    return ', '.join(f'{key}={value}' for key, value in sorted(vars(qos).items()))


@pytest.fixture
def recording(monkeypatch):
    """Make the fake Node also keep message types and QoS of what it creates."""
    def recorder(method, kind):
        original = getattr(Node, method)

        def wrapper(self, type_, name, *args, **kwargs):
            qos = [_qos(args[-1])] if kind != 'services' else []
            self.__dict__.setdefault('contract', {}).setdefault(kind, []).append(
                [name, type_.__name__] + qos)
            return original(self, type_, name, *args, **kwargs)
        monkeypatch.setattr(Node, method, wrapper)

    recorder('create_publisher', 'publishers')
    recorder('create_subscription', 'subscriptions')
    recorder('create_service', 'services')


def _portable(value, tmp_path):
    """Parameter values without this machine's paths."""
    if isinstance(value, str):
        return value.replace(str(tmp_path), '<tmp>').replace(PKG_DIR, '<pkg>')
    return value


def _interface(name, tmp_path):
    module, cls, overrides, kwargs = NODES[name]
    Node.overrides = {
        key: value.replace('<tmp>', str(tmp_path)) if isinstance(value, str) else value
        for key, value in overrides.items()}
    py_trees.blackboard.Blackboard.clear()
    try:
        node = getattr(__import__(f'my_bot.{module}', fromlist=[cls]), cls)(
            **{key: make() for key, make in kwargs.items()})
    finally:
        Node.overrides = {}
        py_trees.blackboard.Blackboard.clear()
    created = getattr(node, 'contract', {})
    destroy = getattr(node, 'destroy_node', None)
    if destroy is not None:
        destroy()  # person_tracker's worker thread
    return {
        'node_name': node.node_name,
        'parameters': {key: _portable(value, tmp_path)
                       for key, value in sorted(node.parameters.items())},
        'publishers': sorted(created.get('publishers', [])),
        'subscriptions': sorted(created.get('subscriptions', [])),
        'services': sorted(created.get('services', [])),
        'timer_periods': sorted(round(timer.period, 6) for timer in node.timers),
    }


def _snapshot():
    with open(SNAPSHOT) as f:
        return json.load(f)


def test_update_snapshot(recording, tmp_path):
    """UPDATE_CONTRACTS=1 rewrites the snapshot from the code as it is."""
    if not os.environ.get('UPDATE_CONTRACTS'):
        pytest.skip('set UPDATE_CONTRACTS=1 to regenerate the snapshot')
    os.makedirs(os.path.dirname(SNAPSHOT), exist_ok=True)
    with open(SNAPSHOT, 'w') as f:
        json.dump({name: _interface(name, tmp_path) for name in sorted(NODES)},
                  f, indent=2, sort_keys=True)
        f.write('\n')


@pytest.mark.parametrize('name', sorted(NODES))
def test_node_interface_matches_snapshot(recording, tmp_path, name):
    expected = _snapshot()[name]
    actual = json.loads(json.dumps(_interface(name, tmp_path)))  # tuples -> lists
    for part in expected:
        assert actual[part] == expected[part], (
            f'{name}: {part} changed. If that is intended, regenerate the '
            'snapshot (see this file\'s docstring) and review its diff.')
    assert sorted(actual) == sorted(expected)


def test_snapshot_covers_every_entry_point():
    """A new console script needs a contract; a removed one drops out."""
    import re
    with open(os.path.join(PKG_DIR, 'setup.py')) as f:
        scripts = set(re.findall(r"'\w+ = my_bot\.(\w+):main'", f.read()))
    assert {module for module, _, _, _ in NODES.values()} == scripts
    assert sorted(_snapshot()) == sorted(NODES)
