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

"""Validate that behavior_params.yaml is a valid ROS 2 params file for its nodes."""

import ast
import os

import pytest
import yaml


def _params_path():
    params_path = os.environ.get('PARAMS_FILE')
    if params_path:
        return params_path
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_path = os.path.dirname(test_dir)
    return os.path.join(pkg_path, 'config', 'behavior_params.yaml')


def _load_file():
    with open(_params_path()) as f:
        return yaml.safe_load(f)


def _load_params():
    """Return {node_name: {param: value}} with the ros__parameters level removed."""
    return {node: body['ros__parameters'] for node, body in _load_file().items()}


# YAML section (node name) -> source file that declares its parameters.
NODE_SOURCES = {
    'ball_controller': 'ball_controller.py',
    'ball_chaser': 'ball_chaser.py',
    'sensor_fusion': 'sensor_fusion.py',
    'person_controller': 'person_controller.py',
    'person_tracker': 'person_tracker.py',
    'security_guard_bt': 'security_guard_bt.py',
    'system_monitor': 'system_monitor.py',
    'target_tracker': 'target_tracker.py',
}


def _declared_params(source_file):
    """Return {name: default_value} for literal declare_parameter() calls."""
    test_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(os.path.dirname(test_dir), 'my_bot', source_file)
    with open(path) as f:
        tree = ast.parse(f.read())
    declared = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and len(node.args) >= 2):
            declared[ast.literal_eval(node.args[0])] = (
                ast.literal_eval(node.args[1]))
    return declared


def _ros_type(value):
    """Map a Python value to the ROS 2 parameter type rcl would infer."""
    if isinstance(value, bool):
        return 'bool'
    if isinstance(value, int):
        return 'integer'
    if isinstance(value, float):
        return 'double'
    if isinstance(value, str):
        return 'string'
    if isinstance(value, list):
        elem_types = {_ros_type(v) for v in value}
        assert len(elem_types) == 1, f'array must be non-empty and single-typed: {value}'
        return elem_types.pop() + '_array'
    raise AssertionError(f'unsupported parameter value: {value!r}')


BALL_CHASER_KEYS = [
    'desired_distance', 'k_lin', 'k_yaw', 'max_linear_speed',
    'safety_distance', 'target_timeout', 'search_secs',
]

SECURITY_GUARD_BT_KEYS = [
    'waypoints', 'waypoint_dwell_secs', 'target_timeout', 'search_secs',
    'desired_distance', 'k_lin', 'k_yaw', 'safety_distance',
]

SENSOR_FUSION_KEYS = [
    'mode',
    'hsv_red_lower1', 'hsv_red_upper1',
    'hsv_red_lower2', 'hsv_red_upper2',
    'min_contour_area', 'range_min', 'range_max',
]


def test_params_file_exists():
    """behavior_params.yaml must exist in the config directory."""
    assert os.path.exists(_params_path()), (
        f'behavior_params.yaml not found at {_params_path()}')


def test_ros_params_file_structure():
    """Each section must be node_name: {ros__parameters: {...}} and nothing else.

    rcl's --params-file parser rejects values placed directly under the node
    name ("Cannot have a value before ros__parameters").
    """
    for node, body in _load_file().items():
        assert isinstance(body, dict), f'{node} must be a mapping'
        assert list(body) == ['ros__parameters'], (
            f'{node} must contain only a ros__parameters key, got {list(body)}')
        assert isinstance(body['ros__parameters'], dict), (
            f'{node}.ros__parameters must be a mapping')


def test_top_level_sections():
    """File must have a section for every node launched with it."""
    params = _load_params()
    for section in NODE_SOURCES:
        assert section in params, f'Missing {section} section'


def test_ball_chaser_keys():
    """ball_chaser section must contain all required keys."""
    bc = _load_params()['ball_chaser']
    for key in BALL_CHASER_KEYS:
        assert key in bc, f'ball_chaser missing key: {key}'


def test_security_guard_bt_keys():
    """security_guard_bt section must contain all required keys."""
    sg = _load_params()['security_guard_bt']
    for key in SECURITY_GUARD_BT_KEYS:
        assert key in sg, f'security_guard_bt missing key: {key}'


def test_sensor_fusion_keys():
    """sensor_fusion section must contain all required keys."""
    sf = _load_params()['sensor_fusion']
    for key in SENSOR_FUSION_KEYS:
        assert key in sf, f'sensor_fusion missing key: {key}'


@pytest.mark.parametrize('section', sorted(NODE_SOURCES))
def test_yaml_params_match_declared_types(section):
    """Every YAML value must be declared by its node with the same ROS type.

    rclpy rejects overrides whose type differs from the declared default
    (e.g. integer 300 for a double parameter).
    """
    declared = _declared_params(NODE_SOURCES[section])
    for key, value in _load_params()[section].items():
        assert key in declared, (
            f'{section}.{key} is not declared by {NODE_SOURCES[section]}')
        assert _ros_type(value) == _ros_type(declared[key]), (
            f'{section}.{key}: YAML type {_ros_type(value)} != declared '
            f'type {_ros_type(declared[key])}')


def test_hsv_ranges_valid():
    """All HSV boundary values must be integers in [0, 255]."""
    params = _load_params()
    for section in ('sensor_fusion',):
        for key in ('hsv_red_lower1', 'hsv_red_upper1',
                    'hsv_red_lower2', 'hsv_red_upper2'):
            values = params[section][key]
            assert len(values) == 3, f'{section}.{key} must have 3 elements'
            for v in values:
                assert 0 <= int(v) <= 255, (
                    f'{section}.{key} value {v} out of [0, 255]')


def test_waypoints_format():
    """Waypoints must be a flat [x0, y0, x1, y1, ...] list of floats."""
    waypoints = _load_params()['security_guard_bt']['waypoints']
    assert isinstance(waypoints, list), 'waypoints must be a list'
    assert len(waypoints) > 0, 'waypoints must not be empty'
    assert len(waypoints) % 2 == 0, 'waypoints must contain x, y pairs'
    for v in waypoints:
        assert isinstance(v, float), f'Waypoint values must be floats: {v!r}'


def test_waypoints_are_open_floor():
    """Each patrol waypoint must be reachable floor on the museum map."""
    from my_bot.clearance_map import ClearanceMap
    test_dir = os.path.dirname(os.path.abspath(__file__))
    cmap = ClearanceMap.from_yaml(
        os.path.join(os.path.dirname(test_dir), 'maps', 'my_map.yaml'))
    flat = _load_params()['security_guard_bt']['waypoints']
    for x, y in zip(flat[0::2], flat[1::2]):
        assert cmap.clearance(x, y) >= 0.5, f'waypoint ({x}, {y}) is too close to a wall'
