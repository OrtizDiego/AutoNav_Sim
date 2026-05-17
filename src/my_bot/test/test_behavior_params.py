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

"""Validate that behavior_params.yaml exists and contains all required keys."""

import os

import yaml


def _params_path():
    params_path = os.environ.get('PARAMS_FILE')
    if params_path:
        return params_path
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_path = os.path.dirname(test_dir)
    return os.path.join(pkg_path, 'config', 'behavior_params.yaml')


def _load_params():
    with open(_params_path()) as f:
        return yaml.safe_load(f)


BALL_CHASER_KEYS = [
    'hsv_red_lower1', 'hsv_red_upper1',
    'hsv_red_lower2', 'hsv_red_upper2',
    'angular_gain', 'linear_speed',
    'min_contour_area', 'search_angular_speed', 'search_timeout_sec',
]

SECURITY_GUARD_KEYS = [
    'hsv_red_lower1', 'hsv_red_upper1',
    'hsv_red_lower2', 'hsv_red_upper2',
    'angular_gain', 'linear_speed', 'max_angular_speed',
    'min_contour_area', 'stop_distance_area', 'waypoints',
]


def test_params_file_exists():
    """behavior_params.yaml must exist in the config directory."""
    assert os.path.exists(_params_path()), (
        f'behavior_params.yaml not found at {_params_path()}')


def test_top_level_sections():
    """File must have ball_chaser and security_guard top-level sections."""
    params = _load_params()
    assert 'ball_chaser' in params, 'Missing ball_chaser section'
    assert 'security_guard' in params, 'Missing security_guard section'


def test_ball_chaser_keys():
    """ball_chaser section must contain all required keys."""
    bc = _load_params()['ball_chaser']
    for key in BALL_CHASER_KEYS:
        assert key in bc, f'ball_chaser missing key: {key}'


def test_security_guard_keys():
    """security_guard section must contain all required keys."""
    sg = _load_params()['security_guard']
    for key in SECURITY_GUARD_KEYS:
        assert key in sg, f'security_guard missing key: {key}'


def test_hsv_ranges_valid():
    """All HSV boundary values must be integers in [0, 255]."""
    params = _load_params()
    for section in ('ball_chaser', 'security_guard'):
        for key in ('hsv_red_lower1', 'hsv_red_upper1',
                    'hsv_red_lower2', 'hsv_red_upper2'):
            values = params[section][key]
            assert len(values) == 3, f'{section}.{key} must have 3 elements'
            for v in values:
                assert 0 <= int(v) <= 255, (
                    f'{section}.{key} value {v} out of [0, 255]')


def test_waypoints_format():
    """Waypoints must be a list of [x, y] pairs."""
    waypoints = _load_params()['security_guard']['waypoints']
    assert isinstance(waypoints, list), 'waypoints must be a list'
    assert len(waypoints) > 0, 'waypoints must not be empty'
    for wp in waypoints:
        assert len(wp) == 2, f'Each waypoint must have 2 elements, got: {wp}'
        assert all(isinstance(v, (int, float)) for v in wp), (
            f'Waypoint values must be numeric: {wp}')
