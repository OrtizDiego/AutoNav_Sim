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

"""Tests for ball-sim: ball path, autopilot, teleop keys and the worlds.

conftest.py stubs ROS; the ball path is checked against the real museum
map (maps/my_map), whose frame equals the Gazebo world frame.
"""

import math
import os
import random
import xml.etree.ElementTree as ET

import pytest

from my_bot import ball_controller as bc
from my_bot import ball_teleop as bt
from my_bot.clearance_map import ClearanceMap

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BALL_RADIUS = 0.5


@pytest.fixture(scope='module')
def museum():
    return ClearanceMap.from_yaml(os.path.join(PKG_DIR, 'maps', 'my_map.yaml'))


def _path():
    return bc.figure_eight(bc.DEFAULT_PATH_CENTER, bc.DEFAULT_PATH_HALF_WIDTH,
                           bc.DEFAULT_PATH_HALF_HEIGHT)


def _world(name):
    return ET.parse(os.path.join(PKG_DIR, 'worlds', name)).getroot()


# ---------------------------------------------------------------------------
# Path and world
# ---------------------------------------------------------------------------

def test_path_keeps_the_ball_off_the_walls(museum):
    for p in _path():
        assert museum.clearance(*p) >= BALL_RADIUS + 0.5, p


def test_path_is_a_figure_eight_around_the_robot_start():
    xs = [p[0] for p in _path()]
    # One lobe in the U where the robot spawns, one in the hall to its right
    assert min(xs) < 0.0 < 3.0 < max(xs)


def test_params_match_module_defaults():
    import yaml
    with open(os.path.join(PKG_DIR, 'config', 'behavior_params.yaml')) as f:
        p = yaml.safe_load(f)['ball_controller']['ros__parameters']
    assert tuple(p['path_center']) == bc.DEFAULT_PATH_CENTER
    assert p['path_half_width'] == bc.DEFAULT_PATH_HALF_WIDTH
    assert p['path_half_height'] == bc.DEFAULT_PATH_HALF_HEIGHT


def test_ball_spawns_on_its_path_in_front_of_the_robot(museum):
    ball = _world('ball.world').find('.//model[@name="ball"]')
    x, y, z = (float(v) for v in ball.findtext('pose').split()[:3])
    assert z == pytest.approx(BALL_RADIUS)        # resting on the floor
    assert min(math.dist((x, y), p) for p in _path()) < 0.05
    assert museum.clearance(x, y) >= BALL_RADIUS + 0.5
    assert abs(math.atan2(y, x)) < 1.089 / 2.0    # in the camera's view


def test_ball_plugin_does_not_clash_with_robot_odom():
    plugin = _world('ball.world').find('.//model[@name="ball"]/plugin')
    assert plugin.get('filename') == 'libgazebo_ros_planar_move.so'
    assert plugin.findtext('ros/namespace') == '/ball'     # /ball/cmd_vel, /ball/odom
    assert plugin.findtext('publish_odom_tf') == 'false'   # no 2nd odom TF


def test_vanilla_world_has_no_ball_or_person():
    root = _world('room.world')
    assert root.find('.//model[@name="target"]') is None
    assert root.find('.//model[@name="ball"]') is None
    assert root.find('.//actor') is None


@pytest.mark.parametrize('world', ['ball.world', 'person.world'])
def test_museum_matches_the_mapped_world(world):
    """The map was built in room.world; scenario worlds must not move it."""
    def museum_xml(root):
        model = root.find('./world/model[@name="iscas_museum"]')
        state = root.find('./world/state/model[@name="iscas_museum"]')
        model.tail = state.tail = None  # whitespace after the element
        return ET.tostring(model), ET.tostring(state)
    assert museum_xml(_world(world)) == museum_xml(_world('room.world'))


# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------

def test_world_to_body_rotates_by_yaw():
    assert bc.world_to_body(1.0, 0.0, 0.0) == pytest.approx((1.0, 0.0))
    assert bc.world_to_body(1.0, 0.0, math.pi / 2) == pytest.approx((0.0, -1.0))
    assert bc.world_to_body(0.0, 1.0, math.pi / 2) == pytest.approx((1.0, 0.0))


def test_robot_view_frame():
    # Robot at origin, ball straight ahead on +x
    assert bc.robot_view_to_world(1.0, 0.0, (3.0, 0.0), (0.0, 0.0)) == \
        pytest.approx((1.0, 0.0))                     # "away" = +x
    assert bc.robot_view_to_world(0.0, 1.0, (3.0, 0.0), (0.0, 0.0)) == \
        pytest.approx((0.0, 1.0))                     # robot's left = +y
    # Ball north of the robot: "away" is +y
    assert bc.robot_view_to_world(1.0, 0.0, (0.0, 2.0), (0.0, 0.0)) == \
        pytest.approx((0.0, 1.0))


def test_clamp_norm():
    assert bc.clamp_norm(3.0, 4.0, 1.0) == pytest.approx((0.6, 0.8))
    assert bc.clamp_norm(0.3, 0.4, 1.0) == pytest.approx((0.3, 0.4))


# ---------------------------------------------------------------------------
# Autopilot
# ---------------------------------------------------------------------------

def _pilot(**kw):
    return bc.BallAutopilot(_path(), rng=random.Random(0), **kw)


def test_point_at_wraps_and_snaps():
    pilot = _pilot()
    assert pilot.point_at(0.0) == pytest.approx(pilot.point_at(pilot.length))
    target = pilot.point_at(5.0)
    pilot.snap_to(target)
    assert math.dist(pilot.point_at(pilot.s), target) < 0.1


def test_speed_depends_on_robot_distance():
    pilot = _pilot(pause_every=(1e9, 1e9))
    assert pilot.target_speed(10.0) == 0.0                 # waits for the robot
    assert pilot.target_speed(1.0) == pilot.flee_speed     # runs away
    assert 0.0 < pilot.target_speed(3.5) <= pilot.flee_speed


def test_ball_follows_its_path_and_plays_with_a_chasing_robot(museum):
    """Simulate ball + a naive chasing robot: ball stays on its safe path."""
    pilot = _pilot()
    ball = pilot.point_at(0.0)
    robot = (ball[0] - 2.0, ball[1])
    vel = (0.0, 0.0)
    dt, accel = 1.0 / 30.0, 1.5
    min_clear, travelled, reversals = math.inf, 0.0, 0
    direction = pilot.direction
    for _ in range(int(180.0 / dt)):
        target = pilot.step(dt, ball, robot)
        dv = bc.clamp_norm(target[0] - vel[0], target[1] - vel[1], accel * dt)
        vel = (vel[0] + dv[0], vel[1] + dv[1])
        ball = (ball[0] + vel[0] * dt, ball[1] + vel[1] * dt)
        travelled += math.hypot(*vel) * dt
        # robot drives at 0.5 m/s toward the ball, stopping 1.5 m short
        dx, dy = ball[0] - robot[0], ball[1] - robot[1]
        d = math.hypot(dx, dy)
        if d > 1.5:
            robot = (robot[0] + dx / d * 0.5 * dt, robot[1] + dy / d * 0.5 * dt)
        min_clear = min(min_clear, museum.clearance(*ball))
        reversals += pilot.direction != direction
        direction = pilot.direction
    assert min_clear >= BALL_RADIUS + 0.3
    assert travelled > 30.0           # it really runs around
    assert reversals >= 1             # and sometimes turns back


# ---------------------------------------------------------------------------
# Teleop keys
# ---------------------------------------------------------------------------

def test_teleop_directions_are_unit_vectors():
    for key in bt.KEYS:
        assert math.hypot(*bt.direction_for(key)) == pytest.approx(1.0)
    assert bt.direction_for('w') == pytest.approx((1.0, 0.0))
    assert bt.direction_for('a') == pytest.approx((0.0, 1.0))
    assert bt.direction_for('W') == bt.direction_for('w')    # caps lock
    assert bt.direction_for('p') is None
