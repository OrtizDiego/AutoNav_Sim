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

"""Tests for person-sim: controller, tracker, fusion/follow helpers, wiring.

Only module-level pure functions are exercised (conftest.py stubs ROS), so
no ROS graph, Gazebo or ONNX model is needed. The controller is also
simulated on the real museum map to prove the person stays off the walls.
"""

import ast
import math
import os
import random
import xml.etree.ElementTree as ET

import pytest
import yaml

from my_bot.clearance_map import ClearanceMap, safe_heading
from my_bot import follow_control as fc
from my_bot import person_controller as pc
from my_bot import person_tracker as pt
from my_bot import sensor_fusion as sf

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.path.join(PKG_DIR, 'maps', 'my_map.yaml')
FX = 320.0 / math.tan(1.089 / 2.0)


@pytest.fixture(scope='module')
def museum():
    return ClearanceMap.from_yaml(MAP_YAML)


# ---------------------------------------------------------------------------
# person_controller
# ---------------------------------------------------------------------------

class TestModes:

    def test_detection_starts_a_sprint(self):
        assert pc.next_mode(pc.WALK, True, 1.0, 0.6) == pc.RUN

    def test_no_sprint_with_empty_tank(self):
        assert pc.next_mode(pc.WALK, True, 0.0, 0.6) == pc.WALK

    def test_sprint_ends_when_stamina_runs_out(self):
        assert pc.next_mode(pc.RUN, True, 0.0, 0.6) == pc.EXHAUSTED

    def test_calm_person_returns_to_walking(self):
        assert pc.next_mode(pc.RUN, False, 0.8, 0.6) == pc.WALK

    def test_exhausted_until_recovered(self):
        assert pc.next_mode(pc.EXHAUSTED, True, 0.5, 0.6) == pc.EXHAUSTED
        assert pc.next_mode(pc.EXHAUSTED, True, 0.7, 0.6) == pc.RUN
        assert pc.next_mode(pc.EXHAUSTED, False, 0.7, 0.6) == pc.WALK

    def test_stamina_drains_and_recovers(self):
        s = pc.update_stamina(1.0, pc.RUN, 1.0, max_run_secs=4.0, recovery_secs=8.0)
        assert s == pytest.approx(0.75)
        s = pc.update_stamina(s, pc.WALK, 2.0, max_run_secs=4.0, recovery_secs=8.0)
        assert s == pytest.approx(1.0)
        assert pc.update_stamina(0.01, pc.RUN, 1.0, 4.0, 8.0) == 0.0

    def test_sprint_lasts_max_run_secs(self):
        """Simulate a full sprint at 20 Hz: it must end after ~max_run_secs."""
        mode, stamina, t, dt = pc.WALK, 1.0, 0.0, 0.05
        mode = pc.next_mode(mode, True, stamina, 0.6)
        while mode == pc.RUN and t < 20.0:
            stamina = pc.update_stamina(stamina, mode, dt, 3.5, 8.0)
            mode = pc.next_mode(mode, True, stamina, 0.6)
            t += dt
        assert mode == pc.EXHAUSTED
        assert t == pytest.approx(3.5, abs=0.1)


class TestFleeHeading:

    def test_points_away_from_robot(self):
        assert pc.flee_heading((3.0, 0.0), (0.0, 0.0)) == pytest.approx(0.0)
        assert pc.flee_heading((0.0, -2.0), (0.0, 0.0)) == pytest.approx(-math.pi / 2)

    def test_coincident_positions_do_not_crash(self):
        assert math.isfinite(pc.flee_heading((1.0, 1.0), (1.0, 1.0)))

    def test_stopping_speed(self):
        assert pc.stopping_speed(0.05, 3.0) == 0.0
        # v^2 = 2 a d  ->  from 2.5 m/s at 3 m/s^2 one needs ~1.04 m + margin
        assert pc.stopping_speed(1.14, 3.0) == pytest.approx(2.5, abs=0.01)


class TestSteer:

    def test_acceleration_is_limited(self):
        v, _ = pc.steer(0.0, 0.0, 2.5, 0.7, 0.05, 1.0, accel=2.0, decel=3.0)
        assert v == pytest.approx(0.8)

    def test_yaw_rate_is_limited(self):
        _, w = pc.steer(0.0, math.pi / 2, 1.0, 1.0, 0.05, 1.0, 2.0, 3.0)
        assert w == pytest.approx(1.0)

    def test_slows_down_for_sharp_turns(self):
        v_straight, _ = pc.steer(0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0)
        v_turn, _ = pc.steer(0.0, math.pi * 0.9, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0)
        assert v_turn < v_straight

    def test_wrap_angle(self):
        assert pc.wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


# ---------------------------------------------------------------------------
# Clearance map + wall avoidance on the real museum map
# ---------------------------------------------------------------------------

class TestClearanceMap:

    def test_spawn_point_is_open_floor(self, museum):
        assert museum.clearance(0.0, 0.0) > 1.0

    def test_outside_the_map_is_blocked(self, museum):
        assert museum.clearance(100.0, 100.0) == 0.0

    def test_free_distance_stops_before_wall(self):
        box = ClearanceMap.from_bounds(5.0)
        # From the centre heading +x the wall is 5 m away; keep 0.5 m
        assert box.free_distance(0.0, 0.0, 0.0, 10.0, 0.5) == pytest.approx(4.5, abs=0.1)

    def test_can_always_move_away_from_a_wall(self):
        box = ClearanceMap.from_bounds(5.0)
        # 0.2 m from the east wall: east is blocked (to within a grid cell),
        # west is open
        assert box.free_distance(4.8, 0.0, 0.0, 2.0, 0.5) <= box.resolution
        assert box.free_distance(4.8, 0.0, math.pi, 2.0, 0.5) == pytest.approx(2.0)

    def test_safe_heading_bends_around_wall(self):
        box = ClearanceMap.from_bounds(5.0)
        # Heading straight at the east wall (x = 5) from x = 3.5: the chosen
        # heading must give 2 m of travel that stays 0.5 m off the wall.
        h, free = safe_heading(box, 3.5, 0.0, 0.0, 2.0, 0.5)
        assert free >= 2.0
        assert 3.5 + 2.0 * math.cos(h) <= 4.5 + 1e-6

    def test_random_free_point_has_clearance(self, museum):
        rng = random.Random(0)
        for _ in range(50):
            assert museum.clearance(*museum.random_free_point(1.0, rng)) >= 1.0


def _simulate_person(cmap, seed, start, seconds=240.0, dt=0.05):
    """Run PersonBrain like the actor plugin does; return (min clearance, metres)."""
    brain = pc.PersonBrain(cmap, rng=random.Random(seed))
    p = pc.PersonPose(start[0], start[1], 0.0, 0.0)
    robot = [0.0, 0.0]
    min_clear, travelled = math.inf, 0.0
    for k in range(int(seconds / dt)):
        alarmed = int(k * dt / 30.0) % 2 == 1  # calm 30 s, chased 30 s, ...
        # A robot that chases at 1 m/s and stops 2.5 m short
        dx, dy = p.x - robot[0], p.y - robot[1]
        d = math.hypot(dx, dy)
        if d > 2.5:
            robot[0] += dx / d * dt
            robot[1] += dy / d * dt
        v, w = brain.step(dt, p, tuple(robot), alarmed)
        p.speed = v = max(0.0, v)  # the plugin never walks backwards
        p.yaw = pc.wrap_angle(p.yaw + w * dt)
        p.x += v * dt * math.cos(p.yaw)
        p.y += v * dt * math.sin(p.yaw)
        travelled += v * dt
        min_clear = min(min_clear, cmap.clearance(p.x, p.y))
    return min_clear, travelled


@pytest.mark.parametrize('seed', range(3))
def test_person_never_walks_into_the_museum_walls(museum, seed):
    min_clear, travelled = _simulate_person(museum, seed, start=(0.5, 7.5))
    params = pc.PersonParams()
    assert min_clear >= params.min_clearance - 0.05
    assert travelled > 100.0, 'the person must keep moving, not freeze at a wall'


# ---------------------------------------------------------------------------
# person_tracker
# ---------------------------------------------------------------------------

class TestTrackerHelpers:

    def test_select_person_box_picks_highest_score_person(self):
        detections = [
            (0, 0, 10, 10, 0, 0.6),
            (5, 5, 30, 30, 0, 0.9),
            (0, 0, 100, 100, 32, 0.95),  # sports ball, ignored
        ]
        assert pt.select_person_box(detections, 0.4) == (5, 5, 25, 25)

    def test_select_person_box_none_without_person(self):
        assert pt.select_person_box([(0, 0, 10, 10, 32, 0.95)]) is None
        assert pt.select_person_box([(0, 0, 10, 10, 0, 0.2)], 0.4) is None

    def test_box_iou(self):
        assert pt.box_iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
        assert pt.box_iou((0, 0, 10, 10), (100, 100, 10, 10)) == 0.0
        assert pt.box_iou((0, 0, 10, 10), (5, 0, 10, 10)) == pytest.approx(1 / 3)

    def test_letterbox_shape_is_square(self):
        assert pt.letterbox_shape((480, 640, 3)) == (640, 640)
        assert pt.letterbox_shape((720, 1280, 3)) == (1280, 1280)

    def test_letterbox_decoding_keeps_y_unscaled(self):
        """A box at y=400 in a 640x480 frame must decode back to y=400."""
        np = pytest.importorskip('numpy')
        pytest.importorskip('cv2.dnn')
        from my_bot.object_detector import postprocess
        out = np.zeros((1, 84, 8400), dtype=np.float32)
        # Letterboxed 640x480 -> 640x640 with scale 1.0: pixels are unchanged
        out[0, :4, 0] = [320.0, 400.0, 50.0, 100.0]
        out[0, 4, 0] = 0.9
        dets = postprocess(out, pt.letterbox_shape((480, 640, 3)), 0.5, 0.45)
        assert len(dets) == 1
        cy = (dets[0][1] + dets[0][3]) / 2
        assert cy == pytest.approx(400.0, abs=1.0)

    def test_clip_box(self):
        assert pt.clip_box((-10, -10, 30, 40), 640, 480) == (0, 0, 20, 30)
        assert pt.clip_box((630, 470, 50, 50), 640, 480) == (630, 470, 10, 10)
        assert pt.clip_box((700, 10, 20, 20), 640, 480) is None


class TestBoxKalman:

    def test_first_update_returns_measurement(self):
        kf = pt.BoxKalman((100, 100, 50, 120))
        kf.predict()
        box = kf.update((100, 100, 50, 120))
        assert box == pytest.approx((100, 100, 50, 120), abs=1)

    def test_learns_velocity_and_predicts_motion(self):
        kf = pt.BoxKalman((100, 100, 50, 120))
        for k in range(1, 20):
            kf.predict()
            kf.update((100 + 5 * k, 100, 50, 120))
        predicted = kf.predict()
        # Last measurement x=195; constant 5 px/frame motion -> ~200
        assert predicted[0] == pytest.approx(200, abs=3)

    def test_smooths_measurement_noise(self):
        kf = pt.BoxKalman((300, 100, 50, 120))
        for k in range(30):
            kf.predict()
            jitter = 6 if k % 2 else -6
            box = kf.update((300 + jitter, 100, 50, 120))
        assert abs(box[0] - 300) < 6


# ---------------------------------------------------------------------------
# sensor_fusion ranging + follow_control
# ---------------------------------------------------------------------------

class TestFollowerGeometry:

    def test_right_of_image_is_negative_ros_angle(self):
        """Image right == robot's right == negative angle in ROS."""
        assert sf.compute_bearing(600.0, 320.0, FX) < 0.0
        assert sf.compute_bearing(40.0, 320.0, FX) > 0.0
        assert sf.compute_bearing(320.0, 320.0, FX) == pytest.approx(0.0)

    def _scan(self, default=10.0):
        n = 360
        inc = 2 * math.pi / n
        return [default] * n, -math.pi, inc

    def test_scan_window_finds_person_on_the_right(self):
        ranges, amin, inc = self._scan()
        # Person at -20 deg (robot's right), 2.5 m
        for deg in range(-22, -17):
            ranges[int(round((math.radians(deg) - amin) / inc))] = 2.5
        r = sf.scan_window_range(ranges, amin, inc, math.radians(-15),
                                 math.radians(-25), 0.3, 12.0)
        assert r == pytest.approx(2.5)
        # Mirrored window (the old bug) sees only background
        r_mirror = sf.scan_window_range(ranges, amin, inc, math.radians(25),
                                        math.radians(15), 0.3, 12.0)
        assert r_mirror == pytest.approx(10.0)

    def test_scan_window_ignores_invalid_returns(self):
        ranges, amin, inc = self._scan(float('inf'))
        assert sf.scan_window_range(ranges, amin, inc, 0.1, -0.1, 0.3, 12.0) is None

    def test_scan_window_wraps_at_pi(self):
        ranges, amin, inc = self._scan()
        ranges[0] = 3.0     # -pi
        ranges[-1] = 3.0    # just below +pi
        r = sf.scan_window_range(ranges, amin, inc, math.pi + 0.02,
                                 math.pi - 0.02, 0.3, 12.0, percentile=0.0)
        assert r == pytest.approx(3.0)

    def test_monocular_range_full_body(self):
        # 1.72 m person, 2.5 m away -> h = fy * 1.72 / 2.5
        h = FX * 1.72 / 2.5
        r = sf.monocular_range([300, 100, 60, h], 480, FX, 240.0, 1.72, 0.103)
        assert r == pytest.approx(2.5, rel=0.01)

    def test_monocular_range_from_feet_when_head_cut(self):
        # Head cut off at the top; feet 0.103 m below the camera at 2.0 m
        feet_y = 240.0 + FX * 0.103 / 2.0
        r = sf.monocular_range([300, 0, 60, feet_y], 480, FX, 240.0, 1.72, 0.103)
        assert r == pytest.approx(2.0, rel=0.01)

    def test_monocular_range_none_when_both_ends_cut(self):
        assert sf.monocular_range([300, 0, 60, 480], 480, FX, 240.0, 1.72, 0.103) is None

    def test_fuse_range(self):
        assert sf.fuse_range(2.4, 2.5) == 2.4          # agree -> lidar
        assert sf.fuse_range(9.0, 2.5) == 2.5          # beam missed legs
        assert sf.fuse_range(None, 2.5) == 2.5
        assert sf.fuse_range(2.4, None) == 2.4
        assert sf.fuse_range(None, None) is None


class TestFollowerControl:

    ARGS = dict(desired_distance=2.5, k_lin=0.8, k_yaw=1.5,
                max_lin=1.0, max_back=0.2, max_yaw=1.5)

    def test_turns_only_without_range(self):
        v, w = fc.compute_command(0.3, None, **self.ARGS)
        assert v == 0.0 and w > 0.0

    def test_advances_when_too_far(self):
        v, _ = fc.compute_command(0.0, 4.0, **self.ARGS)
        assert v == pytest.approx(1.0)

    def test_backs_off_when_too_close(self):
        v, _ = fc.compute_command(0.0, 1.5, **self.ARGS)
        assert v == pytest.approx(-0.2)

    def test_deadband(self):
        v, _ = fc.compute_command(0.0, 2.6, **self.ARGS)
        assert v == 0.0

    def test_turns_toward_person(self):
        _, w_left = fc.compute_command(0.4, 2.5, **self.ARGS)
        _, w_right = fc.compute_command(-0.4, 2.5, **self.ARGS)
        assert w_left > 0.0 > w_right

    def test_turn_before_driving(self):
        v_ahead, _ = fc.compute_command(0.0, 4.0, **self.ARGS)
        v_side, _ = fc.compute_command(1.2, 4.0, **self.ARGS)
        assert v_side < v_ahead

    def test_safety_stop(self):
        v, _ = fc.compute_command(0.0, 4.0, front_clear=0.4,
                                  safety_distance=0.6, **self.ARGS)
        assert v == 0.0


# ---------------------------------------------------------------------------
# Config, world and plugin wiring
# ---------------------------------------------------------------------------

def _declared_defaults(module_file):
    """Map parameter name -> default value from declare_parameter calls."""
    tree = ast.parse(open(module_file).read())
    out = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, 'attr', '') == 'declare_parameter'
                and len(node.args) == 2):
            out[ast.literal_eval(node.args[0])] = ast.literal_eval(node.args[1])
    return out


@pytest.mark.parametrize('node', ['person_controller', 'person_tracker'])
def test_yaml_params_match_declared_types(node):
    """YAML values must match declared types (int vs double fails in ROS)."""
    with open(os.path.join(PKG_DIR, 'config', 'behavior_params.yaml')) as f:
        section = yaml.safe_load(f)[node]['ros__parameters']
    declared = _declared_defaults(os.path.join(PKG_DIR, 'my_bot', f'{node}.py'))
    for key, value in section.items():
        assert key in declared, f'{node}: unknown parameter {key}'
        assert type(value) is type(declared[key]), (
            f'{node}.{key}: yaml {type(value).__name__} vs '
            f'declared {type(declared[key]).__name__}')


def test_person_params_dataclass_matches_node_defaults():
    """PersonParams (used by tests/sim) must mirror the ROS parameter defaults."""
    declared = _declared_defaults(os.path.join(PKG_DIR, 'my_bot', 'person_controller.py'))
    for name, value in vars(pc.PersonParams()).items():
        assert declared[name] == value, name


def _actor(world):
    root = ET.parse(os.path.join(PKG_DIR, 'worlds', world)).getroot()
    actor = root.find('.//actor[@name="person_intruder"]')
    assert actor is not None
    return actor


@pytest.mark.parametrize('world', ['person.world', 'yolo.world'])
def test_actor_uses_plugin_not_script(world):
    actor = _actor(world)
    # A <script> trajectory would override the plugin's pose every frame
    assert actor.find('script') is None
    anims = {a.get('name'): a.findtext('filename')
             for a in actor.findall('animation')}
    assert anims == {'walking': 'walk.dae', 'running': 'run.dae'}
    plugin = actor.find('plugin')
    assert plugin.get('filename') == 'libperson_actor_plugin.so'
    assert plugin.findtext('ros/namespace') == '/person'


def test_person_spawns_on_open_floor(museum):
    x, y = (float(v) for v in _actor('person.world').findtext('pose').split()[:2])
    assert museum.clearance(x, y) >= 1.0


def test_yolo_person_stands_in_front_of_the_camera():
    x, y, *_, yaw = (float(v) for v in _actor('yolo.world').findtext('pose').split())
    bearing = math.atan2(y, x)
    assert 2.0 <= math.hypot(x, y) <= 4.0
    assert abs(bearing) < 1.089 / 2.0           # inside the camera's FOV
    assert math.cos(yaw - (bearing + math.pi)) > 0.9  # facing the robot


def test_person_sim_gives_the_controller_the_museum_map():
    src = open(os.path.join(PKG_DIR, 'launch', 'person_sim.launch.py')).read()
    assert "'map_yaml'" in src and "'my_map.yaml'" in src
    assert "'person.world'" in src


def test_plugin_package_exports_plugin_path():
    xml = os.path.join(os.path.dirname(PKG_DIR), 'person_actor_plugin',
                       'package.xml')
    root = ET.parse(xml).getroot()
    export = root.find('export/gazebo_ros')
    assert export is not None and 'plugin_path' in export.attrib
