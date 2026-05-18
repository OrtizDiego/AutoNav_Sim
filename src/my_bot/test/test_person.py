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

"""Pure-Python tests for the person {controller, tracker, follower} math.

These exercise the module-level helper functions only, so they run without
a ROS graph, Gazebo, or ONNX model.  ROS / OpenCV / cv_bridge imports are
stubbed at module load so the modules-under-test import cleanly.
"""

import math
import os
import sys
import types

# ---------------------------------------------------------------------------
# Stub ROS / OpenCV / cv_bridge so the modules import in a plain Python env
# ---------------------------------------------------------------------------
for _mod in ('cv_bridge', 'rclpy', 'rclpy.node', 'rclpy.qos',
             'sensor_msgs', 'sensor_msgs.msg',
             'geometry_msgs', 'geometry_msgs.msg',
             'nav_msgs', 'nav_msgs.msg',
             'std_msgs', 'std_msgs.msg',
             'visualization_msgs', 'visualization_msgs.msg'):
    sys.modules.setdefault(_mod, types.ModuleType(_mod))

import rclpy.node as _rclpy_node  # noqa: E402
_rclpy_node.Node = object
import rclpy.qos as _rclpy_qos  # noqa: E402
_rclpy_qos.qos_profile_sensor_data = None

import sensor_msgs.msg as _smsg  # noqa: E402
_smsg.Image = object
_smsg.LaserScan = object

import geometry_msgs.msg as _gmsg  # noqa: E402
_gmsg.PointStamped = object
_gmsg.Twist = object
_gmsg.Pose = object

import nav_msgs.msg as _nmsg  # noqa: E402
_nmsg.Odometry = object

import std_msgs.msg as _stdmsg  # noqa: E402
_stdmsg.Bool = object
_stdmsg.Float32 = object
_stdmsg.Float32MultiArray = object
_stdmsg.String = object

import visualization_msgs.msg as _vmsg  # noqa: E402
_vmsg.Marker = object
_vmsg.MarkerArray = object

import cv_bridge as _cvb  # noqa: E402
_cvb.CvBridge = object

# cv2 + numpy stubs (only used so that `import` succeeds; the helper
# functions tested here do not actually invoke OpenCV).  When the real
# packages are installed (CI / Docker) they take precedence over the stubs.
try:
    import cv2  # noqa: F401
except ImportError:
    _cv2 = types.ModuleType('cv2')
    _cv2.legacy = types.SimpleNamespace()
    sys.modules['cv2'] = _cv2
try:
    import numpy  # noqa: F401
except ImportError:
    sys.modules['numpy'] = types.ModuleType('numpy')

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402


# ---------------------------------------------------------------------------
# person_controller — FSM + locomotion primitives
# ---------------------------------------------------------------------------

def test_should_run_triggers_on_detection():
    from my_bot.person_controller import RUN, WALK, should_run
    assert should_run(True, WALK, 0.0, 4.0) == RUN


def test_should_run_persists_during_calm_down():
    from my_bot.person_controller import RUN, should_run
    # Still inside the calm-down window → stay in RUN
    assert should_run(False, RUN, 2.0, 4.0) == RUN


def test_should_run_returns_to_walk_after_calm_down():
    from my_bot.person_controller import RUN, WALK, should_run
    assert should_run(False, RUN, 5.0, 4.0) == WALK


def test_step_walk_advances_toward_waypoint():
    from my_bot.person_controller import PersonState, step_walk
    s = PersonState(0.0, 0.0, 0.0)
    out = step_walk(s, (10.0, 0.0), speed=1.0, dt=1.0)
    assert math.isclose(out.x, 1.0, abs_tol=1e-6)
    assert math.isclose(out.y, 0.0, abs_tol=1e-6)
    assert math.isclose(out.yaw, 0.0, abs_tol=1e-6)


def test_step_walk_snaps_when_within_one_step():
    from my_bot.person_controller import PersonState, step_walk
    s = PersonState(0.0, 0.0, 0.0)
    out = step_walk(s, (0.3, 0.0), speed=1.0, dt=1.0)
    assert math.isclose(out.x, 0.3, abs_tol=1e-6)


def test_step_flee_moves_away_from_robot():
    from my_bot.person_controller import PersonState, step_flee
    s = PersonState(2.0, 0.0, 0.0)
    out = step_flee(s, (0.0, 0.0), speed=1.0, dt=1.0, bounds=8.0)
    # Should move further from origin (the "robot")
    assert math.hypot(out.x, out.y) > math.hypot(s.x, s.y)


def test_step_flee_clamps_to_bounds():
    from my_bot.person_controller import PersonState, step_flee
    s = PersonState(7.9, 0.0, 0.0)
    out = step_flee(s, (0.0, 0.0), speed=5.0, dt=1.0, bounds=8.0)
    assert -8.0 <= out.x <= 8.0
    assert -8.0 <= out.y <= 8.0


# ---------------------------------------------------------------------------
# person_tracker — pure helpers
# ---------------------------------------------------------------------------

def test_select_person_box_picks_highest_score_person():
    from my_bot.person_tracker import select_person_box
    detections = [
        (0, 0, 10, 10, 0, 0.6),   # person, low score
        (5, 5, 30, 30, 0, 0.9),   # person, high score
        (0, 0, 100, 100, 32, 0.95),  # sports ball — must be ignored
    ]
    box = select_person_box(detections, min_conf=0.4)
    assert box == (5, 5, 25, 25)


def test_select_person_box_returns_none_when_no_person():
    from my_bot.person_tracker import select_person_box
    detections = [(0, 0, 10, 10, 32, 0.95)]
    assert select_person_box(detections) is None


def test_select_person_box_respects_min_confidence():
    from my_bot.person_tracker import select_person_box
    detections = [(0, 0, 10, 10, 0, 0.2)]
    assert select_person_box(detections, min_conf=0.4) is None


def test_box_iou_identical_is_one():
    from my_bot.person_tracker import box_iou
    a = (0, 0, 10, 10)
    assert math.isclose(box_iou(a, a), 1.0, abs_tol=1e-6)


def test_box_iou_disjoint_is_zero():
    from my_bot.person_tracker import box_iou
    assert box_iou((0, 0, 10, 10), (100, 100, 10, 10)) == 0.0


def test_box_iou_half_overlap():
    from my_bot.person_tracker import box_iou
    iou = box_iou((0, 0, 10, 10), (5, 0, 10, 10))
    # intersection = 5*10 = 50, union = 100+100-50 = 150 → 1/3
    assert math.isclose(iou, 1.0 / 3.0, abs_tol=1e-6)


# ---------------------------------------------------------------------------
# person_follower — control law
# ---------------------------------------------------------------------------

def test_compute_command_stops_when_no_range():
    from my_bot.person_follower import compute_command
    v, w = compute_command(0.1, -1.0, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2)
    assert v == 0.0 and w == 0.0


def test_compute_command_advances_when_too_far():
    from my_bot.person_follower import compute_command
    v, _ = compute_command(0.0, 3.0, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2,
                           deadband=0.1)
    assert v > 0.0


def test_compute_command_backs_off_when_too_close():
    from my_bot.person_follower import compute_command
    v, _ = compute_command(0.0, 0.5, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2,
                           deadband=0.1)
    assert v < 0.0


def test_compute_command_respects_deadband():
    from my_bot.person_follower import compute_command
    v, _ = compute_command(0.0, 1.55, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2,
                           deadband=0.1)
    assert v == 0.0


def test_compute_command_clips_linear_speed():
    from my_bot.person_follower import compute_command
    v, _ = compute_command(0.0, 100.0, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2)
    assert v == 0.4


def test_compute_command_turns_toward_target():
    """Positive bearing (target to the left) → positive angular.z (turn left)."""
    from my_bot.person_follower import compute_command
    # compute_bearing in sensor_fusion uses atan2(pixel_x - cx, fx), so a target
    # to the *right* of the optical axis gives a positive bearing; the follower
    # control law uses w = -k_yaw * bearing, so a right-of-centre target turns
    # the robot right (negative w).
    _, w_right = compute_command(0.5, 2.0, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2)
    _, w_left = compute_command(-0.5, 2.0, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2)
    assert w_right < 0.0
    assert w_left > 0.0


def test_compute_command_clips_angular_speed():
    from my_bot.person_follower import compute_command
    _, w = compute_command(10.0, 2.0, 1.5, 0.6, 1.2, 0.4, 0.2, 1.2)
    assert abs(w) <= 1.2 + 1e-9


def test_bbox_centroid_x_basic():
    from my_bot.person_follower import bbox_centroid_x
    assert bbox_centroid_x([10.0, 20.0, 40.0, 60.0]) == 30.0


def test_bbox_centroid_x_empty():
    from my_bot.person_follower import bbox_centroid_x
    assert bbox_centroid_x([]) is None


# ---------------------------------------------------------------------------
# person.world — structural sanity
# ---------------------------------------------------------------------------

def test_person_world_exists_and_has_actor():
    test_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_path = os.path.dirname(test_dir)
    world = os.path.join(pkg_path, 'worlds', 'person.world')
    assert os.path.exists(world), 'person.world missing'
    with open(world) as f:
        content = f.read()
    assert '<actor name="person_intruder">' in content
    assert 'walk.dae' in content
    assert 'libgazebo_ros_state.so' in content
