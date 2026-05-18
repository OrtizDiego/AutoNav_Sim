#!/usr/bin/env python3

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

"""Unit tests for security_guard_bt.py leaf nodes and tree structure.

All tests run as pure Python — no ROS runtime required.
ROS/Gazebo/cv2 imports are stubbed via sys.modules before importing the module.
"""

import math
import sys
import types

# ---------------------------------------------------------------------------
# Stub ROS / sensor / cv2 modules so the import succeeds without ROS runtime
# ---------------------------------------------------------------------------

_stub_mods = [
    'rclpy', 'rclpy.node', 'rclpy.executors', 'rclpy.qos',
    'cv_bridge',
    'geometry_msgs', 'geometry_msgs.msg',
    'sensor_msgs', 'sensor_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'nav2_simple_commander', 'nav2_simple_commander.robot_navigator',
]
for _mod in _stub_mods:
    if _mod not in sys.modules:
        sys.modules[_mod] = types.ModuleType(_mod)

# cv_bridge
_bridge = sys.modules['cv_bridge']


class _FakeCvBridge:
    def imgmsg_to_cv2(self, *a, **kw):
        return None


_bridge.CvBridge = _FakeCvBridge

# geometry_msgs.msg
_geo_msg = sys.modules['geometry_msgs.msg']


class _FakeTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class _FakePoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='', stamp=None)
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


_geo_msg.Twist = _FakeTwist
_geo_msg.PoseStamped = _FakePoseStamped

# sensor_msgs.msg
_sen_msg = sys.modules['sensor_msgs.msg']


class _FakeImage:
    pass


_sen_msg.Image = _FakeImage

# std_msgs.msg
_std_msg = sys.modules['std_msgs.msg']


class _FakeFloat32:
    data = 0.0


_std_msg.Float32 = _FakeFloat32

# rclpy.node
_rclpy = sys.modules['rclpy']
_rclpy.ok = lambda: True
_rclpy.init = lambda **kw: None
_rclpy.spin = lambda n: None
_rclpy.shutdown = lambda: None


class _FakeNode:
    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, debug=lambda *a: None,
                                     warn=lambda *a: None)

    def declare_parameter(self, *a, **kw):
        pass

    def get_parameter(self, name):
        defaults = {
            'hsv_red_lower1': [0, 100, 100],
            'hsv_red_upper1': [10, 255, 255],
            'hsv_red_lower2': [160, 100, 100],
            'hsv_red_upper2': [180, 255, 255],
            'angular_gain': 0.01,
            'linear_speed': 0.2,
            'max_angular_speed': 0.5,
            'min_contour_area': 500.0,
            'waypoints': [3.28, 6.85, -0.92, 6.85],
            'waypoint_dwell_secs': 2.0,
            'stop_distance': 0.8,
        }
        val = defaults.get(name, 0)
        return types.SimpleNamespace(value=val)

    def create_subscription(self, *a, **kw):
        return None

    def create_publisher(self, *a, **kw):
        return types.SimpleNamespace(publish=lambda *a: None)

    def create_timer(self, *a, **kw):
        return None


sys.modules['rclpy.node'].Node = _FakeNode

# nav2_simple_commander.robot_navigator
_nav_mod = sys.modules['nav2_simple_commander.robot_navigator']


class _FakeNavigator:
    def isTaskComplete(self):
        return True

    def cancelTask(self):
        pass

    def goToPose(self, *a):
        pass

    def waitUntilNav2Active(self):
        pass

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            to_msg=lambda: None))


_nav_mod.BasicNavigator = _FakeNavigator

# ---------------------------------------------------------------------------
# Now safe to import py_trees (real) and our module
# ---------------------------------------------------------------------------
import py_trees  # noqa: E402 — must come after stubs

sys.path.insert(0, 'src/my_bot')
from my_bot.security_guard_bt import (  # noqa: E402
    BB_HSV_DETECTED,
    BB_INTRUDER_VISIBLE,
    BB_TARGET_RANGE,
    BB_WP_INDEX,
    IntruderVisible,
    TooClose,
    CancelPatrol,
    ChaseIntruder,
    NavigateToWaypoint,
    WaitAtWaypoint,
    IncrementWaypoint,
    build_security_guard_tree,
)


# ---------------------------------------------------------------------------
# Helper: reset the global blackboard between tests
# ---------------------------------------------------------------------------

def _fresh_bb(**kwargs):
    """Return a clean Blackboard with provided keys set."""
    bb = py_trees.blackboard.Blackboard()
    bb.clear()
    for k, v in kwargs.items():
        bb.set(k, v)
    return bb


# ---------------------------------------------------------------------------
# IntruderVisible
# ---------------------------------------------------------------------------

class TestIntruderVisible:

    def test_success_when_hsv_detected(self):
        _fresh_bb(**{BB_HSV_DETECTED: True})
        node = IntruderVisible()
        assert node.update() == py_trees.common.Status.SUCCESS

    def test_success_when_yolo_detected(self):
        _fresh_bb(**{BB_HSV_DETECTED: False, 'yolo_target_detected': True})
        node = IntruderVisible()
        assert node.update() == py_trees.common.Status.SUCCESS

    def test_success_when_both_detected(self):
        _fresh_bb(**{BB_HSV_DETECTED: True, 'yolo_target_detected': True})
        node = IntruderVisible()
        assert node.update() == py_trees.common.Status.SUCCESS

    def test_failure_when_neither_detected(self):
        _fresh_bb(**{BB_HSV_DETECTED: False, 'yolo_target_detected': False})
        node = IntruderVisible()
        assert node.update() == py_trees.common.Status.FAILURE

    def test_failure_when_keys_absent(self):
        # Fresh blackboard with no keys set — getattr should return False defaults
        bb = py_trees.blackboard.Blackboard()
        node = IntruderVisible()
        result = node.update()
        assert result == py_trees.common.Status.FAILURE


# ---------------------------------------------------------------------------
# TooClose
# ---------------------------------------------------------------------------

class TestTooClose:

    def test_success_when_below_threshold(self):
        _fresh_bb(**{BB_TARGET_RANGE: 0.3})
        node = TooClose(stop_distance=0.8)
        assert node.update() == py_trees.common.Status.SUCCESS

    def test_failure_when_above_threshold(self):
        _fresh_bb(**{BB_TARGET_RANGE: 1.5})
        node = TooClose(stop_distance=0.8)
        assert node.update() == py_trees.common.Status.FAILURE

    def test_failure_when_range_invalid(self):
        _fresh_bb(**{BB_TARGET_RANGE: -1.0})
        node = TooClose(stop_distance=0.8)
        assert node.update() == py_trees.common.Status.FAILURE

    def test_failure_at_exactly_threshold(self):
        _fresh_bb(**{BB_TARGET_RANGE: 0.8})
        node = TooClose(stop_distance=0.8)
        # 0.8 is NOT < 0.8, so should be FAILURE
        assert node.update() == py_trees.common.Status.FAILURE

    def test_custom_stop_distance(self):
        _fresh_bb(**{BB_TARGET_RANGE: 0.4})
        node = TooClose(stop_distance=0.5)
        assert node.update() == py_trees.common.Status.SUCCESS


# ---------------------------------------------------------------------------
# CancelPatrol
# ---------------------------------------------------------------------------

class TestCancelPatrol:

    def test_always_succeeds(self):
        nav = _FakeNavigator()
        node = CancelPatrol(nav)
        result = node.update()
        assert result == py_trees.common.Status.SUCCESS

    def test_sets_nav_goal_sent_false(self):
        nav = _FakeNavigator()
        bb = py_trees.blackboard.Blackboard()
        bb.set('nav_goal_sent', True)
        node = CancelPatrol(nav)
        node.update()
        val = bb.get('nav_goal_sent') if bb.exists('nav_goal_sent') else True
        assert val is False

    def test_cancels_incomplete_task(self):
        cancelled = []

        class SlowNav(_FakeNavigator):
            def isTaskComplete(self):
                return False

            def cancelTask(self):
                cancelled.append(True)

        node = CancelPatrol(SlowNav())
        node.update()
        assert len(cancelled) == 1


# ---------------------------------------------------------------------------
# WaitAtWaypoint
# ---------------------------------------------------------------------------

class TestWaitAtWaypoint:

    def test_first_tick_is_running(self):
        bb = py_trees.blackboard.Blackboard()
        bb.set('dwell_start_time', None)
        node = WaitAtWaypoint(dwell_secs=2.0)
        assert node.update() == py_trees.common.Status.RUNNING

    def test_success_after_dwell(self):
        import time
        bb = py_trees.blackboard.Blackboard()
        bb.set('dwell_start_time', time.monotonic() - 5.0)  # 5s ago
        node = WaitAtWaypoint(dwell_secs=2.0)
        assert node.update() == py_trees.common.Status.SUCCESS

    def test_running_before_dwell_expires(self):
        import time
        bb = py_trees.blackboard.Blackboard()
        bb.set('dwell_start_time', time.monotonic() - 0.1)  # just started
        node = WaitAtWaypoint(dwell_secs=60.0)
        assert node.update() == py_trees.common.Status.RUNNING


# ---------------------------------------------------------------------------
# IncrementWaypoint
# ---------------------------------------------------------------------------

class TestIncrementWaypoint:

    def test_increments_index(self):
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_WP_INDEX, 0)
        node = IncrementWaypoint(n_waypoints=6)
        node.update()
        assert bb.get(BB_WP_INDEX) == 1

    def test_wraps_around(self):
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_WP_INDEX, 5)
        node = IncrementWaypoint(n_waypoints=6)
        node.update()
        assert bb.get(BB_WP_INDEX) == 0

    def test_always_succeeds(self):
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_WP_INDEX, 2)
        node = IncrementWaypoint(n_waypoints=6)
        assert node.update() == py_trees.common.Status.SUCCESS


# ---------------------------------------------------------------------------
# Tree structure
# ---------------------------------------------------------------------------

class TestTreeStructure:

    def _make_tree(self):
        fake_node = _FakeNode()
        fake_node._waypoints_bt = [[3.28, 6.85], [-0.92, 6.85],
                                    [5.13, -2.45], [0.23, -2.55],
                                    [-4.52, 8.60], [0.0, 0.0]]
        fake_node._angular_gain_bt = 0.01
        fake_node._linear_speed_bt = 0.2
        fake_node._max_ang_bt = 0.5
        fake_node._vel_pub_bt = types.SimpleNamespace(publish=lambda *a: None)
        nav = _FakeNavigator()
        return build_security_guard_tree(nav, fake_node, stop_distance=0.8, dwell_secs=2.0)

    def test_root_is_selector(self):
        tree = self._make_tree()
        assert isinstance(tree.root, py_trees.composites.Selector)

    def test_root_has_two_children(self):
        tree = self._make_tree()
        assert len(tree.root.children) == 2

    def test_intruder_protocol_is_sequence(self):
        tree = self._make_tree()
        intruder_seq = tree.root.children[0]
        assert isinstance(intruder_seq, py_trees.composites.Sequence)

    def test_intruder_protocol_has_three_children(self):
        tree = self._make_tree()
        intruder_seq = tree.root.children[0]
        assert len(intruder_seq.children) == 3

    def test_patrol_protocol_is_sequence(self):
        tree = self._make_tree()
        patrol_seq = tree.root.children[1]
        assert isinstance(patrol_seq, py_trees.composites.Sequence)

    def test_patrol_protocol_has_three_children(self):
        tree = self._make_tree()
        patrol_seq = tree.root.children[1]
        assert len(patrol_seq.children) == 3

    def test_first_intruder_child_is_intruder_visible(self):
        tree = self._make_tree()
        first_child = tree.root.children[0].children[0]
        assert isinstance(first_child, IntruderVisible)

    def test_approach_control_is_selector(self):
        tree = self._make_tree()
        approach_sel = tree.root.children[0].children[2]
        assert isinstance(approach_sel, py_trees.composites.Selector)

    def test_navigate_to_waypoint_is_first_patrol_child(self):
        tree = self._make_tree()
        first_patrol = tree.root.children[1].children[0]
        assert isinstance(first_patrol, NavigateToWaypoint)
