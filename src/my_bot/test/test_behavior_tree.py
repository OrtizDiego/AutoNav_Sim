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

"""Unit tests for security_guard_bt.py leaf nodes and tree behaviour.

All tests run as pure Python — conftest.py stubs the ROS packages. The
tree is ticked against a fake Nav2 navigator and a fake clock.
"""

import math
import time

import py_trees
import pytest

from my_bot.security_guard_bt import (
    BB_ESTOP,
    BB_FRONT_CLEAR,
    BB_LAST_SEEN,
    BB_NAV_GOAL_SENT,
    BB_TARGET_BEARING,
    BB_TARGET_RANGE,
    BB_WP_INDEX,
    CancelPatrol,
    EStopActive,
    FollowGains,
    FollowIntruder,
    HaltRobot,
    IncrementWaypoint,
    IntruderRecentlyLost,
    IntruderVisible,
    NavigateToWaypoint,
    SearchLastSeen,
    WaitAtWaypoint,
    active_protocol,
    build_security_guard_tree,
)

Status = py_trees.common.Status
WAYPOINTS = [[3.28, 6.85], [-0.92, 6.85], [5.13, -2.45]]


class FakeNavigator:
    """Records goals; tasks finish only when told to."""

    def __init__(self):
        self.goals = []
        self.cancelled = 0
        self.done = True

    def isTaskComplete(self):
        return self.done

    def cancelTask(self):
        self.cancelled += 1
        self.done = True

    def goToPose(self, goal):
        self.goals.append(goal)
        self.done = False


class Recorder:
    """Collects published Twists."""

    def __init__(self):
        self.msgs = []

    def __call__(self, msg):
        self.msgs.append(msg)

    @property
    def last(self):
        return self.msgs[-1]


@pytest.fixture(autouse=True)
def fresh_blackboard():
    """Every test starts from an empty global blackboard."""
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _bb(**kwargs):
    for k, v in kwargs.items():
        py_trees.blackboard.Blackboard.set(k, v)


# ---------------------------------------------------------------------------
# Conditions
# ---------------------------------------------------------------------------

class TestVisibility:

    def test_visible_when_recently_seen(self):
        _bb(**{BB_LAST_SEEN: 100.0})
        assert IntruderVisible(0.5, clock=lambda: 100.3).update() == Status.SUCCESS

    def test_not_visible_after_timeout(self):
        _bb(**{BB_LAST_SEEN: 100.0})
        assert IntruderVisible(0.5, clock=lambda: 100.6).update() == Status.FAILURE

    def test_not_visible_when_never_seen(self):
        assert IntruderVisible(0.5).update() == Status.FAILURE

    def test_recently_lost_window(self):
        _bb(**{BB_LAST_SEEN: 100.0})
        lost = IntruderRecentlyLost(0.5, 6.0, clock=lambda: 100.3)
        assert lost.update() == Status.FAILURE      # still visible
        lost = IntruderRecentlyLost(0.5, 6.0, clock=lambda: 103.0)
        assert lost.update() == Status.SUCCESS      # searching
        lost = IntruderRecentlyLost(0.5, 6.0, clock=lambda: 107.0)
        assert lost.update() == Status.FAILURE      # gave up

    def test_never_seen_is_not_recently_lost(self):
        assert IntruderRecentlyLost().update() == Status.FAILURE

    def test_estop(self):
        assert EStopActive().update() == Status.FAILURE
        _bb(**{BB_ESTOP: True})
        assert EStopActive().update() == Status.SUCCESS


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------

class TestActions:

    def test_follow_drives_toward_far_target(self):
        _bb(**{BB_TARGET_BEARING: 0.0, BB_TARGET_RANGE: 5.0, BB_FRONT_CLEAR: 9.0})
        pub = Recorder()
        assert FollowIntruder(pub, FollowGains()).update() == Status.RUNNING
        assert pub.last.linear.x > 0.0

    def test_follow_turns_toward_target_on_the_left(self):
        _bb(**{BB_TARGET_BEARING: 0.4, BB_TARGET_RANGE: 2.5})
        pub = Recorder()
        FollowIntruder(pub, FollowGains()).update()
        assert pub.last.angular.z > 0.0

    def test_follow_only_turns_without_range(self):
        _bb(**{BB_TARGET_BEARING: -0.3, BB_TARGET_RANGE: -1.0})
        pub = Recorder()
        FollowIntruder(pub, FollowGains()).update()
        assert pub.last.linear.x == 0.0 and pub.last.angular.z < 0.0

    def test_follow_safety_stop(self):
        _bb(**{BB_TARGET_BEARING: 0.0, BB_TARGET_RANGE: 5.0, BB_FRONT_CLEAR: 0.3})
        pub = Recorder()
        FollowIntruder(pub, FollowGains()).update()
        assert pub.last.linear.x == 0.0

    def test_search_turns_to_last_seen_side(self):
        pub = Recorder()
        _bb(**{BB_TARGET_BEARING: -0.2})
        SearchLastSeen(pub, 0.6).update()
        assert pub.last.angular.z == pytest.approx(-0.6)
        _bb(**{BB_TARGET_BEARING: 0.2})
        SearchLastSeen(pub, 0.6).update()
        assert pub.last.angular.z == pytest.approx(0.6)

    def test_cancel_patrol(self):
        nav = FakeNavigator()
        nav.done = False
        _bb(**{BB_NAV_GOAL_SENT: True})
        assert CancelPatrol(nav).update() == Status.SUCCESS
        assert nav.cancelled == 1
        assert py_trees.blackboard.Blackboard.get(BB_NAV_GOAL_SENT) is False

    def test_halt_stops_robot_and_navigation(self):
        nav, pub = FakeNavigator(), Recorder()
        nav.done = False
        assert HaltRobot(nav, pub).update() == Status.RUNNING
        assert nav.cancelled == 1
        assert pub.last.linear.x == 0.0 and pub.last.angular.z == 0.0

    def test_navigate_sends_goal_once(self):
        nav = FakeNavigator()
        _bb(**{BB_WP_INDEX: 1})
        node = NavigateToWaypoint(nav, WAYPOINTS)
        assert node.update() == Status.RUNNING
        assert node.update() == Status.RUNNING
        assert len(nav.goals) == 1
        goal = nav.goals[0]
        assert goal.header.frame_id == 'map'
        assert (goal.pose.position.x, goal.pose.position.y) == (-0.92, 6.85)
        nav.done = True
        assert node.update() == Status.SUCCESS

    def test_wait_at_waypoint(self):
        _bb(**{'dwell_start_time': None})
        assert WaitAtWaypoint(2.0).update() == Status.RUNNING
        _bb(**{'dwell_start_time': time.monotonic() - 5.0})
        assert WaitAtWaypoint(2.0).update() == Status.SUCCESS

    def test_increment_wraps(self):
        _bb(**{BB_WP_INDEX: 2})
        IncrementWaypoint(3).update()
        assert py_trees.blackboard.Blackboard.get(BB_WP_INDEX) == 0


# ---------------------------------------------------------------------------
# Whole tree
# ---------------------------------------------------------------------------

class TestTree:

    def _tree(self, nav, pub):
        return build_security_guard_tree(nav, pub, WAYPOINTS, search_secs=6.0)

    def test_structure(self):
        tree = self._tree(FakeNavigator(), Recorder())
        assert isinstance(tree.root, py_trees.composites.Selector)
        assert [c.name for c in tree.root.children] == [
            'EmergencyStop', 'IntruderProtocol', 'SearchProtocol', 'PatrolProtocol']
        for child in tree.root.children:
            assert isinstance(child, py_trees.composites.Sequence)

    def test_patrols_when_nothing_is_seen(self):
        nav, pub = FakeNavigator(), Recorder()
        tree = self._tree(nav, pub)
        tree.tick()
        assert active_protocol(tree) == 'PatrolProtocol'
        assert len(nav.goals) == 1

    def test_intruder_interrupts_patrol(self):
        nav, pub = FakeNavigator(), Recorder()
        tree = self._tree(nav, pub)
        tree.tick()                                   # patrol goal sent
        _bb(**{BB_LAST_SEEN: time.monotonic(), BB_TARGET_BEARING: 0.1,
               BB_TARGET_RANGE: 4.0})
        tree.tick()
        assert active_protocol(tree) == 'IntruderProtocol'
        assert nav.cancelled == 1
        assert pub.last.linear.x > 0.0

    def test_search_then_resume_patrol(self):
        nav, pub = FakeNavigator(), Recorder()
        tree = self._tree(nav, pub)
        _bb(**{BB_LAST_SEEN: time.monotonic() - 2.0, BB_TARGET_BEARING: 0.3})
        tree.tick()
        assert active_protocol(tree) == 'SearchProtocol'
        assert pub.last.angular.z > 0.0
        _bb(**{BB_LAST_SEEN: time.monotonic() - 60.0})
        tree.tick()
        assert active_protocol(tree) == 'PatrolProtocol'
        assert len(nav.goals) == 1

    def test_patrol_resumes_same_waypoint_after_interruption(self):
        nav, pub = FakeNavigator(), Recorder()
        tree = self._tree(nav, pub)
        _bb(**{BB_WP_INDEX: 2})
        tree.tick()
        _bb(**{BB_LAST_SEEN: time.monotonic()})
        tree.tick()
        _bb(**{BB_LAST_SEEN: -math.inf})
        tree.tick()
        assert len(nav.goals) == 2
        assert nav.goals[1].pose.position.x == nav.goals[0].pose.position.x == 5.13

    def test_estop_overrides_everything(self):
        nav, pub = FakeNavigator(), Recorder()
        tree = self._tree(nav, pub)
        _bb(**{BB_ESTOP: True, BB_LAST_SEEN: time.monotonic(),
               BB_TARGET_BEARING: 0.0, BB_TARGET_RANGE: 5.0})
        tree.tick()
        assert active_protocol(tree) == 'EmergencyStop'
        assert pub.last.linear.x == 0.0 and pub.last.angular.z == 0.0
        assert nav.goals == []
