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

"""Integration tests: scenario node graphs wired together without ROS.

``conftest.wire`` connects publishers to same-topic subscriptions, so a
message published by one node runs the other node's callback, as on a ROS
graph. The camera, lidar and odometry inputs are synthetic; YOLO is a fake
session that "sees" a scripted person box.
"""

import math
import types

import cv2
from nav_msgs.msg import Odometry
import numpy as np
import py_trees
import pytest
from sensor_msgs.msg import Image, LaserScan

from conftest import wire
from my_bot import ball_chaser as bch
from my_bot import ball_controller as bc
from my_bot import ball_teleop as bt
from my_bot import person_controller as pc
from my_bot import person_tracker as pt
from my_bot import security_guard_bt as sg
from my_bot import sensor_fusion as sf
from my_bot import system_monitor as sm
from my_bot import target_tracker as tt

W, H = 320, 240   # camera.xacro


def _scan(value, n=360):
    return LaserScan(ranges=[value] * n, angle_min=-math.pi,
                     angle_increment=2.0 * math.pi / n)


def _odom(x, y):
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    return msg


def _camera(frame):
    return Image(frame=frame)


def _ball_frame(cx):
    frame = np.full((H, W, 3), 90, dtype=np.uint8)
    if cx is not None:
        cv2.circle(frame, (cx, H // 2), 20, (0, 0, 255), -1)
    return frame


@pytest.fixture
def fresh_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


# ---------------------------------------------------------------------------
# ball-sim: camera + lidar -> sensor_fusion(hsv) -> ball_chaser -> /cmd_vel
# ---------------------------------------------------------------------------

class TestBallSim:

    @pytest.fixture
    def graph(self):
        fusion, chaser = sf.SensorFusionNode(), bch.BallChaser()
        wire(fusion, chaser)
        return fusion, chaser

    def _step(self, fusion, chaser, frame, scan):
        fusion.subscriptions['/scan'](scan)
        chaser.subscriptions['/scan'](scan)
        fusion.subscriptions['/camera/image_raw'](_camera(frame))
        chaser.timers[0]()
        cmd = chaser.publishers['/cmd_vel'].last
        return cmd.linear.x, cmd.angular.z

    def test_chases_a_far_ball_on_the_left(self, graph):
        v, w = self._step(*graph, _ball_frame(cx=100), _scan(3.0))
        assert v > 0.0 and w > 0.0

    def test_backs_off_a_ball_that_is_too_close(self, graph):
        v, w = self._step(*graph, _ball_frame(cx=W // 2), _scan(0.6))
        assert v < 0.0
        assert w == pytest.approx(0.0, abs=0.05)

    def test_holds_position_at_the_desired_distance(self, graph):
        v, _ = self._step(*graph, _ball_frame(cx=W // 2), _scan(1.0))
        assert v == 0.0

    def test_searches_toward_where_the_ball_left_the_view(self, graph):
        fusion, chaser = graph
        self._step(fusion, chaser, _ball_frame(cx=280), _scan(3.0))  # right
        chaser.clock.advance(1.0)
        v, w = self._step(fusion, chaser, _ball_frame(cx=None), _scan(3.0))
        assert v == 0.0 and w < 0.0               # turns right to find it


# ---------------------------------------------------------------------------
# ball-sim: keyboard teleop -> ball_controller -> /ball/cmd_vel
# ---------------------------------------------------------------------------

def test_teleop_takes_the_ball_over_from_the_autopilot(monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(bt, 'time', types.SimpleNamespace(monotonic=lambda: now[0]))
    teleop, ball = bt.BallTeleop(), bc.BallControllerNode()
    wire(teleop, ball)
    ball.subscriptions['/ball/odom'](_odom(5.0, 0.0))
    ball.subscriptions['/odom'](_odom(0.0, 0.0))
    teleop.press('w')                             # away from the robot
    for _ in range(30):
        now[0] += 0.05
        teleop.press('w')
        teleop.publish()
        ball.clock.advance(ball._dt)
        ball.timers[0]()
    cmd = ball.publishers['/ball/cmd_vel'].last
    assert ball._manual
    assert cmd.linear.x == pytest.approx(teleop.speed)
    assert cmd.linear.y == pytest.approx(0.0)
    # Keys released and teleop quit: the autopilot takes over again
    ball.clock.advance(2.0)
    ball.timers[0]()
    assert not ball._manual


# ---------------------------------------------------------------------------
# person-sim: tracker -> sensor_fusion(person) -> security guard (+ e-stop)
# ---------------------------------------------------------------------------

class FakeYolo:
    """Sees one person; the box is scripted by the test."""

    def __init__(self):
        self.box = None    # (cx, cy, w, h) in pixels

    def get_inputs(self):
        # The image's 320 px export: input pixels are camera pixels.
        return [types.SimpleNamespace(name='images', shape=[1, 3, 320, 320])]

    def run(self, outputs, feeds):
        assert feeds['images'].shape == (1, 3, 320, 320)
        out = np.zeros((1, 84, 2100), dtype=np.float32)
        if self.box is not None:
            out[0, :4, 0] = self.box
            out[0, 4, 0] = 0.9
        return [out]


class StaticTracker:
    """OpenCV tracker stand-in that holds the box it was given."""

    def __init__(self, preferred='kcf'):
        self.box = None

    def init(self, frame, box):
        self.box = box

    def update(self, frame):
        return True, self.box


@pytest.fixture
def person_sim(monkeypatch, ros_params, fresh_blackboard):
    monkeypatch.setattr(pt, 'make_tracker', StaticTracker)
    ros_params.update(mode='person', waypoint_dwell_secs=0.0,
                      model_path='/nonexistent/yolov8n.onnx',
                      async_detection=False)
    tracker = pt.PersonTrackerNode()
    tracker._session = FakeYolo()
    nodes = types.SimpleNamespace(
        tracker=tracker, fusion=sf.SensorFusionNode(),
        target_tracker=tt.TargetTrackerNode(),
        guard=sg.SecurityGuardBTNode(), monitor=sm.SystemMonitorNode(),
        person=pc.PersonControllerNode())
    wire(*vars(nodes).values())
    nodes.guard.subscriptions['/estop'](nodes.monitor.publishers['/estop'].last)
    return nodes


def _person_box(fusion, distance, cx):
    """Full-body pixel box of a 1.72 m person ``distance`` metres away."""
    h = fusion._fx * fusion._person_h / distance
    return (cx, fusion._cy, h / 3.0, h)


def _frame(sim, scan=None, frames=1):
    """Run ``frames`` camera frames (a track is confirmed after 3 hits)."""
    for _ in range(frames):
        frame = np.full((H, W, 3), 90, dtype=np.uint8)
        for node in (sim.target_tracker, sim.guard):
            node.subscriptions['/odom'](_odom(0.0, 0.0))
        if scan is not None:
            sim.fusion.subscriptions['/scan'](scan)
            sim.target_tracker.subscriptions['/scan'](scan)
            sim.guard.subscriptions['/scan'](scan)
        sim.tracker.subscriptions['/camera/image_raw'](_camera(frame))
        sim.guard.timers[0]()
    return sim.guard.publishers['/security_guard/state'].last.data


def _cmd(sim):
    cmd = sim.guard.publishers['/cmd_vel'].last
    return cmd.linear.x, cmd.angular.z


class TestPersonSim:

    def test_patrols_while_nobody_is_there(self, person_sim):
        assert _frame(person_sim) == 'PatrolProtocol'
        assert len(person_sim.guard._navigator.goals) == 1
        assert person_sim.fusion.publishers['/target_range'].last.data == -1.0

    def test_follows_a_detected_person(self, person_sim):
        _frame(person_sim)
        person_sim.tracker._session.box = _person_box(person_sim.fusion, 5.0, 240.0)
        assert _frame(person_sim, frames=2) == 'PatrolProtocol'  # tentative
        assert person_sim.target_tracker.publishers['/intruder/track'].msgs == []
        assert _frame(person_sim) == 'IntruderProtocol'          # confirmed
        assert person_sim.guard._navigator.cancelled == 1
        rng = person_sim.fusion.publishers['/target_range'].last.data
        assert rng == pytest.approx(5.0, rel=0.05)    # camera-only range
        v, w = _cmd(person_sim)
        assert v > 0.0                                # 5 m > 2.5 m stand-off
        assert w < 0.0                                # person on the right

    def test_lidar_ranges_the_person_when_it_agrees(self, person_sim):
        person_sim.tracker._session.box = _person_box(person_sim.fusion, 2.0, W / 2)
        _frame(person_sim, _scan(2.1), frames=3)
        assert person_sim.fusion.publishers['/target_range'].last.data == \
            pytest.approx(2.1)
        v, _ = _cmd(person_sim)
        assert v < 0.0                                # too close: back off

    def test_person_runs_from_the_robot_that_spotted_it(self, person_sim):
        person = person_sim.person
        person.subscriptions['/person/odom'](_odom(3.0, 0.0))
        person.subscriptions['/odom'](_odom(0.0, 0.0))
        person_sim.tracker._session.box = _person_box(person_sim.fusion, 3.0, W / 2)
        _frame(person_sim)
        assert person_sim.tracker.publishers['/person_detected'].last.data
        person.timers[0]()
        assert person.publishers['/person_mode'].last.data == pc.RUN

    def test_searches_then_returns_to_patrol(self, person_sim, monkeypatch):
        person_sim.tracker._session.box = _person_box(person_sim.fusion, 4.0, 100.0)
        _frame(person_sim, frames=3)
        person_sim.tracker._session.box = None
        last_seen = py_trees.blackboard.Blackboard.get(sg.BB_LAST_SEEN)
        clock = types.SimpleNamespace(now=last_seen + 1.0)
        for leaf in person_sim.guard._bt.root.iterate():
            if hasattr(leaf, '_clock'):
                monkeypatch.setattr(leaf, '_clock', lambda: clock.now)
        assert _frame(person_sim) == 'SearchProtocol'
        assert _cmd(person_sim) == (0.0, pytest.approx(0.6))  # last seen left
        clock.now += 10.0
        assert _frame(person_sim) == 'PatrolProtocol'

    def test_estop_overrides_the_chase_until_cleared(self, person_sim):
        person_sim.tracker._session.box = _person_box(person_sim.fusion, 5.0, W / 2)
        assert _frame(person_sim, frames=3) == 'IntruderProtocol'
        resp = person_sim.monitor.services['/trigger_estop'](
            None, types.SimpleNamespace())
        assert resp.success
        assert _frame(person_sim) == 'EmergencyStop'
        assert _cmd(person_sim) == (0.0, 0.0)
        person_sim.monitor.services['/clear_estop'](None, types.SimpleNamespace())
        assert _frame(person_sim) == 'IntruderProtocol'

    def test_system_monitor_reports_the_sensors_it_hears(self, person_sim):
        monitor = person_sim.monitor
        monitor.clock.advance(5.0)
        monitor.subscriptions['/scan'](_scan(3.0))
        monitor.subscriptions['/camera/image_raw'](_camera(None))
        monitor.timers[0]()
        assert monitor.publishers['/system_health'].last.status[0].message == \
            'All sensors OK'
