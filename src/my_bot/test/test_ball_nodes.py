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

"""Node-level tests for ball_chaser, ball_controller and ball_teleop.

The nodes run on conftest.py's fake rclpy: callbacks are called directly,
timers are fired by hand and the clock only moves when a test advances it.
"""

import math
import os
import runpy
import select
import sys
import termios
import tty
import types

from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from my_bot import ball_chaser as bch
from my_bot import ball_controller as bc
from my_bot import ball_teleop as bt


def _target(node, bearing, rng=-1.0, stamp=None):
    msg = Vector3Stamped()
    msg.header.stamp = stamp
    msg.vector.x, msg.vector.y = bearing, rng
    node.subscriptions['/target'](msg)


def _odom_at(t, x, y, yaw):
    msg = Odometry()
    msg.header.stamp = types.SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9))
    p = msg.pose.pose
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


def _scan(value=5.0, n=360):
    return LaserScan(ranges=[value] * n, angle_min=-math.pi,
                     angle_increment=2.0 * math.pi / n)


def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _teleop(x, y, frame):
    msg = TwistStamped()
    msg.header.frame_id = frame
    msg.twist.linear.x, msg.twist.linear.y = x, y
    return msg


# ---------------------------------------------------------------------------
# ball_chaser
# ---------------------------------------------------------------------------

class TestBallChaser:

    @pytest.fixture
    def node(self):
        return bch.BallChaser()

    def _tick(self, node):
        node.timers[0]()
        return node.publishers['/cmd_vel'].last

    def test_interface(self, node):
        assert set(node.subscriptions) == {'/target', '/odom', '/scan'}
        assert set(node.publishers) == {'/cmd_vel'}
        assert node.timers[0].period == pytest.approx(0.05)
        assert node.parameters['desired_distance'] == 1.0

    def test_parameters_are_read(self, ros_params):
        ros_params.update(desired_distance=2.0, search_secs=5.0)
        node = bch.BallChaser()
        assert node._desired == 2.0
        assert node._search_secs == 5.0

    def test_idle_before_any_target(self, node):
        cmd = self._tick(node)
        assert (cmd.linear.x, cmd.angular.z) == (0.0, 0.0)

    def test_drives_toward_a_far_target(self, node):
        _target(node, 0.2, 3.0)
        cmd = self._tick(node)
        assert cmd.linear.x > 0.0
        assert cmd.angular.z > 0.0

    def test_nan_bearing_is_not_a_sighting(self, node):
        _target(node, float('nan'))
        assert node._last_seen < 0.0

    def test_unknown_range_only_turns(self, node):
        _target(node, -0.3, -1.0)
        assert node._target.relative() == (pytest.approx(-0.3), None)
        cmd = self._tick(node)
        assert cmd.linear.x == 0.0
        assert cmd.angular.z < 0.0

    def test_obstacle_ahead_stops_forward_motion(self, node):
        node.subscriptions['/scan'](_scan(0.2))
        assert node._front_clear == pytest.approx(0.2)
        _target(node, 0.0, 4.0)
        assert self._tick(node).linear.x == 0.0

    def test_steers_on_where_the_ball_is_now_not_where_it_was(self, node):
        # Image taken at t=10 s with the ball 0.4 rad left; by the time the
        # detection arrives the robot has already turned 0.4 rad left.
        node.subscriptions['/odom'](_odom_at(10.0, 0.0, 0.0, 0.0))
        node.subscriptions['/odom'](_odom_at(10.3, 0.0, 0.0, 0.4))
        _target(node, 0.4, 3.0, stamp=types.SimpleNamespace(sec=10, nanosec=0))
        cmd = self._tick(node)
        assert cmd.angular.z == pytest.approx(0.0, abs=1e-6)  # no overshoot
        assert cmd.linear.x > 0.0

    def test_searches_toward_last_seen_side_then_gives_up(self, node):
        _target(node, 0.4)
        node.clock.advance(1.0)                       # lost, searching
        cmd = self._tick(node)
        assert cmd.linear.x == 0.0
        assert cmd.angular.z == pytest.approx(node._search_speed)
        node.clock.advance(node._search_secs)         # search over
        cmd = self._tick(node)
        assert (cmd.linear.x, cmd.angular.z) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# ball_controller
# ---------------------------------------------------------------------------

class TestBallController:

    @pytest.fixture
    def node(self):
        return bc.BallControllerNode()

    def _tick(self, node, n=1, dt=None):
        for _ in range(n):
            node.clock.advance(dt if dt is not None else node._dt)
            node.timers[0]()
        return node.publishers['/ball/cmd_vel'].last

    def test_interface(self, node):
        assert set(node.subscriptions) == {'/ball/odom', '/odom', '/ball/teleop'}
        assert set(node.publishers) == {'/ball/cmd_vel'}
        assert node.timers[0].period == pytest.approx(1.0 / 30.0)
        assert 'autopilot on' in node.logger.messages('info')[0]

    def test_waits_for_the_ball_odometry(self, node):
        assert self._tick(node) is None

    def test_first_odometry_snaps_the_autopilot(self, node):
        start = node._pilot.point_at(7.0)
        node.subscriptions['/ball/odom'](_odom(*start))
        assert math.dist(node._pilot.point_at(node._pilot.s), start) < 0.1
        s = node._pilot.s
        node.subscriptions['/ball/odom'](_odom(start[0] + 1.0, start[1]))
        assert node._pilot.s == s                     # only the first one

    def test_autopilot_drives_the_ball(self, node):
        ball = node._pilot.point_at(0.0)
        node.subscriptions['/ball/odom'](_odom(*ball))
        node.subscriptions['/odom'](_odom(ball[0] - 1.0, ball[1]))  # close: flee
        assert node._robot == (ball[0] - 1.0, ball[1])
        cmd = self._tick(node, 30)
        assert math.hypot(cmd.linear.x, cmd.linear.y) > 0.3
        assert cmd.angular.z == 0.0

    def test_yaw_is_corrected_back_to_zero(self, node):
        node.subscriptions['/ball/odom'](_odom(0.0, 0.0, yaw=0.1))
        assert node._ball_yaw == pytest.approx(0.1)
        assert self._tick(node).angular.z == pytest.approx(-0.2)

    def test_acceleration_is_limited(self, node):
        node.subscriptions['/ball/odom'](_odom(5.0, 0.0))
        node.subscriptions['/ball/teleop'](_teleop(1.0, 0.0, 'world'))
        cmd = self._tick(node, dt=0.0)
        assert math.hypot(cmd.linear.x, cmd.linear.y) == pytest.approx(
            node._accel * node._dt)

    def test_teleop_world_frame(self, node):
        node.subscriptions['/ball/odom'](_odom(5.0, 0.0))
        for _ in range(40):
            node.subscriptions['/ball/teleop'](_teleop(0.0, 0.5, 'world'))
            cmd = self._tick(node)
        assert (cmd.linear.x, cmd.linear.y) == pytest.approx((0.0, 0.5))
        assert 'teleop' in node.logger.messages('info')

    def test_teleop_robot_frame_is_relative_to_the_robot(self, node):
        node.subscriptions['/ball/odom'](_odom(0.0, 3.0))  # north of the robot
        node.subscriptions['/odom'](_odom(0.0, 0.0))
        for _ in range(40):
            node.subscriptions['/ball/teleop'](_teleop(0.5, 0.0, 'robot'))
            cmd = self._tick(node)
        assert (cmd.linear.x, cmd.linear.y) == pytest.approx((0.0, 0.5))  # away

    def test_autopilot_resumes_after_teleop_timeout(self, node):
        node.subscriptions['/ball/odom'](_odom(5.0, 0.0))
        node.subscriptions['/ball/teleop'](_teleop(0.5, 0.0, 'world'))
        self._tick(node)
        assert node._manual
        self._tick(node, dt=node._teleop_timeout + 0.1)
        assert not node._manual
        assert node.logger.messages('info')[-1] == 'autopilot'

    def test_autopilot_off_holds_the_ball(self, ros_params):
        ros_params['autopilot'] = False
        node = bc.BallControllerNode()
        assert 'autopilot off' in node.logger.messages('info')[0]
        node.subscriptions['/ball/odom'](_odom(5.0, 0.0))
        cmd = self._tick(node, 10)
        assert (cmd.linear.x, cmd.linear.y) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# ball_teleop
# ---------------------------------------------------------------------------

class FakeClock:
    """Replaces the time module inside ball_teleop."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


class TestBallTeleop:

    @pytest.fixture
    def clock(self, monkeypatch):
        clock = FakeClock()
        monkeypatch.setattr(bt, 'time', clock)
        return clock

    @pytest.fixture
    def node(self, clock):
        return bt.BallTeleop()

    def _published(self, node):
        node.publish()
        msg = node.publishers['/ball/teleop'].last
        return msg.header.frame_id, msg.twist.linear.x, msg.twist.linear.y

    def test_starts_still_in_robot_frame(self, node):
        assert self._published(node) == ('robot', 0.0, 0.0)

    def test_held_key_moves_then_stops_after_release(self, node, clock):
        assert node.press('a') is False
        assert self._published(node) == ('robot', 0.0, pytest.approx(0.6))
        clock.t += bt.HOLD_SECS + 0.01
        assert self._published(node) == ('robot', 0.0, 0.0)

    def test_stop_key(self, node):
        node.press('w')
        assert node.press('x') is False
        assert self._published(node)[1] == 0.0
        node.press('w')
        node.press(' ')
        assert self._published(node)[1] == 0.0

    def test_frame_toggle(self, node):
        assert node.press('m') is True
        assert node.frame == 'world'
        assert node.press('m') is True
        assert node.frame == 'robot'

    def test_speed_keys_are_clamped(self, node):
        for _ in range(20):
            assert node.press('+') is True
        assert node.speed == pytest.approx(1.5)
        node.press('=')
        assert node.speed == pytest.approx(1.5)
        for _ in range(20):
            node.press('-')
        assert node.speed == pytest.approx(0.1)
        node.press('_')
        assert node.speed == pytest.approx(0.1)

    def test_unknown_key_is_ignored(self, node):
        assert node.press('p') is False
        assert self._published(node) == ('robot', 0.0, 0.0)

    def test_read_key(self, monkeypatch):
        stdin = types.SimpleNamespace(read=lambda n: 'w')
        monkeypatch.setattr(bt, 'sys', types.SimpleNamespace(stdin=stdin))
        ready = []
        monkeypatch.setattr(bt, 'select', types.SimpleNamespace(
            select=lambda r, w, x, timeout: (ready, [], [])))
        assert bt._read_key(0.05) == ''
        ready.append(stdin)
        assert bt._read_key(0.05) == 'w'


# ---------------------------------------------------------------------------
# ball_teleop.main(): the keyboard loop
# ---------------------------------------------------------------------------

class FakeTerminal:
    """termios/tty/sys stand-ins for ball_teleop.main()."""

    def __init__(self, keys):
        self.keys = list(keys)
        self.restored = None
        self.cbreak = False
        self.stdin = types.SimpleNamespace(fileno=lambda: 0)

    def read_key(self, timeout):
        key = self.keys.pop(0)
        if isinstance(key, BaseException):
            raise key
        return key


@pytest.fixture
def terminal(monkeypatch):
    def install(keys):
        term = FakeTerminal(keys)
        monkeypatch.setattr(bt, '_read_key', term.read_key)
        monkeypatch.setattr(bt, 'sys', types.SimpleNamespace(stdin=term.stdin))
        monkeypatch.setattr(bt, 'tty', types.SimpleNamespace(
            setcbreak=lambda fd: setattr(term, 'cbreak', True)))
        monkeypatch.setattr(bt, 'termios', types.SimpleNamespace(
            TCSADRAIN=1, tcgetattr=lambda f: 'saved',
            tcsetattr=lambda f, when, s: setattr(term, 'restored', s)))
        term.nodes = []
        original = bt.BallTeleop

        def make_node():
            term.nodes.append(original())
            return term.nodes[-1]
        monkeypatch.setattr(bt, 'BallTeleop', make_node)
        return term
    return install


def test_teleop_main_loop_until_ctrl_c(terminal, capsys):
    term = terminal(['m', 'w', '', '\x03'])
    bt.main()
    node = term.nodes[0]
    out = capsys.readouterr().out
    assert out.count('Ball teleop') == 2          # start + after 'm'
    assert 'world frame' in out
    pub = node.publishers['/ball/teleop']
    assert len(pub.msgs) == 3                     # one per loop, not after ^C
    assert pub.msgs[1].twist.linear.x > 0.0       # 'w' pressed
    assert term.cbreak and term.restored == 'saved'
    assert node.destroyed


def test_teleop_main_restores_terminal_on_keyboard_interrupt(terminal):
    term = terminal([KeyboardInterrupt()])
    bt.main()
    assert term.restored == 'saved'
    assert term.nodes[0].destroyed


def test_teleop_runs_as_a_script(monkeypatch, capsys):
    """``python3 ball_teleop.py`` with a fake terminal that types Ctrl-C."""
    class Stdin:
        def fileno(self):
            return 0

        def read(self, n):
            return '\x03'
    restored = []
    monkeypatch.setattr(sys, 'stdin', Stdin())
    monkeypatch.setattr(termios, 'tcgetattr', lambda f: 'saved')
    monkeypatch.setattr(termios, 'tcsetattr', lambda f, w, s: restored.append(s))
    monkeypatch.setattr(tty, 'setcbreak', lambda fd: None)
    monkeypatch.setattr(select, 'select', lambda r, w, x, t: (r, [], []))
    runpy.run_path(os.path.join(os.path.dirname(bt.__file__), 'ball_teleop.py'),
                   run_name='__main__')
    assert restored == ['saved']
    assert 'Ball teleop' in capsys.readouterr().out


def test_teleop_main_stops_when_ros_shuts_down(terminal, monkeypatch):
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    term = terminal([])
    bt.main()
    assert term.nodes[0].publishers['/ball/teleop'].msgs == []
    assert term.restored == 'saved'
