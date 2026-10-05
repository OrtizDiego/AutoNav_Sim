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

"""Node-level tests for person_controller and security_guard_bt.

Also covers every node's ``main()`` and ``__main__`` entry point.
"""

import math
import os
import runpy
import types

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import py_trees
import pytest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from my_bot import person_controller as pc
from my_bot import security_guard_bt as sg

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.path.join(PKG_DIR, 'maps', 'my_map.yaml')


def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


# ---------------------------------------------------------------------------
# person_controller
# ---------------------------------------------------------------------------

class TestPersonControllerNode:

    @pytest.fixture
    def node(self, ros_params):
        ros_params['map_yaml'] = MAP_YAML
        return pc.PersonControllerNode()

    def _tick(self, node, n=1):
        for _ in range(n):
            node.clock.advance(node._dt)
            node.timers[0]()
        return (node.publishers['/person/cmd_vel'].last,
                node.publishers['/person_mode'].last)

    def test_interface(self, node):
        assert set(node.subscriptions) == {'/person/odom', '/odom', '/person_detected'}
        assert set(node.publishers) == {'/person/cmd_vel', '/person_mode'}
        assert node.timers[0].period == pytest.approx(0.05)
        assert MAP_YAML in node.logger.messages('info')[0]
        # Wall avoidance really uses the museum map
        assert node._brain.cmap.clearance(0.0, 0.0) > 0.5
        assert node._brain.cmap.clearance(100.0, 0.0) <= 0.0

    def test_without_a_map_it_stays_inside_bounds(self, ros_params):
        ros_params['bounds'] = 5.0
        node = pc.PersonControllerNode()
        assert '+/-5.0 m' in node.logger.messages('warn')[0]
        assert node._brain.cmap.clearance(0.0, 0.0) > 5.0
        assert node._brain.cmap.clearance(8.0, 0.0) <= 0.0

    def test_parameters_reach_the_brain(self, node, ros_params):
        ros_params.update(run_speed=3.0, update_rate_hz=10.0)
        node = pc.PersonControllerNode()
        assert node._brain.p.run_speed == 3.0
        assert node._dt == pytest.approx(0.1)

    def test_waits_for_the_person_odometry(self, node):
        assert self._tick(node) == (None, None)

    def test_odometry(self, node):
        node.subscriptions['/person/odom'](_odom(1.0, 2.0, yaw=0.5))
        node.subscriptions['/odom'](_odom(-1.0, 0.5))
        assert (node._person.x, node._person.y) == (1.0, 2.0)
        assert node._person.yaw == pytest.approx(0.5)
        assert node._robot_xy == (-1.0, 0.5)

    def test_walks_when_calm(self, node):
        node.subscriptions['/person/odom'](_odom(0.0, 0.0))
        cmd, mode = self._tick(node, 20)
        assert mode.data == pc.WALK
        assert 0.0 < cmd.linear.x <= 0.8

    def test_runs_when_detected_nearby(self, node):
        node.subscriptions['/person/odom'](_odom(0.0, 0.0))
        node.subscriptions['/odom'](_odom(-2.0, 0.0))
        node.subscriptions['/person_detected'](Bool(data=True))
        _, mode = self._tick(node)
        assert mode.data == pc.RUN
        assert 'WALK -> RUN' in node.logger.messages('info')[-1]

    def test_far_detection_or_lost_lock_does_not_alarm(self, node):
        node.subscriptions['/person/odom'](_odom(0.0, 0.0))
        node.subscriptions['/odom'](_odom(-20.0, 0.0))     # beyond notice_radius
        node.subscriptions['/person_detected'](Bool(data=True))
        assert self._tick(node)[1].data == pc.WALK
        node.subscriptions['/odom'](_odom(-2.0, 0.0))
        node.clock.advance(node._calm_down)                 # lock is stale
        node.subscriptions['/person_detected'](Bool(data=False))
        assert self._tick(node)[1].data == pc.WALK


# ---------------------------------------------------------------------------
# security_guard_bt
# ---------------------------------------------------------------------------

@pytest.fixture
def fresh_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


@pytest.fixture
def guard(fresh_blackboard, ros_params):
    """Guard on sensor_fusion's /target (use_track: false)."""
    ros_params.update(waypoint_dwell_secs=0.0, use_track=False)
    return sg.SecurityGuardBTNode()


@pytest.fixture
def tracked_guard(fresh_blackboard, ros_params):
    """Guard on target_tracker's /intruder/track (the default)."""
    ros_params['waypoint_dwell_secs'] = 0.0
    return sg.SecurityGuardBTNode()


def _track(node, x, y, vx=0.0, vy=0.0, t=None):
    msg = Odometry()
    if t is not None:
        msg.header.stamp = types.SimpleNamespace(
            sec=int(t), nanosec=round((t % 1) * 1e9))
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
    node.subscriptions['/intruder/track'](msg)


def _tick(node, n=1):
    for _ in range(n):
        node.timers[0]()
    return node.publishers['/security_guard/state'].last.data


def _see(node, bearing=0.0, rng=3.0):
    msg = Vector3Stamped()
    msg.vector.x, msg.vector.y = bearing, rng
    node.subscriptions['/target'](msg)


def _bb(key):
    return py_trees.blackboard.Blackboard.get(key)


class TestSecurityGuardNode:

    def test_interface(self, guard):
        assert set(guard.subscriptions) == {
            '/estop', '/target', '/scan', '/odom'}
        assert set(guard.publishers) == {
            '/cmd_vel', '/security_guard/state', '/security_guard/metrics',
            '/intruder_sightings'}
        assert [t.period for t in guard.timers] == [0.1, 5.0]
        assert guard._navigator.active                # waited for Nav2
        assert 'PatrolProtocol' in guard.logger.messages('info')[1]

    def test_waypoints_parameter_is_split_into_pairs(self, fresh_blackboard,
                                                     ros_params):
        ros_params['waypoints'] = [1.0, 2.0, 3.0, 4.0, 5.0]  # odd tail ignored
        node = sg.SecurityGuardBTNode()
        _tick(node)
        goal = node._navigator.goals[0]
        assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 2.0)
        assert node._bt.root.children[3].children[2]._n == 2

    def test_blackboard_starts_clean(self, guard):
        assert _bb(sg.BB_ESTOP) is False
        assert _bb(sg.BB_LAST_SEEN) == -math.inf
        assert _bb(sg.BB_TARGET_RANGE) == -1.0
        assert _bb(sg.BB_FRONT_CLEAR) == math.inf

    def test_sensor_callbacks_fill_the_blackboard(self, guard):
        _see(guard, float('nan'))
        assert _bb(sg.BB_LAST_SEEN) == -math.inf      # NaN is "no target"
        _see(guard, 0.3, 2.0)
        assert _bb(sg.BB_TARGET_BEARING) == pytest.approx(0.3)
        assert _bb(sg.BB_TARGET_RANGE) == pytest.approx(2.0)
        assert _bb(sg.BB_LAST_SEEN) == pytest.approx(guard.clock.seconds)  # sim time
        guard.subscriptions['/scan'](LaserScan(
            ranges=[0.5] * 360, angle_min=-math.pi, angle_increment=math.pi / 180))
        assert _bb(sg.BB_FRONT_CLEAR) == pytest.approx(0.5)
        guard.subscriptions['/estop'](Bool(data=True))
        assert _bb(sg.BB_ESTOP) is True

    def test_odometry_accumulates_distance(self, guard):
        guard.subscriptions['/odom'](_odom(0.0, 0.0))
        guard.subscriptions['/odom'](_odom(3.0, 4.0, yaw=1.0))
        assert guard._metrics['distance_traveled_m'] == pytest.approx(5.0)
        assert guard._robot_pose == pytest.approx((3.0, 4.0, 1.0))

    def test_target_is_anchored_in_odom(self, guard):
        _see(guard, 0.0, 2.0)
        assert guard._target.target_xy is None        # no robot pose yet
        guard.subscriptions['/odom'](_odom(1.0, 1.0, yaw=math.pi / 2))
        _see(guard, 0.0, 2.0)
        assert guard._target.target_xy == pytest.approx((1.0, 3.0))

    def test_blackboard_target_follows_the_robot_between_detections(self, guard):
        guard.subscriptions['/odom'](_odom(0.0, 0.0))
        _see(guard, 0.5, 3.0)
        guard.subscriptions['/odom'](_odom(0.0, 0.0, yaw=0.5))  # turned to it
        _tick(guard)
        assert _bb(sg.BB_TARGET_BEARING) == pytest.approx(0.0, abs=1e-9)
        assert _bb(sg.BB_TARGET_RANGE) == pytest.approx(3.0)

    def test_patrols_and_counts_waypoints(self, guard):
        assert _tick(guard) == 'PatrolProtocol'
        assert len(guard._navigator.goals) == 1
        guard._navigator.done = True                  # arrived
        _tick(guard, 3)                               # wait, then increment
        assert guard._metrics['waypoints_visited'] == 1
        assert len(guard._navigator.goals) == 2

    def test_intruder_is_followed_and_marked(self, guard):
        guard.subscriptions['/odom'](_odom(0.0, 0.0))
        _tick(guard)
        _see(guard, 0.0, 4.0)
        assert _tick(guard) == 'IntruderProtocol'
        assert guard._navigator.cancelled == 1
        assert guard.publishers['/cmd_vel'].last.linear.x > 0.0
        _see(guard, 0.0, 4.0)
        _tick(guard)
        assert guard._metrics['intruder_detections'] == 1
        assert guard._metrics['time_following_sec'] == pytest.approx(0.2)
        markers = guard.publishers['/intruder_sightings'].last.markers
        sphere, label = markers
        assert (sphere.pose.position.x, sphere.pose.position.y) == (4.0, 0.0)
        assert sphere.type == sphere.SPHERE
        assert label.text.startswith('#1 ')
        assert 'PatrolProtocol -> IntruderProtocol' in guard.logger.messages('info')

    def test_sighting_falls_back_to_the_robot_pose(self, guard):
        guard.subscriptions['/odom'](_odom(2.0, -1.0))
        _see(guard, 0.0, -1.0)                        # seen but not ranged
        _tick(guard)
        sphere = guard.publishers['/intruder_sightings'].last.markers[0]
        assert (sphere.pose.position.x, sphere.pose.position.y) == (2.0, -1.0)

    def test_no_sighting_marker_without_any_pose(self, guard):
        _see(guard)
        _tick(guard)
        assert guard._metrics['intruder_detections'] == 1
        assert guard.publishers['/intruder_sightings'].msgs == []

    def test_searches_after_losing_the_intruder(self, guard):
        _see(guard, -0.5)
        _tick(guard)
        guard.clock.advance(1.0)                      # sim time, not wall time
        assert _tick(guard) == 'SearchProtocol'
        assert guard.publishers['/cmd_vel'].last.angular.z < 0.0
        assert guard._metrics['track_losses'] == 1

    def test_estop_halts(self, guard):
        _tick(guard)
        guard.subscriptions['/estop'](Bool(data=True))
        assert _tick(guard) == 'EmergencyStop'
        cmd = guard.publishers['/cmd_vel'].last
        assert (cmd.linear.x, cmd.angular.z) == (0.0, 0.0)

    def test_metrics(self, guard):
        _tick(guard)
        guard.subscriptions['/odom'](_odom(0.0, 0.0))
        guard.subscriptions['/odom'](_odom(1.5, 0.0))
        guard.clock.advance(12.0)
        guard.timers[1]()
        status = guard.publishers['/security_guard/metrics'].last.status[0]
        values = {kv.key: kv.value for kv in status.values}
        assert status.name == 'SecurityGuardBT'
        assert status.message == 'PatrolProtocol | session 12s'
        assert values['state'] == 'PatrolProtocol'
        assert values['distance_traveled_m'] == '1.50'
        assert values['session_elapsed_sec'] == '12.0'
        assert values['track_losses'] == '0'
        assert values['follow_bearing_rms_deg'] == '0.0'

    def test_follow_quality_metrics(self, guard):
        guard.subscriptions['/odom'](_odom(0.0, 0.0))
        _see(guard, 0.1, 3.5)                         # 1 m beyond the stand-off
        _tick(guard, 2)
        bearing_rms, range_rms = guard._follow_rms()
        assert bearing_rms == pytest.approx(math.degrees(0.1))
        assert range_rms == pytest.approx(1.0)


class TestSecurityGuardOnTrack:

    def test_follows_the_intruder_track_by_default(self, tracked_guard):
        assert set(tracked_guard.subscriptions) == {
            '/estop', '/intruder/track', '/scan', '/odom'}

    def test_track_is_extrapolated_to_now(self, tracked_guard):
        g = tracked_guard
        g.subscriptions['/odom'](_odom(0.0, 0.0))
        # Seen at t=100 (the clock) 3 m ahead, walking left at 1 m/s
        _track(g, 3.0, 0.0, vx=0.0, vy=1.0, t=g.clock.seconds)
        assert _bb(sg.BB_LAST_SEEN) == pytest.approx(g.clock.seconds)
        assert _bb(sg.BB_TARGET_BEARING) == pytest.approx(0.0)
        g.clock.advance(0.5)
        assert _tick(g) == 'IntruderProtocol'
        assert _bb(sg.BB_TARGET_BEARING) == pytest.approx(math.atan2(0.5, 3.0))
        assert _bb(sg.BB_TARGET_RANGE) == pytest.approx(math.hypot(3.0, 0.5))
        assert g.publishers['/cmd_vel'].last.angular.z > 0.0  # leads it left

    def test_extrapolation_is_capped(self, tracked_guard):
        g = tracked_guard
        g.subscriptions['/odom'](_odom(0.0, 0.0))
        _track(g, 3.0, 0.0, vy=1.0, t=g.clock.seconds)
        g.clock.advance(0.9)                          # still within timeout
        g._refresh_target()
        far = _bb(sg.BB_TARGET_BEARING)
        g.clock.advance(5.0)                          # long gone
        g._refresh_target()
        assert _bb(sg.BB_TARGET_BEARING) == pytest.approx(math.atan2(1.0, 3.0))
        assert far == pytest.approx(math.atan2(0.9, 3.0))

    def test_sighting_marker_at_the_track(self, tracked_guard):
        g = tracked_guard
        g.subscriptions['/odom'](_odom(0.0, 0.0))
        _track(g, 2.0, 1.0, t=g.clock.seconds)
        _tick(g)
        sphere = g.publishers['/intruder_sightings'].last.markers[0]
        assert (sphere.pose.position.x, sphere.pose.position.y) == (2.0, 1.0)

    def test_non_finite_track_is_ignored(self, tracked_guard):
        _track(tracked_guard, float('nan'), 0.0)
        assert _bb(sg.BB_LAST_SEEN) == -math.inf


def test_active_protocol_is_idle_before_the_first_tick(fresh_blackboard):
    tree = sg.build_security_guard_tree(
        sg.BasicNavigator(), lambda cmd: None, [[0.0, 0.0]])
    assert sg.active_protocol(tree) == 'Idle'


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

NODES = {
    'ball_chaser': 'BallChaser',
    'ball_controller': 'BallControllerNode',
    'sensor_fusion': 'SensorFusionNode',
    'person_controller': 'PersonControllerNode',
    'person_tracker': 'PersonTrackerNode',
    'security_guard_bt': 'SecurityGuardBTNode',
    'system_monitor': 'SystemMonitorNode',
    'perf_monitor': 'PerfMonitorNode',
}


@pytest.fixture
def spun(monkeypatch):
    """Capture the node rclpy.spin() is given; spin returns at once."""
    import rclpy
    nodes = []
    monkeypatch.setattr(rclpy, 'spin', nodes.append)
    return nodes


@pytest.mark.parametrize('module', sorted(NODES))
def test_main_spins_the_node_and_cleans_up(spun, fresh_blackboard, module):
    mod = __import__(f'my_bot.{module}', fromlist=['main'])
    mod.main()
    assert type(spun[0]).__name__ == NODES[module]
    assert spun[0].destroyed


@pytest.mark.parametrize('module', sorted(NODES))
def test_runs_as_a_script(spun, fresh_blackboard, module):
    path = os.path.join(PKG_DIR, 'my_bot', f'{module}.py')
    runpy.run_path(path, run_name='__main__')
    assert type(spun[0]).__name__ == NODES[module]


def test_main_shuts_down_even_if_spin_fails(monkeypatch):
    import rclpy
    calls = []

    def boom(node):
        raise RuntimeError('spin failed')
    monkeypatch.setattr(rclpy, 'spin', boom)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: calls.append('shutdown'))
    with pytest.raises(RuntimeError):
        pc.main()
    assert calls == ['shutdown']


@pytest.mark.parametrize('module', sorted(NODES))
def test_ctrl_c_exits_cleanly(monkeypatch, fresh_blackboard, module):
    """Ctrl+C: rclpy shuts the context down itself; a second shutdown raised."""
    import rclpy
    calls = []

    def interrupted(node):
        raise KeyboardInterrupt
    monkeypatch.setattr(rclpy, 'spin', interrupted)
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: calls.append('shutdown'))
    __import__(f'my_bot.{module}', fromlist=['main']).main()
    assert calls == []
