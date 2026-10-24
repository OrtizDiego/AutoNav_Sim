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

"""Phase 5: run metrics against ground truth, and the evaluator node."""

import json
import math
import os
import random

from nav_msgs.msg import Odometry
import numpy as np
import pytest
from std_msgs.msg import String

from my_bot import scenario_evaluator as se
from my_bot.clearance_map import ClearanceMap
from my_bot.scenario_metrics import (
    FloorCoverage, ScenarioMetrics, check_targets, format_report, in_camera_view)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.path.join(PKG_DIR, 'maps', 'my_map.yaml')
HFOV = 1.089


@pytest.fixture(scope='module')
def museum():
    return ClearanceMap.from_yaml(MAP_YAML)


def _open_floor():
    return ClearanceMap.from_bounds(10.0)


def test_clearance_many_matches_clearance(museum):
    rng = random.Random(1)
    pts = [(rng.uniform(-9.0, 9.0), rng.uniform(-10.0, 11.0)) for _ in range(300)]
    xs, ys = np.array(pts).T
    assert museum.clearance_many(xs, ys) == pytest.approx(
        [museum.clearance(x, y) for x, y in pts])
    assert museum.clearance_many(np.array([100.0]), np.array([0.0]))[0] == 0.0


class TestInView:

    def test_field_of_view_and_range(self):
        cmap = _open_floor()
        robot = (0.0, 0.0, 0.0)
        assert in_camera_view(cmap, robot, (4.0, 0.0), HFOV, 8.0)
        assert in_camera_view(cmap, robot, (4.0, 2.0), HFOV, 8.0)  # 26.6 deg < 31.2
        assert not in_camera_view(cmap, robot, (4.0, 3.0), HFOV, 8.0)  # 36.9 deg
        assert not in_camera_view(cmap, robot, (-4.0, 0.0), HFOV, 8.0)
        assert not in_camera_view(cmap, robot, (9.0, 0.0), HFOV, 8.0)
        assert in_camera_view(cmap, (0.0, 0.0, math.pi), (-4.0, 0.0), HFOV, 8.0)

    def test_walls_block_the_view(self, museum):
        # From the spawn, facing north, the U-shaped exhibit's top bar
        # (y ~2.7-3.3) hides the north hall.
        robot = (0.0, 0.0, math.pi / 2)
        assert in_camera_view(museum, robot, (0.0, 1.5), HFOV, 8.0)
        assert not in_camera_view(museum, robot, (0.0, 5.0), HFOV, 8.0)


class TestFloorCoverage:

    def test_sees_only_the_cone(self):
        cov = FloorCoverage(_open_floor(), HFOV, 4.0)
        assert cov.fraction == 0.0
        new = cov.observe((0.0, 0.0, 0.0))
        assert new > 0 and cov.fraction == pytest.approx(new / cov.seen.size)
        assert cov.observe((0.0, 0.0, 0.0)) == 0  # nothing new
        seen = np.nonzero(cov.seen)[0]
        assert (cov.xs[seen] > 0.0).all()
        assert (np.hypot(cov.xs[seen], cov.ys[seen]) <= 4.0).all()

    def test_turning_around_sees_more(self):
        cov = FloorCoverage(_open_floor(), HFOV, 6.0)
        for k in range(12):
            cov.observe((0.0, 0.0, k * math.pi / 6))
        # A full turn: the 6 m disc of a 20 m square, about 28 %
        assert cov.fraction == pytest.approx(math.pi * 36 / 400, abs=0.04)

    def test_walls_block_coverage(self, museum):
        cov = FloorCoverage(museum, HFOV, 8.0)
        cov.observe((0.0, 0.0, math.pi / 2))
        seen = np.nonzero(cov.seen)[0]
        assert seen.size > 0
        assert (cov.ys[seen] < 3.0).all()  # nothing beyond the exhibit


def _feed(m, script, dt=0.1):
    """script: list of (seconds, robot, person, track_state, track_xy, guard)."""
    t = 0.0
    for secs, robot, person, state, track, guard in script:
        for _ in range(int(round(secs / dt))):
            m.add(t, robot, person, state, track, guard)
            t += dt
    return t


class TestScenarioMetrics:

    ROBOT = (0.0, 0.0, 0.0)

    def _metrics(self):
        return ScenarioMetrics(_open_floor(), HFOV, 8.0, match_radius=1.0)

    def test_a_clean_detection(self):
        m = self._metrics()
        _feed(m, [
            (2.0, self.ROBOT, (5.0, 6.0), 'none', None, 'PatrolProtocol'),  # out of view
            (1.0, self.ROBOT, (4.0, 0.0), 'tentative', None, 'PatrolProtocol'),
            (5.0, self.ROBOT, (4.0, 0.0), 'confirmed', (4.2, 0.1), 'IntruderProtocol'),
            (2.0, self.ROBOT, (5.0, 6.0), 'lost', None, 'SearchProtocol'),
        ])
        r = m.report()
        assert r['duration_s'] == pytest.approx(9.9)
        assert r['first_in_view_s'] == pytest.approx(2.0)
        assert r['time_to_detect_s'] == pytest.approx(1.0)
        assert r['in_view_pct'] == pytest.approx(5.0 / 7.0 * 100, abs=1.5)
        assert r['tracked_in_view_pct'] == pytest.approx(5.0 / 6.0 * 100, abs=1.5)
        assert r['track_breaks'] == 1
        assert r['track_breaks_per_min'] == pytest.approx(1 / (6.9 / 60), abs=0.2)
        assert r['false_alarms'] == 0 and r['false_track_pct'] == 0.0
        assert r['follow_range_mean_m'] == pytest.approx(4.0)
        assert r['guard_state_pct']['IntruderProtocol'] == pytest.approx(50.5, abs=1.5)
        assert r['patrol_coverage_pct'] > 0.0

    def test_a_track_on_an_exhibit_is_a_false_alarm(self):
        m = self._metrics()
        _feed(m, [
            (2.0, self.ROBOT, (5.0, 6.0), 'confirmed', (3.0, -1.0), ''),
            (1.0, self.ROBOT, (5.0, 6.0), 'none', None, ''),
            (2.0, self.ROBOT, (4.0, 0.0), 'confirmed', (4.0, 0.0), ''),
        ])
        r = m.report()
        assert r['false_alarms'] == 1  # the second episode is on the person
        assert r['false_track_pct'] == pytest.approx(50.0, abs=3.0)
        assert r['track_breaks'] == 1

    def test_never_detected(self):
        m = self._metrics()
        _feed(m, [(3.0, self.ROBOT, (4.0, 0.0), 'none', None, '')])
        r = m.report()
        assert r['time_to_detect_s'] is None
        assert r['in_view_pct'] is None and r['track_breaks_per_min'] is None
        assert r['tracked_in_view_pct'] == 0.0
        assert check_targets(r, {'max_time_to_detect_s': 5.0}) == [
            'the person was never detected']

    def test_events_are_counted(self):
        m = self._metrics()
        for kind in ('intruder_detected', 'intruder_lost', 'intruder_detected'):
            m.event(kind)
        assert m.report()['events'] == {'intruder_detected': 2, 'intruder_lost': 1}


def test_check_targets():
    report = {'time_to_detect_s': 2.0, 'tracked_in_view_pct': 70.0,
              'track_breaks_per_min': 0.5, 'false_alarms': 1,
              'patrol_coverage_pct': 10.0}
    limits = {'max_time_to_detect_s': 5.0, 'min_tracked_in_view_pct': 80.0,
              'max_track_breaks_per_min': 1.0, 'max_false_alarms': 0.0,
              'min_patrol_coverage_pct': -1.0}
    assert check_targets(report, limits) == [
        'tracked_in_view_pct 70.0 (min 80.0)', 'false_alarms 1 (max 0.0)']
    text = format_report('sprint', report, ['x'])
    assert text.startswith('scenario sprint: FAIL') and 'missed: x' in text
    assert format_report('sprint', report, []).startswith('scenario sprint: PASS')


# ---------------------------------------------------------------------------
# scenario_evaluator node
# ---------------------------------------------------------------------------

def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class TestEvaluatorNode:

    @pytest.fixture
    def node(self, ros_params, tmp_path):
        ros_params.update(map_yaml=MAP_YAML, scenario='sprint', linger_secs=2.0,
                          timeout_secs=60.0,
                          report_path=str(tmp_path / 'run' / 'report.json'))
        return se.ScenarioEvaluatorNode()

    def _tick(self, node, secs):
        for _ in range(int(round(secs / 0.1))):
            node.clock.advance(0.1)
            node.timers[0]()

    def test_interface(self, node):
        assert set(node.subscriptions) == {
            '/odom', '/person/odom', '/intruder/track', '/intruder/state',
            '/security_guard/state', '/security_events', '/scenario/status'}
        assert set(node.publishers) == {'/scenario/report'}
        assert node.timers[0].period == pytest.approx(0.1)

    def test_needs_the_map(self, ros_params):
        with pytest.raises(ValueError, match='map_yaml'):
            se.ScenarioEvaluatorNode()

    def test_default_report_path(self, ros_params):
        ros_params.update(map_yaml=MAP_YAML, scenario='hide')
        node = se.ScenarioEvaluatorNode()
        assert os.path.basename(node.report_path).startswith('hide_')
        assert node.report_path.startswith(os.path.expanduser('~/.ros/autonav_scenarios'))

    def test_a_scored_run(self, node):
        sub = node.subscriptions
        self._tick(node, 1.0)
        assert node.metrics.start is None  # no odometry yet
        sub['/odom'](_odom(0.0, 0.0, yaw=math.pi / 2))
        sub['/person/odom'](_odom(0.0, 2.0))
        sub['/security_guard/state'](String(data='PatrolProtocol'))
        self._tick(node, 1.0)
        sub['/intruder/state'](String(data='confirmed'))
        sub['/security_events'](String(data=json.dumps({'event': 'intruder_detected'})))
        sub['/security_events'](String(data='not json'))

        def follow(secs):
            for _ in range(int(round(secs / 0.1))):
                sub['/intruder/track'](_odom(0.1, 2.1))
                self._tick(node, 0.1)
        follow(2.0)
        sub['/scenario/status'](String(data=json.dumps({'phase': 'moving'})))
        sub['/scenario/status'](String(data='{broken'))
        follow(1.0)
        assert not node.finished
        sub['/scenario/status'](String(data=json.dumps({'phase': 'done'})))
        follow(1.5)
        assert not node.finished  # lingering
        follow(1.0)
        assert node.finished
        with open(node.report_path) as f:
            report = json.load(f)
        assert report == json.loads(node.publishers['/scenario/report'].last.data)
        assert report['scenario'] == 'sprint' and report['outcome'] == 'completed'
        assert report['time_to_detect_s'] == pytest.approx(1.0, abs=0.15)
        assert report['events'] == {'intruder_detected': 1, 'unparsed': 1}
        assert report['passed'], report['failures']
        assert 'scenario sprint: PASS' in node.logger.messages('info')[-1]
        last = node.metrics.last
        self._tick(node, 1.0)  # finished: no more samples
        assert node.metrics.last == last

    def test_the_track_coasts_with_its_velocity(self, node):
        sub = node.subscriptions
        sub['/odom'](_odom(0.0, 0.0, yaw=math.pi / 2))
        sub['/intruder/state'](String(data='confirmed'))
        track = _odom(0.0, 2.0)
        track.twist.twist.linear.y = 1.0  # walking away at 1 m/s
        sub['/intruder/track'](track)
        for k in range(15):  # the person walks on; no track message
            sub['/person/odom'](_odom(0.0, 2.0 + 0.1 * k))
            self._tick(node, 0.1)
        report = node.metrics.report()
        assert report['tracked_in_view_pct'] == 100.0
        assert report['false_alarms'] == 0

    def test_a_stale_track_is_unknown_not_false(self, node):
        sub = node.subscriptions
        sub['/odom'](_odom(0.0, 0.0, yaw=math.pi / 2))
        sub['/person/odom'](_odom(0.0, 2.0))
        sub['/intruder/state'](String(data='confirmed'))
        sub['/intruder/track'](_odom(0.0, 2.0))
        self._tick(node, 4.0)
        report = node.metrics.report()
        assert report['tracked_in_view_pct'] == pytest.approx(50.0, abs=3.0)
        assert report['false_track_pct'] == 0.0 and report['false_alarms'] == 0

    def test_timeout(self, node):
        node.subscriptions['/odom'](_odom(0.0, 0.0))
        self._tick(node, 61.0)
        assert node.finished
        assert node.report['outcome'] == 'timeout' and not node.report['passed']
        assert any('timeout' in f for f in node.report['failures'])


class TestEvaluatorMain:

    def test_spins_until_scored(self, monkeypatch, ros_params, tmp_path):
        import rclpy
        ros_params.update(map_yaml=MAP_YAML, report_path=str(tmp_path / 'r.json'))
        spun = []

        def spin_once(node, timeout_sec=None):
            spun.append(node)
            node.finished = len(spun) == 3
        monkeypatch.setattr(rclpy, 'spin_once', spin_once, raising=False)
        se.main()
        assert len(spun) == 3 and spun[0].destroyed

    def test_ctrl_c_scores_what_it_has(self, monkeypatch, ros_params, tmp_path):
        import rclpy
        ros_params.update(map_yaml=MAP_YAML, report_path=str(tmp_path / 'r.json'))
        nodes = []

        def spin_once(node, timeout_sec=None):
            nodes.append(node)
            node.subscriptions['/odom'](_odom(0.0, 0.0))
            node.timers[0]()
            raise KeyboardInterrupt
        monkeypatch.setattr(rclpy, 'spin_once', spin_once, raising=False)
        se.main()
        assert nodes[0].report['outcome'] == 'interrupted'
        assert os.path.exists(tmp_path / 'r.json')
