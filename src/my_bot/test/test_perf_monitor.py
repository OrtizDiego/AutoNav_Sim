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

"""Tests for perf_monitor (Phase 0 measurements)."""

import types

from geometry_msgs.msg import Twist, Vector3Stamped
import pytest
from rclpy.node import Node
from sensor_msgs.msg import Image

from my_bot import perf_monitor as pm


def _stamp(t):
    return types.SimpleNamespace(sec=int(t), nanosec=round((t % 1) * 1e9))


def test_topic_stats_rate_and_age():
    stats = pm.TopicStats()
    stats.add(10.0, 9.9)
    stats.add(10.5, 10.2)
    stats.add(11.0, None)                          # unstamped: rate only
    out = stats.summary(2.0)
    assert out['hz'] == pytest.approx(1.5)
    assert out['age_mean_ms'] == pytest.approx(200.0)
    assert out['age_max_ms'] == pytest.approx(300.0)
    assert stats.summary(2.0) == {'hz': 0.0}       # reset after a summary
    assert stats.summary(0.0) == {'hz': 0.0}


def test_real_time_factor():
    assert pm.real_time_factor(2.0, 4.0) == pytest.approx(0.5)
    assert pm.real_time_factor(1.0, 0.0) == 0.0


def test_report_table():
    text = pm.format_report(0.5, {
        '/target': {'hz': 4.0, 'age_mean_ms': 310.0, 'age_max_ms': 480.0},
        '/cmd_vel': {'hz': 10.0, 'publishers': 2}})
    assert text.splitlines()[0] == 'real-time factor 0.50'
    assert '310/480' in text
    assert text.splitlines()[-2].split() == ['/target', '-', '4.0', '310/480']
    assert text.splitlines()[-1].split() == ['/cmd_vel', '2', '10.0', '-']


class TestPerfMonitorNode:

    @pytest.fixture
    def node(self, monkeypatch):
        wall = [500.0]
        monkeypatch.setattr(pm, 'time', types.SimpleNamespace(monotonic=lambda: wall[0]))
        node = pm.PerfMonitorNode()
        node.wall = wall
        return node

    def test_interface(self, node, ros_params):
        assert set(node.subscriptions) == {
            '/camera/image_raw', '/scan', '/odom', '/person_bbox', '/target',
            '/intruder/track', '/cmd_vel'}
        assert set(node.publishers) == {'/perf_monitor'}
        assert node.timers[0].period == 5.0
        ros_params['watch_camera'] = False
        assert '/camera/image_raw' not in pm.PerfMonitorNode().subscriptions

    def test_first_window_before_clock_is_dropped(self, node):
        node._window = (0.0, node.wall[0])            # started before /clock
        target = Vector3Stamped()
        target.header.stamp = _stamp(node.clock.seconds - 1.0)
        node.subscriptions['/target'](target)
        node.wall[0] += 5.0
        node.timers[0]()
        assert node.publishers['/perf_monitor'].msgs == []
        assert node.logger.messages('info') == ['clock received, measuring']
        assert node._stats['/target'].count == 0
        node.clock.advance(1.0)
        node.wall[0] += 1.0
        node.timers[0]()
        assert node.publishers['/perf_monitor'].last.status[0].message == '1.00'

    def test_reports_rtf_rates_and_ages(self, node, monkeypatch):
        monkeypatch.setattr(Node, 'publisher_counts', {'/target': 1, '/cmd_vel': 1})
        # 2 s of simulation take 4 s of wall time: the sim runs at half speed
        for i in range(4):
            node.clock.advance(0.5)
            target = Vector3Stamped()
            target.header.stamp = _stamp(node.clock.seconds - 0.3)
            node.subscriptions['/target'](target)
            node.subscriptions['/cmd_vel'](Twist())
            node.subscriptions['/camera/image_raw'](Image())  # stamp unset
        node.wall[0] += 4.0
        node.timers[0]()
        status = {s.name: s for s in node.publishers['/perf_monitor'].last.status}
        rtf = status['perf_monitor: real_time_factor']
        assert rtf.message == '0.50'
        assert rtf.level == rtf.WARN
        target = {kv.key: kv.value for kv in status['perf_monitor: /target'].values}
        assert target == {'hz': '2.0', 'age_mean_ms': '300.0', 'age_max_ms': '300.0',
                          'publishers': '1'}
        assert status['perf_monitor: /cmd_vel'].message == '2.0 Hz'
        assert 'real-time factor 0.50' in node.logger.messages('info')[0]
