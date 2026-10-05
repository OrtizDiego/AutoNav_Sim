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

"""Tests for the target_tracker node (/target + /scan -> /intruder/track)."""

import math
import types

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import LaserScan

from my_bot import target_tracker as tt
from my_bot.target_estimate import stamp_to_sec


def _stamp(t):
    return types.SimpleNamespace(sec=int(t), nanosec=round((t % 1) * 1e9))


def _odom(t, x=0.0, y=0.0, yaw=0.0):
    msg = Odometry()
    msg.header.stamp = _stamp(t)
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    q = msg.pose.pose.orientation
    q.z, q.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return msg


def _target(t, bearing, rng):
    msg = Vector3Stamped()
    msg.header.stamp = _stamp(t) if t is not None else None
    msg.vector.x, msg.vector.y = bearing, rng
    return msg


def _leg_scan(t, x, y, n=360):
    """Scan with one person-sized blob (radius 0.15 m) at (x, y), laser frame."""
    inc = 2 * math.pi / n
    ranges = []
    for i in range(n):
        a = -math.pi + i * inc
        proj = x * math.cos(a) + y * math.sin(a)
        perp2 = x * x + y * y - proj * proj
        ranges.append(proj - math.sqrt(0.0225 - perp2)
                      if proj > 0 and perp2 <= 0.0225 else float('inf'))
    msg = LaserScan(ranges=ranges, angle_min=-math.pi, angle_increment=inc,
                    range_min=0.12)
    msg.header.stamp = _stamp(t)
    return msg


@pytest.fixture
def node(ros_params):
    n = tt.TargetTrackerNode()
    n.subscriptions['/odom'](_odom(99.0))
    n.subscriptions['/odom'](_odom(101.0))
    return n


def _confirm(node, bearing=0.0, rng=3.0, t0=100.0):
    for i in range(3):
        node.subscriptions['/target'](_target(t0 + i * 0.1, bearing, rng))


def test_interface(node):
    assert set(node.subscriptions) == {'/odom', '/target', '/scan'}
    assert set(node.publishers) == {
        '/intruder/track', '/intruder/predicted', '/intruder/state',
        '/intruder/markers'}
    assert node.timers[0].period == pytest.approx(0.1)


def test_lidar_can_be_disabled(ros_params):
    ros_params['use_lidar'] = False
    assert '/scan' not in tt.TargetTrackerNode().subscriptions


def test_publishes_only_a_confirmed_track(node):
    track = node.publishers['/intruder/track']
    node.subscriptions['/target'](_target(100.0, 0.0, 3.0))
    node.subscriptions['/target'](_target(100.1, 0.0, 3.0))
    assert track.msgs == []
    node.subscriptions['/target'](_target(100.2, 0.0, 3.0))
    msg = track.last
    assert msg.header.frame_id == 'odom'
    assert msg.child_frame_id == 'intruder'
    assert stamp_to_sec(msg.header.stamp) == pytest.approx(100.2)
    p = msg.pose.pose.position
    assert (p.x, p.y) == (pytest.approx(3.0, abs=1e-6), pytest.approx(0.0, abs=1e-6))
    assert msg.pose.pose.orientation.w == 1.0
    cov = msg.pose.covariance
    assert cov[0] > 0.0 and cov[7] > 0.0 and cov[14] == 0.0
    assert msg.twist.covariance[0] > 0.0
    pred = node.publishers['/intruder/predicted'].last
    assert pred.header.frame_id == 'odom'
    assert pred.point.x == pytest.approx(3.0, abs=0.05)


def test_target_is_placed_with_the_pose_at_its_stamp(node):
    node.subscriptions['/odom'](_odom(102.0, x=1.0, yaw=math.pi / 2))
    # Seen at t=100 (robot at the origin facing +x), received after it moved
    _confirm(node, bearing=0.0, rng=3.0)
    p = node.publishers['/intruder/track'].last.pose.pose.position
    assert (p.x, p.y) == (pytest.approx(3.0, abs=0.05), pytest.approx(0.0, abs=0.05))


def test_walking_person_gets_a_velocity(node):
    for i in range(30):
        t = 100.0 + i / 15.0
        y = 0.9 * (t - 100.0)                       # 0.9 m/s to the left
        node.subscriptions['/target'](
            _target(t, math.atan2(y, 3.0), math.hypot(3.0, y)))
    v = node.publishers['/intruder/track'].last.twist.twist.linear
    assert (v.x, v.y) == (pytest.approx(0.0, abs=0.1), pytest.approx(0.9, abs=0.1))


def test_no_odometry_no_track(ros_params):
    n = tt.TargetTrackerNode()
    n.subscriptions['/target'](_target(100.0, 0.0, 3.0))
    assert n._tracker.filter is None


def test_nan_and_unranged_targets_do_not_start_a_track(node):
    node.subscriptions['/target'](_target(100.0, float('nan'), -1.0))
    node.subscriptions['/target'](_target(100.0, 0.2, -1.0))
    assert node._tracker.filter is None


def test_unstamped_target_uses_the_clock(node):
    node.subscriptions['/target'](_target(None, 0.0, 3.0))
    assert node._tracker.filter.t == pytest.approx(node.clock.seconds)


def test_state_and_markers_follow_the_lifecycle(node):
    state = node.publishers['/intruder/state']
    markers = node.publishers['/intruder/markers']
    node.timers[0]()
    assert state.last.data == 'none'
    assert markers.msgs == []                       # nothing to clear yet
    _confirm(node)
    node.timers[0]()
    assert state.last.data == 'confirmed'
    body, ellipse, arrow = markers.last.markers
    assert (body.type, ellipse.type, arrow.type) == (
        body.CYLINDER, body.CYLINDER, body.ARROW)
    assert body.color.r == 1.0 and body.color.g < 0.5
    assert ellipse.scale.x > 0.0 and ellipse.scale.y > 0.0
    assert len(arrow.points) == 2
    node.clock.advance(5.0)                         # nothing for 5 s
    node.timers[0]()
    assert state.last.data == 'lost'
    assert markers.last.markers[0].action == markers.last.markers[0].DELETEALL
    n = len(markers.msgs)
    node.timers[0]()
    assert len(markers.msgs) == n                   # cleared once


def test_lidar_cluster_refines_a_confirmed_track(node):
    node.subscriptions['/scan'](_leg_scan(100.0, 3.0, 0.0))
    assert node._tracker.filter is None             # clusters never start one
    _confirm(node)
    n = len(node.publishers['/intruder/track'].msgs)
    # The person stepped 0.2 m left; laser_frame is 0.064 m behind base_link
    node.subscriptions['/scan'](_leg_scan(100.3, 3.064 - 0.15, 0.2))
    assert len(node.publishers['/intruder/track'].msgs) == n + 1
    p = node.publishers['/intruder/track'].last.pose.pose.position
    assert p.y > 0.05
    assert node._tracker.last_update == pytest.approx(100.3)
