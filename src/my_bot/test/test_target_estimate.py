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

"""Tests for the latency-compensated target estimate."""

import math
import types

import pytest

from my_bot.follow_control import compute_command
from my_bot.target_estimate import (
    TargetEstimate, stamp_to_sec, wrap_angle, yaw_from_quaternion)


def _stamp(t):
    return types.SimpleNamespace(sec=int(t), nanosec=round((t % 1) * 1e9))


def test_helpers():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert stamp_to_sec(_stamp(12.25)) == pytest.approx(12.25)
    assert stamp_to_sec(None) is None
    assert stamp_to_sec(_stamp(0.0)) is None         # unset header stamp
    q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))
    assert yaw_from_quaternion(q) == pytest.approx(0.8)


def test_pose_is_interpolated_and_clamped():
    est = TargetEstimate()
    assert est.pose_at(1.0) is None
    est.add_pose(1.0, 0.0, 0.0, 0.0)
    est.add_pose(2.0, 2.0, 0.0, 1.0)
    assert est.pose_at(1.5) == pytest.approx((1.0, 0.0, 0.5))
    assert est.pose_at(0.0) == pytest.approx((0.0, 0.0, 0.0))
    assert est.pose_at(9.0) == pytest.approx((2.0, 0.0, 1.0))
    assert est.pose_at(None) == pytest.approx((2.0, 0.0, 1.0))


def test_yaw_interpolates_across_the_pi_seam():
    est = TargetEstimate()
    est.add_pose(0.0, 0.0, 0.0, 3.0)
    est.add_pose(1.0, 0.0, 0.0, -3.0)              # turned 0.28 rad left
    assert abs(est.pose_at(0.5)[2]) == pytest.approx(math.pi, abs=1e-6)


def test_history_is_bounded_and_resets_when_time_jumps_back():
    est = TargetEstimate(history_secs=1.0)
    for i in range(30):
        est.add_pose(i * 0.1, float(i), 0.0, 0.0)
    assert len(est._poses) <= 11
    est.add_pose(0.5, 7.0, 0.0, 0.0)                # Gazebo world reset
    assert list(est._poses) == [(0.5, 7.0, 0.0, 0.0)]


def test_no_target_until_a_finite_bearing_arrives():
    est = TargetEstimate()
    assert not est.has_target
    assert est.relative() is None
    assert not est.update(1.0, float('nan'), 3.0)
    assert not est.has_target


def test_without_odometry_the_raw_measurement_is_used():
    est = TargetEstimate()
    assert est.update(None, 0.3, -1.0)
    assert est.relative() == (0.3, None)
    assert est.target_xy is None


def test_detection_latency_is_compensated_by_the_turn_since():
    # The image is taken at t=5 with the target 0.6 rad left, 4 m away.
    # The detection arrives at t=5.4, after the robot turned 0.5 rad left.
    est = TargetEstimate()
    est.add_pose(5.0, 0.0, 0.0, 0.0)
    est.add_pose(5.4, 0.0, 0.0, 0.5)
    est.update(5.0, 0.6, 4.0)
    bearing, rng = est.relative()
    assert bearing == pytest.approx(0.1)
    assert rng == pytest.approx(4.0)
    assert est.target_xy == pytest.approx((4 * math.cos(0.6), 4 * math.sin(0.6)))


def test_driving_toward_a_ranged_target_shortens_the_range():
    est = TargetEstimate()
    est.add_pose(0.0, 0.0, 0.0, 0.0)
    est.update(0.0, 0.0, 3.0)
    est.add_pose(0.5, 1.0, 0.0, 0.0)
    assert est.relative() == (pytest.approx(0.0), pytest.approx(2.0))


def test_unranged_target_keeps_its_world_direction():
    est = TargetEstimate()
    est.add_pose(0.0, 0.0, 0.0, 0.2)
    est.update(0.0, 0.3, None)
    est.add_pose(0.1, 0.0, 0.0, 0.4)
    bearing, rng = est.relative()
    assert bearing == pytest.approx(0.1)
    assert rng is None


def test_compensation_removes_the_overshoot():
    """Closed loop: dead time makes raw P steering ring; anchoring does not.

    A static target 1 rad left; the robot turns with w = k * bearing, but
    each detection describes the image taken ``delay`` seconds earlier and
    arrives every ``period`` seconds (YOLO on a CPU).
    """
    k, dt, delay, period = 1.5, 0.02, 0.5, 0.2
    target_dir = 1.0

    def simulate(compensate):
        yaw, history, est = 0.0, [], TargetEstimate()
        latest_raw, peak = target_dir, 0.0
        for i in range(int(8.0 / dt)):
            t = i * dt
            history.append((t, yaw))
            est.add_pose(t, 0.0, 0.0, yaw)
            if i % round(period / dt) == 0 and t >= delay:
                t_img = t - delay
                yaw_img = min(history, key=lambda h: abs(h[0] - t_img))[1]
                latest_raw = wrap_angle(target_dir - yaw_img)
                est.update(t_img, latest_raw, 5.0)
            bearing = est.relative()[0] if compensate and est.has_target else latest_raw
            _, w = compute_command(bearing, None, 2.5, 0.8, k, 1.0, 0.2, 1.5)
            yaw += w * dt
            peak = max(peak, yaw - target_dir)
        return peak, abs(yaw - target_dir)

    raw_overshoot, _ = simulate(compensate=False)
    comp_overshoot, comp_final = simulate(compensate=True)
    assert raw_overshoot > 0.2                       # rings past the target
    assert comp_overshoot < 0.02
    assert comp_final < 0.01


def test_track_is_extrapolated_with_its_velocity_up_to_a_cap():
    est = TargetEstimate(max_predict=1.0)
    est.add_pose(10.0, 0.0, 0.0, 0.0)
    assert est.update_track(10.0, 3.0, 0.0, 0.0, 1.0)
    assert est.target_xy == (3.0, 0.0)
    assert est.predicted_xy(10.5) == pytest.approx((3.0, 0.5))
    assert est.predicted_xy(9.0) == pytest.approx((3.0, 0.0))   # never back
    assert est.predicted_xy(20.0) == pytest.approx((3.0, 1.0))  # capped
    assert est.relative(t=10.5) == (pytest.approx(math.atan2(0.5, 3.0)),
                                    pytest.approx(math.hypot(3.0, 0.5)))
    est.add_pose(10.5, 0.0, 0.0, 0.0)
    assert est.relative()[0] == pytest.approx(math.atan2(0.5, 3.0))  # odom time


def test_a_detection_replaces_the_track_velocity():
    est = TargetEstimate()
    est.add_pose(0.0, 0.0, 0.0, 0.0)
    est.update_track(0.0, 3.0, 0.0, 0.0, 1.0)
    est.update(0.0, 0.0, 2.0)
    assert est.predicted_xy(0.5) == pytest.approx((2.0, 0.0))
    assert not est.update_track(0.0, float('inf'), 0.0, 0.0, 0.0)


def test_track_without_odometry_is_seen_from_the_origin():
    est = TargetEstimate()
    est.update_track(None, 0.0, 2.0, 0.0, 0.0)
    assert est.relative() == (pytest.approx(math.pi / 2), pytest.approx(2.0))
