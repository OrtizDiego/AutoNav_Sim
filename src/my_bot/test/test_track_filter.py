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

"""Tests for the world-frame intruder track (EKF, lifecycle, lidar clusters)."""

import math
import random

import numpy as np
import pytest

from my_bot.target_estimate import TargetEstimate, wrap_angle
from my_bot.track_filter import (
    CONFIRMED, ConstantVelocityEKF, IntruderTracker, LOST, NONE, TENTATIVE,
    polar_covariance, scan_clusters, to_world)

ORIGIN = (0.0, 0.0, 0.0)


def _observe(pose, xy):
    """Exact (bearing, range) of ``xy`` from ``pose``."""
    dx, dy = xy[0] - pose[0], xy[1] - pose[1]
    return wrap_angle(math.atan2(dy, dx) - pose[2]), math.hypot(dx, dy)


def _ekf(x=3.0, y=0.0):
    return ConstantVelocityEKF(0.0, x, y, np.eye(2) * 0.04)


# ---------------------------------------------------------------------------
# ConstantVelocityEKF
# ---------------------------------------------------------------------------

class TestEKF:

    def test_prediction_moves_with_the_velocity_and_grows_uncertainty(self):
        f = _ekf()
        f.x[2:] = (1.0, -0.5)
        p0 = np.trace(f.P)
        f.predict(2.0)
        assert f.position == pytest.approx((5.0, -1.0))
        assert f.t == 2.0
        assert np.trace(f.P) > p0
        f.predict(1.0)                               # never backwards
        assert f.t == 2.0
        assert f.position_at(3.0) == pytest.approx((6.0, -1.5))

    def test_estimates_the_velocity_of_a_walking_person(self):
        rng = random.Random(1)
        f = _ekf(3.0, 0.0)
        for i in range(1, 46):                       # 3 s of 15 Hz detections
            t = i / 15.0
            b, r = _observe(ORIGIN, (3.0, 1.2 * t))  # walks left at 1.2 m/s
            assert f.range_bearing(t, ORIGIN, b + rng.gauss(0, 0.02),
                                   r + rng.gauss(0, 0.1), 0.04, 0.2)
        assert f.velocity == pytest.approx((0.0, 1.2), abs=0.25)  # range noise
        assert f.position == pytest.approx((3.0, 3.6), abs=0.15)

    def test_bearing_only_update_corrects_the_direction(self):
        f = _ekf(3.0, 0.0)
        b, _ = _observe(ORIGIN, (3.0, 0.3))
        assert f.range_bearing(0.0, ORIGIN, b, None, 0.02, 0.0)
        assert f.position[1] > 0.1
        assert math.hypot(*f.position) == pytest.approx(3.0, abs=0.05)

    def test_outlier_is_gated(self):
        f = _ekf()
        assert f.range_bearing(0.1, ORIGIN, 1.5, 6.0, 0.04, 0.2) is False
        assert f.position == pytest.approx((3.0, 0.0))
        assert f.update_position(0.1, (6.0, 6.0), 0.1) is False
        assert f.update_position(0.1, (6.0, 6.0), 0.1, gate=False)

    def test_late_measurement_is_retrodicted(self):
        f = _ekf(3.0, 0.0)
        f.x[2:] = (0.0, 1.0)                        # walking left at 1 m/s
        f.predict(1.0)                              # state at t=1: (3, 1)
        b, r = _observe(ORIGIN, (3.0, 0.8))         # where it was at t=0.8
        assert f.range_bearing(0.8, ORIGIN, b, r, 0.02, 0.05)
        assert f.t == 1.0                           # the state stays at t=1
        assert f.position == pytest.approx((3.0, 1.0), abs=0.02)

    def test_too_old_measurement_is_dropped(self):
        f = _ekf()
        f.predict(5.0)
        assert f.range_bearing(3.5, ORIGIN, 0.0, 3.0, 0.04, 0.2) is None
        assert f.position_distance(3.5, (3.0, 0.0), 0.1) is None

    def test_gating_distance_does_not_change_the_state(self):
        f = _ekf()
        x, p, t = f.x.copy(), f.P.copy(), f.t
        assert f.position_distance(1.0, (3.0, 0.0), 0.1) < 1.0
        assert f.position_distance(1.0, (8.0, 0.0), 0.1) > 13.82
        assert (f.x == x).all() and (f.P == p).all() and f.t == t

    def test_polar_covariance_is_wide_across_the_beam(self):
        cov = polar_covariance(0.0, 5.0, 0.04, 0.1)
        assert cov[0, 0] == pytest.approx(0.01)
        assert cov[1, 1] == pytest.approx((5.0 * 0.04) ** 2)


# ---------------------------------------------------------------------------
# Lidar clusters
# ---------------------------------------------------------------------------

def _scan_of(objects, n=360, background=float('inf')):
    """Ranges of a 360-beam scan seeing circles (x, y, radius)."""
    ranges = [background] * n
    inc = 2 * math.pi / n
    for i in range(n):
        a = -math.pi + i * inc
        d = (math.cos(a), math.sin(a))
        for cx, cy, rad in objects:
            proj = cx * d[0] + cy * d[1]
            perp2 = cx * cx + cy * cy - proj * proj
            if proj > 0 and perp2 <= rad * rad:
                hit = proj - math.sqrt(rad * rad - perp2)
                ranges[i] = min(ranges[i], hit)
    return ranges, -math.pi, inc


class TestScanClusters:

    def test_two_legs_make_one_person(self):
        ranges, a0, inc = _scan_of([(2.0, 0.15, 0.06), (2.0, -0.15, 0.06)])
        clusters = scan_clusters(ranges, a0, inc)
        assert len(clusters) == 1
        assert clusters[0] == pytest.approx((1.96, 0.0), abs=0.05)

    def test_wall_is_not_a_person(self):
        n, inc = 360, 2 * math.pi / 360
        ranges = [float('inf')] * n
        for i in range(n):
            a = -math.pi + i * inc
            if abs(a) < 0.6:
                ranges[i] = 2.0 / math.cos(a)        # wall 2 m ahead
        assert scan_clusters(ranges, -math.pi, inc) == []

    def test_single_beams_and_far_returns_are_dropped(self):
        ranges = [float('inf')] * 360
        ranges[10] = 2.0                             # one stray beam
        ranges[100:104] = [9.0] * 4                  # beyond range_max
        assert scan_clusters(ranges, -math.pi, 2 * math.pi / 360) == []

    def test_separate_objects_stay_separate(self):
        ranges, a0, inc = _scan_of([(2.0, 1.5, 0.1), (2.0, -1.5, 0.1)])
        clusters = sorted(scan_clusters(ranges, a0, inc), key=lambda c: c[1])
        assert len(clusters) == 2
        assert clusters[0][1] < -1.0 < 1.0 < clusters[1][1]

    def test_to_world(self):
        pts = to_world([(1.0, 0.0)], (2.0, 1.0, math.pi / 2), offset_x=-0.1)
        assert pts[0] == pytest.approx((2.0, 1.9))


# ---------------------------------------------------------------------------
# IntruderTracker lifecycle
# ---------------------------------------------------------------------------

def _see(trk, t, xy, pose=ORIGIN):
    b, r = _observe(pose, xy)
    return trk.camera(t, pose, b, r)


class TestIntruderTracker:

    def test_confirmed_after_three_hits(self):
        trk = IntruderTracker()
        assert trk.state == NONE
        assert _see(trk, 0.0, (3.0, 0.0))
        assert trk.state == TENTATIVE and trk.track_id == 1
        _see(trk, 0.1, (3.0, 0.0))
        assert trk.state == TENTATIVE
        _see(trk, 0.2, (3.0, 0.0))
        assert trk.confirmed and trk.state == CONFIRMED

    def test_needs_a_range_to_start(self):
        trk = IntruderTracker()
        assert not trk.camera(0.0, ORIGIN, 0.2, None)
        assert not trk.camera(0.0, ORIGIN, 0.2, -1.0)
        assert not trk.camera(0.0, ORIGIN, float('nan'), 3.0)
        assert trk.filter is None

    def test_unranged_detection_updates_an_existing_track(self):
        trk = IntruderTracker()
        _see(trk, 0.0, (3.0, 0.0))
        assert trk.camera(0.1, ORIGIN, 0.0, -1.0)
        assert trk.hits == 2

    def test_tentative_track_expires_quickly(self):
        trk = IntruderTracker(tentative_timeout=0.5)
        _see(trk, 0.0, (3.0, 0.0))
        assert not trk.expire(0.4)
        assert trk.expire(0.6)
        assert trk.state == NONE and trk.filter is None

    def test_confirmed_track_is_lost_after_the_timeout(self):
        trk = IntruderTracker(lost_timeout=1.5)
        for i in range(3):
            _see(trk, i * 0.1, (3.0, 0.0))
        assert not trk.expire(1.6)
        assert trk.expire(1.8)
        assert trk.state == LOST
        _see(trk, 2.0, (1.0, 1.0))                  # a new track
        assert trk.state == TENTATIVE and trk.track_id == 2

    def test_tentative_outlier_restarts_the_track(self):
        trk = IntruderTracker()
        _see(trk, 0.0, (3.0, 0.0))
        assert _see(trk, 0.1, (0.0, 4.0))
        assert trk.track_id == 2
        assert trk.filter.position == pytest.approx((0.0, 4.0), abs=1e-6)

    def test_confirmed_track_survives_single_outliers(self):
        trk = IntruderTracker(max_outliers=3)
        for i in range(3):
            _see(trk, i * 0.1, (3.0, 0.0))
        assert not _see(trk, 0.3, (0.0, 4.0))       # e.g. another person
        assert not _see(trk, 0.4, (0.0, 4.0))
        assert trk.confirmed and trk.track_id == 1
        assert _see(trk, 0.5, (0.0, 4.0))           # third in a row: moved
        assert trk.state == TENTATIVE and trk.track_id == 2

    def test_lidar_updates_a_confirmed_track(self):
        trk = IntruderTracker()
        assert not trk.lidar(0.0, [(3.0, 0.0)])     # nothing to update
        for i in range(3):
            _see(trk, i * 0.1, (3.0, 0.0))
        assert trk.lidar(0.25, [(3.05, 0.0), (6.0, 2.0)])
        assert trk.last_update == 0.25

    def test_lidar_skips_ambiguous_and_stale_cases(self):
        trk = IntruderTracker(lidar_coast_secs=1.5)
        for i in range(3):
            _see(trk, i * 0.1, (3.0, 0.0))
        assert not trk.lidar(0.3, [(3.05, 0.05), (2.95, -0.05)])  # two in gate
        assert not trk.lidar(0.3, [(5.0, 0.0)])                   # outside
        assert trk.lidar(1.0, [(3.0, 0.0)])
        assert not trk.lidar(1.8, [(3.0, 0.0)])     # camera silent > 1.5 s

    def test_lidar_keeps_the_track_through_a_camera_dropout(self):
        trk = IntruderTracker(lost_timeout=1.0, lidar_coast_secs=1.5)
        for i in range(3):
            _see(trk, i * 0.1, (3.0, 0.0))
        for i in range(1, 13):                      # 1.2 s of lidar only
            trk.lidar(0.2 + i * 0.1, [(3.0, 0.0)])
        assert not trk.expire(1.5)
        assert trk.confirmed


def test_track_follows_a_walking_person_better_than_a_static_anchor():
    """The point of Phase 3: a moving target, detections every 0.2 s.

    Between detections a static anchor lags the person by up to
    speed * period; the track extrapolates with its velocity.
    """
    speed, period, dt = 1.5, 0.2, 0.02
    trk, static = IntruderTracker(), TargetEstimate()
    static.add_pose(0.0, *ORIGIN)
    err_static, err_track = [], []
    for i in range(int(6.0 / dt)):
        t = i * dt
        person = (3.0, -4.5 + speed * t)          # walks across the view
        if i % round(period / dt) == 0:
            b, r = _observe(ORIGIN, person)
            trk.camera(t, ORIGIN, b, r)
            static.update(t, b, r)
        if t > 2.0 and trk.confirmed:
            est = TargetEstimate()
            est.update_track(trk.filter.t, *trk.filter.position, *trk.filter.velocity)
            err_track.append(math.dist(est.predicted_xy(t), person))
            err_static.append(math.dist(static.target_xy, person))
    assert max(err_static) > 0.25                   # ~speed * period
    assert np.mean(err_track) < 0.4 * np.mean(err_static)
