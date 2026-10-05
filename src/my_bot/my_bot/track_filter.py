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

"""World-frame intruder track: constant-velocity EKF + track lifecycle.

target_estimate anchors each detection in the odom frame but assumes the
target stands still between detections. A walking (or sprinting) person does
not, so this filter estimates position *and velocity* in the odom frame:

  state    x = [px, py, vx, vy]  (odom frame, m and m/s)
  motion   constant velocity, white-noise acceleration ``accel_noise``
           (m/s^2), time step = the real gap between measurement stamps
  camera   range + bearing from the robot pose at the image's stamp
           (sensor_fusion's /target); bearing only when unranged
  lidar    a person-sized scan cluster, as an odom-frame position

Measurements arrive slightly out of order (the camera pipeline is ~25 ms
behind the scan). A measurement older than the state is applied by
retrodiction: the state is propagated back to the measurement's time to
form the innovation, and the process noise of the gap is added to the
measurement noise (Bar-Shalom's one-step-lag approximation). Older than
``max_lag`` seconds it is dropped.

Every update is gated on the Mahalanobis distance of its innovation
(chi-square, 99.9 %), so a detection of something else does not drag the
track. IntruderTracker adds the lifecycle around the filter:

  tentative   created by a ranged camera detection
  confirmed   after ``confirm_hits`` camera hits; only these are published
  lost        no accepted measurement for ``lost_timeout`` (``tentative_timeout``
              before confirmation); the next detection starts a new track

Lidar clusters only update a confirmed track whose camera detection is at
most ``lidar_coast_secs`` old, and only when exactly one cluster falls in
the gate: on their own they cannot tell a person from an exhibit.

Pure Python + numpy (no ROS): the target_tracker node feeds it.
"""

import copy
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from my_bot.target_estimate import Pose, wrap_angle

# Chi-square 99.9 % gates by measurement dimension
GATE = {1: 10.83, 2: 13.82}

TENTATIVE, CONFIRMED, LOST, NONE = 'tentative', 'confirmed', 'lost', 'none'


def _transition(dt: float) -> np.ndarray:
    f = np.eye(4)
    f[0, 2] = f[1, 3] = dt
    return f


def _process_noise(dt: float, accel_noise: float) -> np.ndarray:
    """Continuous white-noise acceleration, integrated over ``dt``."""
    dt = abs(dt)
    q = accel_noise ** 2
    a, b, c = q * dt ** 3 / 3.0, q * dt ** 2 / 2.0, q * dt
    return np.array([[a, 0, b, 0],
                     [0, a, 0, b],
                     [b, 0, c, 0],
                     [0, b, 0, c]])


class ConstantVelocityEKF:
    """Position + velocity of one target in the odom frame."""

    def __init__(self, t: float, x: float, y: float, pos_cov: np.ndarray,
                 speed_sigma: float = 1.0, accel_noise: float = 1.5,
                 max_lag: float = 1.0):
        self.t = t
        self.x = np.array([x, y, 0.0, 0.0])
        self.P = np.zeros((4, 4))
        self.P[:2, :2] = pos_cov
        self.P[2, 2] = self.P[3, 3] = speed_sigma ** 2
        self.accel_noise = accel_noise
        self.max_lag = max_lag

    # -- prediction --------------------------------------------------------

    def predict(self, t: float):
        """Advance the state to ``t`` (no-op for earlier times)."""
        dt = t - self.t
        if dt <= 0.0:
            return
        f = _transition(dt)
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + _process_noise(dt, self.accel_noise)
        self.t = t

    def position_at(self, t: float) -> Tuple[float, float]:
        """Predicted position at ``t`` without changing the filter."""
        dt = t - self.t
        return (float(self.x[0] + self.x[2] * dt),
                float(self.x[1] + self.x[3] * dt))

    @property
    def position(self) -> Tuple[float, float]:
        """(x, y) at the filter's time."""
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        """(vx, vy) in the odom frame."""
        return float(self.x[2]), float(self.x[3])

    # -- measurements ------------------------------------------------------

    def _innovation(self, t, z, h, jac, r, angle_rows=()):
        """(y, S, H, R) of measurement ``z`` taken at ``t``, or None if too old.

        ``h(state)`` predicts the measurement, ``jac(state)`` is its Jacobian.
        The state must not be older than ``t`` (callers predict first). An
        older measurement is compared with the state propagated back to
        ``t`` (retrodiction), its noise inflated by the gap's process noise.
        """
        dt = min(0.0, t - self.t)
        if dt < -self.max_lag:
            return None
        f = _transition(dt)
        xb = f @ self.x
        hb = jac(xb)
        y = np.asarray(z, dtype=float) - h(xb)
        for i in angle_rows:
            y[i] = wrap_angle(y[i])
        big_h = hb @ f
        if dt < 0.0:
            r = r + hb @ _process_noise(dt, self.accel_noise) @ hb.T
        s = big_h @ self.P @ big_h.T + r
        return y, s, big_h, r

    def _apply(self, t, inn_fn, gate: bool) -> Optional[bool]:
        self.predict(t)
        inn = inn_fn()
        if inn is None:
            return None
        y, s, big_h, r = inn
        s_inv = np.linalg.inv(s)
        if gate and float(y @ s_inv @ y) > GATE[len(y)]:
            return False
        k = self.P @ big_h.T @ s_inv
        self.x = self.x + k @ y
        i_kh = np.eye(4) - k @ big_h
        self.P = i_kh @ self.P @ i_kh.T + k @ r @ k.T  # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        return True

    def range_bearing(self, t: float, pose: Pose, bearing: float,
                      range_m: Optional[float], bearing_sigma: float,
                      range_sigma: float, gate: bool = True) -> Optional[bool]:
        """Camera update from ``pose`` (robot x, y, yaw at ``t``).

        Returns True (applied), False (outside the gate) or None (too old).
        ``range_m`` None: bearing-only update.
        """
        px, py, yaw = pose

        def h(s):
            dx, dy = s[0] - px, s[1] - py
            out = [math.atan2(dy, dx) - yaw]
            if range_m is not None:
                out.append(math.hypot(dx, dy))
            return np.array(out)

        def jac(s):
            dx, dy = s[0] - px, s[1] - py
            q = max(dx * dx + dy * dy, 1e-6)
            rows = [[-dy / q, dx / q, 0.0, 0.0]]
            if range_m is not None:
                d = math.sqrt(q)
                rows.append([dx / d, dy / d, 0.0, 0.0])
            return np.array(rows)

        if range_m is None:
            z, r = [bearing], np.diag([bearing_sigma ** 2])
        else:
            z, r = [bearing, range_m], np.diag([bearing_sigma ** 2, range_sigma ** 2])
        return self._apply(
            t, lambda: self._innovation(t, z, h, jac, r, angle_rows=(0,)), gate)

    def _position_innovation(self, t, xy, sigma):
        h_mat = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        return self._innovation(t, xy, lambda s: h_mat @ s, lambda s: h_mat,
                                np.eye(2) * sigma ** 2)

    def position_distance(self, t: float, xy: Sequence[float], sigma: float
                          ) -> Optional[float]:
        """Mahalanobis distance^2 of a position measurement (state unchanged)."""
        probe = copy.copy(self)  # predict() rebinds x and P, never mutates
        probe.predict(t)
        inn = probe._position_innovation(t, xy, sigma)
        if inn is None:
            return None
        y, s, _, _ = inn
        return float(y @ np.linalg.inv(s) @ y)

    def update_position(self, t: float, xy: Sequence[float], sigma: float,
                        gate: bool = True) -> Optional[bool]:
        """Direct position update (a lidar cluster)."""
        return self._apply(
            t, lambda: self._position_innovation(t, xy, sigma), gate)


def polar_covariance(bearing_world: float, range_m: float,
                     bearing_sigma: float, range_sigma: float) -> np.ndarray:
    """2x2 position covariance of a range/bearing measurement."""
    c, s = math.cos(bearing_world), math.sin(bearing_world)
    j = np.array([[c, -range_m * s], [s, range_m * c]])
    return j @ np.diag([range_sigma ** 2, bearing_sigma ** 2]) @ j.T


# ---------------------------------------------------------------------------
# Lidar clusters
# ---------------------------------------------------------------------------

def scan_clusters(ranges: Sequence[float], angle_min: float,
                  angle_increment: float, range_min: float = 0.12,
                  range_max: float = 8.0, jump: float = 0.1,
                  jump_rel: float = 0.03, merge_distance: float = 0.5,
                  max_width: float = 0.8, min_points: int = 2
                  ) -> List[Tuple[float, float]]:
    """Centroids (x, y in the scan frame) of person-sized scan clusters.

    Consecutive valid beams belong to one segment while their end points
    are within ``jump + jump_rel * range`` of each other. Segments whose
    centroids are within ``merge_distance`` are merged (a person's two legs),
    and merged groups wider than ``max_width`` (walls, furniture) or made of
    fewer than ``min_points`` beams are dropped.
    """
    segments, current, prev = [], [], None
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min <= r <= range_max):
            if current:
                segments.append(current)
            current, prev = [], None
            continue
        a = angle_min + i * angle_increment
        p = (r * math.cos(a), r * math.sin(a))
        if prev is not None and math.dist(p, prev) > jump + jump_rel * r:
            segments.append(current)
            current = []
        current.append(p)
        prev = p
    if current:
        segments.append(current)

    groups = []  # [points, centroid]
    for seg in segments:
        cx = sum(p[0] for p in seg) / len(seg)
        cy = sum(p[1] for p in seg) / len(seg)
        if groups and math.dist(groups[-1][1], (cx, cy)) <= merge_distance:
            pts = groups[-1][0] + seg
            groups[-1] = [pts, (sum(p[0] for p in pts) / len(pts),
                                sum(p[1] for p in pts) / len(pts))]
        else:
            groups.append([seg, (cx, cy)])

    out = []
    for pts, centroid in groups:
        if len(pts) < min_points:
            continue
        width = max(math.dist(a, b) for a in pts for b in (pts[0], pts[-1]))
        if width <= max_width:
            out.append(centroid)
    return out


def to_world(points: Sequence[Tuple[float, float]], pose: Pose,
             offset_x: float = 0.0) -> List[Tuple[float, float]]:
    """Scan-frame points -> odom, the scan frame ``offset_x`` ahead of base_link."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * (px + offset_x) - s * py, y + s * (px + offset_x) + c * py)
            for px, py in points]


# ---------------------------------------------------------------------------
# Track lifecycle
# ---------------------------------------------------------------------------

class IntruderTracker:
    """One intruder track: create, confirm, update, lose."""

    def __init__(self, accel_noise: float = 1.5, bearing_sigma: float = 0.04,
                 range_sigma: float = 0.15, range_sigma_rel: float = 0.05,
                 lidar_sigma: float = 0.12, speed_sigma: float = 1.0,
                 confirm_hits: int = 3, tentative_timeout: float = 0.5,
                 lost_timeout: float = 1.5, lidar_coast_secs: float = 1.5,
                 max_outliers: int = 3, max_lag: float = 1.0):
        self.accel_noise = accel_noise
        self.bearing_sigma = bearing_sigma
        self.range_sigma = range_sigma
        self.range_sigma_rel = range_sigma_rel
        self.lidar_sigma = lidar_sigma
        self.speed_sigma = speed_sigma
        self.confirm_hits = confirm_hits
        self.tentative_timeout = tentative_timeout
        self.lost_timeout = lost_timeout
        self.lidar_coast_secs = lidar_coast_secs
        self.max_outliers = max_outliers
        self.max_lag = max_lag

        self.filter: Optional[ConstantVelocityEKF] = None
        self.state = NONE
        self.hits = 0
        self.outliers = 0
        self.track_id = 0
        self.last_update = -math.inf   # time of the last accepted measurement
        self.last_camera = -math.inf   # time of the last accepted camera hit

    @property
    def confirmed(self) -> bool:
        """Whether the track is confirmed: only these are published."""
        return self.state == CONFIRMED

    def _range_sigma(self, range_m: float) -> float:
        return self.range_sigma + self.range_sigma_rel * range_m

    def _start(self, t: float, pose: Pose, bearing: float, range_m: float):
        world = pose[2] + bearing
        cov = polar_covariance(world, range_m, self.bearing_sigma,
                               self._range_sigma(range_m))
        self.filter = ConstantVelocityEKF(
            t, pose[0] + range_m * math.cos(world),
            pose[1] + range_m * math.sin(world), cov,
            self.speed_sigma, self.accel_noise, self.max_lag)
        self.state = TENTATIVE
        self.hits, self.outliers = 1, 0
        self.track_id += 1
        self.last_update = self.last_camera = t

    def camera(self, t: float, pose: Pose, bearing: float,
               range_m: Optional[float]) -> bool:
        """Update with a /target detection; True if the track changed."""
        if bearing is None or not math.isfinite(bearing):
            return False
        if range_m is not None and not (math.isfinite(range_m) and range_m > 0.0):
            range_m = None
        if self.filter is None:
            if range_m is None:
                return False  # a bearing alone does not place a new track
            self._start(t, pose, bearing, range_m)
            return True
        sigma = self._range_sigma(range_m) if range_m is not None else 0.0
        ok = self.filter.range_bearing(t, pose, bearing, range_m,
                                       self.bearing_sigma, sigma)
        if ok is None:
            return False
        if not ok:
            self.outliers += 1
            if range_m is not None and (
                    self.state == TENTATIVE or self.outliers >= self.max_outliers):
                self._start(t, pose, bearing, range_m)  # it is somewhere else
                return True
            return False
        self.outliers = 0
        self.hits += 1
        self.last_update = max(self.last_update, t)
        self.last_camera = max(self.last_camera, t)
        if self.state == TENTATIVE and self.hits >= self.confirm_hits:
            self.state = CONFIRMED
        return True

    def lidar(self, t: float, clusters: Sequence[Tuple[float, float]]) -> bool:
        """Update with odom-frame scan clusters at ``t``; True if one was used."""
        if not self.confirmed or t - self.last_camera > self.lidar_coast_secs:
            return False
        gated = []
        for xy in clusters:
            d2 = self.filter.position_distance(t, xy, self.lidar_sigma)
            if d2 is not None and d2 <= GATE[2]:
                gated.append(xy)
        if len(gated) != 1:
            return False  # nothing there, or ambiguous
        if self.filter.update_position(t, gated[0], self.lidar_sigma, gate=False):
            self.last_update = max(self.last_update, t)
            return True
        return False

    def expire(self, now: float) -> bool:
        """Drop a track without recent measurements; True if it was dropped."""
        if self.filter is None:
            return False
        timeout = self.lost_timeout if self.confirmed else self.tentative_timeout
        if now - self.last_update <= timeout:
            return False
        self.filter = None
        self.state = LOST if self.state == CONFIRMED else NONE
        return True
