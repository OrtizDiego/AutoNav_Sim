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

"""Latency-compensated target estimate for the follow controllers.

A camera detection reaches the controller hundreds of milliseconds after the
image was taken (render, YOLO, tracker, fusion). Steering on that bearing as
if it were current makes the robot keep turning after it already faces the
target: with a P gain k and a dead time d the heading loop rings once
k * d > 1/e and goes unstable near pi/2.

So the measurement is anchored where it was taken instead: the robot pose
at the image's timestamp (interpolated from odometry history) turns the
relative bearing/range into a fixed point in the odom frame, or into a world
direction when the range is unknown. The controller then asks for the
target relative to the robot's pose *now*, which odometry updates at
50-100 Hz. The target is assumed static between detections; a world-frame
filter with velocity (Phase 3 of the security-guard plan) replaces that.

Pure Python (no ROS): ball_chaser and security_guard_bt feed it odometry and
sensor_fusion's stamped /target.
"""

from collections import deque
import math
from typing import Optional, Tuple

Pose = Tuple[float, float, float]  # x, y, yaw in the odom frame


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def stamp_to_sec(stamp) -> Optional[float]:
    """Seconds of a builtin_interfaces/Time, or None if it carries no time."""
    sec = getattr(stamp, 'sec', None)
    if sec is None:
        return None
    t = sec + getattr(stamp, 'nanosec', 0) * 1e-9
    return t if t > 0.0 else None


def yaw_from_quaternion(q) -> float:
    """Yaw (rad) of a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TargetEstimate:
    """Last target measurement, re-expressed relative to the current pose."""

    def __init__(self, history_secs: float = 2.0):
        self._history_secs = history_secs
        self._poses = deque()  # (t, x, y, yaw), t increasing
        self._xy: Optional[Tuple[float, float]] = None  # odom, when ranged
        self._world_bearing: Optional[float] = None     # odom-frame direction
        self._raw: Optional[Tuple[float, Optional[float]]] = None  # no odom yet
        self.stamp: Optional[float] = None  # capture time of the measurement

    # -- odometry ----------------------------------------------------------

    def add_pose(self, t: Optional[float], x: float, y: float, yaw: float):
        """Record an odometry pose. Unstamped poses replace the history."""
        if t is None or (self._poses and t < self._poses[-1][0]):
            self._poses.clear()  # no time, or the clock jumped back (reset)
            t = 0.0 if t is None else t
        self._poses.append((t, x, y, yaw))
        while self._poses and t - self._poses[0][0] > self._history_secs:
            self._poses.popleft()

    def latest_pose(self) -> Optional[Pose]:
        """Most recent odometry pose, or None before the first one."""
        return self._poses[-1][1:] if self._poses else None

    def pose_at(self, t: Optional[float]) -> Optional[Pose]:
        """Pose at time ``t``, interpolated; clamped to the history's ends."""
        if not self._poses:
            return None
        if t is None or t >= self._poses[-1][0]:
            return self._poses[-1][1:]
        if t <= self._poses[0][0]:
            return self._poses[0][1:]
        later = None
        for entry in reversed(self._poses):  # the latency is short: few steps
            if entry[0] <= t:
                t0, x0, y0, yaw0 = entry
                t1, x1, y1, yaw1 = later
                a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return (x0 + a * (x1 - x0), y0 + a * (y1 - y0),
                        wrap_angle(yaw0 + a * wrap_angle(yaw1 - yaw0)))
            later = entry
        return self._poses[0][1:]  # pragma: no cover — guarded above

    # -- measurements ------------------------------------------------------

    def update(self, t: Optional[float], bearing: float,
               range_m: Optional[float]) -> bool:
        """Anchor a measurement taken at ``t``; False if it holds no target.

        ``bearing`` is relative to the robot (rad, positive left);
        ``range_m`` is None or <= 0 when unknown.
        """
        if bearing is None or not math.isfinite(bearing):
            return False
        if range_m is not None and not (math.isfinite(range_m) and range_m > 0.0):
            range_m = None
        self.stamp = t
        pose = self.pose_at(t)
        if pose is None:
            self._raw = (bearing, range_m)
            self._xy = self._world_bearing = None
            return True
        x, y, yaw = pose
        self._raw = None
        self._world_bearing = wrap_angle(yaw + bearing)
        if range_m is None:
            self._xy = None
        else:
            self._xy = (x + range_m * math.cos(self._world_bearing),
                        y + range_m * math.sin(self._world_bearing))
        return True

    @property
    def has_target(self) -> bool:
        """True once a measurement has been anchored."""
        return self._raw is not None or self._world_bearing is not None

    @property
    def target_xy(self) -> Optional[Tuple[float, float]]:
        """Target position in the odom frame, if it was ranged."""
        return self._xy

    def relative(self, pose: Optional[Pose] = None
                 ) -> Optional[Tuple[float, Optional[float]]]:
        """(bearing, range or None) of the target from ``pose`` (default: now)."""
        if self._raw is not None:
            return self._raw
        if self._world_bearing is None:
            return None
        pose = pose or self.latest_pose()
        x, y, yaw = pose
        if self._xy is None:
            return wrap_angle(self._world_bearing - yaw), None
        dx, dy = self._xy[0] - x, self._xy[1] - y
        return wrap_angle(math.atan2(dy, dx) - yaw), math.hypot(dx, dy)
