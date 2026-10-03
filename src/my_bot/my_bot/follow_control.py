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

"""Stand-off follow controller shared by ball_chaser and security_guard_bt.

Both consume sensor_fusion's /target (via target_estimate, which compensates
the detection latency with odometry), so the same proportional law follows
a ball or a person:

  linear.x  = k_lin * (range - desired_distance), slowed while turning
  angular.z = k_yaw * bearing

Forward motion stops if anything is closer than ``safety_distance`` ahead.
"""

import math
from typing import List, Optional, Tuple


def compute_command(angle: float, range_m: Optional[float],
                    desired_distance: float, k_lin: float, k_yaw: float,
                    max_lin: float, max_back: float, max_yaw: float,
                    deadband: float = 0.15,
                    front_clear: float = float('inf'),
                    safety_distance: float = 0.6) -> Tuple[float, float]:
    """Return (linear.x, angular.z) for one control step.

    ``angle`` is the robot-frame angle to the target (positive = left).
    Without a range the robot only turns toward the target.
    """
    w = max(-max_yaw, min(max_yaw, k_yaw * angle))
    if range_m is None or not math.isfinite(range_m) or range_m <= 0.0:
        return 0.0, w
    error = range_m - desired_distance
    v = 0.0 if abs(error) < deadband else k_lin * error
    v = max(-max_back, min(max_lin, v))
    if v > 0.0:
        v *= max(0.0, math.cos(angle))  # turn first, then drive
        if front_clear < safety_distance:
            v = 0.0
    return float(v), float(w)


def search_command(last_angle: float, speed: float) -> float:
    """Angular velocity that turns toward where the target was last seen."""
    return math.copysign(speed, last_angle if last_angle else 1.0)


def front_clearance(ranges: List[float], angle_min: float,
                    angle_increment: float, half_width: float = 0.44,
                    range_min: float = 0.12) -> float:
    """Closest valid lidar return within +/-``half_width`` rad of straight ahead."""
    n = len(ranges)
    if n == 0:
        return float('inf')
    lo = int(math.floor((-half_width - angle_min) / angle_increment))
    hi = int(math.ceil((half_width - angle_min) / angle_increment))
    best = float('inf')
    for i in range(lo, hi + 1):
        r = ranges[i % n]
        if math.isfinite(r) and r >= range_min:
            best = min(best, r)
    return best
