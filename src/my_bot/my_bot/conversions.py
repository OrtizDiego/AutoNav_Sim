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

"""Angle and time conversions shared by the nodes and libraries.

Pure functions on duck-typed ROS messages (anything with the same fields),
so the libraries that use them stay importable without a ROS installation.
"""

import math
from typing import Optional, Tuple


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q) -> float:
    """Yaw (rad) of a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_zw(yaw: float) -> Tuple[float, float]:
    """(z, w) of the quaternion for a rotation of ``yaw`` about the z axis."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def stamp_to_sec(stamp) -> Optional[float]:
    """Seconds of a builtin_interfaces/Time, or None if it carries no time."""
    sec = getattr(stamp, 'sec', None)
    if sec is None:
        return None
    t = sec + getattr(stamp, 'nanosec', 0) * 1e-9
    return t if t > 0.0 else None
