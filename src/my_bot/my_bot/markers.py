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

"""Build visualization_msgs/Marker messages without the field-by-field noise.

``make_marker`` fills what every RViz marker of the package sets (frame,
stamp, namespace, id, type, ADD, an upright pose); the keyword arguments
cover what differs. Used by target_tracker and guard_markers.
"""

from typing import Optional, Sequence, Tuple

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from my_bot.conversions import quaternion_zw

Colour = Tuple[float, float, float, float]  # r, g, b, a


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    """geometry_msgs/Point at (x, y, z)."""
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def make_marker(frame_id: str, stamp, ns: str, marker_id: int, marker_type: int, *,
                position: Sequence[float] = (0.0, 0.0, 0.0),
                yaw: Optional[float] = None,
                scale: Sequence[float] = (0.0, 0.0, 0.0),
                rgba: Colour = (0.0, 0.0, 0.0, 0.0),
                points: Sequence[Point] = (), text: str = '') -> Marker:
    """Marker to add: ``position`` (x, y, z), heading ``yaw`` (None = none)."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = position
    if yaw is None:
        m.pose.orientation.w = 1.0
    else:
        m.pose.orientation.z, m.pose.orientation.w = quaternion_zw(yaw)
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.points = list(points)
    m.text = text
    return m


def delete_all() -> Marker:
    """Marker that clears every marker of its topic."""
    m = Marker()
    m.action = Marker.DELETEALL
    return m
