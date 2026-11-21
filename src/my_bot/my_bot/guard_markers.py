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

"""RViz markers of the security guard (security_guard_bt publishes them).

``zone_markers``: restricted areas and patrol routes (/guard_zones, map
frame). ``sweep_markers``: the sweep's viewpoints (/sweep_viewpoints, map
frame). ``sighting_markers``: where the intruder was seen
(/intruder_sightings, odom frame).
"""

from typing import List, Sequence, Tuple

from visualization_msgs.msg import Marker

from my_bot.guard_zones import GuardZones
from my_bot.markers import delete_all, make_marker, make_point

RESTRICTED_RED = (1.0, 0.0, 0.0)
ROUTE_BLUE = (0.0, 0.5, 1.0, 0.6)
SWEEP_ORANGE = (1.0, 0.6, 0.0, 0.9)


def _loop(points) -> list:
    """Points of a closed line strip through ``points``, just above the floor."""
    return [make_point(x, y, 0.05) for x, y in points + points[:1]]


def zone_markers(zones: GuardZones, stamp) -> List[Marker]:
    """Restricted areas (red outline + name) and each zone's patrol route."""
    markers = []
    for i, (name, poly) in enumerate(zones.restricted.items()):
        centre = (sum(x for x, _ in poly) / len(poly),
                  sum(y for _, y in poly) / len(poly), 0.5)
        markers += [
            make_marker('map', stamp, 'restricted', i, Marker.LINE_STRIP,
                        scale=(0.08, 0.0, 0.0), rgba=(*RESTRICTED_RED, 0.9),
                        points=_loop(poly)),
            make_marker('map', stamp, 'restricted_labels', i, Marker.TEXT_VIEW_FACING,
                        position=centre, scale=(0.0, 0.0, 0.35),
                        rgba=(*RESTRICTED_RED, 1.0), text=name),
        ]
    for i, (name, waypoints) in enumerate(zones.zones.items()):
        markers.append(make_marker(
            'map', stamp, 'patrol_' + name, i, Marker.LINE_STRIP,
            scale=(0.04, 0.0, 0.0), rgba=ROUTE_BLUE, points=_loop(waypoints)))
    return markers


def sweep_markers(points: Sequence[Tuple[float, float, float]], stamp) -> List[Marker]:
    """Clear the old viewpoints, then one arrow per (x, y, yaw); none clears."""
    return [delete_all()] + [
        make_marker('map', stamp, 'sweep', i, Marker.ARROW, position=(x, y, 0.1),
                    yaw=yaw, scale=(0.6, 0.12, 0.12), rgba=SWEEP_ORANGE)
        for i, (x, y, yaw) in enumerate(points)]


def sighting_markers(index: int, where: Tuple[float, float], stamp,
                     elapsed: float) -> List[Marker]:
    """Red sphere and '#n T=..s' label for sighting number ``index`` (from 0)."""
    x, y = where
    return [
        make_marker('odom', stamp, 'sightings', index, Marker.SPHERE,
                    position=(x, y, 0.5), scale=(0.3, 0.3, 0.3),
                    rgba=(1.0, 0.0, 0.0, 0.8)),
        make_marker('odom', stamp, 'sighting_labels', index, Marker.TEXT_VIEW_FACING,
                    position=(x, y, 0.9), scale=(0.0, 0.0, 0.25),
                    rgba=(1.0, 1.0, 1.0, 1.0), text=f'#{index + 1} T={elapsed:.0f}s'),
    ]
