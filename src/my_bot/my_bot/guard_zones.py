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

"""Patrol zones, their schedule and restricted areas for the security guard.

Read from a YAML file (config/guard_zones.yaml) rather than ROS parameters:
parameters cannot hold nested lists or maps, and the zones are named lists of
points. Everything is in the map frame (= Gazebo world frame, metres)::

  patrol_zones:            # name -> waypoints [[x, y], ...], visited in order
    north_hall: [[3.28, 6.85], [-0.92, 6.85]]
  patrol_schedule:         # cycled on the node clock (simulation seconds)
    - {zone: north_hall, secs: 120.0}
  restricted_areas:        # name -> polygon [[x, y], ...]
    statue_circle: [[-8.0, 0.5], [-4.5, 0.5], [-4.5, 3.5], [-8.0, 3.5]]

Without a schedule the first zone is patrolled all the time.

Pure Python (no ROS).
"""

from typing import Dict, List, Optional, Sequence, Tuple

import yaml

Point = Tuple[float, float]


def point_in_polygon(x: float, y: float, polygon: Sequence[Point]) -> bool:
    """Ray casting: True if (x, y) lies inside ``polygon`` (any winding)."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            if x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


class GuardZones:
    """Patrol route by schedule + restricted-area lookups."""

    def __init__(self, zones: Dict[str, List[Point]],
                 schedule: Sequence[Tuple[str, float]] = (),
                 restricted: Optional[Dict[str, List[Point]]] = None):
        if not zones:
            raise ValueError('at least one patrol zone is needed')
        for name, waypoints in zones.items():
            if not waypoints:
                raise ValueError(f'patrol zone {name} has no waypoints')
        for name, secs in schedule:
            if name not in zones:
                raise ValueError(f'schedule names unknown zone {name}')
            if secs <= 0.0:
                raise ValueError(f'schedule slot for {name} must be > 0 s')
        for name, polygon in (restricted or {}).items():
            if len(polygon) < 3:
                raise ValueError(f'restricted area {name} needs >= 3 points')
        self.zones = {name: [(float(x), float(y)) for x, y in wps]
                      for name, wps in zones.items()}
        self.schedule = [(name, float(secs)) for name, secs in schedule]
        self.restricted = {name: [(float(x), float(y)) for x, y in poly]
                           for name, poly in (restricted or {}).items()}
        self._period = sum(secs for _, secs in self.schedule)

    @classmethod
    def from_yaml(cls, path: str) -> 'GuardZones':
        """Load a guard zones file (format in the module docstring)."""
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        schedule = [(slot['zone'], slot['secs'])
                    for slot in data.get('patrol_schedule') or []]
        return cls(data.get('patrol_zones') or {}, schedule,
                   data.get('restricted_areas') or {})

    @classmethod
    def single(cls, waypoints: Sequence[Point]) -> 'GuardZones':
        """One zone named 'patrol', no schedule, no restricted areas."""
        return cls({'patrol': list(waypoints)})

    # ------------------------------------------------------------------

    def zone_at(self, t: float) -> str:
        """Zone the schedule assigns to clock time ``t`` (s)."""
        if not self.schedule:
            return next(iter(self.zones))
        t %= self._period
        for name, secs in self.schedule:
            if t < secs:
                return name
            t -= secs
        return self.schedule[-1][0]  # float rounding at the period's end

    def route_at(self, t: float) -> Tuple[str, List[Point]]:
        """(zone name, its waypoints) for clock time ``t``."""
        name = self.zone_at(t)
        return name, self.zones[name]

    def areas_containing(self, x: float, y: float) -> List[str]:
        """Names of the restricted areas that contain (x, y)."""
        return [name for name, poly in self.restricted.items()
                if point_in_polygon(x, y, poly)]
