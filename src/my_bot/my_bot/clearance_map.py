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

"""Distance-to-obstacle lookups on the saved occupancy map.

The robot spawns at the world origin with zero yaw and SLAM starts from
there, so map coordinates equal Gazebo world coordinates. Nodes that move
things around the museum (the pedestrian, the ball) use this to stay clear
of walls and exhibits without any physics or planning stack.

Pure Python + numpy + OpenCV, no ROS: importable by unit tests.
"""

import math
import os
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml


# map_saver writes 254 for free, 0 for occupied and 205 for unknown. Only
# confidently free cells count as free; unknown space (outside the walls,
# inside solid exhibits) is treated as blocked.
_FREE_PIXEL = 250


class ClearanceMap:
    """Clearance (metres to the nearest blocked cell) over a 2D grid."""

    def __init__(self, free: np.ndarray, resolution: float,
                 origin: Tuple[float, float]):
        """Build from a boolean free-space grid (row 0 = top = max y)."""
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.height, self.width = free.shape
        dist = cv2.distanceTransform(
            free.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        self._clear = dist.astype(np.float32) * self.resolution
        self._free_cells = {}  # min_clear -> (rows, cols), for sampling

    # ------------------------------------------------------------------

    @classmethod
    def from_yaml(cls, yaml_path: str) -> 'ClearanceMap':
        """Load a map_server YAML + PGM pair."""
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(yaml_path), image)
        pixels = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
        if pixels is None:
            raise IOError(f'cannot read map image {image}')
        if int(meta.get('negate', 0)):
            pixels = 255 - pixels
        return cls(pixels >= _FREE_PIXEL, meta['resolution'],
                   (meta['origin'][0], meta['origin'][1]))

    @classmethod
    def from_bounds(cls, half_size: float,
                    resolution: float = 0.05) -> 'ClearanceMap':
        """Square walled area of +/- half_size metres (fallback, no map)."""
        n = int(round(2.0 * half_size / resolution))
        free = np.ones((n, n), dtype=bool)
        free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False
        return cls(free, resolution, (-half_size, -half_size))

    # ------------------------------------------------------------------

    def clearance(self, x: float, y: float) -> float:
        """Metres from (x, y) to the nearest blocked cell; 0 off the map."""
        col = int((x - self.origin[0]) / self.resolution)
        row = self.height - 1 - int((y - self.origin[1]) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return float(self._clear[row, col])
        return 0.0

    def free_distance(self, x: float, y: float, heading: float,
                      max_dist: float, min_clear: float,
                      step: Optional[float] = None) -> float:
        """How far one can travel along ``heading`` keeping ``min_clear``.

        If (x, y) is already closer than ``min_clear`` to an obstacle, any
        direction that does not reduce clearance further still counts as
        free, so something stuck near a wall can always move away from it.
        """
        step = step or self.resolution
        limit = min(min_clear, self.clearance(x, y) - 0.02)
        c, s = math.cos(heading), math.sin(heading)
        d = step
        while d <= max_dist:
            if self.clearance(x + c * d, y + s * d) < limit:
                return d - step
            d += step
        return max_dist

    def segment_clearance(self, a: Tuple[float, float],
                          b: Tuple[float, float]) -> float:
        """Minimum clearance along the straight segment a -> b."""
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / self.resolution))
        return min(
            self.clearance(a[0] + (b[0] - a[0]) * i / n,
                           a[1] + (b[1] - a[1]) * i / n)
            for i in range(n + 1))

    def random_free_point(self, min_clear: float, rng) -> Tuple[float, float]:
        """Uniformly sample a point with at least ``min_clear`` clearance."""
        if min_clear not in self._free_cells:
            self._free_cells[min_clear] = np.nonzero(self._clear >= min_clear)
        rows, cols = self._free_cells[min_clear]
        if len(rows) == 0:
            raise ValueError(f'no cell has {min_clear} m clearance')
        i = rng.randrange(len(rows))
        x = self.origin[0] + (cols[i] + 0.5) * self.resolution
        y = self.origin[1] + (self.height - 1 - rows[i] + 0.5) * self.resolution
        return x, y


def safe_heading(cmap: ClearanceMap, x: float, y: float, desired: float,
                 lookahead: float, min_clear: float,
                 step_deg: float = 10.0) -> Tuple[float, float]:
    """Heading closest to ``desired`` with a clear path of ``lookahead``.

    Candidates fan out from ``desired`` in ``step_deg`` increments. The first
    one with ``lookahead`` metres of free space wins; if none has, the most
    open direction is returned. Returns (heading, free_distance).
    """
    best = (desired, -1.0)
    n = int(round(180.0 / step_deg))
    for k in range(n + 1):
        for sign in ((1,) if k in (0, n) else (1, -1)):
            h = desired + sign * math.radians(k * step_deg)
            free = cmap.free_distance(x, y, h, lookahead, min_clear)
            if free >= lookahead:
                return h, free
            if free > best[1]:
                best = (h, free)
    return best
