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

"""Frontier-based exploration on an occupancy grid (explorer node).

A frontier cell is known free space next to unknown space: driving there
lets the lidar see past it. Neighbouring frontier cells form a frontier;
each gets one goal, the cell of the frontier with the most room around it
(distance to the nearest occupied cell), so Nav2 can reach and stand on it.

The grid is a nav_msgs/OccupancyGrid as an array: row 0 at the origin
(minimum y), -1 unknown, 0..100 occupancy. Cells below ``free_below`` are
free, cells at or above ``occupied_from`` are occupied.

Pure Python + numpy + OpenCV, no ROS: importable by unit tests.
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

Point = Tuple[float, float]


@dataclass
class Grid:
    """An occupancy grid with its placement in the map frame."""

    data: np.ndarray            # (height, width) int8; row 0 = min y
    resolution: float
    origin: Point               # map-frame position of cell (0, 0)'s corner

    def to_world(self, row, col):
        """Cell centre(s) in the map frame."""
        return (self.origin[0] + (np.asarray(col) + 0.5) * self.resolution,
                self.origin[1] + (np.asarray(row) + 0.5) * self.resolution)

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """(row, col) of the cell holding (x, y); may lie off the grid."""
        return (int(math.floor((y - self.origin[1]) / self.resolution)),
                int(math.floor((x - self.origin[0]) / self.resolution)))

    def inside(self, row: int, col: int) -> bool:
        return 0 <= row < self.data.shape[0] and 0 <= col < self.data.shape[1]


@dataclass
class Frontier:
    """A connected frontier: its goal point, size and centroid."""

    goal: Point
    size: int                   # frontier cells
    centroid: Point
    clearance: float            # metres from the goal to the nearest obstacle


def classify(data: np.ndarray, free_below: int = 25, occupied_from: int = 65):
    """Return boolean (free, unknown, occupied) masks."""
    unknown = data < 0
    free = (data >= 0) & (data < free_below)
    occupied = data >= occupied_from
    return free, unknown, occupied


def frontier_mask(data: np.ndarray, free_below: int = 25,
                  occupied_from: int = 65) -> np.ndarray:
    """Free cells with an unknown 4-neighbour."""
    free, unknown, _ = classify(data, free_below, occupied_from)
    near_unknown = cv2.dilate(unknown.astype(np.uint8),
                              cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))) > 0
    return free & near_unknown


def obstacle_clearance(grid: Grid, occupied_from: int = 65) -> np.ndarray:
    """Metres from every cell to the nearest occupied cell (unknown is open)."""
    _, _, occupied = classify(grid.data, occupied_from=occupied_from)
    dist = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2,
                                 cv2.DIST_MASK_PRECISE)
    return dist * grid.resolution


def find_frontiers(grid: Grid, min_size: int = 8, min_clearance: float = 0.3,
                   free_below: int = 25, occupied_from: int = 65) -> List[Frontier]:
    """Frontiers of at least ``min_size`` cells with a goal ``min_clearance`` clear.

    A frontier whose cells are all closer than ``min_clearance`` to an
    obstacle (a gap the robot cannot fit through) is dropped.
    """
    mask = frontier_mask(grid.data, free_below, occupied_from)
    if not mask.any():
        return []
    clearance = obstacle_clearance(grid, occupied_from)
    count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    frontiers = []
    for label in range(1, count):
        rows, cols = np.nonzero(labels == label)
        if rows.size < min_size:
            continue
        clear = clearance[rows, cols]
        best = int(np.argmax(clear))
        if clear[best] < min_clearance:
            continue
        gx, gy = grid.to_world(rows[best], cols[best])
        cx, cy = grid.to_world(rows.mean(), cols.mean())
        frontiers.append(Frontier(goal=(float(gx), float(gy)), size=int(rows.size),
                                  centroid=(float(cx), float(cy)),
                                  clearance=float(clear[best])))
    return frontiers


def choose_frontier(frontiers: Sequence[Frontier], robot: Point,
                    blacklist: Iterable[Point] = (), blacklist_radius: float = 0.5,
                    size_weight: float = 0.5, resolution: float = 0.05,
                    ) -> Optional[Frontier]:
    """Pick the next frontier: near, long, and not blacklisted.

    Cost = metres to the goal - ``size_weight`` * frontier length in metres
    (cells * ``resolution``), so a long frontier a little further away can
    beat a sliver next to the robot. None when every frontier is blacklisted.
    """
    blacklist = list(blacklist)
    best, best_cost = None, math.inf
    for f in frontiers:
        if any(math.dist(f.goal, b) < blacklist_radius for b in blacklist):
            continue
        cost = math.dist(robot, f.goal) - size_weight * f.size * resolution
        if cost < best_cost:
            best, best_cost = f, cost
    return best


def unknown_near(grid: Grid, point: Point, radius: float) -> bool:
    """Tell whether any unknown cell lies within ``radius`` of ``point``.

    Once false, a goal's frontier has been seen and the goal is moot.
    """
    row, col = grid.to_cell(*point)
    r = int(math.ceil(radius / grid.resolution))
    h, w = grid.data.shape
    r0, r1, c0, c1 = max(0, row - r), min(h, row + r + 1), max(0, col - r), min(w, col + r + 1)
    if r0 >= r1 or c0 >= c1:
        return True  # off the grid: still unexplored
    window = grid.data[r0:r1, c0:c1] < 0
    rows, cols = np.nonzero(window)
    if rows.size == 0:
        return False
    d = np.hypot(rows + r0 - row, cols + c0 - col) * grid.resolution
    return bool((d <= radius).any())


def known_area(grid: Grid, free_below: int = 25) -> float:
    """Square metres of free space seen so far."""
    free, _, _ = classify(grid.data, free_below)
    return float(free.sum()) * grid.resolution ** 2
