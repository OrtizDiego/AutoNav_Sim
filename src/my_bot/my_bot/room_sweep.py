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

"""Viewpoints that sweep the rooms around where an intruder was lost.

After the search of the predicted path, the guard visits a few viewpoints
that between them see the free floor within ``radius`` of the last-seen
spot, so a person standing behind an exhibit or around a corner comes into
view. Greedy set cover on a coarse grid of the saved map:

1. Floor points (``spacing`` apart, free on the map) within ``radius`` of
   the centre are the targets; those already seen from ``seen`` (where the
   search looked) count as covered.
2. Candidate viewpoints are grid points with ``min_clear`` clearance that
   the robot can reach from ``start`` through gaps of ``pass_clear``.
3. A viewpoint sees a target within ``view_range`` when the straight line
   between them stays on free cells. The one that sees the most uncovered
   targets is picked, until ``coverage`` of them is seen, ``max_points``
   are picked, or no viewpoint adds ``min_gain`` targets.

Each viewpoint carries the heading toward the centroid of the targets it
added, so the camera (not just the lidar) faces them on arrival. They are
returned in nearest-neighbour order from ``start``.

Pure Python + numpy (no ROS); positions in the map frame.
"""

from collections import deque
import math
from typing import List, Sequence, Tuple

import numpy as np

from my_bot.clearance_map import ClearanceMap

Point = Tuple[float, float]
Viewpoint = Tuple[float, float, float]  # x, y, yaw (map frame)

_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _visible(cmap: ClearanceMap, origin: Point, targets: np.ndarray,
             view_range: float) -> np.ndarray:
    """Boolean mask: which ``targets`` (N x 2) the point ``origin`` sees."""
    d = targets - np.asarray(origin)
    dist = np.hypot(d[:, 0], d[:, 1])
    mask = dist <= view_range
    if not mask.any():
        return mask
    k = max(2, int(math.ceil(view_range / cmap.resolution)))
    # Samples strictly between the two ends, scaled to each segment's length
    frac = (np.arange(1, k) / k)[None, :]
    xs = origin[0] + d[mask, 0:1] * frac
    ys = origin[1] + d[mask, 1:2] * frac
    clear = cmap.clearance_many(xs, ys).min(axis=1)
    mask[np.nonzero(mask)[0]] = clear > 0.0
    return mask


def _reachable(cmap: ClearanceMap, nodes: np.ndarray, shape: Tuple[int, int],
               start: Point, spacing: float, pass_clear: float) -> np.ndarray:
    """Boolean mask over the grid: nodes connected to ``start``."""
    rows, cols = shape
    grid = nodes.reshape(rows, cols, 2)
    ok = cmap.clearance_many(grid[..., 0], grid[..., 1]) >= pass_clear
    if not ok.any():
        return np.zeros(rows * cols, dtype=bool)
    d = np.hypot(grid[..., 0] - start[0], grid[..., 1] - start[1])
    d[~ok] = np.inf
    seed = np.unravel_index(int(np.argmin(d)), (rows, cols))
    if d[seed] > 2.0 * spacing:
        return np.zeros(rows * cols, dtype=bool)  # start is not near open floor
    seen = np.zeros((rows, cols), dtype=bool)
    seen[seed] = True
    queue = deque([seed])
    while queue:
        r, c = queue.popleft()
        for dr, dc in _NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or seen[nr, nc] or not ok[nr, nc]:
                continue
            a, b = grid[r, c], grid[nr, nc]
            if cmap.segment_clearance(tuple(a), tuple(b)) < pass_clear:
                continue
            seen[nr, nc] = True
            queue.append((nr, nc))
    return seen.ravel()


def plan_sweep(cmap: ClearanceMap, center: Point, start: Point,
               seen: Sequence[Point] = (), radius: float = 5.0,
               spacing: float = 0.5, view_range: float = 4.0,
               min_clear: float = 0.4, pass_clear: float = 0.25,
               max_points: int = 4, coverage: float = 0.9,
               min_gain: int = 3) -> List[Viewpoint]:
    """Viewpoints (x, y, yaw) that sweep the floor around ``center``."""
    lo_x = min(center[0] - radius, start[0]) - spacing
    hi_x = max(center[0] + radius, start[0]) + spacing
    lo_y = min(center[1] - radius, start[1]) - spacing
    hi_y = max(center[1] + radius, start[1]) + spacing
    gx = np.arange(lo_x, hi_x + 1e-9, spacing)
    gy = np.arange(lo_y, hi_y + 1e-9, spacing)
    mx, my = np.meshgrid(gx, gy)
    nodes = np.stack([mx.ravel(), my.ravel()], axis=1)
    clear = cmap.clearance_many(nodes[:, 0], nodes[:, 1])
    near = np.hypot(nodes[:, 0] - center[0], nodes[:, 1] - center[1]) <= radius

    targets = nodes[near & (clear > 0.0)]
    if len(targets) == 0:
        return []
    covered = np.zeros(len(targets), dtype=bool)
    for point in seen:
        covered |= _visible(cmap, point, targets, view_range)

    reach = _reachable(cmap, nodes, mx.shape, start, spacing, pass_clear)
    candidates = nodes[reach & (clear >= min_clear) & near]
    views = [_visible(cmap, tuple(c), targets, view_range) for c in candidates]

    picked = []
    while len(picked) < max_points and covered.mean() < coverage and views:
        gains = [int((v & ~covered).sum()) for v in views]
        best = int(np.argmax(gains))
        if gains[best] < min_gain:
            break
        new = views[best] & ~covered
        cx, cy = candidates[best]
        tx, ty = targets[new].mean(axis=0)
        picked.append((float(cx), float(cy), math.atan2(ty - cy, tx - cx)))
        covered |= views[best]

    ordered, here = [], start
    while picked:
        i = min(range(len(picked)),
                key=lambda j: math.hypot(picked[j][0] - here[0], picked[j][1] - here[1]))
        ordered.append(picked.pop(i))
        here = ordered[-1][:2]
    return ordered
