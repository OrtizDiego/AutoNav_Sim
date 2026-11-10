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

"""Nav2 stand-in for the web demo: A* on the saved map + a path follower.

Nav2 itself (costmaps, planner and controller servers, AMCL, lifecycle
nodes) does not run in a browser. The guard only talks to it through
nav2_simple_commander's ``BasicNavigator``: goToPose, isTaskComplete,
cancelTask, getResult. ``DemoNavigator`` offers those four calls and does
the two jobs behind them in the simplest way that gives the same behaviour
on screen:

* global plan: A* on a 0.1 m grid of the saved map, cells closer to a wall
  than the robot radius are blocked and cells near walls cost more (the
  inflation layer's job), then the path is pulled straight where the map
  allows;
* controller: follows the path with a look-ahead point at up to 0.8 m/s
  (nav2_params.yaml's limit), turns on the spot for large heading errors,
  stops for anything the lidar sees right ahead, and finishes by turning
  to the goal's heading.

It publishes /cmd_vel like Nav2's controller. Odometry is ground truth in
the demo, so the odom and map frames coincide and there is no AMCL.
"""

import heapq
import math

import numpy as np

import rosfake
from rosfake import Node, TaskResult, Twist

SQRT2 = math.sqrt(2.0)


def plan_path(cmap, start, goal, cell=0.1, robot_radius=0.22,
              inflation=0.7, wall_cost=3.0, grid_cache=None):
    """Return [(x, y), ...] from ``start`` to ``goal``, or None if there is none."""
    grid = grid_cache if grid_cache is not None else PlanGrid(
        cmap, cell, robot_radius, inflation, wall_cost)
    return grid.plan(start, goal)


class PlanGrid:
    """Coarse traversability + cost grid over a ClearanceMap."""

    def __init__(self, cmap, cell=0.1, robot_radius=0.22, inflation=0.7,
                 wall_cost=3.0):
        self.cmap = cmap
        self.cell = cell
        self.robot_radius = robot_radius
        self.nx = int(cmap.width * cmap.resolution / cell)
        self.ny = int(cmap.height * cmap.resolution / cell)
        xs = cmap.origin[0] + (np.arange(self.nx) + 0.5) * cell
        ys = cmap.origin[1] + (np.arange(self.ny) + 0.5) * cell
        gx, gy = np.meshgrid(xs, ys)
        clear = cmap.clearance_many(gx.ravel(), gy.ravel()).reshape(self.ny, self.nx)
        self.free = (clear >= robot_radius).ravel().tolist()
        near = np.clip((inflation - clear) / (inflation - robot_radius), 0.0, 1.0)
        self.cost = (1.0 + wall_cost * near).ravel().tolist()

    def cell_of(self, x, y):
        return (int((x - self.cmap.origin[0]) / self.cell),
                int((y - self.cmap.origin[1]) / self.cell))

    def centre(self, i, j):
        return (self.cmap.origin[0] + (i + 0.5) * self.cell,
                self.cmap.origin[1] + (j + 0.5) * self.cell)

    def _nearest_free(self, i, j, max_ring=12):
        """Closest traversable cell to (i, j): the robot may hug a wall."""
        if 0 <= i < self.nx and 0 <= j < self.ny and self.free[j * self.nx + i]:
            return i, j
        best = None
        for ring in range(1, max_ring + 1):
            for di in range(-ring, ring + 1):
                for dj in range(-ring, ring + 1):
                    if max(abs(di), abs(dj)) != ring:
                        continue
                    a, b = i + di, j + dj
                    if 0 <= a < self.nx and 0 <= b < self.ny and self.free[b * self.nx + a]:
                        d = di * di + dj * dj
                        if best is None or d < best[0]:
                            best = (d, a, b)
            if best is not None:
                return best[1], best[2]
        return None

    def plan(self, start, goal):
        s = self._nearest_free(*self.cell_of(*start))
        g = self._nearest_free(*self.cell_of(*goal))
        if s is None or g is None:
            return None
        nx, ny, free, cost = self.nx, self.ny, self.free, self.cost
        si, gi = s[1] * nx + s[0], g[1] * nx + g[0]
        gx, gy = g
        dist = {si: 0.0}
        parent = {}
        heap = [(0.0, si)]
        done = set()
        while heap:
            _, node = heapq.heappop(heap)
            if node in done:
                continue
            if node == gi:
                break
            done.add(node)
            i, j = node % nx, node // nx
            base = dist[node]
            for di, dj, step in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                                 (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2),
                                 (-1, -1, SQRT2)):
                a, b = i + di, j + dj
                if not (0 <= a < nx and 0 <= b < ny):
                    continue
                nb = b * nx + a
                if not free[nb] or nb in done:
                    continue
                if di and dj and not (free[j * nx + a] and free[b * nx + i]):
                    continue  # no cutting corners
                d = base + step * cost[nb]
                if d < dist.get(nb, math.inf):
                    dist[nb] = d
                    parent[nb] = node
                    h = math.hypot(a - gx, b - gy)
                    heapq.heappush(heap, (d + h, nb))
        if gi not in dist:
            return None
        cells = [gi]
        while cells[-1] != si:
            cells.append(parent[cells[-1]])
        cells.reverse()
        points = [self.centre(c % nx, c // nx) for c in cells]
        points[0] = tuple(start)
        if g == self.cell_of(*goal):
            points[-1] = tuple(goal)
        return self._pull(points)

    def _pull(self, points, min_clear=None):
        """Drop the grid's staircase where a straight leg keeps its distance."""
        if min_clear is None:
            min_clear = self.robot_radius + 0.12
        out = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = min(len(points) - 1, i + 40)
            while j > i + 1 and self.cmap.segment_clearance(points[i], points[j]) < min_clear:
                j -= 1
            out.append(points[j])
            i = j
        return out


def _densify(points, step=0.1):
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        for k in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return out


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DemoNavigator(Node):
    """BasicNavigator's interface on the planner and follower above."""

    cmap = None  # ClearanceMap of the saved map, set by the demo before use
    instance = None

    def __init__(self, *args, **kwargs):
        super().__init__('nav2')
        DemoNavigator.instance = self
        self.max_speed = 0.8        # nav2_params.yaml: DWB max_vel_x
        self.max_yaw_rate = 1.5
        self.lookahead = 0.55
        self.xy_tolerance = 0.2
        self.yaw_tolerance = 0.2
        self.stop_distance = 0.38   # lidar return ahead: wait
        self.blocked_timeout = 6.0
        self._grid = PlanGrid(self.cmap) if self.cmap is not None else None
        self._pose = None
        self._front = math.inf
        self._path = []
        self._goal_yaw = None
        self._index = 0
        self._done = True
        self._result = TaskResult.UNKNOWN
        self._blocked_since = None
        self.path_version = 0
        self.plan_ms = 0.0
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(rosfake.Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(rosfake.LaserScan, '/scan', self._scan_cb, 10)
        self.create_timer(0.05, self._control)

    # -- BasicNavigator ----------------------------------------------------

    def waitUntilNav2Active(self, *args, **kwargs):
        self.get_logger().info('Nav2 stand-in active (A* + path follower)')

    def goToPose(self, goal):
        q = goal.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        target = (goal.pose.position.x, goal.pose.position.y)
        self._path, self._index = [], 0
        self.path_version += 1
        self._blocked_since = None
        if self._pose is None or self._grid is None:
            self._finish(TaskResult.FAILED)
            return False
        import time
        t0 = time.perf_counter()
        path = self._grid.plan(self._pose[:2], target)
        self.plan_ms = (time.perf_counter() - t0) * 1e3
        if not path:
            self.get_logger().warn('no path to ({:.2f}, {:.2f})'.format(*target))
            self._finish(TaskResult.FAILED)
            return False
        self._path = _densify(path)
        self._goal_yaw = yaw
        self._done = False
        self._result = TaskResult.UNKNOWN
        return True

    def isTaskComplete(self):
        return self._done

    def cancelTask(self):
        if not self._done:
            self._finish(TaskResult.CANCELED)

    def getResult(self):
        return self._result

    # -- for the demo's display ---------------------------------------------

    @property
    def path(self):
        return self._path

    @property
    def active(self):
        return not self._done

    # ----------------------------------------------------------------------

    def _finish(self, result):
        self._done = True
        self._result = result
        self._path = []
        self.path_version += 1
        self._cmd_pub.publish(Twist())

    def _odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._pose = (p.x, p.y, math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z))

    def _scan_cb(self, msg):
        best = math.inf
        n = len(msg.ranges)
        lo = int(math.floor((-0.5 - msg.angle_min) / msg.angle_increment))
        hi = int(math.ceil((0.5 - msg.angle_min) / msg.angle_increment))
        for i in range(lo, hi + 1):
            r = msg.ranges[i % n]
            if math.isfinite(r) and r >= 0.12:
                best = min(best, r)
        self._front = best

    def _control(self):
        if self._done or self._pose is None:
            return
        x, y, yaw = self._pose
        path = self._path
        gx, gy = path[-1]
        cmd = Twist()
        if math.hypot(gx - x, gy - y) <= self.xy_tolerance:
            err = _wrap(self._goal_yaw - yaw)
            if abs(err) <= self.yaw_tolerance:
                self._finish(TaskResult.SUCCEEDED)
                return
            cmd.angular.z = max(-self.max_yaw_rate, min(self.max_yaw_rate, 2.0 * err))
            self._cmd_pub.publish(cmd)
            return

        # Progress along the path, then the look-ahead point
        i = self._index
        end = min(len(path), i + 30)
        self._index = i = min(
            range(i, end), key=lambda k: (path[k][0] - x) ** 2 + (path[k][1] - y) ** 2)
        j = i
        while j < len(path) - 1 and math.hypot(path[j][0] - x, path[j][1] - y) < self.lookahead:
            j += 1
        tx, ty = path[j]
        err = _wrap(math.atan2(ty - y, tx - x) - yaw)
        cmd.angular.z = max(-self.max_yaw_rate, min(self.max_yaw_rate, 2.2 * err))
        if abs(err) < 0.9:
            remaining = math.hypot(gx - x, gy - y)
            speed = min(self.max_speed, 0.15 + 1.2 * remaining)
            cmd.linear.x = speed * max(0.0, math.cos(err)) ** 2
            if self._front < self.stop_distance:
                cmd.linear.x = 0.0
                now = rosfake.now()
                if self._blocked_since is None:
                    self._blocked_since = now
                elif now - self._blocked_since > self.blocked_timeout:
                    self.get_logger().warn('goal aborted: blocked')
                    self._finish(TaskResult.FAILED)
                    return
            else:
                self._blocked_since = None
        self._cmd_pub.publish(cmd)
