#!/usr/bin/env python3

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

"""Drive the red ball in ball.world: autopilot plus keyboard override.

Autopilot
---------
The ball runs a figure-eight through the museum: one lobe inside the U
where the robot starts, one in the open hall to its right. The path keeps
over 1 m between the ball's centre and every wall in maps/my_map (checked
by test_ball.py). The ball plays with the robot along that path:

* robot close   -> the ball speeds up and runs away
* robot far     -> the ball waits for it to catch up
* every so often the ball pauses, and sometimes reverses direction

Teleop
------
``ball_teleop`` publishes TwistStamped on /ball/teleop. While those keep
arriving the ball is driven by hand; ``teleop_timeout`` seconds after the
last one the autopilot takes over again from the nearest path point.
frame_id selects how the keys are read:

  world  linear.x/y are world +x/+y
  robot  linear.x = away from the robot, linear.y = robot's left, as the
         robot's camera sees the ball

The ball's libgazebo_ros_planar_move plugin takes body-frame velocities, so
every command is rotated by the ball's yaw, and yaw is held at zero.

Subscribes: /ball/odom, /odom (robot; spawns at the world origin), /ball/teleop
Publishes:  /ball/cmd_vel
"""

import math
import random
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

Point = Tuple[float, float]


# ---------------------------------------------------------------------------
# Pure helpers (importable by tests without a ROS runtime)
# ---------------------------------------------------------------------------

def figure_eight(center: Point, half_width: float, half_height: float,
                 n: int = 400) -> List[Point]:
    """Closed figure-eight (lemniscate of Gerono), lobes along x."""
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((center[0] + half_width * math.sin(t),
                    center[1] + 2.0 * half_height * math.sin(t) * math.cos(t)))
    return pts


def world_to_body(vx: float, vy: float, yaw: float) -> Point:
    """Rotate a world-frame velocity into a body frame with heading ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def robot_view_to_world(forward: float, left: float, ball: Point,
                        robot: Point) -> Point:
    """Velocity given as (away from robot, robot's left) -> world frame."""
    dx, dy = ball[0] - robot[0], ball[1] - robot[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return forward, left
    ux, uy = dx / norm, dy / norm
    return forward * ux - left * uy, forward * uy + left * ux


def clamp_norm(vx: float, vy: float, limit: float) -> Point:
    """Scale (vx, vy) down so its length is at most ``limit``."""
    n = math.hypot(vx, vy)
    if n > limit > 0.0:
        return vx * limit / n, vy * limit / n
    return vx, vy


class BallAutopilot:
    """Moves a reference point along a closed path; the ball tracks it."""

    def __init__(self, path: List[Point], cruise_speed: float = 0.35,
                 flee_speed: float = 0.8, flee_dist: float = 1.8,
                 wait_dist: float = 4.5, max_speed: float = 1.0,
                 kp: float = 1.5, pause_every: Tuple[float, float] = (12.0, 25.0),
                 pause_secs: float = 2.5, reverse_prob: float = 0.35,
                 rng: Optional[random.Random] = None):
        self.path = path
        self._cum = [0.0]
        for i in range(1, len(path) + 1):
            self._cum.append(self._cum[-1] + math.dist(path[i - 1], path[i % len(path)]))
        self.length = self._cum[-1]
        self.cruise_speed = cruise_speed
        self.flee_speed = flee_speed
        self.flee_dist = flee_dist
        self.wait_dist = wait_dist
        self.max_speed = max_speed
        self.kp = kp
        self.pause_every = pause_every
        self.pause_secs = pause_secs
        self.reverse_prob = reverse_prob
        self._rng = rng or random.Random()
        self.s = 0.0
        self.direction = 1.0
        self._t = 0.0
        self._pause_until = -1.0
        self._next_pause = self._rng.uniform(*pause_every)

    # ------------------------------------------------------------------

    def point_at(self, s: float) -> Point:
        """Path point at arc length ``s`` (wraps around)."""
        s %= self.length
        lo, hi = 0, len(self.path)
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        a, b = self.path[lo], self.path[(lo + 1) % len(self.path)]
        seg = self._cum[lo + 1] - self._cum[lo]
        f = (s - self._cum[lo]) / seg if seg > 0 else 0.0
        return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f

    def tangent_at(self, s: float) -> Point:
        """Unit tangent (in the direction of increasing s)."""
        a, b = self.point_at(s - 0.05), self.point_at(s + 0.05)
        n = math.dist(a, b) or 1.0
        return (b[0] - a[0]) / n, (b[1] - a[1]) / n

    def snap_to(self, p: Point) -> None:
        """Restart from the path point closest to ``p``."""
        best = min(range(len(self.path)), key=lambda i: math.dist(self.path[i], p))
        self.s = self._cum[best]

    def target_speed(self, robot_dist: float) -> float:
        """How fast to run given the robot's distance."""
        if self._t < self._pause_until or robot_dist > self.wait_dist:
            return 0.0
        if robot_dist < self.flee_dist:
            return self.flee_speed
        # Ease from flee speed down to cruise speed as the robot drops back.
        f = (robot_dist - self.flee_dist) / max(self.wait_dist - self.flee_dist, 1e-6)
        return self.flee_speed + (self.cruise_speed - self.flee_speed) * min(1.0, 2.0 * f)

    def step(self, dt: float, ball: Point, robot: Point) -> Point:
        """Advance by ``dt`` and return the world-frame velocity for the ball."""
        self._t += dt
        robot_dist = math.dist(ball, robot)
        if self._t >= self._next_pause and robot_dist >= self.flee_dist:
            self._pause_until = self._t + self.pause_secs
            self._next_pause = self._pause_until + self._rng.uniform(*self.pause_every)
            if self._rng.random() < self.reverse_prob:
                self.direction = -self.direction

        speed = self.target_speed(robot_dist)
        ref = self.point_at(self.s)
        # Do not let the reference run away if the ball is held up.
        if math.dist(ref, ball) < 0.5:
            self.s = (self.s + self.direction * speed * dt) % self.length
            ref = self.point_at(self.s)
        tx, ty = self.tangent_at(self.s)
        vx = self.direction * speed * tx + self.kp * (ref[0] - ball[0])
        vy = self.direction * speed * ty + self.kp * (ref[1] - ball[1])
        return clamp_norm(vx, vy, self.max_speed)


# Figure-eight through the U and the hall to its right (see module docstring).
DEFAULT_PATH_CENTER = (2.5, -2.5)
DEFAULT_PATH_HALF_WIDTH = 3.5
DEFAULT_PATH_HALF_HEIGHT = 1.25


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class BallControllerNode(Node):
    """Publishes /ball/cmd_vel from the autopilot or the teleop keys."""

    def __init__(self):
        super().__init__('ball_controller')

        self.declare_parameter('autopilot', True)
        # Defaults match DEFAULT_PATH_* (literals so the params test can parse them)
        self.declare_parameter('path_center', [2.5, -2.5])
        self.declare_parameter('path_half_width', 3.5)
        self.declare_parameter('path_half_height', 1.25)
        self.declare_parameter('cruise_speed', 0.35)
        self.declare_parameter('flee_speed', 0.8)
        self.declare_parameter('flee_distance', 1.8)
        self.declare_parameter('wait_distance', 4.5)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('accel', 1.5)
        self.declare_parameter('teleop_timeout', 1.0)
        self.declare_parameter('update_rate_hz', 30.0)

        gp = self.get_parameter
        self._autopilot_enabled = bool(gp('autopilot').value)
        path = figure_eight(tuple(gp('path_center').value),
                            float(gp('path_half_width').value),
                            float(gp('path_half_height').value))
        self._pilot = BallAutopilot(
            path,
            cruise_speed=float(gp('cruise_speed').value),
            flee_speed=float(gp('flee_speed').value),
            flee_dist=float(gp('flee_distance').value),
            wait_dist=float(gp('wait_distance').value),
            max_speed=float(gp('max_speed').value))
        self._accel = float(gp('accel').value)
        self._teleop_timeout = float(gp('teleop_timeout').value)
        self._dt = 1.0 / max(float(gp('update_rate_hz').value), 1.0)

        self._ball: Optional[Point] = None
        self._ball_yaw = 0.0
        self._robot: Point = (0.0, 0.0)
        self._teleop: Optional[TwistStamped] = None
        self._last_teleop = -1.0e9
        self._manual = False
        self._vel = (0.0, 0.0)

        self._cmd_pub = self.create_publisher(Twist, '/ball/cmd_vel', 10)
        self.create_subscription(Odometry, '/ball/odom', self._on_ball_odom, 10)
        self.create_subscription(Odometry, '/odom', self._on_robot_odom, 10)
        self.create_subscription(TwistStamped, '/ball/teleop', self._on_teleop, 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            'ball_controller: autopilot '
            + ('on' if self._autopilot_enabled else 'off')
            + ' (run `make teleop-ball` to take over)')

    # ------------------------------------------------------------------

    def _on_ball_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        first = self._ball is None
        self._ball = (p.x, p.y)
        self._ball_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if first:
            self._pilot.snap_to(self._ball)

    def _on_robot_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot = (p.x, p.y)

    def _on_teleop(self, msg: TwistStamped) -> None:
        self._teleop = msg
        self._last_teleop = self._now()

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._ball is None:
            return  # plugin not up yet
        manual = (self._now() - self._last_teleop) < self._teleop_timeout
        if manual != self._manual:
            self._manual = manual
            if not manual:
                self._pilot.snap_to(self._ball)
            self.get_logger().info('teleop' if manual else 'autopilot')

        if manual:
            t = self._teleop.twist.linear
            if self._teleop.header.frame_id == 'robot':
                target = robot_view_to_world(t.x, t.y, self._ball, self._robot)
            else:
                target = (t.x, t.y)
        elif self._autopilot_enabled:
            target = self._pilot.step(self._dt, self._ball, self._robot)
        else:
            target = (0.0, 0.0)

        # Smooth with an acceleration limit so the ball never jumps.
        dvx, dvy = clamp_norm(target[0] - self._vel[0], target[1] - self._vel[1],
                              self._accel * self._dt)
        self._vel = (self._vel[0] + dvx, self._vel[1] + dvy)

        bx, by = world_to_body(self._vel[0], self._vel[1], self._ball_yaw)
        cmd = Twist()
        cmd.linear.x = bx
        cmd.linear.y = by
        cmd.angular.z = -2.0 * self._ball_yaw  # keep body frame == world frame
        self._cmd_pub.publish(cmd)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Initialize and spin the BallControllerNode."""
    rclpy.init(args=args)
    node = BallControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
