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

"""Behaviour model for the pedestrian intruder in person.world.

This node decides where and how fast the person moves; the Gazebo-side
``PersonActorPlugin`` (package ``person_actor_plugin``) turns the resulting
``/person/cmd_vel`` into actor motion and switches between the walk and run
animations.

Modes
-----
WALK       Wanders between random points at a casual pace.
RUN        Sprints away from the robot after being detected.
EXHAUSTED  Out of breath after a sprint: keeps moving away, but slowly,
           until stamina recovers.

Realism details
---------------
* Speed changes are acceleration-limited and turning is yaw-rate limited,
  so the person never teleports or spins on the spot.
* The person slows down for sharp turns.
* Sprints drain stamina; an empty tank forces EXHAUSTED until it refills.
* Fleeing is steered away from the yard boundary so the person does not
  run into a wall.
* Walking speed has small random variation, as real gait does.

Subscribes
----------
/person/odom       nav_msgs/Odometry   person ground truth (from the plugin)
/odom              nav_msgs/Odometry   robot pose; the robot spawns at the
                                       world origin, so odom ~= world
/person_detected   std_msgs/Bool       robot's tracker has a lock

Publishes
---------
/person/cmd_vel    geometry_msgs/Twist
/person_mode       std_msgs/String     WALK | RUN | EXHAUSTED
"""

from dataclasses import dataclass
import math
import random
from typing import Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


WALK = 'WALK'
RUN = 'RUN'
EXHAUSTED = 'EXHAUSTED'


# ---------------------------------------------------------------------------
# Pure helpers (importable by tests without a ROS runtime)
# ---------------------------------------------------------------------------

def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def update_stamina(stamina: float, mode: str, dt: float,
                   max_run_secs: float, recovery_secs: float) -> float:
    """Drain stamina while running, refill otherwise; result in [0, 1]."""
    if mode == RUN:
        stamina -= dt / max_run_secs
    else:
        stamina += dt / recovery_secs
    return max(0.0, min(1.0, stamina))


def next_mode(mode: str, alarmed: bool, stamina: float,
              resume_stamina: float) -> str:
    """Return the next behaviour mode.

    ``alarmed`` means the person knows it is being watched. A sprint ends
    when stamina runs out; after that the person stays EXHAUSTED until
    stamina is back above ``resume_stamina``.
    """
    if mode == RUN:
        if stamina <= 0.0:
            return EXHAUSTED
        return RUN if alarmed else WALK
    if mode == EXHAUSTED:
        if stamina < resume_stamina:
            return EXHAUSTED
        return RUN if alarmed else WALK
    return RUN if alarmed and stamina > 0.0 else WALK


def flee_heading(person_xy: Tuple[float, float],
                 robot_xy: Tuple[float, float],
                 bounds: float,
                 margin: float = 2.0) -> float:
    """Heading (rad) that moves away from the robot without hitting walls.

    The escape direction points from the robot to the person. Inside
    ``margin`` of the boundary a repulsive term grows linearly and bends the
    path along the wall.
    """
    dx = person_xy[0] - robot_xy[0]
    dy = person_xy[1] - robot_xy[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        dx, dy, norm = 1.0, 0.0, 1.0
    vx, vy = dx / norm, dy / norm

    for axis in (0, 1):
        p = person_xy[axis]
        push = 0.0
        if p > bounds - margin:
            push = -(p - (bounds - margin)) / margin
        elif p < -bounds + margin:
            push = (-bounds + margin - p) / margin
        if axis == 0:
            vx += 2.0 * push
        else:
            vy += 2.0 * push
    return math.atan2(vy, vx)


def steer(heading: float, desired_heading: float, target_speed: float,
          current_speed: float, dt: float, max_yaw_rate: float,
          accel: float, decel: float, k_heading: float = 2.5
          ) -> Tuple[float, float]:
    """Return (speed, yaw_rate) honouring human turn and accel limits."""
    err = wrap_angle(desired_heading - heading)
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, k_heading * err))
    # People slow down to turn sharply.
    target_speed *= max(0.25, math.cos(err)) if abs(err) < math.pi / 2 else 0.25
    if target_speed > current_speed:
        speed = min(target_speed, current_speed + accel * dt)
    else:
        speed = max(target_speed, current_speed - decel * dt)
    return speed, yaw_rate


def random_waypoint(bounds: float, margin: float = 2.0) -> Tuple[float, float]:
    """Pick a random wander target well inside the yard."""
    lim = bounds - margin
    return random.uniform(-lim, lim), random.uniform(-lim, lim)


@dataclass
class PersonPose:
    """Planar pose and speed of the person."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PersonControllerNode(Node):
    """Publish /person/cmd_vel according to the WALK/RUN/EXHAUSTED model."""

    def __init__(self):
        super().__init__('person_controller')

        self.declare_parameter('walk_speed', 0.7)
        self.declare_parameter('walk_speed_jitter', 0.1)
        self.declare_parameter('run_speed', 2.5)
        self.declare_parameter('exhausted_speed', 0.5)
        self.declare_parameter('accel', 2.0)
        self.declare_parameter('decel', 3.0)
        self.declare_parameter('walk_max_yaw_rate', 1.5)
        self.declare_parameter('run_max_yaw_rate', 1.0)
        self.declare_parameter('max_run_secs', 3.5)
        self.declare_parameter('recovery_secs', 8.0)
        self.declare_parameter('resume_stamina', 0.6)
        self.declare_parameter('calm_down_secs', 4.0)
        self.declare_parameter('notice_radius', 7.0)
        self.declare_parameter('bounds', 8.0)
        self.declare_parameter('update_rate_hz', 20.0)

        gp = self.get_parameter
        self._walk_speed = float(gp('walk_speed').value)
        self._walk_jitter = float(gp('walk_speed_jitter').value)
        self._run_speed = float(gp('run_speed').value)
        self._exhausted_speed = float(gp('exhausted_speed').value)
        self._accel = float(gp('accel').value)
        self._decel = float(gp('decel').value)
        self._walk_yaw = float(gp('walk_max_yaw_rate').value)
        self._run_yaw = float(gp('run_max_yaw_rate').value)
        self._max_run_secs = float(gp('max_run_secs').value)
        self._recovery_secs = float(gp('recovery_secs').value)
        self._resume_stamina = float(gp('resume_stamina').value)
        self._calm_down = float(gp('calm_down_secs').value)
        self._notice_radius = float(gp('notice_radius').value)
        self._bounds = float(gp('bounds').value)
        self._dt = 1.0 / max(float(gp('update_rate_hz').value), 1.0)

        self._person = PersonPose()
        self._have_person_pose = False
        self._robot_xy = (0.0, 0.0)
        self._mode = WALK
        self._stamina = 1.0
        self._last_detection = -1.0e9
        self._waypoint = random_waypoint(self._bounds)
        self._walk_target = self._walk_speed

        self._cmd_pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self._mode_pub = self.create_publisher(String, '/person_mode', 10)
        self.create_subscription(
            Odometry, '/person/odom', self._on_person_odom, 10)
        self.create_subscription(Odometry, '/odom', self._on_robot_odom, 10)
        self.create_subscription(
            Bool, '/person_detected', self._on_detection, 10)

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'person_controller: walk {self._walk_speed} m/s, '
            f'run {self._run_speed} m/s for up to {self._max_run_secs} s')

    # ------------------------------------------------------------------

    def _on_person_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._person.x = msg.pose.pose.position.x
        self._person.y = msg.pose.pose.position.y
        self._person.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._have_person_pose = True

    def _on_robot_odom(self, msg: Odometry) -> None:
        self._robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_detection(self, msg: Bool) -> None:
        if msg.data:
            self._last_detection = self._now()

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if not self._have_person_pose:
            return  # plugin not up yet
        p = self._person
        robot_dist = math.hypot(p.x - self._robot_xy[0], p.y - self._robot_xy[1])
        alarmed = ((self._now() - self._last_detection) < self._calm_down
                   and robot_dist < self._notice_radius)

        prev = self._mode
        self._stamina = update_stamina(
            self._stamina, self._mode, self._dt,
            self._max_run_secs, self._recovery_secs)
        self._mode = next_mode(
            self._mode, alarmed, self._stamina, self._resume_stamina)
        if self._mode != prev:
            self.get_logger().info(
                f'{prev} -> {self._mode} (stamina {self._stamina:.2f}, '
                f'robot {robot_dist:.1f} m)')
            if self._mode == WALK:
                self._waypoint = random_waypoint(self._bounds)

        if self._mode == WALK:
            if math.hypot(p.x - self._waypoint[0], p.y - self._waypoint[1]) < 0.5:
                self._waypoint = random_waypoint(self._bounds)
                self._walk_target = self._walk_speed + random.uniform(
                    -self._walk_jitter, self._walk_jitter)
            desired = math.atan2(self._waypoint[1] - p.y, self._waypoint[0] - p.x)
            target, max_yaw = self._walk_target, self._walk_yaw
        else:
            desired = flee_heading((p.x, p.y), self._robot_xy, self._bounds)
            if self._mode == RUN:
                target, max_yaw = self._run_speed, self._run_yaw
            else:
                target, max_yaw = self._exhausted_speed, self._walk_yaw

        p.speed, yaw_rate = steer(
            p.yaw, desired, target, p.speed, self._dt,
            max_yaw, self._accel, self._decel)

        cmd = Twist()
        cmd.linear.x = p.speed
        cmd.angular.z = yaw_rate
        self._cmd_pub.publish(cmd)
        self._mode_pub.publish(String(data=self._mode))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Initialize and spin the PersonControllerNode."""
    rclpy.init(args=args)
    node = PersonControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
