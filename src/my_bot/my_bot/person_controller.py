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
WALK       Wanders between random points of the museum at a casual pace.
RUN        Sprints away from the robot after being detected.
EXHAUSTED  Out of breath after a sprint: keeps moving away, but slowly,
           until stamina recovers.

Staying out of the walls
------------------------
Gazebo actors have no physics, so nothing stops one from walking through a
wall. The person therefore steers on the saved museum map (maps/my_map,
whose frame equals the world frame because the robot spawns at the origin):

* wander targets are sampled only from open floor with a clear line of
  sight from where the person stands;
* every step the heading is the one closest to the desired direction that
  still has ``lookahead`` metres of free floor ahead (wider when running);
* speed is capped so the person can always stop before the free floor
  along its *current* heading runs out, so turning never cuts a corner.

test_person.py simulates minutes of walking and fleeing on the real map and
checks the person never comes closer than ``min_clearance`` to a wall.

Realism details
---------------
* Speed changes are acceleration-limited and turning is yaw-rate limited,
  so the person never teleports or spins on the spot.
* The person slows down for sharp turns.
* Sprints drain stamina; an empty tank forces EXHAUSTED until it refills.
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
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from my_bot.clearance_map import ClearanceMap, safe_heading


WALK = 'WALK'
RUN = 'RUN'
EXHAUSTED = 'EXHAUSTED'

Point = Tuple[float, float]


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


def flee_heading(person_xy: Point, robot_xy: Point) -> float:
    """Heading (rad) pointing from the robot through the person."""
    dx = person_xy[0] - robot_xy[0]
    dy = person_xy[1] - robot_xy[1]
    if math.hypot(dx, dy) < 1e-6:
        return 0.0
    return math.atan2(dy, dx)


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


def stopping_speed(free_distance: float, decel: float,
                   margin: float = 0.1) -> float:
    """Fastest speed from which one can still stop within ``free_distance``."""
    return math.sqrt(2.0 * decel * max(0.0, free_distance - margin))


@dataclass
class PersonPose:
    """Planar pose and speed of the person."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0


@dataclass
class PersonParams:
    """Tunables of the behaviour model (same names as the ROS parameters)."""

    walk_speed: float = 0.7
    walk_speed_jitter: float = 0.1
    run_speed: float = 2.5
    exhausted_speed: float = 0.5
    accel: float = 2.0
    decel: float = 3.0
    walk_max_yaw_rate: float = 1.5
    run_max_yaw_rate: float = 1.0
    max_run_secs: float = 3.5
    recovery_secs: float = 8.0
    resume_stamina: float = 0.6
    min_clearance: float = 0.5
    waypoint_clearance: float = 1.0


class PersonBrain:
    """WALK / RUN / EXHAUSTED behaviour on a clearance map (no ROS)."""

    def __init__(self, cmap: ClearanceMap, params: Optional[PersonParams] = None,
                 rng: Optional[random.Random] = None):
        self.cmap = cmap
        self.p = params or PersonParams()
        self.rng = rng or random.Random()
        self.mode = WALK
        self.stamina = 1.0
        self.waypoint: Optional[Point] = None
        self._walk_target = self.p.walk_speed
        self._best_dist = float('inf')
        self._stuck_for = 0.0

    def pick_waypoint(self, here: Point) -> Point:
        """Random open spot, at least 2 m away, in line of sight of ``here``."""
        need = self.p.min_clearance + 0.15
        candidate = here
        for _ in range(200):
            candidate = self.cmap.random_free_point(self.p.waypoint_clearance, self.rng)
            if (math.dist(candidate, here) >= 2.0
                    and self.cmap.segment_clearance(here, candidate) >= need):
                break
        self._best_dist = float('inf')
        self._stuck_for = 0.0
        self._walk_target = self.p.walk_speed + self.rng.uniform(
            -self.p.walk_speed_jitter, self.p.walk_speed_jitter)
        return candidate

    def step(self, dt: float, pose: PersonPose, robot_xy: Point,
             alarmed: bool) -> Tuple[float, float]:
        """Advance the behaviour by ``dt``; return (speed, yaw_rate)."""
        p = self.p
        here = (pose.x, pose.y)
        prev = self.mode
        self.stamina = update_stamina(
            self.stamina, self.mode, dt, p.max_run_secs, p.recovery_secs)
        self.mode = next_mode(self.mode, alarmed, self.stamina, p.resume_stamina)
        if self.waypoint is None or (self.mode == WALK and prev != WALK):
            self.waypoint = self.pick_waypoint(here)

        if self.mode == WALK:
            dist = math.dist(here, self.waypoint)
            if dist < self._best_dist - 0.3:
                self._best_dist, self._stuck_for = dist, 0.0
            else:
                self._stuck_for += dt
            if dist < 0.6 or self._stuck_for > 8.0:
                self.waypoint = self.pick_waypoint(here)
            desired = math.atan2(self.waypoint[1] - pose.y, self.waypoint[0] - pose.x)
            target, max_yaw = self._walk_target, p.walk_max_yaw_rate
        else:
            desired = flee_heading(here, robot_xy)
            if self.mode == RUN:
                target, max_yaw = p.run_speed, p.run_max_yaw_rate
            else:
                target, max_yaw = p.exhausted_speed, p.walk_max_yaw_rate

        # Look far enough ahead to turn away at this speed (radius v / w).
        lookahead = 1.0 + 1.5 * max(target, pose.speed) / max_yaw
        heading, _ = safe_heading(
            self.cmap, pose.x, pose.y, desired, lookahead, p.min_clearance)
        speed, yaw_rate = steer(pose.yaw, heading, target, pose.speed, dt,
                                max_yaw, p.accel, p.decel)

        # Never outrun the free floor along the current heading.
        free = self.cmap.free_distance(
            pose.x, pose.y, pose.yaw, lookahead, p.min_clearance)
        speed = min(speed, stopping_speed(free, p.decel), free / dt)
        return speed, yaw_rate


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
        self.declare_parameter('min_clearance', 0.5)
        self.declare_parameter('waypoint_clearance', 1.0)
        self.declare_parameter('calm_down_secs', 4.0)
        self.declare_parameter('notice_radius', 7.0)
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('bounds', 8.0)
        self.declare_parameter('update_rate_hz', 20.0)

        gp = self.get_parameter
        params = PersonParams(**{
            name: float(gp(name).value) for name in vars(PersonParams())})
        self._calm_down = float(gp('calm_down_secs').value)
        self._notice_radius = float(gp('notice_radius').value)
        self._dt = 1.0 / max(float(gp('update_rate_hz').value), 1.0)

        map_yaml = str(gp('map_yaml').value)
        if map_yaml:
            cmap = ClearanceMap.from_yaml(map_yaml)
            self.get_logger().info(f'avoiding walls from {map_yaml}')
        else:
            bounds = float(gp('bounds').value)
            cmap = ClearanceMap.from_bounds(bounds + 2.0)
            self.get_logger().warn(
                f'no map_yaml: keeping inside +/-{bounds} m only')
        self._brain = PersonBrain(cmap, params)

        self._person = PersonPose()
        self._have_person_pose = False
        self._robot_xy = (0.0, 0.0)
        self._last_detection = -1.0e9

        self._cmd_pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self._mode_pub = self.create_publisher(String, '/person_mode', 10)
        self.create_subscription(
            Odometry, '/person/odom', self._on_person_odom, 10)
        self.create_subscription(Odometry, '/odom', self._on_robot_odom, 10)
        self.create_subscription(
            Bool, '/person_detected', self._on_detection, 10)

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'person_controller: walk {params.walk_speed} m/s, '
            f'run {params.run_speed} m/s for up to {params.max_run_secs} s')

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

        prev = self._brain.mode
        p.speed, yaw_rate = self._brain.step(self._dt, p, self._robot_xy, alarmed)
        if self._brain.mode != prev:
            self.get_logger().info(
                f'{prev} -> {self._brain.mode} (stamina '
                f'{self._brain.stamina:.2f}, robot {robot_dist:.1f} m)')

        cmd = Twist()
        cmd.linear.x = p.speed
        cmd.angular.z = yaw_rate
        self._cmd_pub.publish(cmd)
        self._mode_pub.publish(String(data=self._brain.mode))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Initialize and spin the PersonControllerNode."""
    rclpy.init(args=args)
    node = PersonControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Ctrl+C: rclpy has already shut the context down
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
