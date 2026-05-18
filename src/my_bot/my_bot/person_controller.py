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

"""Two-mode pedestrian controller for the Gazebo actor in person.world.

States
------
WALK : low speed (~0.8 m/s), patrol-like wandering between waypoints.
RUN  : high speed (~2.0 m/s), evasive — flees away from the robot's heading.

Transition: ``/person_detected`` (std_msgs/Bool) from object_detector flips
the FSM to RUN; after ``calm_down_secs`` without further detection it
returns to WALK.

Position is driven via the Gazebo ``/gazebo/set_entity_state`` service
(provided by ``libgazebo_ros_state.so`` in person.world).  Because the
actor's ``<animation>`` uses ``interpolate_x=true``, faster commanded
displacement automatically produces a faster stride — so the visual
walking-vs-running distinction is emergent rather than a separate
animation file (which gazebo-common does not ship by default).

The position-update math lives in module-level pure functions so unit
tests can exercise it without a running ROS graph.
"""

import math
import random
from dataclasses import dataclass
from typing import Optional, Tuple

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# Pure helpers (importable by tests without a ROS runtime)
# ---------------------------------------------------------------------------

WALK = 'WALK'
RUN = 'RUN'


@dataclass
class PersonState:
    """Mutable pose + heading of the simulated pedestrian."""

    x: float = 3.0
    y: float = 0.0
    yaw: float = 0.0


def step_walk(state: PersonState,
              waypoint: Tuple[float, float],
              speed: float,
              dt: float) -> PersonState:
    """Advance ``state`` toward ``waypoint`` by ``speed * dt`` metres.

    Heading is set so the pedestrian faces the direction of motion.  When
    the waypoint is within one step, the actor snaps to it exactly to
    avoid jitter.
    """
    dx = waypoint[0] - state.x
    dy = waypoint[1] - state.y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return state
    step = speed * dt
    if step >= dist:
        return PersonState(waypoint[0], waypoint[1], math.atan2(dy, dx))
    return PersonState(
        state.x + step * dx / dist,
        state.y + step * dy / dist,
        math.atan2(dy, dx),
    )


def step_flee(state: PersonState,
              robot_xy: Tuple[float, float],
              speed: float,
              dt: float,
              bounds: float = 8.0) -> PersonState:
    """Advance ``state`` directly away from ``robot_xy``.

    Stays inside an axis-aligned square of half-extent ``bounds`` — when
    the pedestrian would otherwise run out of the world it slides along
    the wall instead.
    """
    dx = state.x - robot_xy[0]
    dy = state.y - robot_xy[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        # Robot is on top of us — pick a random direction
        theta = random.uniform(-math.pi, math.pi)
        dx, dy = math.cos(theta), math.sin(theta)
        norm = 1.0
    ux, uy = dx / norm, dy / norm
    nx = state.x + ux * speed * dt
    ny = state.y + uy * speed * dt
    # Clamp inside bounds while preserving heading
    nx = max(-bounds, min(bounds, nx))
    ny = max(-bounds, min(bounds, ny))
    return PersonState(nx, ny, math.atan2(uy, ux))


def should_run(detected: bool,
               current_state: str,
               time_since_last_detection: float,
               calm_down_secs: float) -> str:
    """Pure FSM transition function.

    ``detected`` is the latest ``/person_detected`` flag value.  Once
    triggered, RUN persists until ``calm_down_secs`` of no fresh detection.
    """
    if detected:
        return RUN
    if current_state == RUN and time_since_last_detection < calm_down_secs:
        return RUN
    return WALK


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PersonControllerNode(Node):
    """Drives the ``person_intruder`` actor between WALK and RUN modes."""

    def __init__(self):
        super().__init__('person_controller')

        # --- parameters --------------------------------------------------
        self.declare_parameter('actor_name', 'person_intruder')
        self.declare_parameter('walk_speed', 0.8)
        self.declare_parameter('run_speed', 2.0)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('calm_down_secs', 4.0)
        self.declare_parameter('bounds', 8.0)
        self.declare_parameter(
            'waypoints',
            [3.0, 0.0,
             3.0, 3.0,
             -1.0, 3.0,
             -1.0, 0.0])

        self._actor = str(self.get_parameter('actor_name').value)
        self._walk_speed = float(self.get_parameter('walk_speed').value)
        self._run_speed = float(self.get_parameter('run_speed').value)
        rate = float(self.get_parameter('update_rate_hz').value)
        self._dt = 1.0 / max(rate, 1.0)
        self._calm_down = float(self.get_parameter('calm_down_secs').value)
        self._bounds = float(self.get_parameter('bounds').value)

        flat = list(self.get_parameter('waypoints').value)
        self._waypoints = [(flat[i], flat[i + 1])
                           for i in range(0, len(flat) - 1, 2)]
        self._wp_idx = 0

        # --- state -------------------------------------------------------
        self._state = PersonState(self._waypoints[0][0],
                                  self._waypoints[0][1], 0.0)
        self._mode = WALK
        self._last_detection_time = -1.0e9
        self._robot_xy: Tuple[float, float] = (0.0, 0.0)

        # --- pubs / subs -------------------------------------------------
        self._mode_pub = self.create_publisher(String, '/person_mode', 10)
        self.create_subscription(
            Bool, '/person_detected', self._on_detection, 10)
        self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        # --- gazebo service client --------------------------------------
        self._set_state_client = None
        self._set_state_req = None
        self._init_gazebo_client()

        self._timer = self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'person_controller up: walk={self._walk_speed} m/s, '
            f'run={self._run_speed} m/s, actor={self._actor}')

    # ------------------------------------------------------------------

    def _init_gazebo_client(self) -> None:
        """Best-effort import of gazebo_msgs.  Tests don't need it."""
        try:
            from gazebo_msgs.srv import SetEntityState
            self._set_state_client = self.create_client(
                SetEntityState, '/gazebo/set_entity_state')
            self._set_state_req_cls = SetEntityState.Request
        except ImportError:
            self.get_logger().warn(
                'gazebo_msgs not available — pose updates disabled '
                '(controller will still publish /person_mode)')

    # ------------------------------------------------------------------

    def _on_detection(self, msg: Bool) -> None:
        if msg.data:
            self._last_detection_time = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_xy = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y)

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        now = self._now()
        dt_since = now - self._last_detection_time
        detected_recent = dt_since < self._dt * 1.5  # current-tick detection
        self._mode = should_run(
            detected_recent, self._mode, dt_since, self._calm_down)
        self._mode_pub.publish(String(data=self._mode))

        if self._mode == RUN:
            self._state = step_flee(
                self._state, self._robot_xy, self._run_speed,
                self._dt, self._bounds)
        else:
            target = self._waypoints[self._wp_idx]
            self._state = step_walk(
                self._state, target, self._walk_speed, self._dt)
            if math.hypot(self._state.x - target[0],
                          self._state.y - target[1]) < 0.15:
                self._wp_idx = (self._wp_idx + 1) % len(self._waypoints)

        self._publish_pose()

    def _publish_pose(self) -> None:
        if self._set_state_client is None:
            return
        if not self._set_state_client.service_is_ready():
            return
        req = self._set_state_req_cls()
        req.state.name = self._actor
        req.state.pose.position.x = float(self._state.x)
        req.state.pose.position.y = float(self._state.y)
        req.state.pose.position.z = 0.0
        # Convert yaw → quaternion (Z-axis rotation)
        half = self._state.yaw / 2.0
        req.state.pose.orientation.z = math.sin(half)
        req.state.pose.orientation.w = math.cos(half)
        req.state.reference_frame = 'world'
        self._set_state_client.call_async(req)

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
