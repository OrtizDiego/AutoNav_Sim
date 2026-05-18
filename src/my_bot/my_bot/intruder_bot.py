#!/usr/bin/env python3

# Copyright 2026 root
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

"""Autonomous intruder node — drives the target sphere with a random walk."""

import random

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class IntruderBotNode(Node):
    """Publish random velocity commands to /target_cmd_vel at 10 Hz.

    The target sphere in the Gazebo world files subscribes to /target_cmd_vel
    via the libgazebo_ros_planar_move plugin, so no extra wiring is needed.
    """

    def __init__(self):
        """Initialize node and declare ROS parameters."""
        super().__init__('intruder_bot')

        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('min_duration', 1.5)
        self.declare_parameter('max_duration', 4.0)

        self._linear_speed = float(
            self.get_parameter('linear_speed').value)
        self._angular_speed = float(
            self.get_parameter('angular_speed').value)
        self._min_dur = float(self.get_parameter('min_duration').value)
        self._max_dur = float(self.get_parameter('max_duration').value)

        self._pub = self.create_publisher(Twist, '/target_cmd_vel', 10)
        self._timer = self.create_timer(0.1, self._step)

        self._cmd = Twist()
        self._cmd_end: float = 0.0

    def _step(self):
        """Publish current command; pick a new one when it expires."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now >= self._cmd_end:
            self._cmd = self._random_twist()
            self._cmd_end = now + random.uniform(self._min_dur, self._max_dur)
        self._pub.publish(self._cmd)

    def _random_twist(self) -> Twist:
        """Return a random forward/backward + angular velocity command."""
        t = Twist()
        t.linear.x = random.choice([-1, 0, 1]) * self._linear_speed
        t.angular.z = random.uniform(-self._angular_speed, self._angular_speed)
        return t


def main(args=None):
    """Initialize and spin the IntruderBotNode."""
    rclpy.init(args=args)
    node = IntruderBotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
