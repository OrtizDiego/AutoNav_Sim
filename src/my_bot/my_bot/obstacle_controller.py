#!/usr/bin/env python3
# Copyright 2024 AutoNav Team
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

"""Random-walk controller for dynamic world obstacles.

Drives each obstacle via its own cmd_vel topic in a timed random walk.
Obstacles reverse direction when approaching the boundary_radius.
"""

import random
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class ObstacleControllerNode(Node):
    def __init__(self):
        super().__init__('obstacle_controller')
        self.declare_parameter('obstacle_topics',
                               ['/obstacle1_cmd_vel', '/obstacle2_cmd_vel'])
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('boundary_radius', 4.0)

        topics = list(self.get_parameter('obstacle_topics').value)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._boundary = float(self.get_parameter('boundary_radius').value)

        self._pubs = [self.create_publisher(Twist, t, 10) for t in topics]
        n = len(topics)
        self._cmds = [Twist() for _ in range(n)]
        self._cmd_end_times = [0.0] * n

        self._timer = self.create_timer(0.1, self._step)

    def _step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for i, pub in enumerate(self._pubs):
            if now >= self._cmd_end_times[i]:
                self._cmds[i] = self._random_twist()
                self._cmd_end_times[i] = now + random.uniform(2.0, 5.0)
            pub.publish(self._cmds[i])

    def _random_twist(self) -> Twist:
        t = Twist()
        t.linear.x = random.choice([-1, 0, 1]) * self._linear_speed
        t.angular.z = random.uniform(-0.8, 0.8)
        return t


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
