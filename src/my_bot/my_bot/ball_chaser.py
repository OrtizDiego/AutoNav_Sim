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

"""Chase the red ball using sensor_fusion's range and bearing.

sensor_fusion (mode=hsv) finds the ball in the camera and ranges it with the
lidar. This node keeps the robot ``desired_distance`` metres from the ball's
surface with the shared stand-off controller, and when the ball is lost it
turns toward where it was last seen for ``search_secs``.

Subscribes: /target_bearing, /target_range (std_msgs/Float32), /scan
Publishes:  /cmd_vel
"""

import math
import threading
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from my_bot.follow_control import compute_command, front_clearance, search_command


class BallChaser(Node):
    """Stand-off follower for the fused ball target."""

    def __init__(self):
        """Initialize the node, declare parameters, and set up pub/sub."""
        super().__init__('ball_chaser')

        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('deadband', 0.1)
        self.declare_parameter('k_lin', 0.6)
        self.declare_parameter('k_yaw', 1.8)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_back_speed', 0.15)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('safety_distance', 0.35)
        self.declare_parameter('target_timeout', 0.5)
        self.declare_parameter('search_secs', 20.0)
        self.declare_parameter('search_angular_speed', 0.6)

        gp = self.get_parameter
        self._desired = float(gp('desired_distance').value)
        self._deadband = float(gp('deadband').value)
        self._k_lin = float(gp('k_lin').value)
        self._k_yaw = float(gp('k_yaw').value)
        self._max_lin = float(gp('max_linear_speed').value)
        self._max_back = float(gp('max_back_speed').value)
        self._max_yaw = float(gp('max_angular_speed').value)
        self._safety = float(gp('safety_distance').value)
        self._timeout = float(gp('target_timeout').value)
        self._search_secs = float(gp('search_secs').value)
        self._search_speed = float(gp('search_angular_speed').value)

        self._lock = threading.Lock()
        self._bearing = 0.0
        self._range: Optional[float] = None
        self._last_seen = -1.0e9
        self._front_clear = float('inf')

        self.create_subscription(Float32, '/target_bearing', self._on_bearing, 10)
        self.create_subscription(Float32, '/target_range', self._on_range, 10)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._tick)

    def _on_bearing(self, msg: Float32):
        if math.isfinite(msg.data):
            with self._lock:
                self._bearing = float(msg.data)
                self._last_seen = self._now()

    def _on_range(self, msg: Float32):
        with self._lock:
            self._range = float(msg.data) if msg.data > 0.0 else None

    def _on_scan(self, msg: LaserScan):
        clear = front_clearance(msg.ranges, msg.angle_min, msg.angle_increment)
        with self._lock:
            self._front_clear = clear

    def _tick(self):
        with self._lock:
            lost_for = self._now() - self._last_seen
            bearing, rng, front = self._bearing, self._range, self._front_clear
        cmd = Twist()
        if lost_for <= self._timeout:
            cmd.linear.x, cmd.angular.z = compute_command(
                bearing, rng, self._desired, self._k_lin, self._k_yaw,
                self._max_lin, self._max_back, self._max_yaw,
                self._deadband, front, self._safety)
        elif lost_for <= self._timeout + self._search_secs:
            cmd.angular.z = search_command(bearing, self._search_speed)
        self._cmd_pub.publish(cmd)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Initialize and spin the BallChaser node."""
    rclpy.init(args=args)
    node = BallChaser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
