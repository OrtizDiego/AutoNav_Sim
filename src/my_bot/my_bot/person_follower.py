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

"""Follow the tracked person at a safe stand-off distance.

Subscribes
----------
/person_bbox   std_msgs/Float32MultiArray  [x, y, w, h] in pixels (from person_tracker)
/scan          sensor_msgs/LaserScan       used to read the metric range at the
                                           tracked person's bearing

Publishes
---------
/cmd_vel       geometry_msgs/Twist         differential-drive command

Control law
-----------
* Bearing: ``angular.z = -k_yaw * bearing_rad`` clipped to ``max_angular_speed``.
* Distance: ``linear.x = k_lin * (range - desired_distance)`` clipped to
  ``[-max_back_speed, max_linear_speed]``.  When ``range`` is within
  ``deadband`` of the goal the linear command is zero.
* Safety: if no track for ``track_timeout`` seconds, stop.
"""

import math
import threading
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from my_bot.sensor_fusion import (
    bearing_to_scan_index, compute_bearing, validate_range)


# ---------------------------------------------------------------------------
# Pure control law — exercised by unit tests
# ---------------------------------------------------------------------------

def compute_command(bearing_rad: float,
                    range_m: float,
                    desired_distance: float,
                    k_lin: float,
                    k_yaw: float,
                    max_lin: float,
                    max_back: float,
                    max_yaw: float,
                    deadband: float = 0.1) -> tuple:
    """Return (linear.x, angular.z) for a single control step.

    ``range_m < 0`` indicates "no valid range" and produces a stop command.
    """
    if not math.isfinite(range_m) or range_m < 0.0:
        return 0.0, 0.0

    error = range_m - desired_distance
    if abs(error) < deadband:
        v = 0.0
    else:
        v = k_lin * error
        v = max(-max_back, min(max_lin, v))

    w = -k_yaw * bearing_rad
    w = max(-max_yaw, min(max_yaw, w))
    return float(v), float(w)


def bbox_centroid_x(bbox: list) -> Optional[float]:
    """Return the x-coordinate of the bbox centre, or None if bbox is empty."""
    if not bbox or len(bbox) < 4:
        return None
    return float(bbox[0]) + float(bbox[2]) / 2.0


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PersonFollowerNode(Node):
    """Proportional follower that keeps the robot at ``desired_distance``."""

    # Same intrinsics as sensor_fusion (camera.xacro: FOV 1.089 rad, w=640)
    _CAMERA_CX = 320.0
    _CAMERA_FX = 320.0 / math.tan(1.089 / 2.0)

    def __init__(self):
        super().__init__('person_follower')

        self.declare_parameter('desired_distance', 1.5)
        self.declare_parameter('deadband', 0.15)
        self.declare_parameter('k_lin', 0.6)
        self.declare_parameter('k_yaw', 1.2)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_back_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('track_timeout', 1.5)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 12.0)

        self._desired = float(self.get_parameter('desired_distance').value)
        self._deadband = float(self.get_parameter('deadband').value)
        self._k_lin = float(self.get_parameter('k_lin').value)
        self._k_yaw = float(self.get_parameter('k_yaw').value)
        self._max_lin = float(self.get_parameter('max_linear_speed').value)
        self._max_back = float(self.get_parameter('max_back_speed').value)
        self._max_yaw = float(self.get_parameter('max_angular_speed').value)
        self._timeout = float(self.get_parameter('track_timeout').value)
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)

        self._scan_lock = threading.Lock()
        self._latest_scan: Optional[LaserScan] = None
        self._last_track_time = -1.0e9
        self._latest_centroid_x: Optional[float] = None

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Float32MultiArray, '/person_bbox', self._on_bbox, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._timer = self.create_timer(0.1, self._tick)

    # ------------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = msg

    def _on_bbox(self, msg: Float32MultiArray) -> None:
        cx = bbox_centroid_x(list(msg.data))
        if cx is None:
            return
        self._latest_centroid_x = cx
        self._last_track_time = self._now()

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        cmd = Twist()
        now = self._now()
        if (self._latest_centroid_x is None
                or (now - self._last_track_time) > self._timeout):
            self._cmd_pub.publish(cmd)  # stop
            return

        with self._scan_lock:
            scan = self._latest_scan
        if scan is None:
            self._cmd_pub.publish(cmd)
            return

        bearing = compute_bearing(
            self._latest_centroid_x, self._CAMERA_CX, self._CAMERA_FX)
        idx = bearing_to_scan_index(
            bearing, scan.angle_min, scan.angle_increment, len(scan.ranges))
        r = scan.ranges[idx]
        if not validate_range(r, self._range_min, self._range_max):
            self._cmd_pub.publish(cmd)
            return

        v, w = compute_command(
            bearing, float(r), self._desired,
            self._k_lin, self._k_yaw,
            self._max_lin, self._max_back, self._max_yaw,
            self._deadband)
        cmd.linear.x = v
        cmd.angular.z = w
        self._cmd_pub.publish(cmd)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Initialize and spin the PersonFollowerNode."""
    rclpy.init(args=args)
    node = PersonFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
