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

"""Follow the tracked person while keeping a stand-off distance.

Subscribes
----------
/person_bbox   std_msgs/Float32MultiArray  [x, y, w, h] px (person_tracker)
/scan          sensor_msgs/LaserScan

Publishes
---------
/cmd_vel       geometry_msgs/Twist

Ranging
-------
The lidar sits ~0.13 m above the ground, so it sees the person's shins and
single beams often pass between the legs. The range is therefore a low
percentile over every beam inside the bbox's angular span. It is
cross-checked against a monocular estimate from the bbox: the person's
height when the whole body is visible, otherwise the ground-contact point
of the feet. When the lidar disagrees (beams hit the background), the
monocular estimate is used.

Sign convention: image bearings are positive to the right, but ROS angles
(LaserScan, angular.z) are positive to the left, so bearings are negated
before any scan lookup or steering.

Control
-------
linear.x  = k_lin * (range - desired_distance), slowed while turning
angular.z = k_yaw * angle_to_person
Forward motion stops if anything is closer than ``safety_distance`` ahead.
When the track is lost the robot turns toward where the person was last
seen for ``search_secs``, then stops.
"""

import math
import threading
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


# ---------------------------------------------------------------------------
# Pure helpers (exercised by unit tests)
# ---------------------------------------------------------------------------

def pixel_to_angle(px: float, cx: float, fx: float) -> float:
    """Robot-frame angle (rad, positive = left) of an image column."""
    return -math.atan2(px - cx, fx)


def scan_window_range(ranges: List[float], angle_min: float,
                      angle_increment: float, left_angle: float,
                      right_angle: float, range_min: float, range_max: float,
                      percentile: float = 0.2) -> Optional[float]:
    """Low-percentile range of valid beams between two angles.

    ``left_angle`` >= ``right_angle`` (ROS angles). Returns None if no beam
    in the window has a valid return.
    """
    n = len(ranges)
    if n == 0:
        return None
    lo = int(math.floor((right_angle - angle_min) / angle_increment))
    hi = int(math.ceil((left_angle - angle_min) / angle_increment))
    vals = []
    for i in range(lo, hi + 1):
        r = ranges[i % n]  # wrap across the +/-pi seam
        if math.isfinite(r) and range_min <= r <= range_max:
            vals.append(r)
    if not vals:
        return None
    vals.sort()
    return vals[min(len(vals) - 1, int(percentile * len(vals)))]


def monocular_range(bbox: List[float], img_h: int, fy: float, cy: float,
                    person_height: float, camera_height: float,
                    edge_margin: float = 3.0) -> Optional[float]:
    """Estimate distance to the person from the bbox alone.

    Uses the apparent height when head and feet are both in frame,
    otherwise the feet's ground contact point (needs the feet visible).
    """
    _, y, _, h = bbox
    top_cut = y <= edge_margin
    bottom_cut = y + h >= img_h - edge_margin
    if not top_cut and not bottom_cut and h > 1.0:
        return fy * person_height / h
    if not bottom_cut:
        below = (y + h) - cy  # pixels below the horizon
        if below > 1.0:
            return fy * camera_height / below
    return None


def fuse_range(lidar: Optional[float], mono: Optional[float],
               tolerance: float = 0.4) -> Optional[float]:
    """Prefer lidar unless it disagrees with the monocular estimate."""
    if lidar is None:
        return mono
    if mono is None:
        return lidar
    if abs(lidar - mono) <= tolerance * mono:
        return lidar
    return mono


def compute_command(angle: float, range_m: Optional[float],
                    desired_distance: float, k_lin: float, k_yaw: float,
                    max_lin: float, max_back: float, max_yaw: float,
                    deadband: float = 0.15,
                    front_clear: float = float('inf'),
                    safety_distance: float = 0.6) -> Tuple[float, float]:
    """Return (linear.x, angular.z) for one control step.

    ``angle`` is the robot-frame angle to the person (positive = left).
    Without a range the robot only turns toward the person.
    """
    w = max(-max_yaw, min(max_yaw, k_yaw * angle))
    if range_m is None or not math.isfinite(range_m):
        return 0.0, w
    error = range_m - desired_distance
    v = 0.0 if abs(error) < deadband else k_lin * error
    v = max(-max_back, min(max_lin, v))
    if v > 0.0:
        v *= max(0.0, math.cos(angle))  # turn first, then drive
        if front_clear < safety_distance:
            v = 0.0
    return float(v), float(w)


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PersonFollowerNode(Node):
    """Keeps the robot ``desired_distance`` behind the tracked person."""

    def __init__(self):
        super().__init__('person_follower')

        # 2.5 m keeps the person's torso in view: the camera is ~0.10 m off
        # the ground with a +/-24 deg vertical FOV, so at 1.5 m it sees
        # only up to the hips and YOLO becomes unreliable.
        self.declare_parameter('desired_distance', 2.5)
        self.declare_parameter('deadband', 0.15)
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_yaw', 1.5)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_back_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('safety_distance', 0.6)
        self.declare_parameter('track_timeout', 0.5)
        self.declare_parameter('search_secs', 4.0)
        self.declare_parameter('search_angular_speed', 0.6)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 12.0)
        # Camera model (camera.xacro: 640x480, horizontal FOV 1.089 rad)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('horizontal_fov', 1.089)
        self.declare_parameter('camera_height', 0.103)
        self.declare_parameter('person_height', 1.72)

        gp = self.get_parameter
        self._desired = float(gp('desired_distance').value)
        self._deadband = float(gp('deadband').value)
        self._k_lin = float(gp('k_lin').value)
        self._k_yaw = float(gp('k_yaw').value)
        self._max_lin = float(gp('max_linear_speed').value)
        self._max_back = float(gp('max_back_speed').value)
        self._max_yaw = float(gp('max_angular_speed').value)
        self._safety = float(gp('safety_distance').value)
        self._timeout = float(gp('track_timeout').value)
        self._search_secs = float(gp('search_secs').value)
        self._search_speed = float(gp('search_angular_speed').value)
        self._range_min = float(gp('range_min').value)
        self._range_max = float(gp('range_max').value)
        width = int(gp('image_width').value)
        self._img_h = int(gp('image_height').value)
        self._cx = width / 2.0
        self._cy = self._img_h / 2.0
        self._fx = (width / 2.0) / math.tan(float(gp('horizontal_fov').value) / 2.0)
        self._cam_h = float(gp('camera_height').value)
        self._person_h = float(gp('person_height').value)

        self._scan_lock = threading.Lock()
        self._scan: Optional[LaserScan] = None
        self._bbox: Optional[List[float]] = None
        self._last_track = -1.0e9
        self._last_angle = 0.0

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Float32MultiArray, '/person_bbox', self._on_bbox, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self._tick)

    # ------------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._scan = msg

    def _on_bbox(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            self._bbox = list(msg.data[:4])
            self._last_track = self._now()

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        cmd = Twist()
        lost_for = self._now() - self._last_track
        if self._bbox is None or lost_for > self._timeout:
            if self._bbox is not None and lost_for < self._timeout + self._search_secs:
                # Look where the person was last seen.
                cmd.angular.z = math.copysign(self._search_speed, self._last_angle)
            self._cmd_pub.publish(cmd)
            return

        x, _, w, _ = self._bbox
        angle = pixel_to_angle(x + w / 2.0, self._cx, self._fx)
        self._last_angle = angle

        with self._scan_lock:
            scan = self._scan
        lidar = None
        front_clear = float('inf')
        if scan is not None:
            margin = math.radians(2.0)
            lidar = scan_window_range(
                scan.ranges, scan.angle_min, scan.angle_increment,
                pixel_to_angle(x, self._cx, self._fx) + margin,
                pixel_to_angle(x + w, self._cx, self._fx) - margin,
                self._range_min, self._range_max)
            front = scan_window_range(
                scan.ranges, scan.angle_min, scan.angle_increment,
                math.radians(25.0), math.radians(-25.0),
                self._range_min, self._range_max, percentile=0.0)
            if front is not None:
                front_clear = front

        mono = monocular_range(
            self._bbox, self._img_h, self._fx, self._cy,
            self._person_h, self._cam_h)
        rng = fuse_range(lidar, mono)

        v, w_cmd = compute_command(
            angle, rng, self._desired, self._k_lin, self._k_yaw,
            self._max_lin, self._max_back, self._max_yaw,
            self._deadband, front_clear, self._safety)
        cmd.linear.x = v
        cmd.angular.z = w_cmd
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
