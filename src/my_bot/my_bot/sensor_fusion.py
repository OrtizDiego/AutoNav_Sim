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

"""Fuse camera pixel coordinates with LiDAR scan to estimate target position."""

import math
import threading

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32


# ---------------------------------------------------------------------------
# Pure functions — module-level so unit tests can import them without ROS
# ---------------------------------------------------------------------------

def compute_bearing(pixel_x: float, cx: float, fx: float) -> float:
    """Return bearing (rad) of a pixel column relative to camera optical axis.

    Positive bearing → target is to the robot's left (ROS convention).

    Args:
        pixel_x: Horizontal pixel coordinate of the target centroid.
        cx:      Image optical centre (pixels).  Typically image_width / 2.
        fx:      Focal length in pixels.  fx = (width/2) / tan(FOV_h/2).
    """
    return math.atan2(pixel_x - cx, fx)


def bearing_to_scan_index(bearing: float,
                           angle_min: float,
                           angle_increment: float,
                           n_samples: int) -> int:
    """Map a bearing angle to the nearest LaserScan array index.

    Args:
        bearing:          Bearing from ``compute_bearing`` (rad).
        angle_min:        scan.angle_min from the LaserScan message.
        angle_increment:  scan.angle_increment from the LaserScan message.
        n_samples:        Total number of range samples in the scan.
    """
    idx = round((bearing - angle_min) / angle_increment)
    return max(0, min(n_samples - 1, idx))


def validate_range(r: float, r_min: float, r_max: float) -> bool:
    """Return True if *r* is a valid, in-range lidar measurement."""
    return math.isfinite(r) and r_min <= r <= r_max


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class SensorFusionNode(Node):
    """Combines HSV-based target detection with LiDAR ranging.

    Subscribes to /camera/image_raw and /scan.  When a target is detected in
    the camera frame, projects the pixel bearing onto the lidar scan to
    obtain a metric range measurement.

    Publishes:
        /target_position  (geometry_msgs/PointStamped, frame_id=base_link)
        /target_range     (std_msgs/Float32, metres; -1.0 if no valid reading)
    """

    # Camera intrinsics derived from camera.xacro: FOV=1.089 rad, width=640
    _CAMERA_CX: float = 320.0
    _CAMERA_FX: float = 320.0 / math.tan(1.089 / 2.0)   # ≈ 534.8

    def __init__(self):
        """Declare parameters, create sub/pub."""
        super().__init__('sensor_fusion')

        # HSV detection params — same defaults as behavior_params.yaml
        self.declare_parameter('hsv_red_lower1', [0, 100, 100])
        self.declare_parameter('hsv_red_upper1', [10, 255, 255])
        self.declare_parameter('hsv_red_lower2', [160, 100, 100])
        self.declare_parameter('hsv_red_upper2', [180, 255, 255])
        self.declare_parameter('min_contour_area', 300.0)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 12.0)

        self._lower1 = np.array(
            self.get_parameter('hsv_red_lower1').value, dtype=np.uint8)
        self._upper1 = np.array(
            self.get_parameter('hsv_red_upper1').value, dtype=np.uint8)
        self._lower2 = np.array(
            self.get_parameter('hsv_red_lower2').value, dtype=np.uint8)
        self._upper2 = np.array(
            self.get_parameter('hsv_red_upper2').value, dtype=np.uint8)
        self._min_area = float(self.get_parameter('min_contour_area').value)
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)

        self._bridge = CvBridge()

        # Thread-safe cache for the latest scan
        self._latest_scan: LaserScan | None = None
        self._scan_lock = threading.Lock()

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self._cam_sub = self.create_subscription(
            Image, '/camera/image_raw', self._camera_cb, qos_profile_sensor_data)

        self._pos_pub = self.create_publisher(PointStamped, '/target_position', 10)
        self._range_pub = self.create_publisher(Float32, '/target_range', 10)

    # ------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        """Cache the most recent scan under a lock."""
        with self._scan_lock:
            self._latest_scan = msg

    def _camera_cb(self, msg: Image):
        """Detect target in frame; fuse with latest scan; publish."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        pixel_x = self._detect_target(cv_image)

        range_msg = Float32()
        if pixel_x is None:
            range_msg.data = -1.0
            self._range_pub.publish(range_msg)
            return

        bearing = compute_bearing(float(pixel_x), self._CAMERA_CX, self._CAMERA_FX)

        with self._scan_lock:
            scan = self._latest_scan

        if scan is None:
            range_msg.data = -1.0
            self._range_pub.publish(range_msg)
            return

        idx = bearing_to_scan_index(
            bearing, scan.angle_min, scan.angle_increment, len(scan.ranges))
        r = scan.ranges[idx]

        if not validate_range(r, self._range_min, self._range_max):
            range_msg.data = -1.0
            self._range_pub.publish(range_msg)
            return

        # Publish metric range
        range_msg.data = float(r)
        self._range_pub.publish(range_msg)

        # Publish Cartesian position in base_link frame
        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'base_link'
        pos_msg.point.x = r * math.cos(bearing)
        pos_msg.point.y = r * math.sin(bearing)
        pos_msg.point.z = 0.0
        self._pos_pub.publish(pos_msg)

        self.get_logger().debug(
            f'Target: bearing={math.degrees(bearing):.1f}°  range={r:.2f}m  '
            f'pos=({pos_msg.point.x:.2f}, {pos_msg.point.y:.2f})')

    def _detect_target(self, cv_image) -> int | None:
        """Return pixel_x of the largest red blob centroid, or None."""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(hsv, self._lower1, self._upper1) +
                cv2.inRange(hsv, self._lower2, self._upper2))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        valid = [c for c in contours if cv2.contourArea(c) >= self._min_area]
        if not valid:
            return None

        c = max(valid, key=cv2.contourArea)
        m = cv2.moments(c)
        if m['m00'] <= 0:
            return None
        return int(m['m10'] / m['m00'])


def main(args=None):
    """Initialize and spin the SensorFusionNode."""
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
