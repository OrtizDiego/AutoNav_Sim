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

"""Detect and follow a red object using OpenCV HSV thresholding and ROS 2."""

import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class BallChaser(Node):
    """Node that detects a red object and publishes velocity commands to follow it."""

    def __init__(self):
        """Initialize the node, declare parameters, and set up pub/sub."""
        super().__init__('ball_chaser')

        # --- Parameters (tunable at runtime via ros2 param set) ---
        self.declare_parameter('hsv_red_lower1', [0, 100, 100])
        self.declare_parameter('hsv_red_upper1', [10, 255, 255])
        self.declare_parameter('hsv_red_lower2', [160, 100, 100])
        self.declare_parameter('hsv_red_upper2', [180, 255, 255])
        self.declare_parameter('angular_gain', 100.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('min_contour_area', 300.0)
        self.declare_parameter('search_angular_speed', 0.3)
        self.declare_parameter('search_timeout_sec', 3.0)

        self._load_params()

        self._subscription = self.create_subscription(
            Image, '/camera/image_raw', self._camera_callback, 10)
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._bridge = CvBridge()

        # State for search-rotate behavior
        self._last_seen_time: float = 0.0

        # Display in a background thread so cv2.imshow never blocks ROS spin
        self._display_frame = None
        self._display_lock = threading.Lock()
        self._display_thread = threading.Thread(
            target=self._display_loop, daemon=True)
        self._display_thread.start()

    def _load_params(self):
        """Cache parameter values as instance attributes."""
        self._lower1 = np.array(
            self.get_parameter('hsv_red_lower1').value, dtype=np.uint8)
        self._upper1 = np.array(
            self.get_parameter('hsv_red_upper1').value, dtype=np.uint8)
        self._lower2 = np.array(
            self.get_parameter('hsv_red_lower2').value, dtype=np.uint8)
        self._upper2 = np.array(
            self.get_parameter('hsv_red_upper2').value, dtype=np.uint8)
        self._angular_gain = float(
            self.get_parameter('angular_gain').value)
        self._linear_speed = float(
            self.get_parameter('linear_speed').value)
        self._min_area = float(
            self.get_parameter('min_contour_area').value)
        self._search_speed = float(
            self.get_parameter('search_angular_speed').value)
        self._search_timeout = float(
            self.get_parameter('search_timeout_sec').value)

    def _camera_callback(self, msg: Image):
        """Process a camera frame: detect target and publish velocity."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            mask = (cv2.inRange(hsv, self._lower1, self._upper1) +
                    cv2.inRange(hsv, self._lower2, self._upper2))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            cmd = Twist()
            now = time.monotonic()

            valid = [c for c in contours
                     if cv2.contourArea(c) >= self._min_area]
            if valid:
                c = max(valid, key=cv2.contourArea)
                m = cv2.moments(c)
                if m['m00'] > 0:
                    cx = int(m['m10'] / m['m00'])
                    _, width, _ = cv_image.shape
                    error_x = cx - (width / 2)
                    cmd.angular.z = -float(error_x) / self._angular_gain
                    cmd.linear.x = self._linear_speed
                    self._last_seen_time = now
                    self.get_logger().debug(f'Tracking — error_x={error_x:.1f}')
                    # Draw tracking indicator on display frame
                    cy = int(m['m01'] / m['m00'])
                    cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), 3)
            else:
                # Search: rotate slowly until timeout, then stop
                elapsed = now - self._last_seen_time
                if self._last_seen_time > 0 and elapsed < self._search_timeout:
                    cmd.angular.z = self._search_speed
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

            self._publisher.publish(cmd)

            with self._display_lock:
                self._display_frame = cv_image

        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')

    def _display_loop(self):
        """Render latest frame at ~30 fps in a dedicated thread."""
        while True:
            with self._display_lock:
                frame = self._display_frame
            if frame is not None:
                cv2.imshow('Ball Chaser View', frame)
                cv2.waitKey(1)
            time.sleep(0.033)


def main(args=None):
    """Initialize and spin the BallChaser node."""
    rclpy.init(args=args)
    node = BallChaser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
