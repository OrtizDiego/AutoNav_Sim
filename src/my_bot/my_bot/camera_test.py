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

"""Debug camera feed node — displays live image with center crosshair overlay."""

import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraSubscriber(Node):
    """Node that subscribes to camera images and displays them in a background thread."""

    def __init__(self):
        """Initialize the node and subscriber."""
        super().__init__('camera_subscriber')

        self._subscription = self.create_subscription(
            Image, '/camera/image_raw', self._callback, 10)
        self._bridge = CvBridge()

        # Display in dedicated thread so imshow never blocks ROS callbacks
        self._display_frame = None
        self._display_lock = threading.Lock()
        self._display_thread = threading.Thread(
            target=self._display_loop, daemon=True)
        self._display_thread.start()

    def _callback(self, msg: Image):
        """Convert and annotate image; hand it to the display thread."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape
            cx, cy = w // 2, h // 2
            cv2.circle(cv_image, (cx, cy), 50, (0, 255, 0), 3)
            cv2.putText(cv_image, 'I CAN SEE!', (cx - 60, cy - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            with self._display_lock:
                self._display_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def _display_loop(self):
        """Render latest frame at ~30 fps without blocking ROS spin."""
        while True:
            with self._display_lock:
                frame = self._display_frame
            if frame is not None:
                cv2.imshow('Robot Camera View', frame)
                cv2.waitKey(1)
            time.sleep(0.033)


def main(args=None):
    """Initialize and spin the CameraSubscriber node."""
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
