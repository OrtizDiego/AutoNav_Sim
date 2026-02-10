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

"""Script to detect and follow a red ball using OpenCV and ROS 2."""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class BallChaser(Node):
    """Node that detects a red ball and publishes velocity commands to follow it."""

    def __init__(self):
        """Initialize the node, subscriber and publisher."""
        super().__init__('ball_chaser')

        # Subscriber (The Eyes)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

        # Publisher (The Feet)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

    def camera_callback(self, msg):
        """Process camera images and detect the red ball."""
        try:
            # 1. Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. Convert BGR to HSV (Better for color detection)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 3. Define the Range of "Red" Color
            # Red wraps around 180 in HSV, so we need two ranges
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2  # Combine

            # 4. Find Contours (Blobs of red)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Command to send to the robot
            cmd = Twist()

            if len(contours) > 0:
                # Find the biggest red blob
                c = max(contours, key=cv2.contourArea)
                m_moments = cv2.moments(c)

                if m_moments['m00'] > 0:
                    # Find the Center (cx, cy) of the blob
                    cx = int(m_moments['m10'] / m_moments['m00'])
                    # cy = int(m_moments['m01'] / m_moments['m00'])

                    # Draw a circle on the image for debugging
                    # cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), 3)

                    # --- CONTROL LOGIC ---
                    _, width, _ = cv_image.shape
                    error_x = cx - (width / 2)  # How far from center?

                    # Turn towards the object (Proportional Controller)
                    # Divide by 100 to scale it down
                    cmd.angular.z = -float(error_x) / 100.0

                    # If we see it, drive forward slowly
                    cmd.linear.x = 0.2

                    print(f"Tracking! Error: {error_x}")
            else:
                # If no red detected, stop (or spin to search)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            # 5. Move the Robot
            self.publisher.publish(cmd)

            # 6. Show the view
            cv2.imshow("Ball Chaser View", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    """Initialize and spin the BallChaser node."""
    rclpy.init(args=args)
    node = BallChaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
