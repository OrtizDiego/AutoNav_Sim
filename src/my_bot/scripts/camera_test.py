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

"""Script to test the robot's camera feed by displaying it with a circle overlay."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraSubscriber(Node):
    """Node that subscribes to camera images and displays them."""

    def __init__(self):
        """Initialize the node and subscriber."""
        super().__init__('camera_subscriber')

        # Create the Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)

        # The Bridge object converts ROS messages to OpenCV images
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        """Display the received image with a visual overlay."""
        try:
            # 1. Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. Process the Image (Draw a Green Circle in the center)
            height, width, _ = cv_image.shape
            center_x = int(width / 2)
            center_y = int(height / 2)

            # Draw: (Image, Center, Radius, Color(BGR), Thickness)
            cv2.circle(cv_image, (center_x, center_y), 50, (0, 255, 0), 3)
            cv2.putText(cv_image, "I CAN SEE!", (center_x - 60, center_y - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 3. Display the Image
            cv2.imshow("Robot Camera View", cv_image)
            cv2.waitKey(1)  # Refresh the window

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


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
