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

"""Script to patrol waypoints and chase intruders using Nav2 and OpenCV."""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# --- CONFIGURATION ---
WAYPOINTS = [
    [1.5, 0.0],  # Point A
    [1.5, 1.5],  # Point B
    [0.0, 1.5],  # Point C
    [0.0, 0.0]   # Back Home
]


class SecurityNode(Node):
    """Node that monitors camera for intruders and publishes detection state."""

    def __init__(self):
        """Initialize the node and camera subscriber."""
        super().__init__('security_guard')

        # 1. Setup Camera Subscriber
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.bridge = CvBridge()

        # 2. Setup Publisher for manual control (Chasing)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. State Flags
        self.intruder_detected = False
        self.intruder_position = 0.0  # Error from center
        self.intruder_area = 0.0  # Size of the detected object (for distance estimation)

    def camera_callback(self, msg):
        """Process camera images to detect red intruders."""
        try:
            # Detect Red Object (Same logic as ball_chaser)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Red masks
            mask1 = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 50, 50]), np.array([180, 255, 255]))
            mask = mask1 + mask2

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # Get biggest blob
                c = max(contours, key=cv2.contourArea)
                print(f"Seeing Red! Area size: {cv2.contourArea(c)}")
                self.intruder_area = cv2.contourArea(c)

                if cv2.contourArea(c) > 500:  # Filter small noise
                    self.intruder_detected = True

                    # Calculate position error
                    m_moments = cv2.moments(c)
                    cx = int(m_moments['m10'] / m_moments['m00'])
                    _, width, _ = cv_image.shape
                    self.intruder_position = float(cx - (width / 2))
                else:
                    self.intruder_detected = False
            else:
                self.intruder_detected = False

            # Show Feed
            cv2.imshow("Security Cam", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().debug(f"Image processing error: {e}")


def main():
    """Run the patrol and chase state machine."""
    rclpy.init()

    # Init the Node and Navigator
    security_node = SecurityNode()
    navigator = BasicNavigator()

    # Wait for Nav2
    print("Waiting for Navigation...")
    navigator.waitUntilNav2Active()

    current_wp_index = 0

    # --- MAIN CONTROL LOOP ---
    while rclpy.ok():
        # 1. Process Sensor Data (Camera)
        rclpy.spin_once(security_node, timeout_sec=0.1)

        if security_node.intruder_detected:
            # === CHASE MODE ===
            print("INTRUDER DETECTED! ENGAGING!")

            # A. Cancel any active navigation
            if not navigator.isTaskComplete():
                navigator.cancelTask()

            # B. Calculate Control with Speed Limits
            cmd = Twist()

            # 1. TURNING (Proportional Control)
            # Scaling factor: 0.01 makes it smoother
            turn_speed = -(security_node.intruder_position * 0.01)

            # CLAMP the turning speed (Max 0.5 rad/s)
            # This prevents the "jumping" in RViz caused by spinning too fast
            if turn_speed > 0.5:
                turn_speed = 0.5
            if turn_speed < -0.5:
                turn_speed = -0.5

            cmd.angular.z = turn_speed

            # 2. FORWARD MOVEMENT (With Stopping Distance)
            # We estimate distance roughly by the size of the object (blob area)
            # If the blob is HUGE (area > 30000), we are too close!

            # (You need to capture 'area' from the detection part above)
            # Ensure you add 'self.intruder_area = cv2.contourArea(c)' in the detection loop

            if security_node.intruder_area > 35000:
                # Too close! Stop or reverse slightly
                cmd.linear.x = 0.0
                print("Intruder stopped. Maintaining distance.")
            else:
                # Chase slowly
                cmd.linear.x = 0.2

            security_node.vel_pub.publish(cmd)

        else:
            # === PATROL MODE ===
            # If we were chasing, we stop the twist command first
            # But simpler: Just check if Nav2 is working

            if navigator.isTaskComplete():
                print(f"Patrolling to Waypoint {current_wp_index}...")

                # Create Goal Pose
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = navigator.get_clock().now().to_msg()
                goal.pose.position.x = WAYPOINTS[current_wp_index][0]
                goal.pose.position.y = WAYPOINTS[current_wp_index][1]
                goal.pose.orientation.w = 1.0

                # Send Goal
                navigator.goToPose(goal)

                # Update Index
                current_wp_index = (current_wp_index + 1) % len(WAYPOINTS)

    # Shutdown
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()