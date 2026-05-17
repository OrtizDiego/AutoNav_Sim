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

"""Security guard — patrol + intruder interception as a ROS 2 Lifecycle Node."""

import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State  # noqa: F401 — used in type hints
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class SecurityGuard(LifecycleNode):
    """Patrol waypoints and chase intruders with full lifecycle management.

    State machine:
        Unconfigured → configure → Inactive → activate → Active
        Active → deactivate → Inactive → cleanup → Unconfigured

    The control loop runs as a 10 Hz timer callback (active state only).
    """

    def __init__(self):
        """Declare parameters — no sub/pub yet (lifecycle protocol)."""
        super().__init__('security_guard')

        # Parameters are declared here so they survive configure/cleanup cycles
        self.declare_parameter('hsv_red_lower1', [0, 100, 100])
        self.declare_parameter('hsv_red_upper1', [10, 255, 255])
        self.declare_parameter('hsv_red_lower2', [160, 100, 100])
        self.declare_parameter('hsv_red_upper2', [180, 255, 255])
        self.declare_parameter('angular_gain', 0.01)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_contour_area', 500.0)
        self.declare_parameter('stop_distance_area', 35000.0)
        self.declare_parameter('waypoints', [
            3.28, 6.85, -0.92, 6.85, 5.13, -2.45,
            0.23, -2.55, -4.52, 8.60, 0.0, 0.0,
        ])
        self.declare_parameter('waypoint_dwell_secs', 2.0)

        # Internal state
        self._intruder_detected: bool = False
        self._intruder_position: float = 0.0
        self._intruder_area: float = 0.0
        self._target_range: float = -1.0
        self._current_wp: int = 0

        self._display_frame = None
        self._display_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state) -> TransitionCallbackReturn:
        """Create subscriptions, publishers, load params."""
        self._load_params()

        self._bridge = CvBridge()
        self._cam_sub = self.create_subscription(
            Image, '/camera/image_raw', self._camera_callback, 10)
        self._range_sub = self.create_subscription(
            Float32, '/target_range', self._range_callback, 10)
        self._vel_pub = self.create_lifecycle_publisher(Twist, '/cmd_vel', 10)

        self._navigator = BasicNavigator()

        # Display thread — persists across activate/deactivate cycles
        if not hasattr(self, '_display_thread') or not self._display_thread.is_alive():
            self._display_thread = threading.Thread(
                target=self._display_loop, daemon=True)
            self._display_thread.start()

        self.get_logger().info('SecurityGuard configured.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        """Activate publishers and start the 10 Hz control loop."""
        super().on_activate(state)
        self.get_logger().info('Waiting for Nav2...')
        self._navigator.waitUntilNav2Active()
        self._current_wp = 0
        self._control_timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info('SecurityGuard active — patrolling.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Stop control loop, cancel navigation, zero velocity."""
        self._control_timer.cancel()
        self.destroy_timer(self._control_timer)
        if not self._navigator.isTaskComplete():
            self._navigator.cancelTask()
        self._vel_pub.publish(Twist())  # zero velocity
        super().on_deactivate(state)
        self.get_logger().info('SecurityGuard deactivated.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """Destroy all subscriptions and publishers."""
        self.destroy_subscription(self._cam_sub)
        self.destroy_subscription(self._range_sub)
        self.destroy_publisher(self._vel_pub)
        self.get_logger().info('SecurityGuard cleaned up.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        """Final cleanup on node shutdown."""
        self.get_logger().info('SecurityGuard shutting down.')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------

    def _load_params(self):
        flat = self.get_parameter('waypoints').value
        self._waypoints = [
            [flat[i], flat[i + 1]] for i in range(0, len(flat) - 1, 2)]
        self._lower1 = np.array(
            self.get_parameter('hsv_red_lower1').value, dtype=np.uint8)
        self._upper1 = np.array(
            self.get_parameter('hsv_red_upper1').value, dtype=np.uint8)
        self._lower2 = np.array(
            self.get_parameter('hsv_red_lower2').value, dtype=np.uint8)
        self._upper2 = np.array(
            self.get_parameter('hsv_red_upper2').value, dtype=np.uint8)
        self._angular_gain = float(self.get_parameter('angular_gain').value)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._max_ang = float(self.get_parameter('max_angular_speed').value)
        self._min_area = float(self.get_parameter('min_contour_area').value)
        self._stop_area = float(self.get_parameter('stop_distance_area').value)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _camera_callback(self, msg: Image):
        """Detect red intruder; update shared detection state."""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = (cv2.inRange(hsv, self._lower1, self._upper1) +
                    cv2.inRange(hsv, self._lower2, self._upper2))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            valid = [c for c in contours
                     if cv2.contourArea(c) >= self._min_area]
            if valid:
                c = max(valid, key=cv2.contourArea)
                m = cv2.moments(c)
                if m['m00'] > 0:
                    cx = int(m['m10'] / m['m00'])
                    _, width, _ = cv_image.shape
                    self._intruder_detected = True
                    self._intruder_position = float(cx - (width / 2))
                    self._intruder_area = cv2.contourArea(c)
                    cv2.circle(cv_image,
                               (cx, int(m['m01'] / m['m00'])),
                               12, (0, 0, 255), 3)
                else:
                    self._intruder_detected = False
            else:
                self._intruder_detected = False

            with self._display_lock:
                self._display_frame = cv_image

        except Exception as e:
            self.get_logger().debug(f'Camera callback error: {e}')

    def _range_callback(self, msg: Float32):
        """Receive metric range from sensor_fusion node."""
        self._target_range = msg.data

    # ------------------------------------------------------------------
    # Control loop (10 Hz timer)
    # ------------------------------------------------------------------

    def _control_loop(self):
        """Patrol/chase state machine — runs as timer callback."""
        if self._intruder_detected:
            self.get_logger().info('INTRUDER DETECTED! ENGAGING!')

            if not self._navigator.isTaskComplete():
                self._navigator.cancelTask()

            cmd = Twist()
            turn = -(self._intruder_position * self._angular_gain)
            cmd.angular.z = max(-self._max_ang, min(self._max_ang, turn))

            if self._target_range > 0:
                too_close = self._target_range < 0.8
            else:
                too_close = self._intruder_area > self._stop_area

            if too_close:
                cmd.linear.x = 0.0
                self.get_logger().info('Maintaining distance.')
            else:
                cmd.linear.x = self._linear_speed

            self._vel_pub.publish(cmd)

        else:
            # Patrol
            if self._navigator.isTaskComplete():
                self.get_logger().info(
                    f'Patrolling to waypoint {self._current_wp}...')
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self._navigator.get_clock().now().to_msg()
                wp = self._waypoints[self._current_wp]
                goal.pose.position.x = wp[0]
                goal.pose.position.y = wp[1]
                goal.pose.orientation.w = 1.0
                self._navigator.goToPose(goal)
                self._current_wp = (self._current_wp + 1) % len(self._waypoints)

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def _display_loop(self):
        """Render latest camera frame at ~30 fps in a dedicated thread."""
        while True:
            with self._display_lock:
                frame = self._display_frame
            if frame is not None:
                cv2.imshow('Security Cam', frame)
                cv2.waitKey(1)
            time.sleep(0.033)


def main(args=None):
    """Initialize and run SecurityGuard as a lifecycle node."""
    rclpy.init(args=args)
    node = SecurityGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
