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

"""Security guard implemented as a py_trees Behavior Tree.

Tree structure:
    Selector("SecurityGuardRoot")
    ├── Sequence("IntruderProtocol")
    │   ├── Condition("IntruderVisible")   reads bb.intruder_visible
    │   ├── Action("CancelPatrol")         cancels active Nav2 goal
    │   └── Selector("ApproachControl")
    │       ├── Condition("TooClose")      checks bb.target_range < stop_dist
    │       └── Action("ChaseIntruder")    publishes proportional cmd_vel
    └── Sequence("PatrolProtocol")
        ├── Action("NavigateToWaypoint")   sends goToPose, returns RUNNING
        ├── Action("WaitAtWaypoint")       timer-based dwell
        └── Action("IncrementWaypoint")   advances bb.waypoint_index
"""

import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import py_trees
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


# ---------------------------------------------------------------------------
# Blackboard key constants
# ---------------------------------------------------------------------------
BB_INTRUDER_VISIBLE = 'intruder_visible'
BB_HSV_DETECTED = 'hsv_target_detected'
BB_YOLO_DETECTED = 'yolo_target_detected'
BB_TARGET_RANGE = 'target_range'
BB_TARGET_BEARING = 'target_bearing'
BB_WP_INDEX = 'waypoint_index'
BB_NAV_GOAL_SENT = 'nav_goal_sent'
BB_DWELL_START = 'dwell_start_time'


# ---------------------------------------------------------------------------
# Behavior Tree leaves
# ---------------------------------------------------------------------------

class IntruderVisible(py_trees.behaviour.Behaviour):
    """SUCCESS if intruder is currently visible (HSV or YOLO)."""

    def __init__(self, name: str = 'IntruderVisible'):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        bb = py_trees.blackboard.Blackboard()
        hsv = bb.get(BB_HSV_DETECTED) if bb.exists(BB_HSV_DETECTED) else False
        yolo = bb.get(BB_YOLO_DETECTED) if bb.exists(BB_YOLO_DETECTED) else False
        if hsv or yolo:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class TooClose(py_trees.behaviour.Behaviour):
    """SUCCESS if the target is within stop_distance metres."""

    def __init__(self, name: str = 'TooClose', stop_distance: float = 0.8):
        super().__init__(name)
        self._stop_dist = stop_distance

    def update(self) -> py_trees.common.Status:
        bb = py_trees.blackboard.Blackboard()
        r = bb.get(BB_TARGET_RANGE) if bb.exists(BB_TARGET_RANGE) else -1.0
        if r > 0 and r < self._stop_dist:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class CancelPatrol(py_trees.behaviour.Behaviour):
    """Cancel any active Nav2 navigation goal."""

    def __init__(self, navigator: BasicNavigator, name: str = 'CancelPatrol'):
        super().__init__(name)
        self._nav = navigator

    def update(self) -> py_trees.common.Status:
        if not self._nav.isTaskComplete():
            self._nav.cancelTask()
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_NAV_GOAL_SENT, False)
        return py_trees.common.Status.SUCCESS


class ChaseIntruder(py_trees.behaviour.Behaviour):
    """Publish proportional cmd_vel to approach the detected intruder."""

    def __init__(self, node: Node, name: str = 'ChaseIntruder'):
        super().__init__(name)
        self._node = node

    def update(self) -> py_trees.common.Status:
        bb = py_trees.blackboard.Blackboard()
        bearing = bb.get(BB_TARGET_BEARING) if bb.exists(BB_TARGET_BEARING) else 0.0
        range_ = bb.get(BB_TARGET_RANGE) if bb.exists(BB_TARGET_RANGE) else 5.0

        cmd = Twist()
        turn = -(bearing * self._node._angular_gain_bt)
        max_ang = self._node._max_ang_bt
        cmd.angular.z = max(-max_ang, min(max_ang, turn))
        cmd.linear.x = self._node._linear_speed_bt if range_ > 0.8 else 0.0
        self._node._vel_pub_bt.publish(cmd)
        return py_trees.common.Status.RUNNING


class NavigateToWaypoint(py_trees.behaviour.Behaviour):
    """Send a Nav2 goal on first tick; poll completion on subsequent ticks."""

    def __init__(self, navigator: BasicNavigator, node: Node,
                 name: str = 'NavigateToWaypoint'):
        super().__init__(name)
        self._nav = navigator
        self._node = node

    def update(self) -> py_trees.common.Status:
        bb = py_trees.blackboard.Blackboard()
        goal_sent = bb.get(BB_NAV_GOAL_SENT) if bb.exists(BB_NAV_GOAL_SENT) else False

        if not goal_sent:
            wp_idx = bb.get(BB_WP_INDEX) if bb.exists(BB_WP_INDEX) else 0
            waypoints = self._node._waypoints_bt
            wp = waypoints[wp_idx % len(waypoints)]
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self._nav.get_clock().now().to_msg()
            goal.pose.position.x = wp[0]
            goal.pose.position.y = wp[1]
            goal.pose.orientation.w = 1.0
            self._nav.goToPose(goal)
            bb.set(BB_NAV_GOAL_SENT, True)
            self._node.get_logger().info(f'BT: navigating to waypoint {wp_idx}')

        if self._nav.isTaskComplete():
            bb.set(BB_NAV_GOAL_SENT, False)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitAtWaypoint(py_trees.behaviour.Behaviour):
    """Dwell at the current waypoint for a configured duration."""

    def __init__(self, dwell_secs: float = 2.0, name: str = 'WaitAtWaypoint'):
        super().__init__(name)
        self._dwell = dwell_secs

    def update(self) -> py_trees.common.Status:
        bb = py_trees.blackboard.Blackboard()
        start = bb.get(BB_DWELL_START) if bb.exists(BB_DWELL_START) else None
        now = time.monotonic()
        if start is None:
            bb.set(BB_DWELL_START, now)
            return py_trees.common.Status.RUNNING
        if (now - start) >= self._dwell:
            bb.set(BB_DWELL_START, None)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class IncrementWaypoint(py_trees.behaviour.Behaviour):
    """Advance the waypoint index on the blackboard."""

    def __init__(self, n_waypoints: int, name: str = 'IncrementWaypoint'):
        super().__init__(name)
        self._n = n_waypoints

    def update(self) -> py_trees.common.Status:
        bb = py_trees.blackboard.Blackboard()
        idx = bb.get(BB_WP_INDEX) if bb.exists(BB_WP_INDEX) else 0
        bb.set(BB_WP_INDEX, (idx + 1) % self._n)
        return py_trees.common.Status.SUCCESS


# ---------------------------------------------------------------------------
# Tree factory — exposed for unit tests
# ---------------------------------------------------------------------------

def build_security_guard_tree(
        navigator: BasicNavigator,
        node: Node,
        stop_distance: float = 0.8,
        dwell_secs: float = 2.0,
) -> py_trees.trees.BehaviourTree:
    """Construct and return the security guard behaviour tree."""
    n_wp = len(node._waypoints_bt)

    root = py_trees.composites.Selector('SecurityGuardRoot', memory=False)

    intruder_seq = py_trees.composites.Sequence(
        'IntruderProtocol', memory=False)
    approach_sel = py_trees.composites.Selector(
        'ApproachControl', memory=False)
    approach_sel.add_children([
        TooClose(stop_distance=stop_distance),
        ChaseIntruder(node),
    ])
    intruder_seq.add_children([
        IntruderVisible(),
        CancelPatrol(navigator),
        approach_sel,
    ])

    patrol_seq = py_trees.composites.Sequence(
        'PatrolProtocol', memory=True)
    patrol_seq.add_children([
        NavigateToWaypoint(navigator, node),
        WaitAtWaypoint(dwell_secs),
        IncrementWaypoint(n_wp),
    ])

    root.add_children([intruder_seq, patrol_seq])
    return py_trees.trees.BehaviourTree(root)


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------

class SecurityGuardBTNode(Node):
    """ROS 2 node that ticks a py_trees BehaviourTree at 10 Hz."""

    def __init__(self):
        super().__init__('security_guard_bt')

        self.declare_parameter('hsv_red_lower1', [0, 100, 100])
        self.declare_parameter('hsv_red_upper1', [10, 255, 255])
        self.declare_parameter('hsv_red_lower2', [160, 100, 100])
        self.declare_parameter('hsv_red_upper2', [180, 255, 255])
        self.declare_parameter('angular_gain', 0.01)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_contour_area', 500.0)
        self.declare_parameter('waypoints', [
            3.28, 6.85, -0.92, 6.85, 5.13, -2.45,
            0.23, -2.55, -4.52, 8.60, 0.0, 0.0,
        ])
        self.declare_parameter('waypoint_dwell_secs', 2.0)
        self.declare_parameter('stop_distance', 0.8)

        flat = self.get_parameter('waypoints').value
        self._waypoints_bt = [
            [flat[i], flat[i + 1]] for i in range(0, len(flat) - 1, 2)]
        self._angular_gain_bt = float(self.get_parameter('angular_gain').value)
        self._linear_speed_bt = float(self.get_parameter('linear_speed').value)
        self._max_ang_bt = float(self.get_parameter('max_angular_speed').value)
        self._min_area_bt = float(self.get_parameter('min_contour_area').value)
        dwell = float(self.get_parameter('waypoint_dwell_secs').value)
        stop_dist = float(self.get_parameter('stop_distance').value)

        self._lower1 = np.array(
            self.get_parameter('hsv_red_lower1').value, dtype=np.uint8)
        self._upper1 = np.array(
            self.get_parameter('hsv_red_upper1').value, dtype=np.uint8)
        self._lower2 = np.array(
            self.get_parameter('hsv_red_lower2').value, dtype=np.uint8)
        self._upper2 = np.array(
            self.get_parameter('hsv_red_upper2').value, dtype=np.uint8)

        self._bridge = CvBridge()

        # Subscriptions
        self._cam_sub = self.create_subscription(
            Image, '/camera/image_raw', self._cam_cb, 10)
        self._range_sub = self.create_subscription(
            Float32, '/target_range', self._range_cb, 10)
        self._yolo_sub = self.create_subscription(
            Float32, '/person_detected',   # Bool remapped to Float32 topic stub
            self._yolo_cb, 10)

        self._vel_pub_bt = self.create_publisher(Twist, '/cmd_vel', 10)

        # Navigator and behaviour tree
        self._navigator = BasicNavigator()
        self._navigator.waitUntilNav2Active()

        # Initialise blackboard
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_WP_INDEX, 0)
        bb.set(BB_NAV_GOAL_SENT, False)
        bb.set(BB_HSV_DETECTED, False)
        bb.set(BB_YOLO_DETECTED, False)
        bb.set(BB_TARGET_RANGE, -1.0)
        bb.set(BB_TARGET_BEARING, 0.0)

        self._bt = build_security_guard_tree(
            self._navigator, self, stop_dist, dwell)
        self._bt.setup()

        self._tick_timer = self.create_timer(0.1, self._tick)

        # Display thread
        self._display_frame = None
        self._display_lock = threading.Lock()
        self._display_thread = threading.Thread(
            target=self._display_loop, daemon=True)
        self._display_thread.start()

    def _cam_cb(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = (cv2.inRange(hsv, self._lower1, self._upper1) +
                    cv2.inRange(hsv, self._lower2, self._upper2))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            valid = [c for c in contours
                     if cv2.contourArea(c) >= self._min_area_bt]
            bb = py_trees.blackboard.Blackboard()
            if valid:
                c = max(valid, key=cv2.contourArea)
                m = cv2.moments(c)
                if m['m00'] > 0:
                    cx = int(m['m10'] / m['m00'])
                    _, width, _ = cv_image.shape
                    bearing = float(cx - (width / 2))
                    bb.set(BB_HSV_DETECTED, True)
                    bb.set(BB_TARGET_BEARING, bearing)
                    return
            bb.set(BB_HSV_DETECTED, False)
            with self._display_lock:
                self._display_frame = cv_image
        except Exception as e:
            self.get_logger().debug(f'Camera error: {e}')

    def _range_cb(self, msg: Float32):
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_TARGET_RANGE, msg.data)

    def _yolo_cb(self, msg):
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_YOLO_DETECTED, bool(msg.data))

    def _tick(self):
        """Tick the behaviour tree once."""
        self._bt.tick()

    def _display_loop(self):
        while True:
            with self._display_lock:
                frame = self._display_frame
            if frame is not None:
                cv2.imshow('Security Guard BT', frame)
                cv2.waitKey(1)
            time.sleep(0.033)


def main(args=None):
    """Initialize and spin the SecurityGuardBTNode."""
    rclpy.init(args=args)
    node = SecurityGuardBTNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
