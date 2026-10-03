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

Tree structure (highest priority first):

    Selector("SecurityGuard")
    ├── Sequence("EmergencyStop")
    │   ├── Condition("EStopActive")        /estop latched by system_monitor
    │   └── Action("HaltRobot")             cancel Nav2, publish zero cmd_vel
    ├── Sequence("IntruderProtocol")
    │   ├── Condition("IntruderVisible")    fused target seen < target_timeout ago
    │   ├── Action("CancelPatrol")          cancel the active Nav2 goal
    │   └── Action("FollowIntruder")        stand-off follow on range + bearing
    ├── Sequence("SearchProtocol")
    │   ├── Condition("IntruderRecentlyLost")  lost < search_secs ago
    │   ├── Action("CancelPatrol")
    │   └── Action("SearchLastSeen")        turn toward the last-seen side
    └── Sequence("PatrolProtocol")
        ├── Action("NavigateToWaypoint")    Nav2 goToPose, RUNNING until done
        ├── Action("WaitAtWaypoint")        timed dwell
        └── Action("IncrementWaypoint")     advance bb.waypoint_index

The intruder comes from sensor_fusion's /target, so the tree is
detector-agnostic: in person-sim that is the YOLO person tracker. /target is
stamped with the camera image's time; odometry anchors it in the odom frame
(my_bot.target_estimate) and every tick the blackboard gets the target
relative to the robot's pose *now*. Steering on the raw, already-stale
bearing made the robot overshoot. The follow law is the same one
ball_chaser uses (follow_control).

All timing (target timeout, search, dwell) runs on the node clock, i.e.
simulation time under use_sim_time, so a simulator running slower than real
time does not make the intruder look lost.

Publishes /cmd_vel (follow, search, halt), /security_guard/state (the active
protocol), /security_guard/metrics and /intruder_sightings markers.
"""

from dataclasses import dataclass
import math
import time
from typing import Callable

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from my_bot.follow_control import compute_command, front_clearance, search_command
from my_bot.target_estimate import TargetEstimate, stamp_to_sec, yaw_from_quaternion

# Message types only needed by the running node (metrics, markers). Guarded
# so the unit tests can import the leaves with stubbed ROS packages.
try:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:  # pragma: no cover — only missing outside a ROS install
    DiagnosticArray = DiagnosticStatus = KeyValue = None
    Odometry = Marker = MarkerArray = None


# ---------------------------------------------------------------------------
# Blackboard key constants
# ---------------------------------------------------------------------------
BB_ESTOP = 'estop'
BB_LAST_SEEN = 'target_last_seen'      # clock time (s) of the last sighting
BB_TARGET_RANGE = 'target_range'       # metres, -1.0 if unknown
BB_TARGET_BEARING = 'target_bearing'   # rad, positive left
BB_FRONT_CLEAR = 'front_clear'         # closest lidar return ahead (m)
BB_WP_INDEX = 'waypoint_index'
BB_NAV_GOAL_SENT = 'nav_goal_sent'
BB_DWELL_START = 'dwell_start_time'

Status = py_trees.common.Status
Publish = Callable[[Twist], None]


def _bb_get(key, default):
    bb = py_trees.blackboard.Blackboard()
    return bb.get(key) if bb.exists(key) else default


def _twist(linear: float = 0.0, angular: float = 0.0) -> Twist:
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    return cmd


@dataclass
class FollowGains:
    """Stand-off follow parameters (see follow_control.compute_command)."""

    desired_distance: float = 2.5
    deadband: float = 0.15
    k_lin: float = 0.8
    k_yaw: float = 1.5
    max_linear_speed: float = 1.0
    max_back_speed: float = 0.2
    max_angular_speed: float = 1.5
    safety_distance: float = 0.6


# ---------------------------------------------------------------------------
# Behavior Tree leaves
# ---------------------------------------------------------------------------

class EStopActive(py_trees.behaviour.Behaviour):
    """SUCCESS while the emergency stop is latched."""

    def __init__(self, name: str = 'EStopActive'):
        super().__init__(name)

    def update(self) -> Status:
        return Status.SUCCESS if _bb_get(BB_ESTOP, False) else Status.FAILURE


class HaltRobot(py_trees.behaviour.Behaviour):
    """Cancel navigation and hold the robot still (RUNNING)."""

    def __init__(self, navigator: BasicNavigator, publish: Publish,
                 name: str = 'HaltRobot'):
        super().__init__(name)
        self._nav = navigator
        self._publish = publish

    def update(self) -> Status:
        if not self._nav.isTaskComplete():
            self._nav.cancelTask()
        py_trees.blackboard.Blackboard().set(BB_NAV_GOAL_SENT, False)
        self._publish(_twist())
        return Status.RUNNING


class IntruderVisible(py_trees.behaviour.Behaviour):
    """SUCCESS if the fused target was seen within ``timeout`` seconds."""

    def __init__(self, timeout: float = 0.5, clock=time.monotonic,
                 name: str = 'IntruderVisible'):
        super().__init__(name)
        self._timeout = timeout
        self._clock = clock

    def update(self) -> Status:
        age = self._clock() - _bb_get(BB_LAST_SEEN, -math.inf)
        return Status.SUCCESS if age <= self._timeout else Status.FAILURE


class IntruderRecentlyLost(py_trees.behaviour.Behaviour):
    """SUCCESS for ``search_secs`` after the target drops out of view."""

    def __init__(self, timeout: float = 0.5, search_secs: float = 6.0,
                 clock=time.monotonic, name: str = 'IntruderRecentlyLost'):
        super().__init__(name)
        self._timeout = timeout
        self._search_secs = search_secs
        self._clock = clock

    def update(self) -> Status:
        age = self._clock() - _bb_get(BB_LAST_SEEN, -math.inf)
        if self._timeout < age <= self._timeout + self._search_secs:
            return Status.SUCCESS
        return Status.FAILURE


class CancelPatrol(py_trees.behaviour.Behaviour):
    """Cancel any active Nav2 navigation goal."""

    def __init__(self, navigator: BasicNavigator, name: str = 'CancelPatrol'):
        super().__init__(name)
        self._nav = navigator

    def update(self) -> Status:
        if not self._nav.isTaskComplete():
            self._nav.cancelTask()
        py_trees.blackboard.Blackboard().set(BB_NAV_GOAL_SENT, False)
        return Status.SUCCESS


class FollowIntruder(py_trees.behaviour.Behaviour):
    """Keep ``desired_distance`` from the intruder (RUNNING)."""

    def __init__(self, publish: Publish, gains: FollowGains,
                 name: str = 'FollowIntruder'):
        super().__init__(name)
        self._publish = publish
        self._g = gains

    def update(self) -> Status:
        g = self._g
        rng = _bb_get(BB_TARGET_RANGE, -1.0)
        v, w = compute_command(
            _bb_get(BB_TARGET_BEARING, 0.0), rng if rng > 0 else None,
            g.desired_distance, g.k_lin, g.k_yaw, g.max_linear_speed,
            g.max_back_speed, g.max_angular_speed, g.deadband,
            _bb_get(BB_FRONT_CLEAR, math.inf), g.safety_distance)
        self._publish(_twist(v, w))
        return Status.RUNNING


class SearchLastSeen(py_trees.behaviour.Behaviour):
    """Rotate toward the side where the intruder was last seen (RUNNING).

    The direction is latched when the search starts: the blackboard bearing
    follows the robot's turn, and re-reading it would stop the spin once
    the robot faces the last-seen spot.
    """

    def __init__(self, publish: Publish, speed: float = 0.6,
                 name: str = 'SearchLastSeen'):
        super().__init__(name)
        self._publish = publish
        self._speed = speed
        self._turn = None

    def initialise(self) -> None:
        self._turn = None

    def update(self) -> Status:
        if self._turn is None:
            self._turn = search_command(_bb_get(BB_TARGET_BEARING, 0.0), self._speed)
        self._publish(_twist(0.0, self._turn))
        return Status.RUNNING


class NavigateToWaypoint(py_trees.behaviour.Behaviour):
    """Send a Nav2 goal on first tick; poll completion on subsequent ticks."""

    def __init__(self, navigator: BasicNavigator, waypoints,
                 log=None, name: str = 'NavigateToWaypoint'):
        super().__init__(name)
        self._nav = navigator
        self._waypoints = waypoints
        self._log = log or (lambda _msg: None)

    def update(self) -> Status:
        bb = py_trees.blackboard.Blackboard()
        if not _bb_get(BB_NAV_GOAL_SENT, False):
            wp_idx = _bb_get(BB_WP_INDEX, 0)
            wp = self._waypoints[wp_idx % len(self._waypoints)]
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = Time()  # zero stamp: use the latest transform
            goal.pose.position.x = wp[0]
            goal.pose.position.y = wp[1]
            goal.pose.orientation.w = 1.0
            self._nav.goToPose(goal)
            bb.set(BB_NAV_GOAL_SENT, True)
            self._log(f'patrol: heading to waypoint {wp_idx} {tuple(wp)}')

        if self._nav.isTaskComplete():
            bb.set(BB_NAV_GOAL_SENT, False)
            return Status.SUCCESS
        return Status.RUNNING


class WaitAtWaypoint(py_trees.behaviour.Behaviour):
    """Dwell at the current waypoint for a configured duration."""

    def __init__(self, dwell_secs: float = 2.0, clock=time.monotonic,
                 name: str = 'WaitAtWaypoint'):
        super().__init__(name)
        self._dwell = dwell_secs
        self._clock = clock

    def update(self) -> Status:
        bb = py_trees.blackboard.Blackboard()
        start = _bb_get(BB_DWELL_START, None)
        now = self._clock()
        if start is None:
            bb.set(BB_DWELL_START, now)
            return Status.RUNNING
        if (now - start) >= self._dwell:
            bb.set(BB_DWELL_START, None)
            return Status.SUCCESS
        return Status.RUNNING


class IncrementWaypoint(py_trees.behaviour.Behaviour):
    """Advance the waypoint index on the blackboard."""

    def __init__(self, n_waypoints: int, name: str = 'IncrementWaypoint'):
        super().__init__(name)
        self._n = n_waypoints

    def update(self) -> Status:
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_WP_INDEX, (_bb_get(BB_WP_INDEX, 0) + 1) % self._n)
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Tree factory — exposed for unit tests
# ---------------------------------------------------------------------------

def build_security_guard_tree(
        navigator: BasicNavigator,
        publish: Publish,
        waypoints,
        gains: FollowGains = FollowGains(),
        target_timeout: float = 0.5,
        search_secs: float = 6.0,
        search_speed: float = 0.6,
        dwell_secs: float = 2.0,
        log=None,
        clock=time.monotonic,
) -> py_trees.trees.BehaviourTree:
    """Construct and return the security guard behaviour tree.

    ``clock`` returns seconds; the node passes its (sim-time) clock.
    """
    estop = py_trees.composites.Sequence('EmergencyStop', memory=False)
    estop.add_children([EStopActive(), HaltRobot(navigator, publish)])

    intruder = py_trees.composites.Sequence('IntruderProtocol', memory=False)
    intruder.add_children([
        IntruderVisible(target_timeout, clock),
        CancelPatrol(navigator),
        FollowIntruder(publish, gains),
    ])

    search = py_trees.composites.Sequence('SearchProtocol', memory=False)
    search.add_children([
        IntruderRecentlyLost(target_timeout, search_secs, clock),
        CancelPatrol(navigator),
        SearchLastSeen(publish, search_speed),
    ])

    patrol = py_trees.composites.Sequence('PatrolProtocol', memory=True)
    patrol.add_children([
        NavigateToWaypoint(navigator, waypoints, log),
        WaitAtWaypoint(dwell_secs, clock),
        IncrementWaypoint(len(waypoints)),
    ])

    root = py_trees.composites.Selector('SecurityGuard', memory=False)
    root.add_children([estop, intruder, search, patrol])
    return py_trees.trees.BehaviourTree(root)


def active_protocol(tree: py_trees.trees.BehaviourTree) -> str:
    """Name of the root child that ran last tick, e.g. 'PatrolProtocol'."""
    for child in tree.root.children:
        if child.status in (Status.RUNNING, Status.SUCCESS):
            return child.name
    return 'Idle'


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------

class SecurityGuardBTNode(Node):
    """ROS 2 node that ticks the security guard tree at 10 Hz."""

    def __init__(self):
        super().__init__('security_guard_bt')

        self.declare_parameter('waypoints', [
            3.28, 6.85, -0.92, 6.85, 5.13, -2.45,
            0.23, -2.55, -4.52, 8.60, 0.0, 0.0,
        ])
        self.declare_parameter('waypoint_dwell_secs', 2.0)
        self.declare_parameter('target_timeout', 0.5)
        self.declare_parameter('search_secs', 6.0)
        self.declare_parameter('search_angular_speed', 0.6)
        self.declare_parameter('desired_distance', 2.5)
        self.declare_parameter('deadband', 0.15)
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_yaw', 1.5)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_back_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('safety_distance', 0.6)

        gp = self.get_parameter
        flat = gp('waypoints').value
        waypoints = [[flat[i], flat[i + 1]] for i in range(0, len(flat) - 1, 2)]
        gains = FollowGains(**{
            name: float(gp(name).value) for name in vars(FollowGains())})

        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._state_pub = self.create_publisher(String, '/security_guard/state', 10)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/security_guard/metrics', 10)
        self._sighting_pub = self.create_publisher(
            MarkerArray, '/intruder_sightings', 10)

        # Fed by /odom and /target: the intruder anchored in the odom frame
        self._target = TargetEstimate()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/estop', self._estop_cb, latched)
        self.create_subscription(Vector3Stamped, '/target', self._target_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_ESTOP, False)
        bb.set(BB_WP_INDEX, 0)
        bb.set(BB_NAV_GOAL_SENT, False)
        bb.set(BB_LAST_SEEN, -math.inf)
        bb.set(BB_TARGET_RANGE, -1.0)
        bb.set(BB_TARGET_BEARING, 0.0)
        bb.set(BB_FRONT_CLEAR, math.inf)

        # Blocks until AMCL and bt_navigator are active (AMCL gets its
        # initial pose from nav2_params.yaml, so no RViz click is needed).
        self._navigator = BasicNavigator()
        self.get_logger().info('waiting for Nav2...')
        self._navigator.waitUntilNav2Active()

        self._bt = build_security_guard_tree(
            self._navigator, self._vel_pub.publish, waypoints, gains,
            target_timeout=float(gp('target_timeout').value),
            search_secs=float(gp('search_secs').value),
            search_speed=float(gp('search_angular_speed').value),
            dwell_secs=float(gp('waypoint_dwell_secs').value),
            log=self.get_logger().info, clock=self._now)
        self._bt.setup()
        self.get_logger().info(
            'behaviour tree:\n' + py_trees.display.unicode_tree(self._bt.root))

        self._metrics = {
            'waypoints_visited': 0,
            'intruder_detections': 0,
            'track_losses': 0,
            'time_following_sec': 0.0,
            'distance_traveled_m': 0.0,
        }
        # Follow quality while IntruderProtocol runs (Phase 0 baseline):
        # sums of squared bearing and stand-off range errors.
        self._desired = gains.desired_distance
        self._follow_err = {'n': 0, 'bearing_sq': 0.0, 'ranged': 0, 'range_sq': 0.0}
        self._session_start = self.get_clock().now()
        self._robot_pose = None  # (x, y, yaw) in odom
        self._sightings = MarkerArray()
        self._state = ''
        self._prev_wp_idx = 0

        self.create_timer(0.1, self._tick)
        self.create_timer(5.0, self._publish_metrics)

    # ------------------------------------------------------------------

    def _estop_cb(self, msg: Bool):
        py_trees.blackboard.Blackboard().set(BB_ESTOP, bool(msg.data))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, msg: Vector3Stamped):
        if self._target.update(stamp_to_sec(msg.header.stamp),
                               float(msg.vector.x), float(msg.vector.y)):
            py_trees.blackboard.Blackboard().set(BB_LAST_SEEN, self._now())
            self._refresh_target()

    def _refresh_target(self):
        """Blackboard target = the estimate relative to the robot now."""
        rel = self._target.relative()
        if rel is None:
            return
        bearing, rng = rel
        bb = py_trees.blackboard.Blackboard()
        bb.set(BB_TARGET_BEARING, bearing)
        bb.set(BB_TARGET_RANGE, rng if rng is not None else -1.0)

    def _scan_cb(self, msg: LaserScan):
        py_trees.blackboard.Blackboard().set(
            BB_FRONT_CLEAR,
            front_clearance(msg.ranges, msg.angle_min, msg.angle_increment))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._target.add_pose(stamp_to_sec(msg.header.stamp), p.x, p.y, yaw)
        if self._robot_pose is not None:
            self._metrics['distance_traveled_m'] += math.hypot(
                p.x - self._robot_pose[0], p.y - self._robot_pose[1])
        self._robot_pose = (p.x, p.y, yaw)

    # ------------------------------------------------------------------

    def _tick(self):
        self._refresh_target()
        self._bt.tick()
        state = active_protocol(self._bt)
        if state != self._state:
            self.get_logger().info(f'{self._state or "start"} -> {state}')
            if state == 'IntruderProtocol':
                self._metrics['intruder_detections'] += 1
                self._add_sighting_marker()
            elif self._state == 'IntruderProtocol' and state == 'SearchProtocol':
                self._metrics['track_losses'] += 1
            self._state = state
        self._state_pub.publish(String(data=state))

        if state == 'IntruderProtocol':
            self._metrics['time_following_sec'] += 0.1
            self._record_follow_error()
        wp_now = _bb_get(BB_WP_INDEX, 0)
        if wp_now != self._prev_wp_idx:
            self._metrics['waypoints_visited'] += 1
            self._prev_wp_idx = wp_now

    def _record_follow_error(self):
        err = self._follow_err
        err['n'] += 1
        err['bearing_sq'] += _bb_get(BB_TARGET_BEARING, 0.0) ** 2
        rng = _bb_get(BB_TARGET_RANGE, -1.0)
        if rng > 0.0:
            err['ranged'] += 1
            err['range_sq'] += (rng - self._desired) ** 2

    def _follow_rms(self):
        """(bearing RMS in degrees, stand-off range RMS in m) while following."""
        err = self._follow_err
        bearing = math.degrees(math.sqrt(err['bearing_sq'] / err['n'])) if err['n'] else 0.0
        rng = math.sqrt(err['range_sq'] / err['ranged']) if err['ranged'] else 0.0
        return bearing, rng

    def _publish_metrics(self):
        elapsed = (self.get_clock().now() - self._session_start).nanoseconds / 1e9
        bearing_rms, range_rms = self._follow_rms()
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'SecurityGuardBT'
        status.hardware_id = 'autonav_sim'
        status.level = DiagnosticStatus.OK
        status.message = f'{self._state} | session {elapsed:.0f}s'
        status.values = [
            KeyValue(key='state', value=self._state),
            KeyValue(key='waypoints_visited',
                     value=str(self._metrics['waypoints_visited'])),
            KeyValue(key='intruder_detections',
                     value=str(self._metrics['intruder_detections'])),
            KeyValue(key='track_losses',
                     value=str(self._metrics['track_losses'])),
            KeyValue(key='follow_bearing_rms_deg', value=f'{bearing_rms:.1f}'),
            KeyValue(key='follow_range_rms_m', value=f'{range_rms:.2f}'),
            KeyValue(key='time_following_sec',
                     value=f"{self._metrics['time_following_sec']:.1f}"),
            KeyValue(key='distance_traveled_m',
                     value=f"{self._metrics['distance_traveled_m']:.2f}"),
            KeyValue(key='session_elapsed_sec', value=f'{elapsed:.1f}'),
        ]
        diag.status = [status]
        self._diag_pub.publish(diag)

    def _add_sighting_marker(self):
        """Mark where the intruder was seen (or the robot, if not ranged)."""
        where = self._target.target_xy or (
            self._robot_pose[:2] if self._robot_pose else None)
        if where is None:
            return
        stamp = self.get_clock().now().to_msg()
        elapsed = (self.get_clock().now() - self._session_start).nanoseconds / 1e9
        n = len(self._sightings.markers) // 2

        sphere = Marker()
        sphere.header.frame_id = 'odom'
        sphere.header.stamp = stamp
        sphere.ns = 'sightings'
        sphere.id = n
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y = where
        sphere.pose.position.z = 0.5
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
        sphere.color.r, sphere.color.a = 1.0, 0.8

        label = Marker()
        label.header = sphere.header
        label.ns = 'sighting_labels'
        label.id = n
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x, label.pose.position.y = where
        label.pose.position.z = 0.9
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'#{n + 1} T={elapsed:.0f}s'

        self._sightings.markers += [sphere, label]
        self._sighting_pub.publish(self._sightings)


def main(args=None):
    """Initialize and spin the SecurityGuardBTNode."""
    rclpy.init(args=args)
    node = SecurityGuardBTNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
