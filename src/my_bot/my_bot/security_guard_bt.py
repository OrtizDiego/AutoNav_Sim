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

"""Security guard: the ROS node around the py_trees tree of guard_tree.py.

The tree (my_bot.guard_tree, highest priority first): EmergencyStop,
IntruderProtocol, SearchProtocol, SweepProtocol, ReportProtocol,
PatrolProtocol. This node feeds its blackboard from the subscriptions,
ticks it at 10 Hz and turns its state changes into alerts, metrics and
markers (my_bot.guard_markers).

Detect and confirm happen upstream: target_tracker publishes
/intruder/track only for a confirmed odom-frame track with velocity, built
from sensor_fusion's /target (camera + lidar) and lidar clusters, so the
tree is detector-agnostic. Every tick
the blackboard gets the track extrapolated to now, relative to the robot's
pose now (my_bot.target_estimate). Steering on the raw, already-stale
bearing made the robot overshoot, and a static target lagged a walking
person. ``use_track: false`` follows sensor_fusion's /target directly
instead (anchored at its stamp, static between detections). The follow law
is the same one ball_chaser uses (follow_control), plus a feedforward of
the rate at which the intruder's own motion turns its bearing: a P-loop alone
lags a person crossing the view by that rate / k_yaw (16-17 deg bearing RMS
measured in person-sim).

Alerts (my_bot.incident_log): entering IntruderProtocol opens an incident
(intruder_detected, with the position in the map frame and a camera
snapshot) or adds a sighting to the open one (intruder_reacquired); leaving
it for the search logs intruder_lost; ReportProtocol closes the incident
once the search gives up, then the patrol resumes. Each event is published
as JSON on /security_events and appended to ``incident_dir``/incidents.jsonl.

The search drives (Nav2, goal in the odom frame) to where the track was
heading: its last position plus ``search_lead_secs`` of its velocity, at
most ``search_max_distance`` further. Nav2 plans around the walls the
intruder went behind. Then, or at once without a ranged track, the robot
turns toward the side it was last seen on. SweepProtocol then visits the
viewpoints my_bot.room_sweep picked on the saved map around the last-seen
spot (map frame, planned when the intruder is lost), each facing the floor
it uncovers, so someone behind an exhibit or around a corner comes into
view. Without a map (no zones file, or no map -> odom yet) there is no sweep.

Patrol zones, their schedule and restricted areas come from ``zones_file``
(my_bot.guard_zones); without one the ``waypoints`` parameter is a single
zone. The schedule counts from the moment Nav2 is active (so a slow
bringup does not eat the first zone's slot). The zone switches when the
current leg ends, never mid-goal. An
intruder inside a restricted area raises a restricted_area event, once per
area and incident.

All timing (target timeout, search, dwell) runs on the node clock, i.e.
simulation time under use_sim_time, so a simulator running slower than real
time does not make the intruder look lost.

Publishes /cmd_vel (follow, search, halt), /security_guard/state (the active
protocol), /security_guard/metrics, /security_events, /intruder_sightings
markers, /guard_zones (restricted areas and patrol waypoints, latched) and
/sweep_viewpoints markers.
"""

import json
import math
import time
from typing import Optional, Tuple

from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, Vector3Stamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time as RosTime
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from my_bot.clearance_map import ClearanceMap
from my_bot.conversions import stamp_to_sec, yaw_from_quaternion
from my_bot.follow_control import front_clearance
from my_bot.guard_markers import sighting_markers, sweep_markers, zone_markers
from my_bot.guard_tree import (
    active_protocol, BB_ESTOP, BB_FRONT_CLEAR, BB_LAST_SEEN, BB_NAV_GOAL_SENT,
    BB_PATROL_ZONE, BB_SEARCH_GOAL, BB_SWEEP, BB_TARGET_BEARING, BB_TARGET_RANGE,
    BB_TARGET_YAW_RATE, BB_WP_INDEX, BB_WP_VISITS, bb_get, bb_set,
    build_security_guard_tree, FollowGains)
from my_bot.guard_zones import GuardZones
from my_bot.incident_log import IncidentLog
from my_bot.node_utils import spin_node
from my_bot.room_sweep import plan_sweep
from my_bot.target_estimate import TargetEstimate

# What a fresh blackboard holds before the first message arrives
BLACKBOARD_DEFAULTS = {
    BB_ESTOP: False,
    BB_WP_INDEX: 0,
    BB_WP_VISITS: 0,
    BB_NAV_GOAL_SENT: False,
    BB_SWEEP: [],
    BB_LAST_SEEN: -math.inf,
    BB_TARGET_RANGE: -1.0,
    BB_TARGET_BEARING: 0.0,
    BB_TARGET_YAW_RATE: 0.0,
    BB_SEARCH_GOAL: None,
    BB_FRONT_CLEAR: math.inf,
}


class GuardNavigator(BasicNavigator):
    """BasicNavigator whose wait for Nav2 cannot hang on a lost reply.

    Humble's waitUntilNav2Active() asks each node's get_state service with
    no timeout. Asked while AMCL was still being created, the reply never
    came and the guard waited forever: no patrol, no /security_guard/state
    (the CI sprint run of 2026-11-02, and a local one in Phase 7b). Here
    each request gives up after ``call_timeout`` seconds and is sent again.
    """

    call_timeout = 2.0
    poll_secs = 1.0

    def _waitForNodeToActivate(self, node_name):  # noqa: N802 - overrides Nav2
        from lifecycle_msgs.srv import GetState
        client = self.create_client(GetState, f'{node_name}/get_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_name}/get_state service not available, waiting...')
        state = 'unknown'
        while state != 'active':
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout)
            if not future.done():
                future.cancel()
                self.info(f'{node_name}/get_state did not answer; asking again')
                continue
            if future.result() is not None:
                state = future.result().current_state.label
            if state != 'active':
                time.sleep(self.poll_secs)


class SecurityGuardBTNode(Node):
    """ROS 2 node that ticks the security guard tree at 10 Hz."""

    def __init__(self):
        super().__init__('security_guard_bt')

        self.declare_parameter('waypoints', [
            3.28, 6.85, -0.92, 6.85, 5.13, -2.45,
            0.23, -2.55, -4.52, 8.60, 0.0, 0.0,
        ])
        self.declare_parameter('waypoint_dwell_secs', 2.0)
        self.declare_parameter('zones_file', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('use_track', True)
        self.declare_parameter('track_max_predict', 1.0)
        self.declare_parameter('target_timeout', 0.5)
        self.declare_parameter('search_secs', 20.0)
        self.declare_parameter('search_angular_speed', 0.6)
        self.declare_parameter('search_lead_secs', 2.0)
        self.declare_parameter('search_max_distance', 4.0)
        self.declare_parameter('sweep_secs', 60.0)
        self.declare_parameter('sweep_dwell_secs', 2.0)
        self.declare_parameter('sweep_radius', 5.0)
        self.declare_parameter('sweep_view_range', 4.0)
        self.declare_parameter('sweep_max_points', 4)
        self.declare_parameter('incident_dir', '')
        self.declare_parameter('snapshot_topic', '/camera/image_raw')
        self.declare_parameter('desired_distance', 2.5)
        self.declare_parameter('deadband', 0.15)
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_yaw', 1.5)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_back_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('safety_distance', 0.6)
        self.declare_parameter('k_ff', 1.0)

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
        self._event_pub = self.create_publisher(String, '/security_events', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._zones_pub = self.create_publisher(MarkerArray, '/guard_zones', latched)
        self._sweep_pub = self.create_publisher(MarkerArray, '/sweep_viewpoints', 10)

        self._zones = self._load_zones(str(gp('zones_file').value), waypoints)
        self._cmap = self._load_map(str(gp('map_yaml').value))
        self._sweep_cfg = {
            'radius': float(gp('sweep_radius').value),
            'view_range': float(gp('sweep_view_range').value),
            'max_points': int(gp('sweep_max_points').value),
        }

        self._incidents = IncidentLog(str(gp('incident_dir').value))
        if self._incidents.directory:
            self.get_logger().info(f'incident log: {self._incidents.directory}')
        self._search_lead = float(gp('search_lead_secs').value)
        self._search_max = float(gp('search_max_distance').value)
        # Alert positions in the map frame when AMCL provides map -> odom
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # Latest camera image, saved as the alert snapshot
        self._bridge = CvBridge()
        self._last_image = None

        # Fed by /odom and the intruder track (or /target): the intruder in
        # the odom frame
        self._target = TargetEstimate(
            max_predict=float(gp('track_max_predict').value))

        self.create_subscription(Bool, '/estop', self._estop_cb, latched)
        if bool(gp('use_track').value):
            self.create_subscription(Odometry, '/intruder/track', self._track_cb, 10)
        else:
            self.create_subscription(Vector3Stamped, '/target', self._target_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        snapshot_topic = str(gp('snapshot_topic').value)
        if snapshot_topic and self._incidents.directory:
            self.create_subscription(
                Image, snapshot_topic, self._image_cb, qos_profile_sensor_data)

        for key, value in BLACKBOARD_DEFAULTS.items():
            bb_set(key, list(value) if isinstance(value, list) else value)

        # Blocks until AMCL and bt_navigator are active (AMCL gets its
        # initial pose from nav2_params.yaml, so no RViz click is needed).
        self._navigator = GuardNavigator()
        self.get_logger().info('waiting for Nav2...')
        self._navigator.waitUntilNav2Active()
        # The patrol schedule counts from the first patrol leg, not from the
        # simulation's start: on a slow machine Gazebo, YOLO and Nav2 bringup
        # took most of the first zone's slot, and the guard left it almost
        # unseen. The clock cannot be read here: under use_sim_time it
        # follows /clock only while this node is spun, and so far only the
        # navigator's node was. It still reads 0, and a schedule counted
        # from 0 starts wherever the world file's saved sim time (788 s in
        # the museum) falls in it.
        self._patrol_start: Optional[float] = None

        self._bt = build_security_guard_tree(
            self._navigator, self._vel_pub.publish,
            self._patrol_route, gains,
            target_timeout=float(gp('target_timeout').value),
            search_secs=float(gp('search_secs').value),
            search_speed=float(gp('search_angular_speed').value),
            dwell_secs=float(gp('waypoint_dwell_secs').value),
            sweep_secs=float(gp('sweep_secs').value),
            sweep_dwell_secs=float(gp('sweep_dwell_secs').value),
            log=self.get_logger().info, clock=self._now,
            incident_open=lambda: self._incidents.is_open,
            report=self._report_incident)
        self._bt.setup()
        self.get_logger().info(
            'behaviour tree:\n' + py_trees.display.unicode_tree(self._bt.root))

        self._metrics = {
            'intruder_detections': 0,
            'track_losses': 0,
            'restricted_entries': 0,
            'time_following_sec': 0.0,
            'distance_traveled_m': 0.0,
            'incidents': 0,
        }
        # Follow quality while IntruderProtocol runs (Phase 0 baseline):
        # sums of squared bearing and stand-off range errors.
        self._desired = gains.desired_distance
        self._follow_err = {'n': 0, 'bearing_sq': 0.0, 'ranged': 0, 'range_sq': 0.0}
        self._session_start = self.get_clock().now()
        self._robot_pose = None  # (x, y, yaw) in odom
        self._sightings = MarkerArray()
        self._state = ''
        self._publish_zones()

        self.create_timer(0.1, self._tick)
        self.create_timer(5.0, self._publish_metrics)

    # ------------------------------------------------------------------

    def _load_zones(self, path: str, waypoints) -> GuardZones:
        """Zones from ``path``; the ``waypoints`` parameter if unset or bad."""
        if path:
            try:
                zones = GuardZones.from_yaml(path)
                self.get_logger().info(
                    f'guard zones: {", ".join(zones.zones)}; restricted: '
                    f'{", ".join(zones.restricted) or "none"}')
                return zones
            except (OSError, ValueError, KeyError, TypeError) as exc:
                self.get_logger().error(
                    f'zones_file {path} not usable ({exc}): patrolling the '
                    'waypoints parameter')
        return GuardZones.single(waypoints)

    def _load_map(self, path: str) -> Optional[ClearanceMap]:
        """Load the saved map for room sweeps; None if unset or bad (no sweep)."""
        if not path:
            return None
        try:
            return ClearanceMap.from_yaml(path)
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(f'map_yaml {path} not usable ({exc}): no room sweep')
            return None

    def _estop_cb(self, msg: Bool):
        bb_set(BB_ESTOP, bool(msg.data))

    def _patrol_route(self):
        """(zone, waypoints) the schedule assigns now, timed from the first call.

        The tree asks when it starts a patrol leg, i.e. from a timer
        callback: the node is spun by then and its clock is the simulation's.
        """
        now = self._now()
        if self._patrol_start is None:
            self._patrol_start = now
        return self._zones.route_at(now - self._patrol_start)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, msg: Vector3Stamped):
        if self._target.update(stamp_to_sec(msg.header.stamp),
                               float(msg.vector.x), float(msg.vector.y)):
            bb_set(BB_LAST_SEEN, self._now())
            self._refresh_target()

    def _track_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        if self._target.update_track(stamp_to_sec(msg.header.stamp),
                                     p.x, p.y, v.x, v.y):
            bb_set(BB_LAST_SEEN, self._now())
            self._refresh_target()

    def _refresh_target(self):
        """Blackboard target = the estimate relative to the robot now."""
        now = self._now()
        rel = self._target.relative(t=now)
        if rel is None:
            return
        bearing, rng = rel
        bb_set(BB_TARGET_BEARING, bearing)
        bb_set(BB_TARGET_RANGE, rng if rng is not None else -1.0)
        bb_set(BB_TARGET_YAW_RATE, self._target.bearing_rate(t=now))
        bb_set(BB_SEARCH_GOAL, self._search_goal())

    def _search_goal(self) -> Optional[Tuple[float, float]]:
        """Odom point the track was heading for, or None if never ranged."""
        xy = self._target.target_xy
        if xy is None:
            return None
        vx, vy = self._target.velocity or (0.0, 0.0)
        dx, dy = vx * self._search_lead, vy * self._search_lead
        lead = math.hypot(dx, dy)
        if lead > self._search_max:
            dx, dy = dx * self._search_max / lead, dy * self._search_max / lead
        return (xy[0] + dx, xy[1] + dy)

    def _image_cb(self, msg):
        self._last_image = msg

    def _scan_cb(self, msg: LaserScan):
        bb_set(BB_FRONT_CLEAR,
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
                self._alert_sighting()
            elif self._state == 'IntruderProtocol' and state == 'SearchProtocol':
                self._metrics['track_losses'] += 1
                where, frame = self._to_map(self._target_or_robot_xy())
                self._publish_event(self._incidents.lost(self._now(), where, frame))
                self._plan_sweep()
            self._state = state
        self._state_pub.publish(String(data=state))

        if state == 'IntruderProtocol':
            self._metrics['time_following_sec'] += 0.1
            self._record_follow_error()
            self._check_restricted()

    # -- sweep and restricted areas ------------------------------------------

    def _plan_sweep(self):
        """Sweep viewpoints (map) around where the intruder was heading."""
        bb_set(BB_SWEEP, [])
        if self._cmap is None or self._robot_pose is None:
            return
        robot, frame = self._to_map(self._robot_pose[:2])
        last, _ = self._to_map(self._target.target_xy)
        goal, _ = self._to_map(bb_get(BB_SEARCH_GOAL, None))
        if frame != 'map':
            self.get_logger().info('sweep: no map -> odom yet, skipping it')
            return
        center = goal or last or robot
        seen = [p for p in (robot, last, goal) if p is not None]
        points = plan_sweep(self._cmap, center, goal or robot, seen=seen,
                            radius=self._sweep_cfg['radius'],
                            view_range=self._sweep_cfg['view_range'],
                            max_points=self._sweep_cfg['max_points'])
        bb_set(BB_SWEEP, points)
        self.get_logger().info('sweep: {} viewpoints around ({:.2f}, {:.2f})'.format(
            len(points), *center))
        self._publish_sweep(points)

    def _check_restricted(self):
        """Alert once per area and incident when the intruder is inside one."""
        if not self._zones.restricted or self._target.target_xy is None:
            return
        where, frame = self._to_map(self._target.target_xy)
        if frame != 'map':
            return  # the areas are drawn on the map
        for area in self._zones.areas_containing(*where):
            event = self._incidents.entered_area(self._now(), area, where, frame)
            if event is not None:
                self._metrics['restricted_entries'] += 1
                self._publish_event(event)

    # -- alerts ------------------------------------------------------------

    def _target_or_robot_xy(self) -> Optional[Tuple[float, float]]:
        """Intruder position (odom), or the robot's if it was never ranged."""
        return self._target.target_xy or (
            self._robot_pose[:2] if self._robot_pose else None)

    def _to_map(self, xy):
        """(xy, frame): ``xy`` (odom) in the map frame when TF has it."""
        if xy is None:
            return None, ''
        try:
            tf = self._tf_buffer.lookup_transform('map', 'odom', RosTime())
        except Exception:  # no AMCL yet, or no map (TransformException)
            return xy, 'odom'
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = yaw_from_quaternion(q)
        c, s = math.cos(yaw), math.sin(yaw)
        return (t.x + c * xy[0] - s * xy[1], t.y + s * xy[0] + c * xy[1]), 'map'

    def _alert_sighting(self):
        """Open an incident (intruder_detected) or add a sighting to it."""
        if not self._incidents.is_open:
            self._metrics['incidents'] += 1
        where, frame = self._to_map(self._target_or_robot_xy())
        event = self._incidents.sighted(
            self._now(), where, frame, snapshot=self._last_image is not None)
        if event.get('snapshot'):
            event['snapshot'] = self._save_snapshot(event['snapshot'])
        self._publish_event(event)

    def _save_snapshot(self, path: str) -> Optional[str]:
        """Write the latest camera image to ``path``; None if that fails."""
        try:
            import cv2  # only needed with an incident directory
            frame = self._bridge.imgmsg_to_cv2(self._last_image, 'bgr8')
            if not cv2.imwrite(path, frame):
                raise OSError('cv2.imwrite failed')
        except Exception as exc:  # a bad frame must not stop the guard
            self.get_logger().warn(f'snapshot {path} not saved: {exc}')
            return None
        return path

    def _report_incident(self):
        """ReportProtocol: close the incident once search and sweep gave up."""
        bb_set(BB_SWEEP, [])
        self._publish_sweep([])
        self._publish_event(self._incidents.close(self._now()))

    def _publish_event(self, event: Optional[dict]):
        if event is None:
            return
        text = json.dumps(event)
        self._event_pub.publish(String(data=text))
        self.get_logger().info(f'security event: {text}')

    def _record_follow_error(self):
        err = self._follow_err
        err['n'] += 1
        err['bearing_sq'] += bb_get(BB_TARGET_BEARING, 0.0) ** 2
        rng = bb_get(BB_TARGET_RANGE, -1.0)
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
        elapsed = self._session_elapsed()
        bearing_rms, range_rms = self._follow_rms()
        diag = DiagnosticArray()
        diag.header.stamp = self._stamp()
        status = DiagnosticStatus()
        status.name = 'SecurityGuardBT'
        status.hardware_id = 'autonav_sim'
        status.level = DiagnosticStatus.OK
        status.message = f'{self._state} | session {elapsed:.0f}s'
        values = {
            'state': self._state,
            'patrol_zone': str(bb_get(BB_PATROL_ZONE, '')),
            'waypoints_visited': str(bb_get(BB_WP_VISITS, 0)),
            'intruder_detections': str(self._metrics['intruder_detections']),
            'track_losses': str(self._metrics['track_losses']),
            'incidents': str(self._metrics['incidents']),
            'restricted_entries': str(self._metrics['restricted_entries']),
            'follow_bearing_rms_deg': f'{bearing_rms:.1f}',
            'follow_range_rms_m': f'{range_rms:.2f}',
            'time_following_sec': f"{self._metrics['time_following_sec']:.1f}",
            'distance_traveled_m': f"{self._metrics['distance_traveled_m']:.2f}",
            'session_elapsed_sec': f'{elapsed:.1f}',
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        diag.status = [status]
        self._diag_pub.publish(diag)

    def _session_elapsed(self) -> float:
        return (self.get_clock().now() - self._session_start).nanoseconds / 1e9

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_zones(self):
        """Restricted areas (red outlines) and patrol waypoints, latched."""
        self._zones_pub.publish(
            MarkerArray(markers=zone_markers(self._zones, self._stamp())))

    def _publish_sweep(self, points):
        """Sweep viewpoints as arrows (map frame); none clears them."""
        self._sweep_pub.publish(
            MarkerArray(markers=sweep_markers(points, self._stamp())))

    def _add_sighting_marker(self):
        """Mark where the intruder was seen (or the robot, if not ranged)."""
        where = self._target_or_robot_xy()
        if where is None:
            return
        self._sightings.markers += sighting_markers(
            len(self._sightings.markers) // 2, where, self._stamp(),
            self._session_elapsed())
        self._sighting_pub.publish(self._sightings)


def main(args=None):
    """Initialize and spin the SecurityGuardBTNode."""
    spin_node(SecurityGuardBTNode, args)


if __name__ == '__main__':
    main()
