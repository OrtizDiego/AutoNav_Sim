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

"""Track the intruder in the odom frame: position, velocity, covariance.

Fuses sensor_fusion's stamped /target (camera box + lidar range) with
person-sized lidar clusters in a constant-velocity EKF (my_bot.track_filter).
Each measurement is placed with the robot pose at its own stamp (odometry
history), so the camera pipeline's latency does not bend the track. The
lidar keeps a confirmed track updated through short camera dropouts.

The odom frame, not map: it exists in every scenario (no AMCL needed), is
continuous, and drifts little over the seconds a track lives.

Subscribes
----------
/target   geometry_msgs/Vector3Stamped   x = bearing (NaN = none), y = range
/scan     sensor_msgs/LaserScan
/odom     nav_msgs/Odometry

Publishes
---------
/intruder/track      nav_msgs/Odometry      confirmed track after every
                                            accepted measurement: stamp =
                                            state time, frame odom, child
                                            'intruder' (axes = odom, so the
                                            twist is the odom-frame
                                            velocity); x/y covariances set
/intruder/predicted  geometry_msgs/PointStamped  where the track will be
                                            ``prediction_horizon`` s later
/intruder/state      std_msgs/String        none | tentative | confirmed |
                                            lost, at ``publish_rate``
/intruder/markers    visualization_msgs/MarkerArray  position, velocity
                                            arrow, 2-sigma ellipse
"""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from my_bot.target_estimate import PoseHistory, stamp_to_sec, yaw_from_quaternion
from my_bot.track_filter import CONFIRMED, IntruderTracker, scan_clusters, to_world

STATE_COLOURS = {'tentative': (1.0, 0.8, 0.0), CONFIRMED: (1.0, 0.1, 0.1)}


class TargetTrackerNode(Node):
    """Runs IntruderTracker on /target, /scan and /odom."""

    def __init__(self):
        super().__init__('target_tracker')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('accel_noise', 1.5)
        self.declare_parameter('bearing_sigma', 0.04)
        self.declare_parameter('range_sigma', 0.15)
        self.declare_parameter('range_sigma_rel', 0.05)
        self.declare_parameter('speed_sigma', 1.0)
        self.declare_parameter('confirm_hits', 3)
        self.declare_parameter('tentative_timeout', 0.5)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('max_outliers', 3)
        self.declare_parameter('max_lag', 1.0)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('lidar_sigma', 0.12)
        self.declare_parameter('lidar_coast_secs', 1.5)
        self.declare_parameter('lidar_offset_x', -0.064)
        self.declare_parameter('cluster_range_max', 8.0)
        self.declare_parameter('cluster_jump', 0.1)
        self.declare_parameter('cluster_jump_rel', 0.03)
        self.declare_parameter('cluster_merge_distance', 0.5)
        self.declare_parameter('cluster_max_width', 0.8)
        self.declare_parameter('prediction_horizon', 1.0)
        self.declare_parameter('publish_rate', 10.0)

        def gp(name):
            return self.get_parameter(name).value

        self._frame = str(gp('frame_id'))
        self._tracker = IntruderTracker(
            accel_noise=float(gp('accel_noise')),
            bearing_sigma=float(gp('bearing_sigma')),
            range_sigma=float(gp('range_sigma')),
            range_sigma_rel=float(gp('range_sigma_rel')),
            lidar_sigma=float(gp('lidar_sigma')),
            speed_sigma=float(gp('speed_sigma')),
            confirm_hits=int(gp('confirm_hits')),
            tentative_timeout=float(gp('tentative_timeout')),
            lost_timeout=float(gp('lost_timeout')),
            lidar_coast_secs=float(gp('lidar_coast_secs')),
            max_outliers=int(gp('max_outliers')),
            max_lag=float(gp('max_lag')))
        self._use_lidar = bool(gp('use_lidar'))
        self._lidar_x = float(gp('lidar_offset_x'))
        self._cluster_args = {
            'range_max': float(gp('cluster_range_max')),
            'jump': float(gp('cluster_jump')),
            'jump_rel': float(gp('cluster_jump_rel')),
            'merge_distance': float(gp('cluster_merge_distance')),
            'max_width': float(gp('cluster_max_width')),
        }
        self._horizon = float(gp('prediction_horizon'))
        self._poses = PoseHistory()
        self._markers_shown = False

        self._track_pub = self.create_publisher(Odometry, '/intruder/track', 10)
        self._pred_pub = self.create_publisher(PointStamped, '/intruder/predicted', 10)
        self._state_pub = self.create_publisher(String, '/intruder/state', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/intruder/markers', 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Vector3Stamped, '/target', self._target_cb, 10)
        if self._use_lidar:
            self.create_subscription(
                LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / float(gp('publish_rate')), self._tick)

    # ------------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self, msg) -> float:
        """Return a message's stamp in seconds; now if it carries none."""
        t = stamp_to_sec(msg.header.stamp)
        return self._now() if t is None else t

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._poses.add_pose(stamp_to_sec(msg.header.stamp), p.x, p.y,
                             yaw_from_quaternion(msg.pose.pose.orientation))

    def _target_cb(self, msg: Vector3Stamped):
        t = self._stamp(msg)
        pose = self._poses.pose_at(t)
        if pose is None:
            return  # no odometry yet: nowhere to place the target
        if self._tracker.camera(t, pose, float(msg.vector.x), float(msg.vector.y)):
            self._publish_track()

    def _scan_cb(self, msg: LaserScan):
        if not self._tracker.confirmed:
            return  # clusters only refine a confirmed track
        t = self._stamp(msg)
        pose = self._poses.pose_at(t)
        if pose is None:
            return
        clusters = scan_clusters(msg.ranges, msg.angle_min, msg.angle_increment,
                                 range_min=float(msg.range_min), **self._cluster_args)
        if self._tracker.lidar(t, to_world(clusters, pose, self._lidar_x)):
            self._publish_track()

    def _tick(self):
        self._tracker.expire(self._now())
        self._state_pub.publish(String(data=self._tracker.state))
        self._publish_markers()

    # ------------------------------------------------------------------

    def _publish_track(self):
        if not self._tracker.confirmed:
            return
        f = self._tracker.filter
        stamp = _to_stamp(f.t)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame
        msg.child_frame_id = 'intruder'
        msg.pose.pose.position.x, msg.pose.pose.position.y = f.position
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = f.velocity
        msg.pose.covariance = _cov6(f.P[:2, :2])
        msg.twist.covariance = _cov6(f.P[2:, 2:])
        self._track_pub.publish(msg)

        pred = PointStamped()
        pred.header.stamp = stamp
        pred.header.frame_id = self._frame
        pred.point.x, pred.point.y = f.position_at(f.t + self._horizon)
        self._pred_pub.publish(pred)

    def _publish_markers(self):
        f = self._tracker.filter
        markers = MarkerArray()
        if f is None:
            if self._markers_shown:  # clear them once
                gone = Marker()
                gone.action = Marker.DELETEALL
                markers.markers.append(gone)
                self._marker_pub.publish(markers)
                self._markers_shown = False
            return
        self._markers_shown = True
        colour = STATE_COLOURS.get(self._tracker.state, (0.5, 0.5, 0.5))
        x, y = f.position
        vx, vy = f.velocity
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame)

        def marker(mid, mtype):
            m = Marker()
            m.header = header
            m.ns = 'intruder'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b = colour
            m.color.a = 0.9
            return m

        body = marker(0, Marker.CYLINDER)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.9
        body.scale.x = body.scale.y = 0.35
        body.scale.z = 1.8

        # 2-sigma position ellipse, flat on the floor
        vals, vecs = np.linalg.eigh(f.P[:2, :2])
        ellipse = marker(1, Marker.CYLINDER)
        ellipse.pose.position.x, ellipse.pose.position.y = x, y
        ellipse.pose.position.z = 0.01
        yaw = math.atan2(vecs[1, 1], vecs[0, 1])
        ellipse.pose.orientation.z = math.sin(yaw / 2.0)
        ellipse.pose.orientation.w = math.cos(yaw / 2.0)
        ellipse.scale.x = 4.0 * math.sqrt(max(vals[1], 1e-6))
        ellipse.scale.y = 4.0 * math.sqrt(max(vals[0], 1e-6))
        ellipse.scale.z = 0.02
        ellipse.color.a = 0.3

        arrow = marker(2, Marker.ARROW)
        arrow.points = [Point(x=x, y=y, z=0.1),
                        Point(x=x + vx * self._horizon, y=y + vy * self._horizon, z=0.1)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.12, 0.15

        markers.markers = [body, ellipse, arrow]
        self._marker_pub.publish(markers)


def _to_stamp(t: float) -> Time:
    """builtin_interfaces/Time of ``t`` seconds."""
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _cov6(cov2) -> list:
    """6x6 row-major covariance with the 2x2 block in its x/y corner."""
    out = [0.0] * 36
    out[0], out[1] = float(cov2[0, 0]), float(cov2[0, 1])
    out[6], out[7] = float(cov2[1, 0]), float(cov2[1, 1])
    return out


def main(args=None):
    """Initialize and spin the TargetTrackerNode."""
    rclpy.init(args=args)
    node = TargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Ctrl+C: rclpy has already shut the context down
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
