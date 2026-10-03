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

"""Fuse a camera detection with the LiDAR scan into a range and bearing.

The camera says *where* the target is (a box in the image); the lidar says
*how far* it is. Every behaviour (ball chaser, security guard BT) consumes
the fused output, so they do not care which detector produced the box.

Modes (parameter ``mode``)
--------------------------
hsv     Largest red blob in /camera/image_raw (the ball in ball-sim).
person  Box from person_tracker on /person_bbox (YOLO + tracker, a
        PolygonStamped stamped with the camera image's time).

Ranging
-------
The lidar sits ~0.12 m above the ground. The range is a low percentile over
every beam inside the box's angular span, which finds the target's near
surface even when single beams slip past it (between a person's legs). In
person mode it is cross-checked against a monocular estimate from the box
(person height, or the feet's ground contact when the head is cut off); if
the lidar disagrees (beams hit the background) the monocular value wins.

Sign convention: image columns grow to the right, ROS angles grow to the
left, so bearings are negated (REP-103, same as LaserScan and angular.z).

Timing: every output carries the camera image's timestamp, and the scan used
for ranging is the one closest to it, so the followers can compensate the
detection pipeline's latency with odometry (my_bot.target_estimate).

Publishes
---------
/target            geometry_msgs/Vector3Stamped vector.x = bearing (rad,
                                                positive left, NaN if no
                                                target), vector.y = range (m,
                                                -1.0 if unknown); stamp = image
                                                capture time. What followers
                                                use.
/target_range      std_msgs/Float32             metres, -1.0 if unknown
/target_bearing    std_msgs/Float32             rad, positive left; NaN if
                                                no target
/target_position   geometry_msgs/PointStamped   base_link, when ranged
/sensor_fusion/image  sensor_msgs/Image         annotated debug view
"""

from collections import deque
import math
import threading
from typing import List, Optional, Tuple

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PolygonStamped, Vector3Stamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Header

from my_bot.target_estimate import stamp_to_sec

Box = Tuple[float, float, float, float]


# ---------------------------------------------------------------------------
# Pure functions — module-level so unit tests can import them without ROS
# ---------------------------------------------------------------------------

def compute_bearing(pixel_x: float, cx: float, fx: float) -> float:
    """Return bearing (rad) of a pixel column relative to camera optical axis.

    Positive bearing → target is to the robot's left (ROS REP-103), i.e.
    pixels left of the image centre give positive angles, matching the
    LaserScan angle convention.

    Args:
        pixel_x: Horizontal pixel coordinate of the target centroid.
        cx:      Image optical centre (pixels).  Typically image_width / 2.
        fx:      Focal length in pixels.  fx = (width/2) / tan(FOV_h/2).
    """
    return math.atan2(cx - pixel_x, fx)


def bearing_to_scan_index(bearing: float,
                          angle_min: float,
                          angle_increment: float,
                          n_samples: int) -> int:
    """Map a bearing angle to the nearest LaserScan array index.

    Args:
        bearing:          Bearing from ``compute_bearing`` (rad).
        angle_min:        scan.angle_min from the LaserScan message.
        angle_increment:  scan.angle_increment from the LaserScan message.
        n_samples:        Total number of range samples in the scan.
    """
    idx = round((bearing - angle_min) / angle_increment)
    return max(0, min(n_samples - 1, idx))


def validate_range(r: float, r_min: float, r_max: float) -> bool:
    """Return True if *r* is a valid, in-range lidar measurement."""
    return math.isfinite(r) and r_min <= r <= r_max


def scan_window_range(ranges: List[float], angle_min: float,
                      angle_increment: float, left_angle: float,
                      right_angle: float, range_min: float, range_max: float,
                      percentile: float = 0.2) -> Optional[float]:
    """Low-percentile range of valid beams between two angles.

    ``left_angle`` >= ``right_angle`` (ROS angles). Returns None if no beam
    in the window has a valid return.
    """
    n = len(ranges)
    if n == 0:
        return None
    lo = int(math.floor((right_angle - angle_min) / angle_increment))
    hi = int(math.ceil((left_angle - angle_min) / angle_increment))
    vals = []
    for i in range(lo, hi + 1):
        r = ranges[i % n]  # wrap across the +/-pi seam
        if validate_range(r, range_min, range_max):
            vals.append(r)
    if not vals:
        return None
    vals.sort()
    return vals[min(len(vals) - 1, int(percentile * len(vals)))]


def monocular_range(bbox: Box, img_h: int, fy: float, cy: float,
                    person_height: float, camera_height: float,
                    edge_margin: float = 3.0) -> Optional[float]:
    """Estimate distance to a person from the bbox alone.

    Uses the apparent height when head and feet are both in frame,
    otherwise the feet's ground contact point (needs the feet visible).
    """
    _, y, _, h = bbox
    top_cut = y <= edge_margin
    bottom_cut = y + h >= img_h - edge_margin
    if not top_cut and not bottom_cut and h > 1.0:
        return fy * person_height / h
    if not bottom_cut:
        below = (y + h) - cy  # pixels below the horizon
        if below > 1.0:
            return fy * camera_height / below
    return None


def fuse_range(lidar: Optional[float], mono: Optional[float],
               tolerance: float = 0.4) -> Optional[float]:
    """Prefer lidar unless it disagrees with the monocular estimate."""
    if lidar is None:
        return mono
    if mono is None:
        return lidar
    if abs(lidar - mono) <= tolerance * mono:
        return lidar
    return mono


def polygon_to_box(points) -> Optional[Box]:
    """(x, y, w, h) bounding box of polygon points, or None if empty."""
    if not points:
        return None
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)


def nearest_scan(scans, t: Optional[float]):
    """Return the scan stamped closest to ``t`` (the newest if unknown)."""
    if not scans:
        return None
    if t is None:
        return scans[-1]
    best, best_dt = scans[-1], math.inf
    for scan in scans:
        ts = stamp_to_sec(scan.header.stamp)
        if ts is not None and abs(ts - t) < best_dt:
            best, best_dt = scan, abs(ts - t)
    return best


def detect_red_blob(bgr: np.ndarray, lower1, upper1, lower2, upper2,
                    min_area: float) -> Optional[Box]:
    """Bounding box (x, y, w, h) of the largest red blob, or None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not valid:
        return None
    x, y, w, h = cv2.boundingRect(max(valid, key=cv2.contourArea))
    return float(x), float(y), float(w), float(h)


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class SensorFusionNode(Node):
    """Turns an image-space box into range + bearing using the lidar."""

    def __init__(self):
        """Declare parameters, create sub/pub."""
        super().__init__('sensor_fusion')

        self.declare_parameter('mode', 'hsv')
        self.declare_parameter('hsv_red_lower1', [0, 100, 100])
        self.declare_parameter('hsv_red_upper1', [10, 255, 255])
        self.declare_parameter('hsv_red_lower2', [160, 100, 100])
        self.declare_parameter('hsv_red_upper2', [180, 255, 255])
        self.declare_parameter('min_contour_area', 300.0)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 12.0)
        # Camera model (camera.xacro: 640x480, horizontal FOV 1.089 rad)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('horizontal_fov', 1.089)
        self.declare_parameter('camera_height', 0.103)
        self.declare_parameter('person_height', 1.72)

        gp = self.get_parameter
        self._mode = str(gp('mode').value)
        if self._mode not in ('hsv', 'person'):
            raise ValueError(f"mode must be 'hsv' or 'person', got {self._mode!r}")
        self._lower1 = np.array(gp('hsv_red_lower1').value, dtype=np.uint8)
        self._upper1 = np.array(gp('hsv_red_upper1').value, dtype=np.uint8)
        self._lower2 = np.array(gp('hsv_red_lower2').value, dtype=np.uint8)
        self._upper2 = np.array(gp('hsv_red_upper2').value, dtype=np.uint8)
        self._min_area = float(gp('min_contour_area').value)
        self._range_min = float(gp('range_min').value)
        self._range_max = float(gp('range_max').value)
        width = int(gp('image_width').value)
        self._img_h = int(gp('image_height').value)
        self._cx = width / 2.0
        self._cy = self._img_h / 2.0
        self._fx = (width / 2.0) / math.tan(float(gp('horizontal_fov').value) / 2.0)
        self._cam_h = float(gp('camera_height').value)
        self._person_h = float(gp('person_height').value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        # ~1 s of 10 Hz scans: the image a box came from is older than the
        # newest scan by the detection latency.
        self._scans = deque(maxlen=10)
        self._frame = None  # latest camera frame, person mode debug view

        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/image_raw', self._camera_cb, qos_profile_sensor_data)
        if self._mode == 'person':
            self.create_subscription(
                PolygonStamped, '/person_bbox', self._bbox_cb, 10)

        self._target_pub = self.create_publisher(Vector3Stamped, '/target', 10)
        self._pos_pub = self.create_publisher(PointStamped, '/target_position', 10)
        self._range_pub = self.create_publisher(Float32, '/target_range', 10)
        self._bearing_pub = self.create_publisher(Float32, '/target_bearing', 10)
        self._debug_pub = self.create_publisher(Image, '/sensor_fusion/image', 1)
        self.get_logger().info(f'sensor_fusion running in {self._mode} mode')

    # ------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scans.append(msg)

    def _camera_cb(self, msg: Image):
        if self._mode == 'person' and not self._want_debug():
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Image conversion failed: {e}')
            return
        if self._mode == 'person':
            with self._lock:
                self._frame = frame
            return
        box = detect_red_blob(frame, self._lower1, self._upper1,
                              self._lower2, self._upper2, self._min_area)
        self._fuse(box, msg.header, frame, 'ball')

    def _bbox_cb(self, msg: PolygonStamped):
        box = polygon_to_box(msg.polygon.points)
        with self._lock:
            frame = self._frame
        stamp = msg.header.stamp
        if stamp_to_sec(stamp) is None:  # unstamped box: best guess is now
            stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='camera_link_optical')
        self._fuse(box, header, frame, 'person')

    # ------------------------------------------------------------------

    def _fuse(self, box: Optional[Box], header, frame, label: str):
        bearing = float('nan')
        rng = None
        source = ''
        if box is not None:
            x, _, w, _ = box
            bearing = compute_bearing(x + w / 2.0, self._cx, self._fx)
            rng, source = self._range_for(box, stamp_to_sec(header.stamp))

        target = Vector3Stamped()
        target.header.stamp = header.stamp
        target.header.frame_id = 'base_link'
        target.vector.x = bearing
        target.vector.y = float(rng) if rng else -1.0
        target.vector.z = 0.0
        self._target_pub.publish(target)
        self._bearing_pub.publish(Float32(data=bearing))
        self._range_pub.publish(Float32(data=float(rng) if rng else -1.0))
        if rng:
            pos = PointStamped()
            pos.header.stamp = header.stamp
            pos.header.frame_id = 'base_link'
            pos.point.x = rng * math.cos(bearing)
            pos.point.y = rng * math.sin(bearing)
            self._pos_pub.publish(pos)

        if frame is not None and self._want_debug():
            self._publish_debug(frame, header, box, rng, bearing, source, label)

    def _range_for(self, box: Box, t: Optional[float] = None
                   ) -> Tuple[Optional[float], str]:
        """Fused range to the boxed target and which sensor produced it.

        ``t`` is the image's capture time; the scan closest to it is used.
        """
        x, _, w, _ = box
        with self._lock:
            scan = nearest_scan(self._scans, t)
        lidar = None
        if scan is not None:
            margin = math.radians(1.0)
            lidar = scan_window_range(
                scan.ranges, scan.angle_min, scan.angle_increment,
                compute_bearing(x, self._cx, self._fx) - margin,
                compute_bearing(x + w, self._cx, self._fx) + margin,
                self._range_min, self._range_max)
        if self._mode != 'person':
            return lidar, 'lidar' if lidar else ''
        mono = monocular_range(box, self._img_h, self._fx, self._cy,
                               self._person_h, self._cam_h)
        rng = fuse_range(lidar, mono)
        if rng is None:
            return None, ''
        return rng, 'lidar' if rng == lidar else 'camera'

    # ------------------------------------------------------------------

    def _want_debug(self) -> bool:
        return self._debug_pub.get_subscription_count() > 0

    def _publish_debug(self, frame, header, box, rng, bearing, source, label):
        vis = frame.copy()
        if box is None:
            cv2.putText(vis, f'no {label}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            x, y, w, h = (int(v) for v in box)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 0), 2)
            text = f'{label} {rng:.2f} m ({source})' if rng else f'{label} (no range)'
            cv2.putText(vis, text, (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
            cv2.putText(vis, f'bearing {math.degrees(bearing):+.1f} deg',
                        (10, vis.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out = self._bridge.cv2_to_imgmsg(vis, 'bgr8')
        out.header = header
        self._debug_pub.publish(out)


def main(args=None):
    """Initialize and spin the SensorFusionNode."""
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
