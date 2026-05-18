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

"""Hybrid YOLO-detector + OpenCV-tracker for the person intruder.

YOLOv8-nano supplies a high-confidence person bounding box every
``redetect_every`` frames (default 10).  In between, the OpenCV CSRT
(or KCF fallback) tracker propagates the box frame-to-frame, giving:

  * stable identity across frames,
  * lower CPU than running YOLO every frame,
  * graceful re-acquisition if the person re-enters the scene.

Published topics
----------------
/person_bbox      std_msgs/Float32MultiArray  [x, y, w, h] in pixels, or empty
/person_track     geometry_msgs/PointStamped   bbox centroid (pixels)
/person_detected  std_msgs/Bool                True iff tracker has a lock
"""

from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

from my_bot.object_detector import postprocess, preprocess


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def select_person_box(detections: list,
                      min_conf: float = 0.4) -> Optional[Tuple[int, int, int, int]]:
    """Return the highest-confidence person box (x, y, w, h), or None.

    ``detections`` is the list of tuples produced by
    ``object_detector.postprocess`` — each entry is
    ``(x1, y1, x2, y2, class_id, score)``.  Only ``class_id == 0`` (person)
    is considered.
    """
    people = [d for d in detections
              if d[4] == 0 and d[5] >= min_conf]
    if not people:
        return None
    best = max(people, key=lambda d: d[5])
    x1, y1, x2, y2, _, _ = best
    return (int(x1), int(y1), int(x2 - x1), int(y2 - y1))


def box_iou(a: Tuple[int, int, int, int],
            b: Tuple[int, int, int, int]) -> float:
    """Intersection-over-union of two (x, y, w, h) boxes."""
    ax2, ay2 = a[0] + a[2], a[1] + a[3]
    bx2, by2 = b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def make_tracker():
    """Construct an OpenCV tracker, preferring CSRT (more accurate).

    Falls back to KCF on builds where the contrib trackers are missing.
    """
    for ctor in (
            getattr(cv2, 'TrackerCSRT_create', None),
            getattr(getattr(cv2, 'legacy', None), 'TrackerCSRT_create', None),
            getattr(cv2, 'TrackerKCF_create', None),
            getattr(getattr(cv2, 'legacy', None), 'TrackerKCF_create', None)):
        if ctor is not None:
            return ctor()
    raise RuntimeError(
        'No OpenCV tracker available — install opencv-contrib-python')


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PersonTrackerNode(Node):
    """YOLO-seeded CSRT tracker for the person class."""

    def __init__(self):
        super().__init__('person_tracker')

        self.declare_parameter('model_path', '/root/models/yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('nms_iou_threshold', 0.45)
        self.declare_parameter('redetect_every', 10)
        self.declare_parameter('reseed_iou_threshold', 0.3)

        self._conf = float(self.get_parameter('confidence_threshold').value)
        self._iou = float(self.get_parameter('nms_iou_threshold').value)
        self._redetect_every = int(self.get_parameter('redetect_every').value)
        self._reseed_iou = float(
            self.get_parameter('reseed_iou_threshold').value)
        model_path = str(self.get_parameter('model_path').value)

        self._bridge = CvBridge()
        self._session = None
        self._load_model(model_path)

        self._tracker = None
        self._last_box: Optional[Tuple[int, int, int, int]] = None
        self._frame_idx = 0

        self.create_subscription(
            Image, '/camera/image_raw', self._on_image,
            qos_profile_sensor_data)
        self._bbox_pub = self.create_publisher(
            Float32MultiArray, '/person_bbox', 10)
        self._track_pub = self.create_publisher(
            PointStamped, '/person_track', 10)
        self._detected_pub = self.create_publisher(
            Bool, '/person_detected', 10)

    # ------------------------------------------------------------------

    def _load_model(self, model_path: str) -> None:
        import os
        if not os.path.exists(model_path):
            self.get_logger().warn(
                f'YOLO model not found at {model_path} — tracker disabled')
            return
        try:
            import onnxruntime as ort
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self._session = ort.InferenceSession(model_path, providers=providers)
            self.get_logger().info(
                f'person_tracker: YOLO loaded ({self._session.get_providers()[0]})')
        except Exception as e:
            self.get_logger().warn(f'YOLO load failed: {e}')

    # ------------------------------------------------------------------

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().debug(f'cv_bridge: {e}')
            return

        self._frame_idx += 1
        need_detect = (
            self._tracker is None
            or self._frame_idx % self._redetect_every == 0)

        detected_box: Optional[Tuple[int, int, int, int]] = None
        if need_detect:
            detected_box = self._run_yolo(frame)

        if detected_box is not None:
            self._seed_or_correct_tracker(frame, detected_box)
            self._last_box = detected_box
        elif self._tracker is not None:
            ok, bb = self._tracker.update(frame)
            if ok:
                x, y, w, h = (int(v) for v in bb)
                self._last_box = (x, y, w, h)
            else:
                self._tracker = None
                self._last_box = None

        self._publish(msg, frame.shape)

    def _run_yolo(self, frame: np.ndarray
                  ) -> Optional[Tuple[int, int, int, int]]:
        if self._session is None:
            return None
        try:
            blob = preprocess(frame)
            inp = self._session.get_inputs()[0].name
            raw = self._session.run(None, {inp: blob})[0]
            dets = postprocess(raw, frame.shape, self._conf, self._iou)
        except Exception as e:
            self.get_logger().debug(f'YOLO inference: {e}')
            return None
        return select_person_box(dets, self._conf)

    def _seed_or_correct_tracker(self, frame, box) -> None:
        """Re-initialise tracker if it drifted (low IoU) or wasn't running."""
        if self._tracker is not None and self._last_box is not None:
            if box_iou(self._last_box, box) >= self._reseed_iou:
                # Track is consistent; no need to reseed.
                return
        try:
            self._tracker = make_tracker()
            self._tracker.init(frame, tuple(box))
        except Exception as e:
            self.get_logger().warn(f'tracker init failed: {e}')
            self._tracker = None

    def _publish(self, header_msg, frame_shape) -> None:
        has_lock = self._last_box is not None
        self._detected_pub.publish(Bool(data=has_lock))

        bbox_msg = Float32MultiArray()
        if has_lock:
            x, y, w, h = self._last_box
            bbox_msg.data = [float(x), float(y), float(w), float(h)]
        self._bbox_pub.publish(bbox_msg)

        if not has_lock:
            return
        x, y, w, h = self._last_box
        point = PointStamped()
        point.header = header_msg.header
        point.point.x = float(x + w / 2.0)
        point.point.y = float(y + h / 2.0)
        point.point.z = 0.0
        self._track_pub.publish(point)


def main(args=None):
    """Initialize and spin the PersonTrackerNode."""
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
