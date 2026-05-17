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

"""YOLOv8-nano ONNX object detector node.

Subscribes to /camera/image_raw, runs inference, publishes:
  /detections       (visualization_msgs/MarkerArray) — one marker per detection
  /person_detected  (std_msgs/Bool) — True when class 0 (person) seen
  /target_detected  (std_msgs/Bool) — True when any target class seen

GPU acceleration via onnxruntime-gpu (CUDAExecutionProvider) with automatic
CPU fallback when CUDA is unavailable.

Module-level functions (preprocess, postprocess) are importable for unit tests
without instantiating a ROS node.
"""

import os
import threading

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# COCO class index → label (subset used here)
# ---------------------------------------------------------------------------
_COCO_NAMES = {
    0: 'person',
    32: 'sports ball',
}

# YOLOv8n input size
_INPUT_SIZE = 640


# ---------------------------------------------------------------------------
# Pure functions — importable by unit tests without ROS runtime
# ---------------------------------------------------------------------------

def preprocess(frame: np.ndarray) -> np.ndarray:
    """Resize (letterbox) and normalise a BGR frame for YOLOv8 inference.

    Returns float32 array of shape [1, 3, 640, 640] with values in [0, 1].
    """
    h, w = frame.shape[:2]
    scale = _INPUT_SIZE / max(h, w)
    new_h, new_w = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(frame, (new_w, new_h))
    # Pad to square
    canvas = np.zeros((_INPUT_SIZE, _INPUT_SIZE, 3), dtype=np.uint8)
    canvas[:new_h, :new_w] = resized
    # BGR → RGB, HWC → CHW, normalise
    rgb = canvas[:, :, ::-1].astype(np.float32) / 255.0
    chw = rgb.transpose(2, 0, 1)
    return chw[np.newaxis]  # [1, 3, 640, 640]


def postprocess(
        output: np.ndarray,
        orig_shape: tuple,
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.45,
) -> list:
    """Decode YOLOv8 raw output into a list of (x1, y1, x2, y2, class_id, score).

    output shape: [1, 84, 8400]
    Returns list of tuples (x1, y1, x2, y2, class_id, confidence) in
    original image coordinates.
    """
    if output.ndim == 3:
        output = output[0]  # [84, 8400]
    preds = output.T  # [8400, 84]

    # Extract class scores
    class_scores = preds[:, 4:]  # [8400, 80]
    class_ids = np.argmax(class_scores, axis=1)
    confidences = class_scores[np.arange(len(class_ids)), class_ids]

    # Filter by confidence
    mask = confidences >= conf_threshold
    if not np.any(mask):
        return []

    boxes_xywh = preds[mask, :4]  # cx, cy, w, h (normalised to 640)
    confs = confidences[mask]
    cls_ids = class_ids[mask]

    # Scale boxes from 640 back to original image size
    orig_h, orig_w = orig_shape[:2]
    scale_x = orig_w / _INPUT_SIZE
    scale_y = orig_h / _INPUT_SIZE
    cx = boxes_xywh[:, 0] * scale_x
    cy = boxes_xywh[:, 1] * scale_y
    bw = boxes_xywh[:, 2] * scale_x
    bh = boxes_xywh[:, 3] * scale_y
    x1 = cx - bw / 2
    y1 = cy - bh / 2
    x2 = cx + bw / 2
    y2 = cy + bh / 2

    # NMS per class
    results = []
    for unique_cls in np.unique(cls_ids):
        cls_mask = cls_ids == unique_cls
        cls_boxes = np.column_stack([x1[cls_mask], y1[cls_mask],
                                     x2[cls_mask] - x1[cls_mask],
                                     y2[cls_mask] - y1[cls_mask]])
        cls_confs = confs[cls_mask].tolist()
        indices = cv2.dnn.NMSBoxes(
            cls_boxes.tolist(), cls_confs, conf_threshold, iou_threshold)
        if len(indices) == 0:
            continue
        for i in np.array(indices).flatten():
            results.append((
                float(x1[cls_mask][i]),
                float(y1[cls_mask][i]),
                float(x2[cls_mask][i]),
                float(y2[cls_mask][i]),
                int(unique_cls),
                float(cls_confs[i]),
            ))
    return results


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------

class ObjectDetectorNode(Node):
    """YOLOv8-nano inference node with GPU/CPU auto-selection."""

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('model_path', '/root/models/yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_iou_threshold', 0.45)
        self.declare_parameter('target_classes', [0, 32])

        self._conf = float(self.get_parameter('confidence_threshold').value)
        self._iou = float(self.get_parameter('nms_iou_threshold').value)
        self._target_classes = list(self.get_parameter('target_classes').value)
        model_path = str(self.get_parameter('model_path').value)

        self._session = None
        self._load_model(model_path)

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._cam_sub = self.create_subscription(
            Image, '/camera/image_raw', self._cam_cb, 10)

        self._markers_pub = self.create_publisher(MarkerArray, '/detections', 10)
        self._person_pub = self.create_publisher(Bool, '/person_detected', 10)
        self._target_pub = self.create_publisher(Bool, '/target_detected', 10)

    def _load_model(self, model_path: str):
        """Load ONNX model, prefer GPU provider, fall back to CPU."""
        if not os.path.exists(model_path):
            self.get_logger().warn(
                f'Model not found at {model_path} — detector disabled, '
                'HSV fallback active')
            return
        try:
            import onnxruntime as ort
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self._session = ort.InferenceSession(model_path, providers=providers)
            active = self._session.get_providers()[0]
            self.get_logger().info(
                f'ONNX model loaded from {model_path} using {active}')
        except Exception as e:
            self.get_logger().warn(
                f'Failed to load ONNX model: {e} — detector disabled')

    def _cam_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().debug(f'imgmsg_to_cv2 error: {e}')
            return

        if self._session is None:
            self._publish_flags(False, False)
            return

        try:
            blob = preprocess(frame)
            input_name = self._session.get_inputs()[0].name
            raw = self._session.run(None, {input_name: blob})[0]
            detections = postprocess(raw, frame.shape, self._conf, self._iou)
        except Exception as e:
            self.get_logger().debug(f'Inference error: {e}')
            return

        filtered = [d for d in detections if d[4] in self._target_classes]
        person_seen = any(d[4] == 0 for d in filtered)
        target_seen = len(filtered) > 0

        self._publish_markers(filtered, msg.header)
        self._publish_flags(person_seen, target_seen)

    def _publish_markers(self, detections: list, header):
        arr = MarkerArray()
        for i, (x1, y1, x2, y2, cls_id, score) in enumerate(detections):
            m = Marker()
            m.header = header
            m.ns = 'detections'
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float((x1 + x2) / 2)
            m.pose.position.y = float((y1 + y2) / 2)
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 1.0
            label = _COCO_NAMES.get(cls_id, str(cls_id))
            m.text = f'{label} {score:.2f}'
            arr.markers.append(m)
        self._markers_pub.publish(arr)

    def _publish_flags(self, person: bool, target: bool):
        self._person_pub.publish(Bool(data=person))
        self._target_pub.publish(Bool(data=target))


def main(args=None):
    """Initialize and spin the ObjectDetectorNode."""
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
