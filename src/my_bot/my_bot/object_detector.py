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

"""YOLOv8-nano pre/post-processing shared by person_tracker.

person_tracker runs the ONNX model (CUDA if available, else CPU); this
module holds the pure parts so unit tests can exercise them without ROS or
a model file.

The network's input side is a parameter: the Docker image exports the
model at 320 px (a quarter of the 640 px work, matching the 320x240
camera), and person_tracker reads the size from the model, so a model
exported at 640 keeps working.
"""

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# COCO class index → label (subset used here)
# ---------------------------------------------------------------------------
_COCO_NAMES = {
    0: 'person',
    32: 'sports ball',
}

# Ultralytics' default export size; person_tracker passes the model's own.
DEFAULT_INPUT_SIZE = 640


# ---------------------------------------------------------------------------
# Pure functions — importable by unit tests without ROS runtime
# ---------------------------------------------------------------------------

def preprocess(frame: np.ndarray,
               input_size: int = DEFAULT_INPUT_SIZE) -> np.ndarray:
    """Resize (letterbox) and normalise a BGR frame for YOLOv8 inference.

    Returns float32 array of shape [1, 3, input_size, input_size] with
    values in [0, 1].
    """
    h, w = frame.shape[:2]
    scale = input_size / max(h, w)
    new_h, new_w = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(frame, (new_w, new_h))
    # Pad to square
    canvas = np.zeros((input_size, input_size, 3), dtype=np.uint8)
    canvas[:new_h, :new_w] = resized
    # BGR → RGB, HWC → CHW, normalise
    rgb = canvas[:, :, ::-1].astype(np.float32) / 255.0
    chw = rgb.transpose(2, 0, 1)
    return chw[np.newaxis]  # [1, 3, input_size, input_size]


def postprocess(
        output: np.ndarray,
        orig_shape: tuple,
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.45,
        input_size: int = DEFAULT_INPUT_SIZE,
) -> list:
    """Decode YOLOv8 raw output into a list of (x1, y1, x2, y2, class_id, score).

    output shape: [1, 84, N] (N = 8400 at 640 px input, 2100 at 320 px)
    Returns list of tuples (x1, y1, x2, y2, class_id, confidence) in
    original image coordinates. ``input_size`` must be the one
    ``preprocess`` used.
    """
    if output.ndim == 3:
        output = output[0]  # [84, N]
    preds = output.T  # [N, 84]

    # Extract class scores
    class_scores = preds[:, 4:]  # [N, 80]
    class_ids = np.argmax(class_scores, axis=1)
    confidences = class_scores[np.arange(len(class_ids)), class_ids]

    # Filter by confidence
    mask = confidences >= conf_threshold
    if not np.any(mask):
        return []

    boxes_xywh = preds[mask, :4]  # cx, cy, w, h in input pixels
    confs = confidences[mask]
    cls_ids = class_ids[mask]

    # Undo the uniform letterbox scale from preprocess(). Padding is added
    # bottom/right only, so no offset needs subtracting.
    orig_h, orig_w = orig_shape[:2]
    scale = max(orig_h, orig_w) / input_size
    cx = boxes_xywh[:, 0] * scale
    cy = boxes_xywh[:, 1] * scale
    bw = boxes_xywh[:, 2] * scale
    bh = boxes_xywh[:, 3] * scale
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
