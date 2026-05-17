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

"""Unit tests for object_detector.py pure functions.

No ROS runtime or ONNX model required — only tests preprocess() and
postprocess() which depend only on numpy and OpenCV.
"""

import sys
import types

# ---------------------------------------------------------------------------
# Stub ROS modules so the import succeeds without ROS runtime
# ---------------------------------------------------------------------------

_stub_mods = [
    'rclpy', 'rclpy.node',
    'cv_bridge',
    'sensor_msgs', 'sensor_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'visualization_msgs', 'visualization_msgs.msg',
]
for _mod in _stub_mods:
    if _mod not in sys.modules:
        sys.modules[_mod] = types.ModuleType(_mod)

_rclpy = sys.modules['rclpy']
_rclpy.init = lambda **kw: None
_rclpy.spin = lambda n: None
_rclpy.shutdown = lambda: None


class _FakeNode:
    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                     debug=lambda *a: None)

    def declare_parameter(self, *a, **kw):
        pass

    def get_parameter(self, name):
        defaults = {
            'model_path': '/nonexistent/model.onnx',
            'confidence_threshold': 0.5,
            'nms_iou_threshold': 0.45,
            'target_classes': [0, 32],
        }
        return types.SimpleNamespace(value=defaults.get(name, 0))

    def create_subscription(self, *a, **kw):
        return None

    def create_publisher(self, *a, **kw):
        return types.SimpleNamespace(publish=lambda *a: None)


sys.modules['rclpy.node'].Node = _FakeNode

_bridge_mod = sys.modules['cv_bridge']


class _FakeCvBridge:
    def imgmsg_to_cv2(self, *a, **kw):
        return None


_bridge_mod.CvBridge = _FakeCvBridge

_sen_msg = sys.modules['sensor_msgs.msg']


class _FakeImage:
    pass


_sen_msg.Image = _FakeImage

_std_msg = sys.modules['std_msgs.msg']


class _FakeBool:
    def __init__(self, data=False):
        self.data = data


_std_msg.Bool = _FakeBool


class _FakeMarker:
    TEXT_VIEW_FACING = 9
    ADD = 0

    def __init__(self):
        self.header = None
        self.ns = ''
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.scale = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.color = types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=1.0)
        self.text = ''


class _FakeMarkerArray:
    def __init__(self):
        self.markers = []


_viz_msg = sys.modules['visualization_msgs.msg']
_viz_msg.Marker = _FakeMarker
_viz_msg.MarkerArray = _FakeMarkerArray

# ---------------------------------------------------------------------------
# Now import the module under test
# ---------------------------------------------------------------------------

import numpy as np  # noqa: E402

sys.path.insert(0, 'src/my_bot')
from my_bot.object_detector import preprocess, postprocess, _INPUT_SIZE  # noqa: E402


# ---------------------------------------------------------------------------
# preprocess() tests
# ---------------------------------------------------------------------------

class TestPreprocess:

    def test_output_shape(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        blob = preprocess(frame)
        assert blob.shape == (1, 3, _INPUT_SIZE, _INPUT_SIZE)

    def test_output_dtype(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        blob = preprocess(frame)
        assert blob.dtype == np.float32

    def test_white_pixel_normalised_to_one(self):
        frame = np.full((640, 640, 3), 255, dtype=np.uint8)
        blob = preprocess(frame)
        # First pixel, first channel should be 1.0 for a white frame
        assert float(blob[0, 0, 0, 0]) == pytest.approx(1.0, abs=1e-3)

    def test_black_pixel_normalised_to_zero(self):
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        blob = preprocess(frame)
        assert float(blob.max()) == pytest.approx(0.0, abs=1e-6)

    def test_square_input_no_padding(self):
        frame = np.full((640, 640, 3), 128, dtype=np.uint8)
        blob = preprocess(frame)
        # No padding rows — all values should be equal
        expected = 128.0 / 255.0
        assert float(blob.mean()) == pytest.approx(expected, abs=0.01)

    def test_portrait_input_letterboxed(self):
        # Tall image (480x240) — should still produce 640x640 output
        frame = np.zeros((480, 240, 3), dtype=np.uint8)
        blob = preprocess(frame)
        assert blob.shape == (1, 3, _INPUT_SIZE, _INPUT_SIZE)

    def test_values_in_unit_range(self):
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        blob = preprocess(frame)
        assert float(blob.min()) >= 0.0
        assert float(blob.max()) <= 1.0


# ---------------------------------------------------------------------------
# postprocess() tests
# ---------------------------------------------------------------------------

import pytest  # noqa: E402 — needed for approx above


def _make_output(cx, cy, w, h, class_id, score, n_classes=80):
    """Build a minimal [1, 84, 8400] array with one detection."""
    output = np.zeros((1, 4 + n_classes, 8400), dtype=np.float32)
    output[0, 0, 0] = cx
    output[0, 1, 0] = cy
    output[0, 2, 0] = w
    output[0, 3, 0] = h
    output[0, 4 + class_id, 0] = score
    return output


class TestPostprocess:

    def test_filters_low_confidence(self):
        output = _make_output(320, 240, 100, 100, class_id=0, score=0.3)
        results = postprocess(output, orig_shape=(480, 640), conf_threshold=0.5)
        assert results == []

    def test_accepts_high_confidence(self):
        output = _make_output(320, 240, 100, 100, class_id=0, score=0.8)
        results = postprocess(output, orig_shape=(640, 640), conf_threshold=0.5)
        assert len(results) == 1

    def test_output_tuple_has_six_fields(self):
        output = _make_output(320, 320, 100, 100, class_id=0, score=0.9)
        results = postprocess(output, orig_shape=(640, 640), conf_threshold=0.5)
        assert len(results) == 1
        assert len(results[0]) == 6  # x1, y1, x2, y2, class_id, score

    def test_correct_class_id_extracted(self):
        output = _make_output(320, 320, 100, 100, class_id=32, score=0.9)
        results = postprocess(output, orig_shape=(640, 640), conf_threshold=0.5)
        assert len(results) == 1
        assert results[0][4] == 32

    def test_nms_removes_overlapping_boxes(self):
        """Two nearly identical boxes for same class → NMS keeps one."""
        output = _make_output(320, 320, 200, 200, class_id=0, score=0.9)
        # Add second overlapping box in slot 1
        output[0, 0, 1] = 322
        output[0, 1, 1] = 322
        output[0, 2, 1] = 200
        output[0, 3, 1] = 200
        output[0, 4, 1] = 0.85  # class 0, high confidence
        results = postprocess(output, orig_shape=(640, 640), conf_threshold=0.5,
                              iou_threshold=0.45)
        assert len(results) == 1

    def test_empty_output_returns_empty_list(self):
        output = np.zeros((1, 84, 8400), dtype=np.float32)
        results = postprocess(output, orig_shape=(480, 640), conf_threshold=0.5)
        assert results == []

    def test_coordinates_scaled_to_original_size(self):
        """Box at image centre should map to roughly (270,360) for 480x640."""
        output = _make_output(320, 240, 64, 48, class_id=0, score=0.9)
        results = postprocess(output, orig_shape=(480, 640), conf_threshold=0.5)
        assert len(results) == 1
        x1, y1, x2, y2 = results[0][:4]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        # Scale: 640→640 (x unchanged), 640→480 (y * 480/640 = 0.75)
        assert cx == pytest.approx(320.0, abs=2.0)
        assert cy == pytest.approx(240.0 * (480 / 640), abs=2.0)

    def test_3d_input_accepted(self):
        """Ensure output with leading batch dim is handled correctly."""
        output = _make_output(320, 320, 100, 100, class_id=0, score=0.9)
        assert output.shape == (1, 84, 8400)
        results = postprocess(output, orig_shape=(640, 640), conf_threshold=0.5)
        assert len(results) == 1
