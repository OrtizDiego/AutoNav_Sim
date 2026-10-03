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

"""Node-level tests for sensor_fusion, person_tracker and system_monitor.

Images are numpy frames carried inside the fake Image message (see
conftest.py); YOLO is a fake onnxruntime session and the OpenCV tracker a
scripted fake, so every branch of the tracking pipeline is deterministic.
"""

import math
import sys
import types

import cv2
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PolygonStamped
import numpy as np
import pytest
from sensor_msgs.msg import Image, LaserScan

from my_bot import person_tracker as pt
from my_bot import sensor_fusion as sf
from my_bot import system_monitor as sm

W, H = 640, 480


def _scan(value=2.0, n=360):
    return LaserScan(ranges=[value] * n, angle_min=-math.pi,
                     angle_increment=2.0 * math.pi / n)


def _frame():
    return np.full((H, W, 3), 90, dtype=np.uint8)


def _ball_frame(cx=W // 2, cy=H // 2, radius=40):
    frame = _frame()
    cv2.circle(frame, (cx, cy), radius, (0, 0, 255), -1)  # BGR red
    return frame


def _image(frame):
    msg = Image(frame=frame)
    msg.header.frame_id = 'camera_link_optical'
    return msg


# ---------------------------------------------------------------------------
# sensor_fusion
# ---------------------------------------------------------------------------

class TestSensorFusionHsv:

    @pytest.fixture
    def node(self):
        return sf.SensorFusionNode()

    def _out(self, node):
        p = node.publishers
        return (p['/target_bearing'].last.data, p['/target_range'].last.data,
                p['/target_position'].last)

    def test_interface(self, node):
        assert set(node.subscriptions) == {'/scan', '/camera/image_raw'}
        assert set(node.publishers) == {
            '/target', '/target_position', '/target_range', '/target_bearing',
            '/sensor_fusion/image'}
        assert 'hsv mode' in node.logger.messages('info')[0]

    def test_rejects_unknown_mode(self, ros_params):
        ros_params['mode'] = 'lidar'
        with pytest.raises(ValueError, match='mode'):
            sf.SensorFusionNode()

    def test_ball_ahead_is_ranged_with_the_lidar(self, node):
        node.subscriptions['/scan'](_scan(2.0))
        node.subscriptions['/camera/image_raw'](_image(_ball_frame()))
        bearing, rng, pos = self._out(node)
        assert bearing == pytest.approx(0.0, abs=0.01)
        assert rng == pytest.approx(2.0)
        assert pos.header.frame_id == 'base_link'
        assert (pos.point.x, pos.point.y) == pytest.approx((2.0, 0.0), abs=0.02)

    def test_ball_on_the_left_has_positive_bearing(self, node):
        node.subscriptions['/camera/image_raw'](_image(_ball_frame(cx=100)))
        assert node.publishers['/target_bearing'].last.data > 0.2

    def test_ball_without_scan_has_no_range(self, node):
        node.subscriptions['/camera/image_raw'](_image(_ball_frame()))
        bearing, rng, pos = self._out(node)
        assert math.isfinite(bearing)
        assert rng == -1.0
        assert pos is None

    def test_no_ball(self, node):
        node.subscriptions['/scan'](_scan(2.0))
        node.subscriptions['/camera/image_raw'](_image(_frame()))
        bearing, rng, _ = self._out(node)
        assert math.isnan(bearing)
        assert rng == -1.0

    def test_debug_image_only_with_a_viewer(self, node):
        debug = node.publishers['/sensor_fusion/image']
        node.subscriptions['/camera/image_raw'](_image(_ball_frame()))
        assert debug.msgs == []
        debug.subscribers = 1
        node.subscriptions['/scan'](_scan(2.0))
        node.subscriptions['/camera/image_raw'](_image(_ball_frame()))
        node.subscriptions['/camera/image_raw'](_image(_frame()))
        node.subscriptions['/scan'](_scan(float('inf')))
        node.subscriptions['/camera/image_raw'](_image(_ball_frame()))
        assert len(debug.msgs) == 3
        ranged, empty, unranged = debug.msgs
        assert ranged.header.frame_id == 'camera_link_optical'
        assert ranged.frame.shape == (H, W, 3)
        # Annotations are drawn on a copy, not on the camera frame
        assert not np.array_equal(ranged.frame, _ball_frame())
        assert not np.array_equal(empty.frame, _frame())
        assert not np.array_equal(unranged.frame, _ball_frame())

    def test_bad_image_is_logged_and_skipped(self, node):
        node.subscriptions['/camera/image_raw'](Image())
        assert 'Image conversion failed' in node.logger.messages('error')[0]
        assert node.publishers['/target_bearing'].msgs == []


class TestSensorFusionPerson:

    @pytest.fixture
    def node(self, ros_params):
        ros_params['mode'] = 'person'
        return sf.SensorFusionNode()

    def _bbox(self, node, box):
        msg = PolygonStamped()
        msg.polygon.points = pt.box_to_polygon(box)
        node.subscriptions['/person_bbox'](msg)
        p = node.publishers
        return p['/target_bearing'].last.data, p['/target_range'].last.data

    def _full_body_box(self, node, distance, cx=W / 2):
        h = node._fx * node._person_h / distance
        return (cx - h / 6, node._cy - h / 2, h / 3, h)

    def test_interface(self, node):
        assert '/person_bbox' in node.subscriptions

    def test_camera_frames_are_skipped_without_a_viewer(self, node):
        node.subscriptions['/camera/image_raw'](_image(_frame()))
        assert node._frame is None
        node.publishers['/sensor_fusion/image'].subscribers = 1
        node.subscriptions['/camera/image_raw'](_image(_frame()))
        assert node._frame is not None
        assert node.publishers['/target_bearing'].msgs == []  # boxes drive output

    def test_lidar_range_when_it_agrees_with_the_camera(self, node):
        node.subscriptions['/scan'](_scan(3.1))
        bearing, rng = self._bbox(node, self._full_body_box(node, 3.0))
        assert bearing == pytest.approx(0.0, abs=0.01)
        assert rng == pytest.approx(3.1)
        assert node._range_for(self._full_body_box(node, 3.0))[1] == 'lidar'

    def test_camera_range_when_the_lidar_sees_the_background(self, node):
        node.subscriptions['/scan'](_scan(9.0))
        _, rng = self._bbox(node, self._full_body_box(node, 3.0))
        assert rng == pytest.approx(3.0, rel=0.01)
        assert node._range_for(self._full_body_box(node, 3.0))[1] == 'camera'

    def test_camera_range_without_scan(self, node):
        _, rng = self._bbox(node, self._full_body_box(node, 4.0))
        assert rng == pytest.approx(4.0, rel=0.01)

    def test_no_range_when_head_and_feet_are_cut(self, node):
        bearing, rng = self._bbox(node, (300.0, 0.0, 40.0, float(H)))
        assert math.isfinite(bearing)
        assert rng == -1.0

    def test_empty_box_means_no_target(self, node):
        bearing, rng = self._bbox(node, [])
        assert math.isnan(bearing)
        assert rng == -1.0

    def test_target_carries_the_image_time_and_its_scan(self, node):
        # The box was seen at t=20.0; scans arrive at 10 Hz meanwhile. The
        # scan closest to the image ranges it, not the newest one.
        for t, r in ((19.9, 5.0), (20.0, 3.05), (20.1, 9.0), (20.2, 9.0)):
            scan = _scan(r)
            scan.header.stamp = types.SimpleNamespace(sec=int(t), nanosec=round(t % 1 * 1e9))
            node.subscriptions['/scan'](scan)
        msg = PolygonStamped()
        msg.header.stamp = types.SimpleNamespace(sec=20, nanosec=0)
        msg.polygon.points = pt.box_to_polygon(self._full_body_box(node, 3.0, cx=200.0))
        node.subscriptions['/person_bbox'](msg)
        target = node.publishers['/target'].last
        assert target.header.stamp.sec == 20
        assert target.header.frame_id == 'base_link'
        assert target.vector.x > 0.0                  # left of centre
        assert target.vector.y == pytest.approx(3.05)
        assert node.publishers['/target_position'].last.header.stamp.sec == 20

    def test_unstamped_box_is_stamped_on_arrival(self, node):
        node.clock.advance(7.0)
        self._bbox(node, self._full_body_box(node, 3.0))
        assert node.publishers['/target'].last.header.stamp.sec == 107

    def test_debug_view_uses_the_latest_frame(self, node):
        debug = node.publishers['/sensor_fusion/image']
        debug.subscribers = 1
        self._bbox(node, self._full_body_box(node, 3.0))
        assert debug.msgs == []                       # no frame yet
        node.subscriptions['/camera/image_raw'](_image(_frame()))
        self._bbox(node, self._full_body_box(node, 3.0))
        self._bbox(node, (300.0, 0.0, 40.0, float(H)))
        assert len(debug.msgs) == 2
        assert debug.last.header.frame_id == 'camera_link_optical'


# ---------------------------------------------------------------------------
# person_tracker
# ---------------------------------------------------------------------------

def _yolo_output(boxes):
    """Raw [1, 84, 8400] YOLOv8 output; boxes are (cx, cy, w, h, cls, score)."""
    out = np.zeros((1, 84, 8400), dtype=np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(boxes):
        out[0, :4, i] = [cx, cy, w, h]
        out[0, 4 + cls, i] = score
    return out


class FakeSession:
    """onnxruntime.InferenceSession stand-in returning scripted detections."""

    def __init__(self, path=None, providers=None):
        self.path = path
        self.providers = providers
        self.boxes = []
        self.fail = False
        self.calls = 0

    def get_providers(self):
        return ['CPUExecutionProvider']

    def get_inputs(self):
        return [types.SimpleNamespace(name='images')]

    def run(self, outputs, feeds):
        self.calls += 1
        assert feeds['images'].shape == (1, 3, 640, 640)
        if self.fail:
            raise RuntimeError('inference failed')
        return [_yolo_output(self.boxes)]


class FakeTracker:
    """OpenCV tracker stand-in: reports ``box`` or a failure."""

    instances = []

    def __init__(self):
        self.box = None
        self.ok = True
        self.fail_init = False
        FakeTracker.instances.append(self)

    def init(self, frame, box):
        if self.fail_init:
            raise RuntimeError('init failed')
        self.box = box

    def update(self, frame):
        return self.ok, self.box


PERSON = (320.0, 240.0, 100.0, 200.0, 0, 0.9)   # box (270, 140, 100, 200)


@pytest.fixture
def tracker_node(monkeypatch, ros_params):
    FakeTracker.instances = []
    monkeypatch.setattr(pt, 'make_tracker', FakeTracker)
    # Never the image's real /root/models/yolov8n.onnx: FakeSession instead.
    ros_params.update(redetect_every=3, max_yolo_misses=2, max_coast_frames=2,
                      model_path='/nonexistent/yolov8n.onnx')
    node = pt.PersonTrackerNode()
    node._session = FakeSession()
    node._session.boxes = [PERSON]
    return node


def _see(node, n=1):
    for _ in range(n):
        node.subscriptions['/camera/image_raw'](_image(_frame()))
    p = node.publishers
    box = sf.polygon_to_box(p['/person_bbox'].last.polygon.points)
    return p['/person_detected'].last.data, list(box) if box else []


class TestPersonTrackerNode:

    def test_box_keeps_the_image_timestamp(self, tracker_node):
        img = _image(_frame())
        img.header.stamp = types.SimpleNamespace(sec=42, nanosec=5)
        tracker_node.subscriptions['/camera/image_raw'](img)
        bbox = tracker_node.publishers['/person_bbox'].last
        assert bbox.header is img.header
        assert len(bbox.polygon.points) == 2

    def test_stage_timings_are_logged_periodically(self, tracker_node, monkeypatch):
        now = [1000.0]
        monkeypatch.setattr(pt, 'time', types.SimpleNamespace(monotonic=lambda: now[0]))
        tracker_node._stats_start = now[0]
        _see(tracker_node, 4)                         # YOLO, then the tracker
        assert not any('frames/s' in m for m in tracker_node.logger.messages('info'))
        now[0] += tracker_node._stats_period
        _see(tracker_node)
        line = [m for m in tracker_node.logger.messages('info') if 'frames/s' in m][-1]
        assert line.startswith('0.5 frames/s')
        assert 'YOLO 0 ms x' in line and 'tracker 0 ms x' in line
        assert tracker_node._frames == 0

    def test_stage_timer_reports_and_resets(self):
        timer = pt.StageTimer()
        assert timer.report() == 'not run'
        timer.add(0.02)
        timer.add(0.04)
        assert timer.report() == '30 ms x 2'
        assert timer.report() == 'not run'

    def test_interface_and_missing_model(self, tmp_path, ros_params):
        ros_params['model_path'] = str(tmp_path / 'missing.onnx')
        node = pt.PersonTrackerNode()
        assert set(node.subscriptions) == {'/camera/image_raw'}
        assert set(node.publishers) == {
            '/person_bbox', '/person_track', '/person_detected',
            '/person_tracker/image'}
        assert node._session is None
        assert 'not found' in node.logger.messages('error')[0]
        # docker compose build alone leaves the container on its old image.
        assert 'make image' in node.logger.messages('error')[0]
        assert 'make image' in node._model_error
        assert _see(node) == (False, [])
        assert node.publishers['/person_track'].msgs == []

    def test_loads_the_model_with_onnxruntime(self, tmp_path, monkeypatch,
                                              ros_params):
        model = tmp_path / 'yolo.onnx'
        model.write_bytes(b'onnx')
        monkeypatch.setitem(sys.modules, 'onnxruntime',
                            types.SimpleNamespace(InferenceSession=FakeSession))
        ros_params['model_path'] = str(model)
        node = pt.PersonTrackerNode()
        assert node._session.path == str(model)
        assert node._session.providers[0] == 'CUDAExecutionProvider'
        assert 'CPUExecutionProvider' in node.logger.messages('info')[0]
        # CUDA was asked for but not granted: say why YOLO runs on the CPU.
        assert 'CUDA' in node.logger.messages('info')[1]

    def test_cuda_fallback_noise_is_silenced_during_load(
            self, tmp_path, monkeypatch, ros_params):
        model = tmp_path / 'yolo.onnx'
        model.write_bytes(b'onnx')
        severities = []
        monkeypatch.setitem(sys.modules, 'onnxruntime', types.SimpleNamespace(
            InferenceSession=FakeSession,
            set_default_logger_severity=severities.append))
        ros_params['model_path'] = str(model)
        pt.PersonTrackerNode()
        assert severities == [4, 2]           # fatal only, then restored

    def test_model_load_failure_is_logged(self, tmp_path, monkeypatch, ros_params):
        model = tmp_path / 'yolo.onnx'
        model.write_bytes(b'not a model')

        def broken(*args, **kwargs):
            raise RuntimeError('bad model')
        monkeypatch.setitem(sys.modules, 'onnxruntime',
                            types.SimpleNamespace(InferenceSession=broken))
        ros_params['model_path'] = str(model)
        node = pt.PersonTrackerNode()
        assert node._session is None
        assert 'bad model' in node.logger.messages('error')[0]
        assert 'failed' in node._model_error

    def test_empty_model_file_is_reported(self, tmp_path, ros_params):
        # What a failed download at image build time used to leave behind.
        model = tmp_path / 'yolo.onnx'
        model.write_bytes(b'')
        ros_params['model_path'] = str(model)
        node = pt.PersonTrackerNode()
        assert node._session is None
        assert 'empty' in node.logger.messages('error')[0]
        assert 'make image' in node._model_error   # drawn below
        debug = node.publishers['/person_tracker/image']
        debug.subscribers = 1
        _see(node)
        assert not np.array_equal(debug.last.frame, _frame())   # warning drawn

    def test_detection_starts_a_track(self, tracker_node):
        detected, box = _see(tracker_node)
        assert detected
        assert box == [270.0, 140.0, 100.0, 200.0]
        track = tracker_node.publishers['/person_track'].last
        assert (track.point.x, track.point.y) == (320.0, 240.0)
        assert track.header.frame_id == 'camera_link_optical'
        assert len(FakeTracker.instances) == 1

    def test_tracker_follows_between_detections(self, tracker_node):
        _see(tracker_node)
        tracker = FakeTracker.instances[0]
        tracker.box = (280, 140, 100, 200)
        _see(tracker_node)
        assert tracker_node._session.calls == 1       # tracker, not YOLO
        assert tracker_node._box[0] > 270             # moved toward tracker

    def test_every_detection_reseeds_the_tracker(self, tracker_node):
        _see(tracker_node)
        # CSRT grew the box a little: still overlapping, but YOLO wins.
        FakeTracker.instances[0].box = (265, 130, 115, 230)
        _see(tracker_node, 3)
        assert tracker_node._session.calls == 2       # frames 1 and 4
        assert len(FakeTracker.instances) == 2
        assert FakeTracker.instances[1].box == (270, 140, 100, 200)

    def test_slow_camera_redetects_by_time(self, tracker_node):
        _see(tracker_node)
        tracker_node.clock.advance(0.1)
        _see(tracker_node)
        assert tracker_node._session.calls == 1       # frame 2: tracker
        tracker_node.clock.advance(0.5)
        _see(tracker_node)
        assert tracker_node._session.calls == 2       # 0.6 s later: YOLO

    def test_yolo_misses_drop_the_track(self, tracker_node):
        _see(tracker_node)
        tracker_node._session.boxes = []
        assert _see(tracker_node, 3)[0]               # first miss: still tracking
        assert tracker_node._yolo_misses == 1
        assert _see(tracker_node, 3) == (False, [])   # second miss: dropped
        assert 'dropping track' in tracker_node.logger.messages('info')[0]

    def test_detections_of_other_classes_are_ignored(self, tracker_node):
        tracker_node._session.boxes = [(320.0, 240.0, 100.0, 200.0, 32, 0.9)]
        assert _see(tracker_node) == (False, [])

    def test_inference_errors_count_as_no_detection(self, tracker_node):
        tracker_node._session.fail = True
        assert _see(tracker_node) == (False, [])

    def test_kalman_coasts_through_tracker_failures(self, tracker_node):
        _see(tracker_node)
        FakeTracker.instances[0].ok = False
        tracker_node._session.boxes = []
        detected, _ = _see(tracker_node)
        assert detected and tracker_node._coast == 1
        # A coasting track asks YOLO again on the next frame
        assert tracker_node._frames_since_detect == tracker_node._redetect_every
        _see(tracker_node)                             # YOLO miss 1, coast 2
        assert tracker_node._coast == 2
        assert _see(tracker_node) == (False, [])       # miss 2: track dropped

    def test_coasting_ends_after_max_frames(self, tracker_node, ros_params):
        _see(tracker_node)
        FakeTracker.instances[0].ok = False
        tracker_node._max_misses = 100
        tracker_node._session.boxes = []
        assert _see(tracker_node, 2)[0]
        assert _see(tracker_node) == (False, [])       # coast 3 > max 2
        assert tracker_node._kf is None

    def test_tracker_init_failure_is_survivable(self, tracker_node, monkeypatch):
        def failing():
            t = FakeTracker()
            t.fail_init = True
            return t
        monkeypatch.setattr(pt, 'make_tracker', failing)
        assert _see(tracker_node)[0]                   # YOLO box still used
        assert tracker_node._tracker is None
        assert 'tracker init failed' in tracker_node.logger.messages('warn')[0]

    def test_bad_image_is_skipped(self, tracker_node):
        tracker_node.subscriptions['/camera/image_raw'](Image())
        assert tracker_node.publishers['/person_detected'].msgs == []
        assert 'cv_bridge' in tracker_node.logger.messages('debug')[0]

    def test_debug_image(self, tracker_node):
        debug = tracker_node.publishers['/person_tracker/image']
        _see(tracker_node)
        assert debug.msgs == []                        # nobody watching
        debug.subscribers = 1
        tracker_node._session.boxes = []
        FakeTracker.instances[0].ok = False
        _see(tracker_node)                             # coasting box
        tracker_node._reset()
        _see(tracker_node)                             # no box
        assert len(debug.msgs) == 2
        coasting, empty = debug.msgs
        assert not np.array_equal(coasting.frame, _frame())
        assert np.array_equal(empty.frame, _frame())

    def test_debug_image_can_be_disabled(self, tracker_node, ros_params):
        ros_params['publish_debug_image'] = False
        node = pt.PersonTrackerNode()
        node.publishers['/person_tracker/image'].subscribers = 1
        _see(node)
        assert node.publishers['/person_tracker/image'].msgs == []


class TestMakeTracker:

    def test_prefers_csrt(self, monkeypatch):
        fake = types.SimpleNamespace(TrackerCSRT_create=lambda: 'csrt',
                                     TrackerMIL_create=lambda: 'mil')
        monkeypatch.setattr(pt, 'cv2', fake)
        assert pt.make_tracker() == 'csrt'

    def test_falls_back_to_the_legacy_module(self, monkeypatch):
        fake = types.SimpleNamespace(legacy=types.SimpleNamespace(
            TrackerKCF_create=lambda: 'kcf'))
        monkeypatch.setattr(pt, 'cv2', fake)
        assert pt.make_tracker() == 'kcf'

    def test_raises_without_any_tracker(self, monkeypatch):
        monkeypatch.setattr(pt, 'cv2', types.SimpleNamespace())
        with pytest.raises(RuntimeError, match='No OpenCV tracker'):
            pt.make_tracker()

    def test_real_opencv_has_a_tracker(self):
        tracker = pt.make_tracker()
        tracker.init(_ball_frame(), (280, 200, 80, 80))
        ok, box = tracker.update(_ball_frame())
        assert ok
        assert abs(box[0] - 280) <= 10


# ---------------------------------------------------------------------------
# system_monitor
# ---------------------------------------------------------------------------

class TestSystemMonitor:

    @pytest.fixture
    def node(self):
        return sm.SystemMonitorNode()

    def _health(self, node):
        node.timers[0]()
        status = node.publishers['/system_health'].last.status[0]
        return status.level, {kv.key: kv.value for kv in status.values}

    def test_interface(self, node):
        assert set(node.subscriptions) == {'/scan', '/camera/image_raw'}
        assert set(node.services) == {'/trigger_estop', '/clear_estop'}
        assert node.timers[0].period == pytest.approx(1.0)
        # The e-stop starts released (latched for late subscribers)
        assert node.publishers['/estop'].last.data is False

    def test_publish_rate_parameter(self, ros_params):
        ros_params['health_publish_rate'] = 4.0
        assert sm.SystemMonitorNode().timers[0].period == pytest.approx(0.25)

    def test_healthy_while_sensors_publish(self, node):
        node.clock.advance(1.5)
        node.subscriptions['/scan'](LaserScan())
        node.subscriptions['/camera/image_raw'](Image())
        node.clock.advance(1.0)
        level, values = self._health(node)
        assert level == DiagnosticStatus.OK
        assert values['scan_ok'] == 'True' and values['camera_ok'] == 'True'
        assert values['scan_age_sec'] == '1.00'
        assert node.logger.messages('warn') == []

    def test_warns_on_silent_sensors(self, node):
        node.clock.advance(2.5)
        node.subscriptions['/scan'](LaserScan())
        level, values = self._health(node)
        assert level == DiagnosticStatus.WARN
        assert values['scan_ok'] == 'True'
        assert values['camera_ok'] == 'False'
        assert node.publishers['/system_health'].last.status[0].message == 'Sensor timeout'
        node.clock.advance(2.5)
        self._health(node)
        warnings = node.logger.messages('warn')
        assert any('LiDAR silent' in w for w in warnings)
        assert any('Camera silent' in w for w in warnings)

    def test_estop_latches_and_clears(self, node):
        resp = node.services['/trigger_estop'](None, types.SimpleNamespace())
        assert resp.success and resp.message == 'E-stop latched'
        assert node.publishers['/estop'].last.data is True
        cmd = node.publishers['/cmd_vel'].last
        assert (cmd.linear.x, cmd.angular.z) == (0.0, 0.0)
        resp = node.services['/clear_estop'](None, types.SimpleNamespace())
        assert resp.success and resp.message == 'E-stop cleared'
        assert node.publishers['/estop'].last.data is False
