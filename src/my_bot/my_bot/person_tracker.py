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

"""Person tracker: YOLOv8 detection + OpenCV tracker + Kalman filter.

Pipeline per camera frame
-------------------------
1. Every ``redetect_every`` frames or ``redetect_period`` seconds, whichever
   comes first (or whenever there is no track), YOLOv8n looks for people.
   The time bound keeps YOLO in charge on a slow, software-rendered camera
   where ten frames can take seconds. The highest-scoring person (re)seeds
   the OpenCV tracker, so tracker drift never outlives one cycle.
   If YOLO misses the person ``max_yolo_misses`` times in a row, the track
   is dropped, so a tracker stuck on the background cannot hold a lock.
2. Between detections the OpenCV tracker (``tracker``: KCF by default, a few
   ms per frame where CSRT took tens; falls back to CSRT, then MIL,
   whichever this OpenCV build has) follows the box frame to frame.
3. A constant-velocity Kalman filter smooths the box and predicts it
   through short tracker failures (up to ``max_coast_frames``).

Detection in a worker thread (``async_detection``, the default)
---------------------------------------------------------------
YOLO runs on a background thread (onnxruntime releases the GIL), so the
camera callback never waits for it: every frame is tracked and published
right away, and /person_bbox is only as old as the camera plus the tracker.
A YOLO result belongs to the frame it was run on, which is a few frames
old when it arrives; it re-seeds the tracker on that frame and the tracker
replays the frames since, so the correction lands on the newest frame.
With ``async_detection: false`` YOLO runs inline, as before Phase 1.

Published topics
----------------
/person_bbox           geometry_msgs/PolygonStamped  box corners (px):
                       top-left then bottom-right, empty when nobody is
                       tracked; header = the camera image's, so consumers
                       know when the box was seen (latency compensation)
/person_track          geometry_msgs/PointStamped  bbox centre (px)
/person_detected       std_msgs/Bool               True while a track exists
/person_tracker/image  sensor_msgs/Image           annotated debug view

Every ``stats_period`` seconds (wall clock) the node logs its frame rate and
the mean YOLO and tracker time per call, to tell CPU-bound inference from a
slow simulator (Phase 0 of the security-guard plan).
"""

from collections import deque
import math
import os
import threading
import time
from typing import Callable, Optional, Tuple

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from my_bot.object_detector import DEFAULT_INPUT_SIZE, postprocess, preprocess

Box = Tuple[int, int, int, int]

# OpenCV trackers in fallback order; the ``tracker`` parameter picks the first.
TRACKERS = ('kcf', 'csrt', 'mil')


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def letterbox_shape(frame_shape: tuple) -> tuple:
    """Shape to pass to ``postprocess`` so boxes map back correctly.

    ``preprocess`` scales the frame uniformly by 640 / max(h, w) and pads
    the bottom/right. Decoding against a square max(h, w) canvas therefore
    undoes the scaling on both axes. Passing the real (h, w) would scale y
    by h / 640 instead and squash every box vertically on a 640x480 camera.
    """
    side = max(frame_shape[0], frame_shape[1])
    return (side, side)


def clip_box(box: Box, width: int, height: int) -> Optional[Box]:
    """Clip an (x, y, w, h) box to the image; None if nothing is left."""
    x1 = max(0, min(width - 1, int(box[0])))
    y1 = max(0, min(height - 1, int(box[1])))
    x2 = max(0, min(width, int(box[0] + box[2])))
    y2 = max(0, min(height, int(box[1] + box[3])))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    return (x1, y1, x2 - x1, y2 - y1)


def select_person_box(detections: list,
                      min_conf: float = 0.4) -> Optional[Box]:
    """Return the highest-confidence person box (x, y, w, h), or None.

    ``detections`` holds ``(x1, y1, x2, y2, class_id, score)`` tuples from
    ``object_detector.postprocess``. Only class 0 (person) is considered.
    """
    people = [d for d in detections if d[4] == 0 and d[5] >= min_conf]
    if not people:
        return None
    x1, y1, x2, y2, _, _ = max(people, key=lambda d: d[5])
    return (int(x1), int(y1), int(x2 - x1), int(y2 - y1))


def box_to_polygon(box: Optional[Box]) -> list:
    """Top-left and bottom-right corners of a box, or [] for no box."""
    if not box:
        return []
    x, y, w, h = (float(v) for v in box)
    return [Point32(x=x, y=y, z=0.0), Point32(x=x + w, y=y + h, z=0.0)]


class StageTimer:
    """Mean duration of a repeated stage, reset on every report."""

    def __init__(self):
        self.calls = 0
        self.total = 0.0

    def add(self, seconds: float) -> None:
        """Record one call's duration."""
        self.calls += 1
        self.total += seconds

    def report(self) -> str:
        """'<mean> ms x <calls>' since the last report, then reset."""
        text = (f'{1000.0 * self.total / self.calls:.0f} ms x {self.calls}'
                if self.calls else 'not run')
        self.calls, self.total = 0, 0.0
        return text


def model_input_size(session) -> int:
    """Square input side the ONNX model was exported with (640 if unknown)."""
    try:
        side = session.get_inputs()[0].shape[-1]
    except (AttributeError, IndexError, TypeError):
        return DEFAULT_INPUT_SIZE
    return side if isinstance(side, int) and side > 0 else DEFAULT_INPUT_SIZE


def make_tracker(preferred: str = 'kcf'):
    """Create an OpenCV single-object tracker, ``preferred`` first.

    KCF is the default: CSRT's tighter box cost tens of ms per frame on the
    CPU, and YOLO re-seeds the box every ``redetect_period`` anyway. The
    others in ``TRACKERS`` are fallbacks for OpenCV builds without it.
    """
    names = [preferred] + [n for n in TRACKERS if n != preferred]
    legacy = getattr(cv2, 'legacy', None)
    for name in names:
        attr = f'Tracker{name.upper()}_create'
        for mod in (cv2, legacy):
            ctor = getattr(mod, attr, None) if mod is not None else None
            if ctor is not None:
                return ctor()
    raise RuntimeError('No OpenCV tracker available')


class DetectionWorker:
    """Runs ``detect(frame)`` on a background thread, one frame at a time.

    The camera callback hands a frame over with ``submit`` and keeps
    tracking the frames that follow; ``take`` returns the finished
    ``(seq, box, seconds)`` once. The worker stays busy until its result is
    taken, so a result is never overwritten by the next one.
    """

    def __init__(self, detect: Callable[[np.ndarray], Optional[Box]]):
        self._detect = detect
        self._cond = threading.Condition()
        self._job = None
        self._result = None
        self._busy = False
        self._stopped = False
        self._thread = threading.Thread(
            target=self._run, name='yolo_worker', daemon=True)
        self._thread.start()

    @property
    def busy(self) -> bool:
        """True from ``submit`` until the result has been taken."""
        with self._cond:
            return self._busy

    def submit(self, seq: int, frame: np.ndarray) -> bool:
        """Start detecting on ``frame``; False if busy or stopped."""
        with self._cond:
            if self._busy or self._stopped:
                return False
            self._busy = True
            self._job = (seq, frame)
            self._cond.notify_all()
        return True

    def take(self):
        """Return the finished (seq, box, seconds), or None if none is ready."""
        with self._cond:
            result, self._result = self._result, None
            if result is not None:
                self._busy = False
            return result

    def wait(self, timeout: float) -> bool:
        """Block until a result is ready to take; False on timeout."""
        with self._cond:
            return self._cond.wait_for(
                lambda: self._result is not None or self._stopped, timeout)

    def stop(self) -> None:
        """Stop the thread (the job in flight, if any, is discarded)."""
        with self._cond:
            self._stopped = True
            self._cond.notify_all()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while True:
            with self._cond:
                self._cond.wait_for(
                    lambda: self._job is not None or self._stopped)
                if self._stopped:
                    return
                seq, frame = self._job
                self._job = None
            start = time.monotonic()
            try:
                box = self._detect(frame)
            except Exception:  # noqa: BLE001 -- a lost frame, not a dead thread
                box = None
            with self._cond:
                self._result = (seq, box, time.monotonic() - start)
                self._cond.notify_all()


class BoxKalman:
    """Constant-velocity Kalman filter on a box centre and size.

    State: [cx, cy, w, h, vcx, vcy, vw, vh] in pixels and pixels/frame.
    """

    def __init__(self, box: Box, meas_noise: float = 4.0,
                 process_noise: float = 1.0):
        x, y, w, h = (float(v) for v in box)
        self.x = np.array([x + w / 2, y + h / 2, w, h, 0, 0, 0, 0], dtype=float)
        self.P = np.diag([10, 10, 10, 10, 100, 100, 100, 100]).astype(float)
        self.F = np.eye(8)
        self.F[:4, 4:] = np.eye(4)
        self.H = np.zeros((4, 8))
        self.H[:4, :4] = np.eye(4)
        self.Q = np.eye(8) * process_noise
        self.Q[4:, 4:] *= 0.5
        self.R = np.eye(4) * meas_noise ** 2

    def predict(self) -> Box:
        """Advance one frame and return the predicted box."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.box()

    def update(self, box: Box) -> Box:
        """Fuse a measured box and return the corrected box."""
        x, y, w, h = (float(v) for v in box)
        z = np.array([x + w / 2, y + h / 2, w, h])
        innov = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.P = (np.eye(8) - K @ self.H) @ self.P
        return self.box()

    def box(self) -> Box:
        """Current estimate as an integer (x, y, w, h) box."""
        cx, cy, w, h = self.x[:4]
        w, h = max(w, 1.0), max(h, 1.0)
        return (int(round(cx - w / 2)), int(round(cy - h / 2)),
                int(round(w)), int(round(h)))


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PersonTrackerNode(Node):
    """YOLO-seeded, Kalman-smoothed single-person tracker."""

    def __init__(self):
        super().__init__('person_tracker')

        self.declare_parameter('model_path', '/root/models/yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('nms_iou_threshold', 0.45)
        self.declare_parameter('redetect_every', 10)
        self.declare_parameter('redetect_period', 0.5)
        self.declare_parameter('max_yolo_misses', 2)
        self.declare_parameter('max_coast_frames', 10)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('stats_period', 10.0)
        self.declare_parameter('tracker', 'kcf')
        self.declare_parameter('async_detection', True)
        self.declare_parameter('inference_threads', 0)

        gp = self.get_parameter
        self._conf = float(gp('confidence_threshold').value)
        self._iou = float(gp('nms_iou_threshold').value)
        self._redetect_every = max(1, int(gp('redetect_every').value))
        self._redetect_period = float(gp('redetect_period').value)
        self._max_misses = int(gp('max_yolo_misses').value)
        self._max_coast = int(gp('max_coast_frames').value)
        self._debug = bool(gp('publish_debug_image').value)
        self._stats_period = float(gp('stats_period').value)
        self._tracker_name = str(gp('tracker').value).lower()
        if self._tracker_name not in TRACKERS:
            raise ValueError(
                f'tracker must be one of {TRACKERS}, got {self._tracker_name!r}')
        self._yolo_timer = StageTimer()
        self._track_timer = StageTimer()
        self._frames = 0
        self._stats_start = time.monotonic()

        self._bridge = CvBridge()
        self._session = None
        # Why there is no session, drawn on the debug image.
        self._model_error = ''
        self._load_model(str(gp('model_path').value),
                         int(gp('inference_threads').value))

        self._tracker = None
        self._kf: Optional[BoxKalman] = None
        self._box: Optional[Box] = None
        self._frames_since_detect = 0
        self._last_yolo = -math.inf
        self._yolo_misses = 0
        self._coast = 0

        # Async detection: frames since the one YOLO is working on, so its
        # result can be replayed forward (~2 s at the camera's 15 Hz).
        self._seq = 0
        self._history = deque(maxlen=30)
        self._worker: Optional[DetectionWorker] = None
        if bool(gp('async_detection').value):
            self._worker = DetectionWorker(self._run_yolo)

        # Depth 1: when inference is slower than the camera, work on the
        # newest frame instead of a queue of stale ones (the box would lag).
        latest_frame = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, latest_frame)
        self._bbox_pub = self.create_publisher(PolygonStamped, '/person_bbox', 10)
        self._track_pub = self.create_publisher(PointStamped, '/person_track', 10)
        self._detected_pub = self.create_publisher(Bool, '/person_detected', 10)
        # Best effort: a reliable debug image stalls this node while a slow
        # RViz (software rendering) acknowledges it.
        self._debug_pub = self.create_publisher(
            Image, '/person_tracker/image', latest_frame)

    # ------------------------------------------------------------------

    def _load_model(self, model_path: str, threads: int = 0) -> None:
        # The Docker image exports the model at build time; a missing or
        # empty file means the container runs an image from before that.
        # Rebuilding alone does not help: a running container keeps its
        # image until it is recreated, which `make image` also does.
        fix = 'run make image on the host'
        hint = (f'nothing will be tracked; {fix} (rebuilds the image and '
                'recreates the container: docker compose build alone keeps '
                'the old one)')
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'YOLO model not found at {model_path}; {hint}')
            self._model_error = f'No YOLO model: {fix}'
            return
        if os.path.getsize(model_path) == 0:
            self.get_logger().error(f'YOLO model {model_path} is empty; {hint}')
            self._model_error = f'Empty YOLO model: {fix}'
            return
        try:
            import onnxruntime as ort
            # Without CUDA 12 + cuDNN 9 in the image, onnxruntime prints a
            # red error before falling back to CPU, which looks like the
            # cause when something else fails. Report the provider instead.
            set_severity = getattr(ort, 'set_default_logger_severity', None)
            if set_severity is not None:
                set_severity(4)
            # Threads for one inference; 0 = onnxruntime's default (every
            # core), which competes with gzserver for the CPU.
            options = {}
            if threads > 0 and hasattr(ort, 'SessionOptions'):
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = threads
                options['sess_options'] = opts
            try:
                self._session = ort.InferenceSession(
                    model_path,
                    providers=['CUDAExecutionProvider', 'CPUExecutionProvider'],
                    **options)
            finally:
                if set_severity is not None:
                    set_severity(2)
            provider = self._session.get_providers()[0]
            side = model_input_size(self._session)
            self.get_logger().info(f'YOLO loaded ({provider}, {side} px input)')
            if provider != 'CUDAExecutionProvider':
                self.get_logger().info(
                    'CUDA unavailable to onnxruntime (needs a GPU plus the '
                    'CUDA 12 and cuDNN 9 libraries); YOLO runs on the CPU')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'YOLO load failed: {e}; {hint}')
            self._model_error = 'YOLO model failed to load (see log)'

    def _reset(self) -> None:
        self._tracker = None
        self._kf = None
        self._box = None
        self._coast = 0
        self._yolo_misses = 0

    # ------------------------------------------------------------------

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f'cv_bridge: {e}')
            return
        h, w = frame.shape[:2]
        self._seq += 1

        # A finished background detection re-seeds the tracker first, so
        # this frame is tracked from the corrected box.
        if self._worker is not None:
            self._history.append((self._seq, frame))
            result = self._worker.take()
            if result is not None:
                self._apply_async_detection(*result)

        self._frames_since_detect += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        detected: Optional[Box] = None
        if ((self._tracker is None
                or self._frames_since_detect >= self._redetect_every
                or now - self._last_yolo >= self._redetect_period)
                and (self._worker is None or not self._worker.busy)):
            self._frames_since_detect = 0
            self._last_yolo = now
            if self._worker is not None:
                self._worker.submit(self._seq, frame)
            else:
                start = time.monotonic()
                detected = self._run_yolo(frame)
                self._yolo_timer.add(time.monotonic() - start)
                self._count_detection(detected)

        measured: Optional[Box] = None
        if detected is not None:
            measured = detected
            self._seed_tracker(frame, detected)
        elif self._tracker is not None:
            start = time.monotonic()
            ok, bb = self._tracker.update(frame)
            self._track_timer.add(time.monotonic() - start)
            if ok:
                measured = clip_box(tuple(int(v) for v in bb), w, h)

        if measured is not None:
            if self._kf is None:
                self._kf = BoxKalman(measured)
            else:
                self._kf.predict()
            self._box = clip_box(self._kf.update(measured), w, h)
            self._coast = 0
        elif self._kf is not None:
            # Tracker failed: coast on the motion model, ask YOLO next frame.
            self._coast += 1
            self._frames_since_detect = self._redetect_every
            if self._coast > self._max_coast:
                self._reset()
            else:
                self._box = clip_box(self._kf.predict(), w, h)

        self._publish(msg, frame)
        self._log_stats()

    def _count_detection(self, detected: Optional[Box]) -> None:
        """Track YOLO's misses; enough in a row drop the track."""
        if detected is None and self._tracker is not None:
            self._yolo_misses += 1
            if self._yolo_misses >= self._max_misses:
                self.get_logger().info('YOLO lost the person; dropping track')
                self._reset()
        elif detected is not None:
            self._yolo_misses = 0

    def _apply_async_detection(self, seq: int, box: Optional[Box],
                               seconds: float) -> None:
        """Fold in a detection made on the earlier frame ``seq``.

        The tracker is re-seeded on that frame and run over the frames
        after it up to (not including) the newest, which the caller then
        tracks as usual. If the frame has left the history, the box seeds
        the newest frame directly.
        """
        self._yolo_timer.add(seconds)
        self._count_detection(box)
        if box is None:
            return
        frames = [f for s, f in self._history if s >= seq]
        if self._history[0][0] > seq:
            frames = [self._history[-1][1]]
        self._seed_tracker(frames[0], box)
        for frame in frames[1:-1]:
            if self._tracker is None:
                break
            start = time.monotonic()
            self._tracker.update(frame)
            self._track_timer.add(time.monotonic() - start)

    def _log_stats(self) -> None:
        self._frames += 1
        elapsed = time.monotonic() - self._stats_start
        if self._stats_period <= 0.0 or elapsed < self._stats_period:
            return
        self.get_logger().info(
            f'{self._frames / elapsed:.1f} frames/s (wall); '
            f'YOLO {self._yolo_timer.report()}, '
            f'tracker {self._track_timer.report()}')
        self._frames = 0
        self._stats_start = time.monotonic()

    def _run_yolo(self, frame: np.ndarray) -> Optional[Box]:
        """Best person box in ``frame``; runs on the worker thread if async."""
        session = self._session
        if session is None:
            return None
        try:
            side = model_input_size(session)
            blob = preprocess(frame, side)
            inp = session.get_inputs()[0].name
            raw = session.run(None, {inp: blob})[0]
            dets = postprocess(raw, letterbox_shape(frame.shape),
                               self._conf, self._iou, side)
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f'YOLO inference: {e}')
            return None
        box = select_person_box(dets, self._conf)
        if box is None:
            return None
        return clip_box(box, frame.shape[1], frame.shape[0])

    def _seed_tracker(self, frame, box: Box) -> None:
        """(Re)start the tracker on a YOLO box; YOLO is the reference."""
        try:
            self._tracker = make_tracker(self._tracker_name)
            self._tracker.init(frame, tuple(box))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'tracker init failed: {e}')
            self._tracker = None

    # ------------------------------------------------------------------

    def _publish(self, img_msg: Image, frame) -> None:
        box = self._box
        self._detected_pub.publish(Bool(data=box is not None))

        bbox_msg = PolygonStamped()
        bbox_msg.header = img_msg.header
        bbox_msg.polygon.points = box_to_polygon(box)
        if box is not None:
            pt = PointStamped()
            pt.header = img_msg.header
            pt.point.x = float(box[0] + box[2] / 2.0)
            pt.point.y = float(box[1] + box[3] / 2.0)
            self._track_pub.publish(pt)
        self._bbox_pub.publish(bbox_msg)

        if self._debug and self._debug_pub.get_subscription_count() > 0:
            vis = frame.copy()
            if self._session is None:
                cv2.putText(vis, self._model_error or 'YOLO model not loaded',
                            (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (0, 0, 255), 1)   # fits the 320 px frame
            if box is not None:
                color = (0, 200, 0) if self._coast == 0 else (0, 200, 255)
                x, y, w, h = box
                cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
                label = 'person' if self._coast == 0 else f'coast {self._coast}'
                cv2.putText(vis, label, (x, max(0, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            out = self._bridge.cv2_to_imgmsg(vis, 'bgr8')
            out.header = img_msg.header
            self._debug_pub.publish(out)

    def destroy_node(self):
        """Stop the detection thread, then the node."""
        if self._worker is not None:
            self._worker.stop()
        super().destroy_node()


def main(args=None):
    """Initialize and spin the PersonTrackerNode."""
    rclpy.init(args=args)
    node = PersonTrackerNode()
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
