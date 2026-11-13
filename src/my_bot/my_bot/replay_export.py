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

"""Turn a scenario's rosbag into a replay for the web demo (demo/).

    python3 -m my_bot.replay_export <run dir> <out dir> [--image-fps 4]

``<run dir>`` is what scenario.launch.py writes (``bag/`` + ``report.json``);
``make demo-bag NAME=<scenario>`` records one with the camera image and
exports it into demo/replays/<scenario>/. The output is what a browser can
play without ROS:

replay.json
    meta (scenario, description, duration, the report's result, metrics and
    missed targets) and time series on simulation time from 0:
    ``frames`` at ``rate`` Hz (robot, person, second person, guard state,
    intruder state, track), ``scans`` (ranges in cm, base64 uint16 LE, 0 =
    no return), ``plans`` (Nav2's /plan), ``sweeps`` (/sweep_viewpoints),
    ``events`` (/security_events), ``images``.
images/NNNNN.jpg
    /person_tracker/image (YOLO box drawn by the tracker) at ``image_fps``.

The robot pose is odometry and the people's poses are Gazebo ground truth;
the robot spawns at the world origin, so both frames are the map's up to
odometry drift (as in scenario_metrics). Timestamps come from /clock: the
bag stores receive times on the wall clock.

``ReplayBuilder`` is pure Python (numpy for the covariance); the bag reader
and the JPEG encoder need ROS 2 and OpenCV, as in the container.
"""

import argparse
import base64
import json
import math
import os
import shutil
import struct
import sys
from typing import Callable, Dict, List, Optional

import numpy as np

GUARD_STATES = ['', 'PatrolProtocol', 'IntruderProtocol', 'SearchProtocol',
                'SweepProtocol', 'ReportProtocol', 'EmergencyStop', 'Idle']
INTRUDER_STATES = ['none', 'tentative', 'confirmed', 'lost']

# What the replay reads (scenario.launch.py records these)
TOPICS = [
    '/clock', '/odom', '/person/odom', '/person2/odom', '/scan',
    '/intruder/track', '/intruder/state', '/security_guard/state',
    '/security_events', '/plan', '/sweep_viewpoints', '/person_mode',
    '/person_tracker/image',
]


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pose(msg) -> List[float]:
    p = msg.pose.pose.position
    return [round(p.x, 3), round(p.y, 3), round(_yaw(msg.pose.pose.orientation), 3)]


def ellipse(cov_xx: float, cov_xy: float, cov_yy: float):
    """2-sigma semi-axes (major, minor) and the major axis' heading."""
    vals, vecs = np.linalg.eigh(np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]]))
    a = 2.0 * math.sqrt(max(float(vals[1]), 1e-6))
    b = 2.0 * math.sqrt(max(float(vals[0]), 1e-6))
    return a, b, math.atan2(float(vecs[1, 1]), float(vecs[0, 1]))


def pack_ranges(ranges, range_max: float = 12.0) -> str:
    """Ranges as base64 uint16 LE centimetres; 0 = no return."""
    out = []
    for r in ranges:
        r = float(r)
        out.append(int(round(r * 100.0)) if math.isfinite(r) and 0.0 < r <= range_max else 0)
    return base64.b64encode(struct.pack(f'<{len(out)}H', *out)).decode('ascii')


def unpack_ranges(text: str) -> List[float]:
    data = base64.b64decode(text)
    return [v / 100.0 if v else math.inf for v in struct.unpack(f'<{len(data) // 2}H', data)]


class ReplayBuilder:
    """Feed it the bag in order (``add``); ``result()`` is replay.json's content.

    ``encode_image(msg)`` returns JPEG bytes or None; ``save_image(i, data)``
    stores one. Both are injected so the builder runs without OpenCV.
    """

    def __init__(self, rate: float = 10.0, scan_rate: float = 5.0,
                 image_fps: float = 4.0,
                 encode_image: Optional[Callable] = None,
                 save_image: Optional[Callable] = None):
        self.rate = rate
        self.scan_period = 1.0 / scan_rate
        self.image_period = 1.0 / image_fps if image_fps > 0 else math.inf
        self._encode = encode_image
        self._save = save_image
        self.now = None          # latest /clock (sim s)
        self.t0 = None
        self._latest: Dict[str, object] = {
            'robot': None, 'person': None, 'person2': None, 'guard': 0,
            'intruder': 0, 'track': None, 'mode': ''}
        self._next_frame = None
        self._next_scan = None
        self._next_image = None
        self.frames = {k: [] for k in
                       ('t', 'robot', 'person', 'person2', 'guard', 'intruder', 'track', 'mode')}
        self.scans: List[dict] = []
        self.scan_info = None
        self.plans: List[dict] = []
        self.sweeps: List[dict] = []
        self.events: List[dict] = []
        self.images: List[dict] = []
        self._sweep: List[List[float]] = []
        self.guard_states = list(GUARD_STATES)

    def _t(self) -> float:
        return round(self.now - self.t0, 3)

    # -- input ----------------------------------------------------------------

    def add(self, topic: str, msg) -> None:
        if topic == '/clock':
            t = msg.clock.sec + msg.clock.nanosec * 1e-9
            if t <= 0.0:
                return
            if self.t0 is None:
                self.t0 = t
                self._next_frame = self._next_scan = self._next_image = t
            self.now = max(self.now or t, t)
            self._sample()
            return
        if self.now is None:
            return  # nothing before the simulation's clock runs
        handler = getattr(self, '_on_' + topic.strip('/').replace('/', '_'), None)
        if handler is not None:
            handler(msg)

    def _sample(self):
        """One frame per 1/rate s of simulation time, once the robot is known."""
        while self._next_frame is not None and self.now >= self._next_frame:
            self._next_frame += 1.0 / self.rate
            if self._latest['robot'] is None:
                continue
            f = self.frames
            f['t'].append(round(self._next_frame - 1.0 / self.rate - self.t0, 3))
            for key in ('robot', 'person', 'person2', 'guard', 'intruder', 'track', 'mode'):
                f[key].append(self._latest[key])

    def _on_odom(self, msg):
        self._latest['robot'] = _pose(msg)

    def _on_person_odom(self, msg):
        self._latest['person'] = _pose(msg)

    def _on_person2_odom(self, msg):
        self._latest['person2'] = _pose(msg)

    def _on_person_mode(self, msg):
        self._latest['mode'] = str(msg.data)

    def _on_security_guard_state(self, msg):
        name = str(msg.data)
        if name not in self.guard_states:
            self.guard_states.append(name)
        self._latest['guard'] = self.guard_states.index(name)

    def _on_intruder_state(self, msg):
        name = str(msg.data)
        self._latest['intruder'] = INTRUDER_STATES.index(name) if name in INTRUDER_STATES else 0
        if name in ('none', 'lost'):
            self._latest['track'] = None if name == 'none' else self._latest['track']

    def _on_intruder_track(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        c = msg.pose.covariance
        a, b, yaw = ellipse(c[0], c[1], c[7])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest['track'] = [round(p.x, 3), round(p.y, 3), round(v.x, 3), round(v.y, 3),
                                 round(a, 3), round(b, 3), round(yaw, 3),
                                 round(stamp - self.t0, 3) if stamp > 0 else self._t()]

    def _on_scan(self, msg):
        if self.now < self._next_scan:
            return
        self._next_scan = self.now + self.scan_period
        if self.scan_info is None:
            self.scan_info = {'angle_min': float(msg.angle_min),
                              'angle_increment': float(msg.angle_increment),
                              'samples': len(msg.ranges)}
        self.scans.append({'t': self._t(), 'r': pack_ranges(msg.ranges, float(msg.range_max))})

    def _on_plan(self, msg):
        pts = [[round(ps.pose.position.x, 2), round(ps.pose.position.y, 2)] for ps in msg.poses]
        if len(pts) > 200:
            step = len(pts) / 200.0
            pts = [pts[int(i * step)] for i in range(200)] + [pts[-1]]
        self.plans.append({'t': self._t(), 'points': pts})

    def _on_sweep_viewpoints(self, msg):
        points = []
        for m in msg.markers:
            if m.action == 3:  # DELETEALL
                points = []
            elif getattr(m, 'ns', '') == 'sweep':
                p = m.pose.position
                points.append([round(p.x, 2), round(p.y, 2), round(_yaw(m.pose.orientation), 2)])
        if points != self._sweep:
            self._sweep = points
            self.sweeps.append({'t': self._t(), 'points': points})

    def _on_security_events(self, msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        event.pop('wall', None)
        event.pop('snapshot', None)
        event['t'] = self._t()
        self.events.append(event)

    def _on_person_tracker_image(self, msg):
        if self._encode is None or self.now < self._next_image:
            return
        self._next_image = self.now + self.image_period
        data = self._encode(msg)
        if data is None:
            return
        index = len(self.images)
        name = f'images/{index:05d}.jpg'
        if self._save is not None:
            self._save(name, data)
        self.images.append({'t': self._t(), 'file': name})

    # -- output ------------------------------------------------------------------

    def result(self, meta: Optional[dict] = None) -> dict:
        out = dict(meta or {})
        out.update({
            'version': 1,
            'duration': self.frames['t'][-1] if self.frames['t'] else 0.0,
            'rate': self.rate,
            'guard_states': self.guard_states,
            'intruder_states': INTRUDER_STATES,
            'frames': self.frames,
            'scan': self.scan_info,
            'scans': self.scans,
            'plans': self.plans,
            'sweeps': self.sweeps,
            'events': self.events,
            'images': self.images,
        })
        return out


def report_meta(report: Optional[dict], scenarios_file: Optional[str] = None) -> dict:
    """Return the report's headline numbers and the scenario's description."""
    meta = {}
    if report:
        name = report.get('scenario', '')
        meta['scenario'] = name
        if report.get('outcome') != 'completed':
            meta['result'] = str(report.get('outcome'))
        else:
            meta['result'] = 'PASS' if report.get('passed') else 'FAIL'
        keys = ('time_to_detect_s', 'in_view_pct', 'tracked_in_view_pct', 'track_breaks_per_min',
                'false_alarms', 'track_switches', 'patrol_coverage_pct', 'real_time_factor')
        meta['metrics'] = {k: report.get(k) for k in keys if report.get(k) is not None}
        meta['failures'] = list(report.get('failures', []))
    if scenarios_file and meta.get('scenario'):
        try:
            from my_bot.scenario_script import load_scenarios
            scenario = load_scenarios(scenarios_file).get(meta['scenario'])
            if scenario is not None:
                meta['description'] = ' '.join(str(scenario.description).split())
        except (OSError, ValueError, KeyError):
            pass
    return meta


# ---------------------------------------------------------------------------
# ROS 2 side: read the bag, encode images
# ---------------------------------------------------------------------------

def _storage_id(bag_dir: str) -> str:
    """storage_identifier from the bag's metadata.yaml (sqlite3 or mcap)."""
    try:
        with open(os.path.join(bag_dir, 'metadata.yaml')) as f:
            for line in f:
                if 'storage_identifier:' in line:
                    return line.split(':', 1)[1].strip()
    except OSError:
        pass
    return 'sqlite3'


def read_bag(bag_dir: str, topics=TOPICS):
    """Yield (topic, message) in recorded order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=_storage_id(bag_dir)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, classes[topic])


def jpeg_encoder(quality: int = 70):
    import cv2

    def encode(msg):
        channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
        if channels is None:
            return None
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = frame.reshape(msg.height, msg.step)[:, :msg.width * channels]
        frame = frame.reshape(msg.height, msg.width, channels)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else None
    return encode


def export(run_dir: str, out_dir: str, image_fps: float = 4.0, quality: int = 70,
           scenarios_file: Optional[str] = None, messages=None) -> dict:
    """Write ``out_dir``/replay.json (+ images/) from ``run_dir``/bag."""
    report = None
    try:
        with open(os.path.join(run_dir, 'report.json')) as f:
            report = json.load(f)
    except (OSError, ValueError):
        pass
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(os.path.join(out_dir, 'images'))

    def save(name, data):
        with open(os.path.join(out_dir, name), 'wb') as f:
            f.write(data)

    builder = ReplayBuilder(image_fps=image_fps,
                            encode_image=jpeg_encoder(quality) if image_fps > 0 else None,
                            save_image=save)
    for topic, msg in (messages if messages is not None
                       else read_bag(os.path.join(run_dir, 'bag'))):
        builder.add(topic, msg)
    replay = builder.result(report_meta(report, scenarios_file))
    with open(os.path.join(out_dir, 'replay.json'), 'w') as f:
        json.dump(replay, f, separators=(',', ':'))
    if not replay['images']:
        os.rmdir(os.path.join(out_dir, 'images'))
    return replay


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog='python3 -m my_bot.replay_export',
        description='Export a scenario run (bag + report) as a web demo replay.')
    parser.add_argument('run_dir', help='scenario output: contains bag/ and report.json')
    parser.add_argument('out_dir', help='replay directory to write (replaced)')
    parser.add_argument('--image-fps', type=float, default=4.0,
                        help='camera frames per simulated second (0 = none)')
    parser.add_argument('--quality', type=int, default=70, help='JPEG quality')
    parser.add_argument('--scenarios-file', default='',
                        help="config/scenarios.yaml, for the scenario's description")
    args = parser.parse_args(argv)
    if not os.path.isdir(os.path.join(args.run_dir, 'bag')):
        print(f'no bag in {args.run_dir} (record:=true?)', file=sys.stderr)
        return 1
    scenarios = args.scenarios_file
    if not scenarios:
        try:
            from ament_index_python.packages import get_package_share_directory
            scenarios = os.path.join(get_package_share_directory('my_bot'),
                                     'config', 'scenarios.yaml')
        except Exception:  # noqa: BLE001 - outside a sourced workspace
            scenarios = ''
    replay = export(args.run_dir, args.out_dir, args.image_fps, args.quality, scenarios)
    size = sum(os.path.getsize(os.path.join(root, f))
               for root, _, files in os.walk(args.out_dir) for f in files)
    print(f'{args.out_dir}: {replay["duration"]:.0f} s, {len(replay["frames"]["t"])} frames, '
          f'{len(replay["scans"])} scans, {len(replay["images"])} images, '
          f'{len(replay["events"])} events, {size / 1e6:.1f} MB')
    return 0 if replay['frames']['t'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
