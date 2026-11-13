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

"""A scenario bag becomes a web demo replay (my_bot.replay_export)."""

import json
import math
import os
from types import SimpleNamespace as NS

import numpy as np

from my_bot import replay_export as rx

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def clock(t):
    return NS(clock=NS(sec=int(t), nanosec=int(round((t % 1) * 1e9))))


def stamp(t):
    return NS(sec=int(t), nanosec=int(round((t % 1) * 1e9)))


def odom(x, y, yaw=0.0, vx=0.0, vy=0.0, cov=(0.0, 0.0, 0.0), t=0.0):
    pose_cov = [0.0] * 36
    pose_cov[0], pose_cov[1], pose_cov[6], pose_cov[7] = cov[0], cov[1], cov[1], cov[2]
    return NS(header=NS(stamp=stamp(t)),
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0),
                              orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                             w=math.cos(yaw / 2))),
                      covariance=pose_cov),
              twist=NS(twist=NS(linear=NS(x=vx, y=vy, z=0.0))))


def text(data):
    return NS(data=data)


def scan(ranges):
    return NS(angle_min=-3.14, angle_increment=6.28 / 359, range_max=12.0, ranges=ranges)


def image(t):
    frame = np.zeros((240, 320, 3), np.uint8)
    frame[100:140, 150:170] = (0, 220, 0)
    return NS(header=NS(stamp=stamp(t)), encoding='bgr8', height=240, width=320,
              step=960, data=frame.tobytes())


def run(messages, **kwargs):
    builder = rx.ReplayBuilder(**kwargs)
    for topic, msg in messages:
        builder.add(topic, msg)
    return builder


def test_frames_follow_sim_time_from_zero():
    msgs = [('/odom', odom(5, 5)),  # before /clock: dropped
            ('/clock', clock(100.0)), ('/odom', odom(1.0, 2.0, 0.5)),
            ('/person/odom', odom(4.0, 3.0)), ('/security_guard/state', text('PatrolProtocol'))]
    t = 100.0
    for _ in range(20):
        t += 0.05
        msgs.append(('/clock', clock(t)))
    b = run(msgs)
    f = b.frames
    assert f['t'][0] == 0.1 and len(f['t']) == 10  # 1 s at 10 Hz, robot known from t=0.05
    assert f['robot'][0] == [1.0, 2.0, 0.5]
    assert f['person'][-1] == [4.0, 3.0, 0.0]
    assert b.result()['guard_states'][f['guard'][-1]] == 'PatrolProtocol'
    assert all(b2 - a >= 0.099 for a, b2 in zip(f['t'], f['t'][1:]))


def test_track_carries_its_2_sigma_ellipse():
    b = run([('/clock', clock(10.0)), ('/odom', odom(0, 0)),
             ('/intruder/state', text('confirmed')),
             ('/intruder/track', odom(3.0, 1.0, vx=0.5, cov=(0.04, 0.0, 0.01), t=10.0)),
             ('/clock', clock(10.2))])
    x, y, vx, vy, a, b_, yaw, t = b.frames['track'][-1]
    assert (x, y, vx, vy, t) == (3.0, 1.0, 0.5, 0.0, 0.0)
    assert math.isclose(a, 0.4, rel_tol=1e-3) and math.isclose(b_, 0.2, rel_tol=1e-3)
    assert math.isclose(abs(math.cos(yaw)), 1.0, abs_tol=1e-6)  # major axis along x
    assert b.result()['intruder_states'][b.frames['intruder'][-1]] == 'confirmed'


def test_scan_round_trip_and_rate():
    ranges = [1.234, math.inf, float('nan'), 0.0, 12.5] + [3.0] * 355
    msgs = [('/clock', clock(1.0))]
    for i in range(10):  # 10 Hz scans for 1 s -> 5 Hz kept
        msgs += [('/clock', clock(1.0 + i * 0.1)), ('/scan', scan(ranges))]
    b = run(msgs)
    assert len(b.scans) == 5
    back = rx.unpack_ranges(b.scans[0]['r'])
    assert back[0] == 1.23 and back[5] == 3.0
    assert all(math.isinf(v) for v in back[1:5])  # inf, nan, 0, beyond range_max
    assert b.scan_info['samples'] == 360


def test_events_plans_and_sweeps():
    plan = NS(poses=[NS(pose=NS(position=NS(x=i * 0.01, y=0.0))) for i in range(1000)])
    arrow = NS(action=0, ns='sweep', pose=NS(position=NS(x=1.0, y=2.0),
                                             orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
    b = run([('/clock', clock(5.0)), ('/plan', plan),
             ('/clock', clock(6.0)),
             ('/security_events', text(json.dumps(
                 {'event': 'intruder_detected', 'wall': 'x', 'snapshot': 'y', 'x': 1.0}))),
             ('/security_events', text('not json')),
             ('/sweep_viewpoints', NS(markers=[NS(action=3), arrow])),
             ('/sweep_viewpoints', NS(markers=[NS(action=3), arrow])),  # unchanged
             ('/sweep_viewpoints', NS(markers=[NS(action=3)]))])
    assert len(b.plans[0]['points']) == 201 and b.plans[0]['points'][-1] == [9.99, 0.0]
    assert b.events == [{'event': 'intruder_detected', 'x': 1.0, 't': 1.0}]
    assert [s['points'] for s in b.sweeps] == [[[1.0, 2.0, 0.0]], []]


def test_images_are_throttled_and_saved():
    saved = {}
    msgs = [('/clock', clock(1.0))]
    for i in range(30):  # 15 Hz for 2 s -> 4 per second
        msgs += [('/clock', clock(1.0 + i / 15)), ('/person_tracker/image', image(1.0))]
    b = run(msgs, image_fps=4.0, encode_image=lambda m: b'jpg',
            save_image=lambda name, data: saved.update({name: data}))
    assert 7 <= len(b.images) <= 9
    assert b.images[0] == {'t': 0.0, 'file': 'images/00000.jpg'}
    assert saved['images/00000.jpg'] == b'jpg'


def test_jpeg_encoder_keeps_the_picture():
    import cv2
    data = rx.jpeg_encoder(80)(image(1.0))
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert frame.shape == (240, 320, 3)
    assert frame[120, 160, 1] > 180 and frame[10, 10, 1] < 30
    assert rx.jpeg_encoder()(NS(encoding='32FC1')) is None


def test_export_writes_a_replay(tmp_path):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'report.json').write_text(json.dumps({
        'scenario': 'sprint', 'outcome': 'completed', 'passed': False,
        'failures': ['track_breaks_per_min 3.0 (max 1.0)'], 'time_to_detect_s': 1.5,
        'real_time_factor': 0.8, 'in_view_pct': None}))
    msgs = [('/clock', clock(2.0)), ('/odom', odom(0, 0))]
    for i in range(1, 21):
        msgs += [('/clock', clock(2.0 + i * 0.1)), ('/person_tracker/image', image(2.0))]
    out = tmp_path / 'replay'
    replay = rx.export(str(run_dir), str(out), image_fps=2.0,
                       scenarios_file=os.path.join(PKG, 'config', 'scenarios.yaml'),
                       messages=msgs)
    on_disk = json.loads((out / 'replay.json').read_text())
    assert on_disk == json.loads(json.dumps(replay))
    assert on_disk['result'] == 'FAIL' and on_disk['scenario'] == 'sprint'
    assert on_disk['metrics'] == {'time_to_detect_s': 1.5, 'real_time_factor': 0.8}
    assert on_disk['description'].startswith('Waits to be seen')
    assert len(on_disk['images']) == 4
    assert all((out / img['file']).stat().st_size > 500 for img in on_disk['images'])


def test_report_meta_outcomes():
    assert rx.report_meta({'outcome': 'timeout', 'scenario': 'x'})['result'] == 'timeout'
    assert rx.report_meta(None) == {}


def test_launch_records_what_the_replay_reads():
    """scenario.launch.py records every topic the exporter uses."""
    path = os.path.join(PKG, 'launch', 'scenario.launch.py')
    with open(path) as f:
        source = f.read()
    for topic in rx.TOPICS:
        assert f"'{topic}'" in source, topic
