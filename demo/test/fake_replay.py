"""Write a replay without Gazebo, for testing the web player.

    python3 demo/test/fake_replay.py <out dir> [seconds]

Runs the demo's robot software on minisim.py (the person walks away, then
hides far off), turns what it publishes into the messages a real scenario
bag holds and passes them through my_bot.replay_export, so the file format
is exactly the exporter's. The images are drawn boxes, not camera renders.
Real replays come from `make demo-bag` or the smoke workflow's
replay-<scenario> artifacts.
"""

import json
import math
import os
import sys
import tempfile
from types import SimpleNamespace as NS

import numpy as np

from minisim import IMG_H, IMG_W, MiniSim

from my_bot import replay_export as rx  # noqa: I100 (minisim puts src/my_bot on the path)


def _stamp(t):
    return NS(sec=int(t), nanosec=int(round((t % 1) * 1e9)))


def _odom(x, y, yaw, t, vx=0.0, vy=0.0, cov=None):
    c = [0.0] * 36
    if cov is not None:
        c[0], c[1], c[6], c[7] = cov[0][0], cov[0][1], cov[1][0], cov[1][1]
    return NS(header=NS(stamp=_stamp(t)),
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0),
                              orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                             w=math.cos(yaw / 2))),
                      covariance=c),
              twist=NS(twist=NS(linear=NS(x=vx, y=vy, z=0.0))))


def _image(box, t):
    frame = np.full((IMG_H, IMG_W, 3), (200, 205, 210), np.uint8)
    frame[IMG_H // 2:] = (150, 160, 170)
    if box:
        x, y, w, h = (int(v) for v in box)
        frame[y:y + h, x:x + w] = (60, 120, 230)
        frame[y:y + 2, x:x + w] = frame[y + h - 2:y + h, x:x + w] = (0, 220, 0)
        frame[y:y + h, x:x + 2] = frame[y:y + h, x + w - 2:x + w] = (0, 220, 0)
    return NS(header=NS(stamp=_stamp(t)), encoding='bgr8', height=IMG_H, width=IMG_W,
              step=IMG_W * 3, data=frame.tobytes())


def messages(seconds):
    sim = MiniSim(person=(5.0, 6.0, 0.0), start_time=10.0)
    n = 0
    hidden = False
    while sim.t < 10.0 + seconds:
        if not hidden and sim.t > 22.0:
            sim.person_vel = (0.0, 0.0)
            sim.person[0], sim.person[1] = -7.0, -8.5  # out of sight
            hidden = True
        elif not hidden and sim.t > 16.0:
            sim.person_vel = (0.0, 0.9)
        out = sim.step()
        n += 1
        t = sim.t
        viz = out['viz']
        yield '/clock', NS(clock=_stamp(t))
        r = sim.robot
        yield '/odom', _odom(r[0], r[1], r[2], t)
        p = sim.person
        yield '/person/odom', _odom(p[0], p[1], p[2], t)
        yield '/security_guard/state', NS(data=viz['state'])
        yield '/intruder/state', NS(data=viz['intruder'])
        tr = viz['track']
        if tr and viz['intruder'] == 'confirmed':
            c, s = math.cos(tr['yaw']), math.sin(tr['yaw'])
            rot = np.array([[c, -s], [s, c]])
            cov = rot @ np.diag([(tr['a'] / 2) ** 2, (tr['b'] / 2) ** 2]) @ rot.T
            yield '/intruder/track', _odom(tr['x'], tr['y'], 0.0, tr['t'],
                                           tr['vx'], tr['vy'], cov.tolist())
        for e in viz['events']:
            yield '/security_events', NS(data=json.dumps(e))
        if 'path' in viz:
            yield '/plan', NS(poses=[NS(pose=NS(position=NS(x=x, y=y)))
                                     for x, y in viz['path']])
        markers = [NS(action=3)] + [
            NS(action=0, ns='sweep', pose=NS(position=NS(x=x, y=y), orientation=NS(
                x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))
            for x, y, yaw in viz['sweep']]
        yield '/sweep_viewpoints', NS(markers=markers)
        if n % 3 == 0:
            yield '/scan', NS(angle_min=-3.14, angle_increment=6.28 / 359,
                              range_max=12.0, ranges=sim._scan())
        if n % 2 == 0:
            yield '/person_tracker/image', _image(sim._box(), t)


def main():
    out = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    run_dir = tempfile.mkdtemp(prefix='fake_run_')
    with open(os.path.join(run_dir, 'report.json'), 'w') as f:
        json.dump({'scenario': 'synthetic', 'outcome': 'completed', 'passed': True,
                   'failures': [], 'time_to_detect_s': 1.0, 'tracked_in_view_pct': 95.0,
                   'real_time_factor': 1.0}, f)
    replay = rx.export(run_dir, out, image_fps=4.0, messages=messages(seconds))
    replay_path = os.path.join(out, 'replay.json')
    with open(replay_path) as f:
        data = json.load(f)
    data['description'] = ('Synthetic test recording: the demo\'s own simulation through '
                           'the exporter, not Gazebo.')
    with open(replay_path, 'w') as f:
        json.dump(data, f, separators=(',', ':'))
    print(f'{out}: {replay["duration"]:.0f} s, {len(replay["images"])} images, '
          f'{len(replay["events"])} events')


if __name__ == '__main__':
    main()
