"""A plain-Python stand-in for the page's simulation, to test the brain.

Same message format as js/sim.js sends: robot and person kinematics, a
ray-cast lidar on the saved map (plus the person as a disc) and the
person's box in the camera image (pinhole model, line of sight on the map).
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
PKG = os.path.join(REPO, 'src', 'my_bot')
for path in (os.path.join(os.path.dirname(HERE), 'py'), PKG):
    if path not in sys.path:
        sys.path.insert(0, path)

import brain as brain_module  # noqa: E402

IMG_W, IMG_H, HFOV = 320, 240, 1.089
FX = (IMG_W / 2) / math.tan(HFOV / 2)
CAM_X, CAM_Z = 0.14, 0.103
PERSON_H, PERSON_R = 1.72, 0.22
LIDAR_X = -0.064


class MiniSim:
    def __init__(self, person=(4.0, 3.0, 0.0), start_time=1.0):
        self.brain = brain_module.Brain(PKG, start_time)
        self.cmap = brain_module.demo_nav.DemoNavigator.cmap
        self.t = start_time
        self.robot = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.person = [person[0], person[1], person[2], 0.0]
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1.0
        self.autopilot = False
        self.estop = False
        self.person_vel = (0.0, 0.0)  # world vx, vy when not on autopilot
        self.last = None
        self.states = []
        self.events = []
        self.log = []
        self._n = 0

    def _scan(self):
        x, y, yaw = self.robot[:3]
        ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        ang = yaw + brain_module.SCAN_ANGLE_MIN + brain_module.SCAN_ANGLE_INCREMENT * np.arange(360)
        d = np.arange(0.12, 12.0, 0.04)
        xs = ox + np.outer(np.cos(ang), d)
        ys = oy + np.outer(np.sin(ang), d)
        blocked = self.cmap.clearance_many(xs.ravel(), ys.ravel()).reshape(xs.shape) <= 0.0
        hit = blocked.argmax(axis=1)
        ranges = np.where(blocked.any(axis=1), d[hit], np.inf)
        # the person as a disc
        px, py = self.person[0] - ox, self.person[1] - oy
        b = px * np.cos(ang) + py * np.sin(ang)
        disc = b * b - (px * px + py * py - PERSON_R ** 2)
        with np.errstate(invalid='ignore'):
            tp = b - np.sqrt(disc)
        ok = (disc >= 0) & (tp > 0.12)
        ranges = np.where(ok & (tp < ranges), tp, ranges)
        return ranges.tolist()

    def _box(self):
        x, y, yaw = self.robot[:3]
        cx, cy = x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw)
        dx, dy = self.person[0] - cx, self.person[1] - cy
        fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if fwd < 0.4 or fwd > 8.0:
            return None
        if self.cmap.segment_clearance((cx, cy), (self.person[0], self.person[1])) <= 0.0:
            return None
        u = IMG_W / 2 - FX * left / fwd
        half = FX * PERSON_R / fwd
        top = IMG_H / 2 - FX * (PERSON_H - CAM_Z) / fwd
        bottom = IMG_H / 2 + FX * CAM_Z / fwd
        x0, x1 = max(0.0, u - half), min(IMG_W - 1.0, u + half)
        y0, y1 = max(0.0, top), min(IMG_H - 1.0, bottom)
        if x1 - x0 < 3:
            return None
        return [x0, y0, x1 - x0, y1 - y0]

    def step(self, dt=1 / 30):
        self.t += dt
        self._n += 1
        v, w = self.cmd if self.t - self.cmd_t < 0.5 else (0.0, 0.0)
        r = self.robot
        nx, ny = r[0] + v * math.cos(r[2]) * dt, r[1] + v * math.sin(r[2]) * dt
        if self.cmap.clearance(nx, ny) > 0.18:
            r[0], r[1] = nx, ny
        r[2] += w * dt
        r[3], r[4] = v, w
        p = self.person
        if self.autopilot and self.last and self.last['person_cmd']:
            pv, pw = self.last['person_cmd']
            p[2] += pw * dt
            p[0] += pv * math.cos(p[2]) * dt
            p[1] += pv * math.sin(p[2]) * dt
            p[3] = pv
        else:
            p[0] += self.person_vel[0] * dt
            p[1] += self.person_vel[1] * dt
            p[3] = math.hypot(*self.person_vel)
        msg = {'t': self.t, 'robot': list(r), 'person': list(p), 'estop': self.estop}
        scan = None
        if self._n % 3 == 0:
            scan = self._scan()
        if self._n % 2 == 0:
            msg['bbox'] = {'t': self.t, 'box': self._box()}
        out = self.brain.step(msg, scan)
        if out['cmd'] is not None:
            self.cmd, self.cmd_t = out['cmd'], self.t
        self.last = out
        viz = out['viz']
        if not self.states or self.states[-1][1] != viz['state']:
            self.states.append((round(self.t, 1), viz['state']))
        self.events += viz['events']
        self.log += viz['log']
        return out

    def run(self, secs, until=None):
        end = self.t + secs
        while self.t < end:
            out = self.step()
            if until is not None and until(out):
                return True
        return until is None
