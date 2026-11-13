// Recorded runs: the real stack (Gazebo, YOLOv8, Nav2, the ROS 2 nodes) on a
// scripted scenario, recorded with ros2 bag and exported by
// my_bot.replay_export. A replay is played back through the same 3D view and
// panels as the live demo: ReplayPlayer exposes the fields scene.js and ui.js
// read from the live simulation, plus the display state ("viz") the robot's
// software would send.

import { CAMERA, LIDAR } from './sim.js';

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// Index of the last entry with entry.t <= t (or -1)
function lastAtOrBefore(list, t, key = (e) => e.t) {
  let lo = 0, hi = list.length - 1, best = -1;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (key(list[mid]) <= t) { best = mid; lo = mid + 1; } else hi = mid - 1;
  }
  return best;
}

function decodeRanges(text) {
  const bin = atob(text);
  const n = bin.length >> 1;
  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    const v = bin.charCodeAt(2 * i) | (bin.charCodeAt(2 * i + 1) << 8);
    out[i] = v ? v / 100 : Infinity;
  }
  return out;
}

export async function loadIndex(base) {
  try {
    const res = await fetch(base + 'index.json');
    if (!res.ok) return [];
    return (await res.json()).replays || [];
  } catch (e) {
    return [];
  }
}

export class ReplayPlayer {
  constructor(data, base) {
    this.data = data;
    this.base = base;
    const f = data.frames;
    this.times = f.t;
    this.duration = data.duration;
    this.t = 0;
    this.speed = 1;
    this.playing = true;
    // What scene.update() and ui read from the live simulation
    this.robot = { x: 0, y: 0, yaw: 0, v: 0, w: 0 };
    this.person = { x: 0, y: 0, yaw: 0, speed: 0, phase: 0, stamina: 1, mode: '' };
    this.person2 = null;
    this.scan = new Float32Array(LIDAR.samples).fill(Infinity);
    this.scanT = -1;
    this.box = null;
    this.autopilot = true;
    this.input = { goal: null };
    this._scanIndex = -1;
    this._planIndex = -2;
    this._sweepIndex = -2;
    this._eventIndex = -1;
    this._imageIndex = -1;
    this._images = new Map();
    this._prev = null;
    if (data.scan && data.scan.samples !== LIDAR.samples) {
      this.scan = new Float32Array(data.scan.samples).fill(Infinity);
    }
    this.scanInfo = data.scan;
  }

  static async load(base) {
    const data = await (await fetch(base + 'replay.json')).json();
    return new ReplayPlayer(data, base);
  }

  cameraPose() {
    const r = this.robot;
    return [r.x + CAMERA.x * Math.cos(r.yaw), r.y + CAMERA.x * Math.sin(r.yaw), r.yaw];
  }

  get finished() { return this.t >= this.duration; }

  advance(dt) {
    if (!this.playing) return;
    this.t = Math.min(this.duration, this.t + dt * this.speed);
    if (this.finished) this.playing = false;
  }

  seek(t) {
    this.t = Math.max(0, Math.min(this.duration, t));
    // Everything incremental starts over from the beginning
    this._eventIndex = -1;
    this._planIndex = -2;
    this._sweepIndex = -2;
    this._prev = null;
  }

  // Pose state at this.t, interpolated between frames; returns viz for the panels
  update() {
    const f = this.data.frames;
    const t = this.t;
    let i = lastAtOrBefore(this.times, t, (x) => x);
    if (i < 0) i = 0;
    const j = Math.min(i + 1, this.times.length - 1);
    const span = this.times[j] - this.times[i];
    const k = span > 0 ? Math.max(0, Math.min(1, (t - this.times[i]) / span)) : 0;
    const lerp = (a, b) => (a && b ? [a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k,
      a[2] + wrap(b[2] - a[2]) * k] : a || b);

    const r = lerp(f.robot[i], f.robot[j]);
    const p = lerp(f.person[i], f.person[j]);
    if (this._prev) {
      const dt = Math.max(1e-3, t - this._prev.t);
      this.robot.v = Math.hypot(r[0] - this.robot.x, r[1] - this.robot.y) / dt;
      if (p) {
        const moved = Math.hypot(p[0] - this.person.x, p[1] - this.person.y);
        this.person.speed = Math.min(3, moved / dt);
        this.person.phase += moved * 3.4;
      }
    }
    this._prev = { t };
    Object.assign(this.robot, { x: r[0], y: r[1], yaw: r[2] });
    if (p) Object.assign(this.person, { x: p[0], y: p[1], yaw: p[2] });
    this.person.mode = f.mode[i] || '';
    const p2 = f.person2 ? lerp(f.person2[i], f.person2[j]) : null;
    this.person2 = p2 ? { x: p2[0], y: p2[1], yaw: p2[2] } : null;

    // Lidar
    const si = lastAtOrBefore(this.data.scans, t);
    if (si >= 0 && si !== this._scanIndex) {
      this._scanIndex = si;
      this.scan = decodeRanges(this.data.scans[si].r);
      this.scanT = this.data.scans[si].t;
    }

    const guard = this.data.guard_states[f.guard[i]] || '';
    const intruder = this.data.intruder_states[f.intruder[i]] || 'none';
    const tr = f.track[i];
    this.box = intruder === 'confirmed' ? [0, 0, 1, 1] : null; // camera wedge colour
    const viz = {
      state: guard,
      bt: null,
      zone: '',
      waypoint: 0,
      intruder,
      track: tr ? { x: tr[0], y: tr[1], vx: tr[2], vy: tr[3], a: tr[4], b: tr[5], yaw: tr[6], t: tr[7] } : null,
      target: null,
      source: '',
      follow: guard === 'IntruderProtocol' ? [] : null,
      search_goal: null,
      sweep: [],
      events: [],
      log: [],
      metrics: {},
    };

    const pi = lastAtOrBefore(this.data.plans, t);
    if (pi !== this._planIndex) {
      this._planIndex = pi;
      viz.path = pi >= 0 ? this.data.plans[pi].points : [];
    }
    const wi = lastAtOrBefore(this.data.sweeps, t);
    viz.sweep = wi >= 0 ? this.data.sweeps[wi].points : [];

    const ei = lastAtOrBefore(this.data.events, t);
    if (ei > this._eventIndex) {
      viz.events = this.data.events.slice(this._eventIndex + 1, ei + 1);
    }
    this._eventIndex = ei;
    return viz;
  }

  // The recorded camera frame for this.t: { index, url } or null
  image() {
    const i = lastAtOrBefore(this.data.images, this.t);
    if (i < 0) return null;
    return { index: i, url: this.base + this.data.images[i].file, t: this.data.images[i].t };
  }

  // A decoded <img> for frame i (cached), and the next few loading
  imageElement(i) {
    for (let k = i; k < Math.min(i + 12, this.data.images.length); k++) {
      if (!this._images.has(k)) {
        const img = new Image();
        img.decoding = 'async';
        img.src = this.base + this.data.images[k].file;
        this._images.set(k, img);
      }
    }
    if (this._images.size > 120) {
      for (const key of this._images.keys()) {
        if (key < i - 20 || key > i + 40) this._images.delete(key);
      }
    }
    return this._images.get(i);
  }
}
