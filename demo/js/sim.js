// What Gazebo does in the real project, reduced to what the robot's software
// can tell apart: a differential-drive robot and a walking person on the
// museum map, a 360-beam lidar, and the box a person detector would put
// around the person in the 320x240 camera image.

export const CAMERA = {
  width: 320, height: 240, hfov: 1.089, // urdf/camera.xacro
  x: 0.076, z: 0.103,                   // on the robot, metres from base_link
  rate: 15,
};
CAMERA.fx = (CAMERA.width / 2) / Math.tan(CAMERA.hfov / 2);
CAMERA.vfov = 2 * Math.atan(Math.tan(CAMERA.hfov / 2) * CAMERA.height / CAMERA.width);

export const LIDAR = {
  samples: 360, angleMin: -3.14, angleIncrement: 6.28 / 359, // urdf/lidar.xacro
  rangeMin: 0.12, rangeMax: 12.0, noise: 0.01, rate: 10,
  x: -0.064, z: 0.122,
};

export const PERSON = {
  height: 1.72, radius: 0.22, legRadius: 0.075, legGap: 0.11,
  walkSpeed: 1.3, runSpeed: 2.5, accel: 5.0,
  maxRunSecs: 3.5, recoverySecs: 8.0, // behavior_params.yaml: person_controller
};

const ROBOT_RADIUS = 0.17;
const DETECT_RANGE = 8.0;      // how far the detector still finds a person
const DETECT_LATENCY = 0.12;   // s from image to box (YOLO + tracker)
const CMD_TIMEOUT = 0.5;

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const toward = (v, target, maxStep) => v + clamp(target - v, -maxStep, maxStep);

function gauss() {
  let u = 0;
  while (u === 0) u = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random());
}

export class Sim {
  constructor(world) {
    this.world = world;
    this.dt = 1 / 60;
    this.reset();
  }

  reset() {
    this.t = 1.0; // simulation seconds; > 0 so every stamp is a valid ROS time
    this.robot = { x: 0, y: 0, yaw: 0, v: 0, w: 0 };
    this.person = { x: 4.6, y: 3.2, yaw: Math.PI, speed: 0, stamina: 1, mode: 'WALK', phase: 0 };
    this.cmd = { v: 0, w: 0, t: -1 };
    this.personCmd = null;     // autopilot: [speed, yaw rate] from person_controller
    this.autopilot = true;
    this.input = { x: 0, y: 0, run: false, goal: null }; // world-frame wish
    this.scan = new Float32Array(LIDAR.samples).fill(Infinity);
    this.scanT = 0;
    this.scanFresh = false;
    this.box = null;           // latest box delivered (after the detector's latency)
    this.boxT = 0;
    this._pendingBoxes = [];
    this._freshBoxes = [];
    this._nextScan = this.t;
    this._nextFrame = this.t;
    this.frameDue = false;
    this.estop = false;
  }

  setCommand(v, w) { this.cmd = { v, w, t: this.t }; }

  step() {
    const dt = this.dt;
    this.t += dt;
    this._stepRobot(dt);
    this._stepPerson(dt);
    if (this.t >= this._nextScan) {
      this._nextScan += 1 / LIDAR.rate;
      if (this._nextScan < this.t) this._nextScan = this.t;
      this._castScan();
    }
    if (this.t >= this._nextFrame) {
      this._nextFrame += 1 / CAMERA.rate;
      if (this._nextFrame < this.t) this._nextFrame = this.t;
      this.frameDue = true; // the page renders the robot's camera image now
      this._pendingBoxes.push({ t: this.t, due: this.t + DETECT_LATENCY, box: this._detect() });
    }
    while (this._pendingBoxes.length && this._pendingBoxes[0].due <= this.t) {
      const b = this._pendingBoxes.shift();
      this.box = b.box;
      this.boxT = b.t;
      this._freshBoxes.push({ t: b.t, box: b.box });
    }
  }

  // Sensor data since the last call, in the format brain.py takes
  takeMessage() {
    const r = this.robot, p = this.person;
    const msg = {
      t: this.t,
      robot: [r.x, r.y, r.yaw, r.v, r.w],
      person: [p.x, p.y, p.yaw, p.speed],
      estop: this.estop,
    };
    if (this._freshBoxes.length) msg.bbox = this._freshBoxes[this._freshBoxes.length - 1];
    this._freshBoxes = [];
    let scan = null;
    if (this.scanFresh) {
      this.scanFresh = false;
      scan = this.scan.slice();
      msg.scan_t = this.scanT;
    }
    return { msg, scan };
  }

  // -- robot: gazebo_ros_diff_drive ------------------------------------------

  _stepRobot(dt) {
    const r = this.robot;
    const live = this.t - this.cmd.t < CMD_TIMEOUT;
    r.v = toward(r.v, live ? this.cmd.v : 0, 2.5 * dt);
    r.w = toward(r.w, live ? this.cmd.w : 0, 6.0 * dt);
    r.yaw = wrap(r.yaw + r.w * dt);
    const dx = r.v * Math.cos(r.yaw) * dt, dy = r.v * Math.sin(r.yaw) * dt;
    let [nx, ny] = this.world.moveDisc(r.x, r.y, dx, dy, ROBOT_RADIUS);
    // The person is solid too
    const p = this.person;
    if (Math.hypot(nx - p.x, ny - p.y) < ROBOT_RADIUS + PERSON.radius &&
        Math.hypot(nx - p.x, ny - p.y) < Math.hypot(r.x - p.x, r.y - p.y)) {
      nx = r.x; ny = r.y;
    }
    if (nx === r.x && ny === r.y && Math.abs(r.v) > 0) r.v = 0;
    r.x = nx; r.y = ny;
  }

  // -- person: the visitor's keys, or person_controller's /person/cmd_vel -------

  _stepPerson(dt) {
    const p = this.person;
    let vx = 0, vy = 0;
    if (this.autopilot) {
      if (this.personCmd) {
        p.yaw = wrap(p.yaw + this.personCmd[1] * dt);
        p.speed = this.personCmd[0];
        vx = p.speed * Math.cos(p.yaw);
        vy = p.speed * Math.sin(p.yaw);
      } else {
        p.speed = 0;
      }
    } else {
      const inp = this.input;
      let wx = inp.x, wy = inp.y;
      if (wx === 0 && wy === 0 && inp.goal) {
        const gx = inp.goal[0] - p.x, gy = inp.goal[1] - p.y;
        const d = Math.hypot(gx, gy);
        if (d < 0.15) inp.goal = null; else { wx = gx / d; wy = gy / d; }
      }
      const mag = Math.hypot(wx, wy);
      const moving = mag > 0.05;
      // Sprinting drains stamina; out of breath, the person can only walk
      const wantRun = moving && inp.run && p.mode !== 'EXHAUSTED';
      if (wantRun) {
        p.stamina = Math.max(0, p.stamina - dt / PERSON.maxRunSecs);
        p.mode = p.stamina <= 0 ? 'EXHAUSTED' : 'RUN';
      } else {
        p.stamina = Math.min(1, p.stamina + dt / PERSON.recoverySecs);
        if (p.mode === 'EXHAUSTED' ? p.stamina >= 0.6 : true) p.mode = 'WALK';
      }
      const top = p.mode === 'RUN' ? PERSON.runSpeed : PERSON.walkSpeed;
      p.speed = toward(p.speed, moving ? top * Math.min(1, mag) : 0, PERSON.accel * dt);
      if (moving) {
        const want = Math.atan2(wy, wx);
        p.yaw = wrap(p.yaw + clamp(wrap(want - p.yaw), -7 * dt, 7 * dt));
      }
      vx = p.speed * Math.cos(p.yaw);
      vy = p.speed * Math.sin(p.yaw);
    }
    const [nx, ny] = this.world.moveDisc(p.x, p.y, vx * dt, vy * dt, PERSON.radius + 0.03);
    const moved = Math.hypot(nx - p.x, ny - p.y);
    if (!this.autopilot && moved < 0.2 * p.speed * dt) p.speed *= 0.8; // walked into a wall
    // Keep out of the robot
    const r = this.robot;
    const gap = Math.hypot(nx - r.x, ny - r.y);
    if (gap >= ROBOT_RADIUS + PERSON.radius || gap > Math.hypot(p.x - r.x, p.y - r.y)) {
      p.x = nx; p.y = ny;
      p.phase += moved * 3.4; // gait: one stride per ~1.85 m... two steps
    }
  }

  // Leg centres (the lidar plane is 0.12 m up: it sees legs, not a torso)
  legs() {
    const p = this.person;
    const swing = 0.16 * Math.min(1, p.speed / 1.0) * Math.sin(p.phase);
    const c = Math.cos(p.yaw), s = Math.sin(p.yaw);
    return [
      [p.x - s * PERSON.legGap + c * swing, p.y + c * PERSON.legGap + s * swing],
      [p.x + s * PERSON.legGap - c * swing, p.y - c * PERSON.legGap - s * swing],
    ];
  }

  // -- lidar ---------------------------------------------------------------

  _castScan() {
    const r = this.robot;
    const ox = r.x + LIDAR.x * Math.cos(r.yaw), oy = r.y + LIDAR.x * Math.sin(r.yaw);
    const legs = this.legs();
    for (let i = 0; i < LIDAR.samples; i++) {
      const a = r.yaw + LIDAR.angleMin + i * LIDAR.angleIncrement;
      const dx = Math.cos(a), dy = Math.sin(a);
      let d = this.world.raycast(ox, oy, a, LIDAR.rangeMax);
      for (const [lx, ly] of legs) {
        const px = lx - ox, py = ly - oy;
        const b = px * dx + py * dy;
        const disc = b * b - (px * px + py * py - PERSON.legRadius * PERSON.legRadius);
        if (disc >= 0) {
          const hit = b - Math.sqrt(disc);
          if (hit > 0 && hit < d) d = hit;
        }
      }
      if (d < LIDAR.rangeMin || d > LIDAR.rangeMax) d = Infinity;
      else d += LIDAR.noise * gauss();
      this.scan[i] = d;
    }
    this.scanT = this.t;
    this.scanFresh = true;
  }

  // -- camera + person detector ---------------------------------------------------
  //
  // The real pipeline runs YOLOv8n on the image. Here the box is computed
  // from where the person is: the part of the body the camera has a line of
  // sight to, projected with the camera's pinhole model, with pixel noise.
  // [x, y, w, h] in pixels, or null.

  cameraPose() {
    const r = this.robot;
    return [r.x + CAMERA.x * Math.cos(r.yaw), r.y + CAMERA.x * Math.sin(r.yaw), r.yaw];
  }

  _detect() {
    const [cx, cy, yaw] = this.cameraPose();
    const p = this.person;
    const dx = p.x - cx, dy = p.y - cy;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const fwd = dx * c + dy * s;
    if (fwd < 0.35 || fwd > DETECT_RANGE) return null;
    // Columns of the body the camera can see past the walls
    const dist = Math.hypot(dx, dy);
    const px = -dy / dist, py = dx / dist; // across the line of sight
    let uMin = Infinity, uMax = -Infinity, seen = 0;
    const N = 7;
    for (let k = 0; k < N; k++) {
      const f = (k / (N - 1)) * 2 - 1;
      const sx = p.x + px * PERSON.radius * f, sy = p.y + py * PERSON.radius * f;
      if (!this.world.lineOfSight(cx, cy, sx - (dx / dist) * 0.05, sy - (dy / dist) * 0.05)) continue;
      const sf = (sx - cx) * c + (sy - cy) * s;
      const sl = -(sx - cx) * s + (sy - cy) * c;
      const u = CAMERA.width / 2 - CAMERA.fx * sl / sf;
      const half = CAMERA.fx * (PERSON.radius / (N - 1)) / sf;
      uMin = Math.min(uMin, u - half);
      uMax = Math.max(uMax, u + half);
      seen++;
    }
    if (seen < 2) return null;
    let x0 = Math.max(0, uMin), x1 = Math.min(CAMERA.width - 1, uMax);
    let y0 = CAMERA.height / 2 - CAMERA.fx * (PERSON.height - CAMERA.z) / fwd;
    let y1 = CAMERA.height / 2 + CAMERA.fx * CAMERA.z / fwd;
    y0 = Math.max(0, y0); y1 = Math.min(CAMERA.height - 1, y1);
    if (x1 - x0 < 5 || y1 - y0 < 24) return null;
    // Very close, the camera (0.1 m up) sees legs only: detectors get unreliable
    if (fwd < 0.9 && Math.random() < 0.6) return null;
    if (Math.random() < 0.02) return null; // the odd missed frame
    const jitter = () => 1.2 * gauss();
    x0 = clamp(x0 + jitter(), 0, CAMERA.width - 2);
    x1 = clamp(x1 + jitter(), x0 + 1, CAMERA.width - 1);
    if (y0 > 0) y0 = Math.max(0, y0 + jitter());
    if (y1 < CAMERA.height - 1) y1 = Math.min(CAMERA.height - 1, y1 + jitter());
    return [x0, y0, x1 - x0, y1 - y0];
  }
}
