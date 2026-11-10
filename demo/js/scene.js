// The 3D view: the museum built from the saved map, the robot, the person,
// and the overlays RViz shows in the real project (scan, plan, track, zones).
// A second camera on the robot renders the 320x240 image its detector sees.
//
// Coordinates are ROS's: x forward, y left, z up, metres, map frame.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { CAMERA, LIDAR, PERSON } from './sim.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const L_MAIN = 1;   // only the big view (cut-away walls, overlays)
const L_ROBOT = 2;  // only the robot's camera (full-height walls)

export const STATE_COLOURS = {
  PatrolProtocol: 0x4f9dff,
  IntruderProtocol: 0xff4d5e,
  SearchProtocol: 0xffb020,
  SweepProtocol: 0xff7a2f,
  ReportProtocol: 0x9d7bff,
  EmergencyStop: 0xff2222,
  '': 0x66707f,
};

function onLayer(obj, layer) {
  obj.traverse((o) => o.layers.set(layer));
  return obj;
}

// Flat band of `width` along a polyline (WebGL lines are 1 px wide)
function ribbonGeometry(points, width, z = 0.03, closed = false) {
  const pts = closed ? [...points, points[0]] : points;
  const pos = [];
  for (let i = 0; i + 1 < pts.length; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1];
    const len = Math.hypot(bx - ax, by - ay);
    if (len < 1e-6) continue;
    const nx = -(by - ay) / len * width / 2, ny = (bx - ax) / len * width / 2;
    pos.push(ax + nx, ay + ny, z, ax - nx, ay - ny, z, bx + nx, by + ny, z,
      bx + nx, by + ny, z, ax - nx, ay - ny, z, bx - nx, by - ny, z);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  return g;
}

function flatMaterial(color, opacity = 1) {
  return new THREE.MeshBasicMaterial({
    color, transparent: opacity < 1, opacity, depthWrite: opacity >= 1, side: THREE.DoubleSide,
  });
}

function label(text, color = '#ffffff', scale = 1) {
  const canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 96;
  const ctx = canvas.getContext('2d');
  ctx.font = '600 44px system-ui, -apple-system, Segoe UI, sans-serif';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.lineWidth = 8; ctx.strokeStyle = 'rgba(8,12,20,0.85)';
  ctx.strokeText(text, 256, 48);
  ctx.fillStyle = color;
  ctx.fillText(text, 256, 48);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sprite.scale.set(3.2 * scale, 0.6 * scale, 1);
  sprite.renderOrder = 10;
  return sprite;
}

function arrowMesh(color) {
  const g = new THREE.Group();
  const shaft = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.07, 0.02), flatMaterial(color));
  shaft.position.x = 0.21;
  const head = new THREE.Mesh(new THREE.ConeGeometry(0.13, 0.26, 3), flatMaterial(color));
  head.rotation.z = -Math.PI / 2;
  head.scale.z = 0.15;
  head.position.x = 0.52;
  g.add(shaft, head);
  return g;
}

export class Scene3D {
  constructor(canvas, world) {
    this.canvas = canvas;
    this.world = world;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.scene = new THREE.Scene();

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.1, 200);
    this.camera.layers.enable(L_MAIN);
    this.camera.position.set(world.cx + 1, world.cy - 24, 23);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(world.cx, world.cy - 1.0, 0);
    this.controls.enableDamping = true;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.08;
    this.controls.minDistance = 3;
    this.controls.maxDistance = 60;
    this.controls.update();
    this.viewMode = 'overview';
    this._followPrev = null;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x3a4658, 1.3));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(6, -9, 14);
    this.scene.add(sun);

    this.layers = {}; // name -> Object3D the Layers menu toggles
    this._buildMuseum();
    this._buildRobot();
    this._buildPerson();
    this._buildOverlays();
    this._buildRobotCamera();
    this._raycaster = new THREE.Raycaster();
    this._ground = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    this.resize();
  }

  // -- the museum, from the occupancy grid -------------------------------------

  _wallMask() {
    const { w, h, free } = this.world;
    const n = w * h;
    // Non-free cells reachable from the image border are "outside"
    const outside = new Uint8Array(n);
    const stack = [];
    const push = (i) => { if (!free[i] && !outside[i]) { outside[i] = 1; stack.push(i); } };
    for (let c = 0; c < w; c++) { push(c); push((h - 1) * w + c); }
    for (let r = 0; r < h; r++) { push(r * w); push(r * w + w - 1); }
    while (stack.length) {
      const i = stack.pop(), c = i % w, r = (i - c) / w;
      if (c > 0) push(i - 1);
      if (c < w - 1) push(i + 1);
      if (r > 0) push(i - w);
      if (r < h - 1) push(i + w);
    }
    // Walls: blocked cells within 3 cells of free floor, plus enclosed solids
    let near = Uint8Array.from(free);
    for (let pass = 0; pass < 3; pass++) {
      const next = Uint8Array.from(near);
      for (let r = 0; r < h; r++) {
        for (let c = 0; c < w; c++) {
          const i = r * w + c;
          if (near[i]) continue;
          if ((c > 0 && near[i - 1]) || (c < w - 1 && near[i + 1]) ||
              (r > 0 && near[i - w]) || (r < h - 1 && near[i + w])) next[i] = 1;
        }
      }
      near = next;
    }
    const mask = new Uint8Array(n);
    for (let i = 0; i < n; i++) mask[i] = !free[i] && (near[i] || !outside[i]) ? 1 : 0;
    return mask;
  }

  _buildMuseum() {
    const world = this.world;
    const mask = this._wallMask();
    const rects = world.rectangles((c, r) => mask[r * world.w + c] === 1);
    const pos = [], norm = [];
    const quad = (a, b, c, d, n) => {
      pos.push(...a, ...b, ...c, ...a, ...c, ...d);
      for (let i = 0; i < 6; i++) norm.push(...n);
    };
    for (const [c, r, cols, rows] of rects) {
      const x0 = world.ox + c * world.res, x1 = x0 + cols * world.res;
      const y1 = world.oy + (world.h - r) * world.res, y0 = y1 - rows * world.res;
      quad([x0, y0, 1], [x1, y0, 1], [x1, y1, 1], [x0, y1, 1], [0, 0, 1]);
      quad([x0, y0, 0], [x1, y0, 0], [x1, y0, 1], [x0, y0, 1], [0, -1, 0]);
      quad([x1, y1, 0], [x0, y1, 0], [x0, y1, 1], [x1, y1, 1], [0, 1, 0]);
      quad([x1, y0, 0], [x1, y1, 0], [x1, y1, 1], [x1, y0, 1], [1, 0, 0]);
      quad([x0, y1, 0], [x0, y0, 0], [x0, y0, 1], [x0, y1, 1], [-1, 0, 0]);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('normal', new THREE.Float32BufferAttribute(norm, 3));

    // Big view: walls cut at knee height, so nothing hides the action
    const cut = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: 0x7f93b3 }));
    cut.scale.z = 0.55;
    this.scene.add(onLayer(cut, L_MAIN));
    // Robot's camera: the same walls at full height
    const tall = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: 0xd8d2c6 }));
    tall.scale.z = 3.2;
    this.scene.add(onLayer(tall, L_ROBOT));

    // Floor of the big view: the occupancy grid itself, with a 1 m grid
    const k = 4;
    const canvas = document.createElement('canvas');
    canvas.width = world.w * k; canvas.height = world.h * k;
    const ctx = canvas.getContext('2d');
    const img = ctx.createImageData(world.w, world.h);
    for (let i = 0; i < world.w * world.h; i++) {
      const f = world.free[i];
      img.data[i * 4] = f ? 28 : 10;
      img.data[i * 4 + 1] = f ? 36 : 13;
      img.data[i * 4 + 2] = f ? 52 : 20;
      img.data[i * 4 + 3] = 255;
    }
    const small = document.createElement('canvas');
    small.width = world.w; small.height = world.h;
    small.getContext('2d').putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(small, 0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = 'rgba(140,170,220,0.10)';
    ctx.lineWidth = 1;
    const perMetre = k / world.res;
    for (let x = Math.ceil(world.ox); x < world.ox + world.sizeX; x++) {
      const px = (x - world.ox) * perMetre;
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, canvas.height); ctx.stroke();
    }
    for (let y = Math.ceil(world.oy); y < world.oy + world.sizeY; y++) {
      const py = canvas.height - (y - world.oy) * perMetre;
      ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(canvas.width, py); ctx.stroke();
    }
    const tex = new THREE.CanvasTexture(canvas);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 4;
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(world.sizeX, world.sizeY), new THREE.MeshBasicMaterial({ map: tex }));
    floor.position.set(world.cx, world.cy, 0);
    this.scene.add(onLayer(floor, L_MAIN));
    const skirt = new THREE.Mesh(
      new THREE.PlaneGeometry(400, 400), new THREE.MeshBasicMaterial({ color: 0x0a0d14 }));
    skirt.position.set(world.cx, world.cy, -0.02);
    this.scene.add(onLayer(skirt, L_MAIN));

    const camFloor = new THREE.Mesh(
      new THREE.PlaneGeometry(world.sizeX + 20, world.sizeY + 20),
      new THREE.MeshLambertMaterial({ color: 0x9c9485 }));
    camFloor.position.set(world.cx, world.cy, 0);
    this.scene.add(onLayer(camFloor, L_ROBOT));
  }

  // -- robot (TurtleBot3 Waffle Pi proportions) -----------------------------------

  _buildRobot() {
    const g = new THREE.Group();
    const dark = new THREE.MeshLambertMaterial({ color: 0x2c313a });
    const plate = new THREE.MeshLambertMaterial({ color: 0x3d4450 });
    for (const z of [0.03, 0.075, 0.118]) {
      const p = new THREE.Mesh(new THREE.BoxGeometry(0.281, 0.306, 0.012), plate);
      p.position.set(-0.064, 0, z);
      g.add(p);
    }
    for (const [x, y] of [[0.05, 0.12], [0.05, -0.12], [-0.18, 0.12], [-0.18, -0.12]]) {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.014, 0.09), dark);
      post.position.set(x, y, 0.075);
      g.add(post);
    }
    for (const y of [0.144, -0.144]) {
      const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.033, 0.033, 0.018, 20), dark);
      wheel.position.set(0, y, 0.033);
      g.add(wheel);
    }
    const lidarBase = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.02, 20), dark);
    lidarBase.rotation.x = Math.PI / 2;
    lidarBase.position.set(LIDAR.x, 0, 0.134);
    const lidarTop = new THREE.Mesh(
      new THREE.CylinderGeometry(0.033, 0.033, 0.022, 20),
      new THREE.MeshLambertMaterial({ color: 0x1f6fd6 }));
    lidarTop.rotation.x = Math.PI / 2;
    lidarTop.position.set(LIDAR.x, 0, 0.155);
    const cam = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.06, 0.03), dark);
    cam.position.set(CAMERA.x, 0, CAMERA.z);
    g.add(lidarBase, lidarTop, cam);

    // What makes a 30 cm robot readable from across the hall
    this.halo = new THREE.Mesh(new THREE.RingGeometry(0.3, 0.37, 48), flatMaterial(0x4f9dff, 0.9));
    this.halo.position.z = 0.02;
    g.add(this.halo);
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.07, 0.16, 3), flatMaterial(0xffffff, 0.9));
    nose.rotation.z = -Math.PI / 2;
    nose.scale.z = 0.1;
    nose.position.set(0.45, 0, 0.02);
    g.add(nose);
    this.haloNose = nose;

    // Camera field of view on the floor, cut where walls block it
    this._fovSteps = 24;
    this._fovPos = new Float32Array(this._fovSteps * 9);
    const fovGeo = new THREE.BufferGeometry();
    fovGeo.setAttribute('position', new THREE.BufferAttribute(this._fovPos, 3));
    this.fov = new THREE.Mesh(fovGeo, flatMaterial(0xffffff, 0.07));
    this.fov.frustumCulled = false;
    this.fov.position.z = 0.015;
    g.add(this.fov);
    this.layers.fov = this.fov;

    this.robotLabel = label('Guard robot', '#9cc7ff', 0.8);
    this.robotLabel.position.z = 0.75;
    g.add(this.robotLabel);
    this.robot = onLayer(g, L_MAIN);
    this.scene.add(g);
  }

  // -- person ------------------------------------------------------------------

  _buildPerson() {
    const g = new THREE.Group();
    const jacket = new THREE.MeshLambertMaterial({ color: 0xff8a3d });
    const trousers = new THREE.MeshLambertMaterial({ color: 0x2f3b55 });
    const skin = new THREE.MeshLambertMaterial({ color: 0xe9b995 });
    const torso = new THREE.Mesh(new THREE.CapsuleGeometry(0.17, 0.42, 6, 14), jacket);
    torso.rotation.x = Math.PI / 2;
    torso.position.z = 1.16;
    torso.scale.set(0.72, 1, 1.12); // x is the body's depth: the person faces +x
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.115, 18, 14), skin);
    head.position.z = 1.6;
    g.add(torso, head);
    const limb = (radius, length, material, y, z) => {
      const pivot = new THREE.Group();
      pivot.position.set(0, y, z);
      const m = new THREE.Mesh(new THREE.CapsuleGeometry(radius, length - 2 * radius, 4, 10), material);
      m.rotation.x = Math.PI / 2;
      m.position.z = -length / 2;
      pivot.add(m);
      g.add(pivot);
      return pivot;
    };
    this.legs = [limb(0.075, 0.86, trousers, PERSON.legGap, 0.88),
      limb(0.075, 0.86, trousers, -PERSON.legGap, 0.88)];
    this.arms = [limb(0.05, 0.62, jacket, 0.225, 1.42), limb(0.05, 0.62, jacket, -0.225, 1.42)];
    this.person = g;
    this.scene.add(g); // layer 0: both cameras see the person

    this.personRing = new THREE.Mesh(new THREE.RingGeometry(0.32, 0.38, 40), flatMaterial(0xff8a3d, 0.85));
    this.personRing.position.z = 0.02;
    this.personLabel = label('You', '#ffb27a', 0.8);
    this.autoLabel = label('Intruder', '#ffb27a', 0.8);
    this.personLabel.position.z = this.autoLabel.position.z = 2.1;
    const extras = new THREE.Group();
    extras.add(this.personRing, this.personLabel, this.autoLabel);
    this.personExtras = onLayer(extras, L_MAIN);
    this.scene.add(extras);
    this.goalMarker = onLayer(new THREE.Mesh(
      new THREE.RingGeometry(0.12, 0.18, 24), flatMaterial(0xffb27a, 0.9)), L_MAIN);
    this.goalMarker.visible = false;
    this.scene.add(this.goalMarker);
  }

  // -- RViz-style overlays ----------------------------------------------------------

  _buildOverlays() {
    const add = (name, obj) => {
      this.layers[name] = obj;
      this.scene.add(onLayer(obj, L_MAIN));
      return obj;
    };
    // /scan
    const scanGeo = new THREE.BufferGeometry();
    this._scanPos = new Float32Array(LIDAR.samples * 3);
    scanGeo.setAttribute('position', new THREE.BufferAttribute(this._scanPos, 3));
    scanGeo.setDrawRange(0, 0);
    this.scanPoints = add('scan', new THREE.Points(scanGeo, new THREE.PointsMaterial({
      color: 0x5ee6ff, size: 4, sizeAttenuation: false })));
    this.scanPoints.frustumCulled = false;

    // Nav plan
    this.path = add('plan', new THREE.Mesh(new THREE.BufferGeometry(), flatMaterial(0x3ee08a, 0.9)));
    this.path.frustumCulled = false;

    // /intruder/markers: 2-sigma ellipse, body, velocity arrow
    const track = new THREE.Group();
    this.trackEllipse = new THREE.Mesh(new THREE.CircleGeometry(1, 40), flatMaterial(0xff4d5e, 0.28));
    this.trackEllipse.position.z = 0.025;
    this.trackBody = new THREE.Mesh(
      new THREE.CylinderGeometry(0.2, 0.2, 1.8, 20, 1, true),
      new THREE.MeshBasicMaterial({ color: 0xff4d5e, transparent: true, opacity: 0.22, depthWrite: false, wireframe: true }));
    this.trackBody.rotation.x = Math.PI / 2;
    this.trackBody.position.z = 0.9;
    this.trackArrow = arrowMesh(0xff4d5e);
    this.trackArrow.position.z = 0.06;
    this.trackGroup = new THREE.Group();
    this.trackGroup.add(this.trackBody, this.trackArrow);
    track.add(this.trackEllipse, this.trackGroup);
    this.track = add('track', track);
    this.trackInner = track;
    this.trackEllipse.visible = this.trackGroup.visible = false;

    // Line from the robot to what it follows
    this.followLine = add('follow', new THREE.Mesh(new THREE.BufferGeometry(), flatMaterial(0xff4d5e, 0.55)));
    this.followLine.frustumCulled = false;

    // Search goal and sweep viewpoints
    const search = new THREE.Group();
    this.searchGoal = new THREE.Mesh(new THREE.RingGeometry(0.22, 0.3, 4), flatMaterial(0xffb020, 0.95));
    this.searchGoal.position.z = 0.05;
    this.searchGoal.visible = false;
    this.sweep = new THREE.Group();
    search.add(this.searchGoal, this.sweep);
    add('search', search);

    this.sightings = add('sightings', new THREE.Group());
    this.zones = add('zones', new THREE.Group());
    this.restricted = add('restricted', new THREE.Group());
    this._sweepKey = '';
  }

  // Zones arrive with the robot's software (config/guard_zones.yaml)
  setZones(info) {
    this.zones.clear();
    this.restricted.clear();
    this._zoneMeshes = {};
    for (const [name, pts] of Object.entries(info.zones)) {
      const g = new THREE.Group();
      g.add(new THREE.Mesh(ribbonGeometry(pts, 0.05, 0.02, true), flatMaterial(0x4f9dff, 0.35)));
      for (const [x, y] of pts) {
        const dot = new THREE.Mesh(new THREE.CircleGeometry(0.11, 16), flatMaterial(0x4f9dff, 0.7));
        dot.position.set(x, y, 0.022);
        g.add(dot);
      }
      this._zoneMeshes[name] = g;
      this.zones.add(g);
    }
    for (const [name, pts] of Object.entries(info.restricted)) {
      const shape = new THREE.Shape(pts.map(([x, y]) => new THREE.Vector2(x, y)));
      const fill = new THREE.Mesh(new THREE.ShapeGeometry(shape), flatMaterial(0xff4d5e, 0.10));
      fill.position.z = 0.012;
      const edge = new THREE.Mesh(ribbonGeometry(pts, 0.07, 0.03, true), flatMaterial(0xff4d5e, 0.8));
      const text = label('restricted: ' + name.replace(/_/g, ' '), '#ff8f9a', 0.75);
      text.position.set(pts.reduce((s, p) => s + p[0], 0) / pts.length,
        pts.reduce((s, p) => s + p[1], 0) / pts.length, 0.9);
      this.restricted.add(fill, edge, text);
    }
    onLayer(this.zones, L_MAIN);
    onLayer(this.restricted, L_MAIN);
  }

  // -- the robot's camera -------------------------------------------------------------

  _buildRobotCamera() {
    this.robotCam = new THREE.PerspectiveCamera(
      THREE.MathUtils.radToDeg(CAMERA.vfov), CAMERA.width / CAMERA.height, 0.03, 60);
    this.robotCam.layers.enable(L_ROBOT);
    this.robotCam.layers.disable(L_MAIN);
    this._target = new THREE.WebGLRenderTarget(CAMERA.width, CAMERA.height);
    this._pixels = new Uint8Array(CAMERA.width * CAMERA.height * 4);
    this._frame = new ImageData(CAMERA.width, CAMERA.height);
    this._gamma = new Uint8ClampedArray(256);
    for (let i = 0; i < 256; i++) {
      const c = i / 255;
      this._gamma[i] = 255 * (c <= 0.0031308 ? 12.92 * c : 1.055 * Math.pow(c, 1 / 2.4) - 0.055);
    }
  }

  // Render what the robot sees; returns the frame as ImageData
  renderRobotCamera(sim) {
    const [cx, cy, yaw] = sim.cameraPose();
    this.robotCam.position.set(cx, cy, CAMERA.z);
    this.robotCam.lookAt(cx + Math.cos(yaw), cy + Math.sin(yaw), CAMERA.z);
    const r = this.renderer;
    r.setRenderTarget(this._target);
    r.setClearColor(0xbfd0e4, 1);
    r.clear();
    r.render(this.scene, this.robotCam);
    r.readRenderTargetPixels(this._target, 0, 0, CAMERA.width, CAMERA.height, this._pixels);
    r.setRenderTarget(null);
    const src = this._pixels, dst = this._frame.data, w = CAMERA.width, h = CAMERA.height;
    for (let y = 0; y < h; y++) {
      const from = (h - 1 - y) * w * 4, to = y * w * 4;
      for (let i = 0; i < w * 4; i += 4) {
        dst[to + i] = this._gamma[src[from + i]];
        dst[to + i + 1] = this._gamma[src[from + i + 1]];
        dst[to + i + 2] = this._gamma[src[from + i + 2]];
        dst[to + i + 3] = 255;
      }
    }
    return this._frame;
  }

  // -- per frame ----------------------------------------------------------------------

  update(sim) {
    const r = sim.robot, p = sim.person;
    this.robot.position.set(r.x, r.y, 0);
    this.robot.rotation.z = r.yaw;
    const far = this._distance() > 9;
    this.robotLabel.visible = far;
    this.personLabel.visible = far && !sim.autopilot;
    this.autoLabel.visible = far && sim.autopilot;

    this.person.position.set(p.x, p.y, 0);
    this.person.rotation.z = p.yaw;
    const amp = Math.min(1, p.speed / 1.2) * (p.speed > 1.8 ? 0.85 : 0.55);
    const swing = amp * Math.sin(p.phase);
    this.legs[0].rotation.y = -swing; this.legs[1].rotation.y = swing;
    this.arms[0].rotation.y = swing * 0.8; this.arms[1].rotation.y = -swing * 0.8;
    this.personExtras.position.set(p.x, p.y, 0);
    const goal = !sim.autopilot && sim.input.goal;
    this.goalMarker.visible = !!goal;
    if (goal) this.goalMarker.position.set(goal[0], goal[1], 0.03);

    if (sim.scanT !== this._scanT) {
      this._scanT = sim.scanT;
      const ox = r.x + LIDAR.x * Math.cos(r.yaw), oy = r.y + LIDAR.x * Math.sin(r.yaw);
      let n = 0;
      for (let i = 0; i < LIDAR.samples; i++) {
        const d = sim.scan[i];
        if (!isFinite(d)) continue;
        const a = r.yaw + LIDAR.angleMin + i * LIDAR.angleIncrement;
        this._scanPos[n * 3] = ox + d * Math.cos(a);
        this._scanPos[n * 3 + 1] = oy + d * Math.sin(a);
        this._scanPos[n * 3 + 2] = LIDAR.z;
        n++;
      }
      this.scanPoints.geometry.setDrawRange(0, n);
      this.scanPoints.geometry.attributes.position.needsUpdate = true;
    }

    // The wedge in the robot's frame, each ray as long as the map lets it be
    const [camX, camY] = sim.cameraPose();
    const steps = this._fovSteps, fp = this._fovPos;
    let prev = null;
    for (let i = 0; i <= steps; i++) {
      const a = -CAMERA.hfov / 2 + (i / steps) * CAMERA.hfov;
      const d = Math.min(8, this.world.raycast(camX, camY, r.yaw + a, 8));
      const point = [CAMERA.x + d * Math.cos(a), d * Math.sin(a)];
      if (prev) {
        const k = (i - 1) * 9;
        fp[k] = CAMERA.x; fp[k + 1] = 0; fp[k + 2] = 0;
        fp[k + 3] = prev[0]; fp[k + 4] = prev[1]; fp[k + 5] = 0;
        fp[k + 6] = point[0]; fp[k + 7] = point[1]; fp[k + 8] = 0;
      }
      prev = point;
    }
    this.fov.geometry.attributes.position.needsUpdate = true;

    const detected = !!sim.box;
    this.fov.material.color.setHex(detected ? 0x3ee08a : 0xffffff);
    this.fov.material.opacity = detected ? 0.16 : 0.08;

    if (this._viz) this._updateTrack(sim, this._viz);
    this._updateView(sim);
  }

  // Every frame: the track, extrapolated to now like the guard does
  _updateTrack(sim, viz) {
    const t = viz.track;
    const show = !!t && viz.intruder !== 'none';
    this.trackEllipse.visible = this.trackGroup.visible = show;
    if (show) {
      const c = viz.intruder === 'confirmed' ? 0xff4d5e : (viz.intruder === 'tentative' ? 0xffd23e : 0x8a93a3);
      const age = Math.min(1.0, Math.max(0, sim.t - t.t));
      const x = t.x + t.vx * age, y = t.y + t.vy * age;
      this.trackEllipse.position.set(x, y, 0.025);
      this.trackEllipse.rotation.z = t.yaw;
      this.trackEllipse.scale.set(Math.max(0.08, t.a), Math.max(0.08, t.b), 1);
      this.trackEllipse.material.color.setHex(c);
      this.trackGroup.position.set(x, y, 0);
      this.trackBody.material.color.setHex(c);
      const speed = Math.hypot(t.vx, t.vy);
      this.trackArrow.visible = speed > 0.15;
      this.trackArrow.rotation.z = Math.atan2(t.vy, t.vx);
      this.trackArrow.scale.x = Math.max(0.6, speed * 1.6);
      this.trackArrow.traverse((o) => { if (o.material) o.material.color.setHex(c); });
    }
    this.followLine.geometry.dispose();
    if (viz.follow && show) {
      const r = sim.robot;
      this.followLine.geometry = ribbonGeometry(
        [[r.x, r.y], [this.trackGroup.position.x, this.trackGroup.position.y]], 0.03, 0.04);
    } else {
      this.followLine.geometry = new THREE.BufferGeometry();
    }
    if (this.searchGoal.visible) this.searchGoal.rotation.z += 0.02;
  }

  // Once per message from the robot's software
  applyViz(sim, viz) {
    this._viz = viz;
    const colour = STATE_COLOURS[viz.state] ?? STATE_COLOURS[''];
    this.halo.material.color.setHex(colour);
    this.haloNose.material.color.setHex(colour);

    if (viz.path) {
      this.path.geometry.dispose();
      this.path.geometry = viz.path.length > 1 ? ribbonGeometry(viz.path, 0.06, 0.035) : new THREE.BufferGeometry();
    }

    this.searchGoal.visible = !!viz.search_goal;
    if (viz.search_goal) this.searchGoal.position.set(viz.search_goal[0], viz.search_goal[1], 0.05);
    const key = JSON.stringify(viz.sweep);
    if (key !== this._sweepKey) {
      this._sweepKey = key;
      this.sweep.clear();
      for (const [x, y, yaw] of viz.sweep) {
        const a = arrowMesh(0xff7a2f);
        a.position.set(x, y, 0.05);
        a.rotation.z = yaw;
        a.scale.setScalar(1.5);
        const ring = new THREE.Mesh(new THREE.RingGeometry(0.2, 0.26, 24), flatMaterial(0xff7a2f, 0.9));
        ring.position.set(x, y, 0.045);
        this.sweep.add(a, ring);
      }
      onLayer(this.sweep, L_MAIN);
    }

    for (const e of viz.events) {
      if ((e.event === 'intruder_detected' || e.event === 'intruder_reacquired') && e.x != null) {
        const pin = new THREE.Mesh(new THREE.SphereGeometry(0.13, 14, 10), flatMaterial(0xff4d5e, 0.75));
        pin.position.set(e.x, e.y, 0.5);
        const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.5, 6), flatMaterial(0xff4d5e, 0.75));
        stem.rotation.x = Math.PI / 2;
        stem.position.set(e.x, e.y, 0.25);
        this.sightings.add(pin, stem);
        onLayer(this.sightings, L_MAIN);
        while (this.sightings.children.length > 24) this.sightings.remove(this.sightings.children[0]);
      }
    }

    if (this._zoneMeshes && viz.zone !== this._zone) {
      this._zone = viz.zone;
      for (const [name, g] of Object.entries(this._zoneMeshes)) {
        g.traverse((o) => { if (o.material) o.material.opacity = name === viz.zone ? 0.75 : 0.22; });
      }
    }
  }

  clearRun() {
    this._viz = null;
    this._sweepKey = '';
    this.sweep.clear();
    this.searchGoal.visible = false;
    this.sightings.clear();
    this.path.geometry = new THREE.BufferGeometry();
    this.trackEllipse.visible = this.trackGroup.visible = false;
  }

  // -- view -----------------------------------------------------------------------------

  setView(mode) {
    this.viewMode = mode;
    this._followPrev = null;
    if (mode === 'overview') {
      this.camera.position.set(this.world.cx + 1, this.world.cy - 24, 23);
      this.controls.target.set(this.world.cx, this.world.cy - 1.0, 0);
    } else if (mode === 'top') {
      this.camera.position.set(this.world.cx, this.world.cy - 0.5, 34);
      this.controls.target.set(this.world.cx, this.world.cy, 0);
    }
  }

  _updateView(sim) {
    const who = this.viewMode === 'robot' ? sim.robot : (this.viewMode === 'person' ? sim.person : null);
    if (who) {
      if (!this._followPrev) {
        // Swing in behind whoever is followed, once
        const back = this.viewMode === 'robot' ? 4.5 : 6.5;
        this.controls.target.set(who.x, who.y, 0.6);
        this.camera.position.set(
          who.x - back * Math.cos(who.yaw), who.y - back * Math.sin(who.yaw), back * 0.8);
        this._followPrev = [who.x, who.y];
      }
      const dx = who.x - this._followPrev[0], dy = who.y - this._followPrev[1];
      this.controls.target.x += dx; this.controls.target.y += dy;
      this.camera.position.x += dx; this.camera.position.y += dy;
      this._followPrev = [who.x, who.y];
    }
    this.controls.update();
  }

  _distance() { return this.camera.position.distanceTo(this.controls.target); }

  // Heading (rad, map frame) the view looks along on the floor: "up" for the keys
  viewHeading() {
    const dx = this.controls.target.x - this.camera.position.x;
    const dy = this.controls.target.y - this.camera.position.y;
    if (Math.hypot(dx, dy) < 0.5) {
      // looking straight down: use the camera's own up direction
      const up = new THREE.Vector3(0, 1, 0).applyQuaternion(this.camera.quaternion);
      return Math.atan2(up.y, up.x);
    }
    return Math.atan2(dy, dx);
  }

  // Map point under a screen position, or null
  groundPoint(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this._raycaster.setFromCamera(ndc, this.camera);
    const hit = new THREE.Vector3();
    return this._raycaster.ray.intersectPlane(this._ground, hit) ? [hit.x, hit.y] : null;
  }

  setLayer(name, visible) {
    if (this.layers[name]) this.layers[name].visible = visible;
  }

  resize() {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  render() {
    this.renderer.setClearColor(0x0a0d14, 1);
    this.renderer.render(this.scene, this.camera);
  }
}
