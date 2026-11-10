// The museum as the robot knows it: the occupancy grid SLAM Toolbox saved
// (maps/my_map.pgm). The page uses the same grid for physics (collisions),
// the lidar (ray casting) and line of sight, so the browser world is exactly
// the world the robot's map describes.

const FREE_PIXEL = 250; // my_bot/clearance_map.py: only confidently free cells are free

export class World {
  constructor(meta, pixels) {
    this.res = meta.resolution;
    this.ox = meta.origin[0];
    this.oy = meta.origin[1];
    this.w = meta.width;
    this.h = meta.height;
    this.pixels = pixels; // row 0 = top = max y
    const n = this.w * this.h;
    this.free = new Uint8Array(n);
    for (let i = 0; i < n; i++) this.free[i] = pixels[i] >= FREE_PIXEL ? 1 : 0;
    this.clear = distanceTransform(this.free, this.w, this.h, this.res);
    this.sizeX = this.w * this.res;
    this.sizeY = this.h * this.res;
    this.cx = this.ox + this.sizeX / 2;
    this.cy = this.oy + this.sizeY / 2;
  }

  static async load(base) {
    const meta = await (await fetch(base + 'world.json')).json();
    const buf = new Uint8Array(await (await fetch(base + meta.image)).arrayBuffer());
    const { width, height, data } = parsePGM(buf);
    return new World({ ...meta, width, height }, data);
  }

  col(x) { return Math.floor((x - this.ox) / this.res); }
  row(y) { return this.h - 1 - Math.floor((y - this.oy) / this.res); }
  cellX(col) { return this.ox + (col + 0.5) * this.res; }
  cellY(row) { return this.oy + (this.h - 1 - row + 0.5) * this.res; }

  isFree(x, y) {
    const c = this.col(x), r = this.row(y);
    return c >= 0 && c < this.w && r >= 0 && r < this.h && this.free[r * this.w + c] === 1;
  }

  // Metres to the nearest blocked cell (0 inside one, or off the map)
  clearance(x, y) {
    const c = this.col(x), r = this.row(y);
    if (c < 0 || c >= this.w || r < 0 || r >= this.h) return 0;
    return this.clear[r * this.w + c];
  }

  // Distance along a ray until it leaves free space (at most maxRange)
  raycast(x, y, angle, maxRange) {
    const dx = Math.cos(angle), dy = Math.sin(angle);
    const step = this.res * 0.5;
    let d = 0;
    while (d < maxRange) {
      // Skip ahead by the known clearance: nothing is closer than that
      const c = this.clearance(x + dx * d, y + dy * d);
      if (c <= 0) {
        // refine back to the cell boundary
        let lo = Math.max(0, d - step), hi = d;
        for (let i = 0; i < 4; i++) {
          const mid = (lo + hi) / 2;
          if (this.isFree(x + dx * mid, y + dy * mid)) lo = mid; else hi = mid;
        }
        return hi;
      }
      d += Math.max(step, c - 1.5 * this.res);
    }
    return Infinity;
  }

  // True when the straight segment a -> b stays on free cells
  lineOfSight(ax, ay, bx, by) {
    const dist = Math.hypot(bx - ax, by - ay);
    if (dist < 1e-6) return this.isFree(ax, ay);
    return this.raycast(ax, ay, Math.atan2(by - ay, bx - ax), dist) >= dist;
  }

  // Move a disc from (x, y) by (dx, dy), sliding along walls
  moveDisc(x, y, dx, dy, radius) {
    if (this.clearance(x + dx, y + dy) >= radius) return [x + dx, y + dy];
    if (this.clearance(x + dx, y) >= radius) return [x + dx, y];
    if (this.clearance(x, y + dy) >= radius) return [x, y + dy];
    if (this.clearance(x, y) < radius) {
      // Already too close (pushed, or a tight start): let it move away
      const here = this.clearance(x, y);
      if (this.clearance(x + dx, y + dy) > here) return [x + dx, y + dy];
    }
    return [x, y];
  }

  // Rectangles [col, row, cols, rows] covering every cell where solid(col,row)
  rectangles(solid) {
    const rects = [];
    let open = new Map(); // "start:len" -> rect still growing downward
    for (let r = 0; r < this.h; r++) {
      const next = new Map();
      let c = 0;
      while (c < this.w) {
        if (!solid(c, r)) { c++; continue; }
        const start = c;
        while (c < this.w && solid(c, r)) c++;
        const key = start + ':' + (c - start);
        const grown = open.get(key);
        if (grown) { grown[3]++; next.set(key, grown); } else {
          const rect = [start, r, c - start, 1];
          rects.push(rect);
          next.set(key, rect);
        }
      }
      open = next;
    }
    return rects;
  }
}

function parsePGM(buf) {
  if (buf[0] !== 0x50 || buf[1] !== 0x35) throw new Error('map image is not a binary PGM');
  let pos = 2;
  const fields = [];
  const isSpace = (b) => b === 32 || b === 10 || b === 13 || b === 9;
  while (fields.length < 3) {
    while (isSpace(buf[pos])) pos++;
    if (buf[pos] === 35) { while (buf[pos] !== 10) pos++; continue; }
    let end = pos;
    while (!isSpace(buf[end])) end++;
    fields.push(parseInt(String.fromCharCode(...buf.subarray(pos, end)), 10));
    pos = end;
  }
  const [width, height] = fields;
  return { width, height, data: buf.subarray(pos + 1, pos + 1 + width * height) };
}

// Exact Euclidean distance transform (Felzenszwalb & Huttenlocher), in metres
function distanceTransform(free, w, h, res) {
  const INF = 1e12;
  const f = new Float64Array(w * h);
  for (let i = 0; i < w * h; i++) f[i] = free[i] ? INF : 0;
  const n = Math.max(w, h);
  const d = new Float64Array(n), v = new Int32Array(n), z = new Float64Array(n + 1);
  const line = new Float64Array(n);
  const pass = (len, get, set) => {
    for (let i = 0; i < len; i++) line[i] = get(i);
    let k = 0;
    v[0] = 0; z[0] = -INF; z[1] = INF;
    for (let q = 1; q < len; q++) {
      let s;
      for (;;) {
        const p = v[k];
        s = ((line[q] + q * q) - (line[p] + p * p)) / (2 * q - 2 * p);
        if (s <= z[k]) k--; else break;
      }
      k++;
      v[k] = q; z[k] = s; z[k + 1] = INF;
    }
    k = 0;
    for (let q = 0; q < len; q++) {
      while (z[k + 1] < q) k++;
      const p = v[k];
      d[q] = (q - p) * (q - p) + line[p];
    }
    for (let i = 0; i < len; i++) set(i, d[i]);
  };
  for (let c = 0; c < w; c++) pass(h, (i) => f[i * w + c], (i, val) => { f[i * w + c] = val; });
  for (let r = 0; r < h; r++) pass(w, (i) => f[r * w + i], (i, val) => { f[r * w + i] = val; });
  const out = new Float32Array(w * h);
  for (let i = 0; i < w * h; i++) out[i] = Math.sqrt(f[i]) * res;
  return out;
}
