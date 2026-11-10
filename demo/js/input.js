// The visitor drives the person: WASD / arrow keys (relative to the view),
// Shift to sprint, click or tap the floor to walk there, or the on-screen
// stick on touch devices. Any of them takes over from the autopilot.

const KEYS = {
  KeyW: [1, 0], ArrowUp: [1, 0], KeyS: [-1, 0], ArrowDown: [-1, 0],
  KeyA: [0, 1], ArrowLeft: [0, 1], KeyD: [0, -1], ArrowRight: [0, -1],
};

export class Input {
  constructor(canvas, scene, onTakeOver) {
    this.scene = scene;
    this.onTakeOver = onTakeOver;
    this.down = new Set();
    this.stick = [0, 0]; // forward, left from the touch stick
    this.goal = null;
    this.run = false;

    window.addEventListener('keydown', (e) => {
      if (e.target instanceof HTMLInputElement || e.metaKey || e.ctrlKey) return;
      if (KEYS[e.code]) {
        this.down.add(e.code);
        this.goal = null;
        this.onTakeOver();
        e.preventDefault();
      }
      if (e.code === 'ShiftLeft' || e.code === 'ShiftRight') this.run = true;
    });
    window.addEventListener('keyup', (e) => {
      this.down.delete(e.code);
      if (e.code === 'ShiftLeft' || e.code === 'ShiftRight') this.run = false;
    });
    window.addEventListener('blur', () => { this.down.clear(); this.run = false; });

    // A click that is not a drag sends the person to that spot
    let press = null;
    canvas.addEventListener('pointerdown', (e) => {
      press = { x: e.clientX, y: e.clientY, t: performance.now() };
    });
    canvas.addEventListener('pointerup', (e) => {
      if (!press) return;
      const moved = Math.hypot(e.clientX - press.x, e.clientY - press.y);
      const quick = performance.now() - press.t < 400;
      press = null;
      if (moved > 6 || !quick) return;
      const point = scene.groundPoint(e.clientX, e.clientY);
      if (point && scene.world.clearance(point[0], point[1]) > 0.2) {
        this.goal = point;
        this.onTakeOver();
      }
    });
  }

  // Bind the on-screen stick (touch devices)
  bindStick(base, knob, runButton) {
    let id = null;
    const set = (e) => {
      const rect = base.getBoundingClientRect();
      let dx = (e.clientX - (rect.left + rect.width / 2)) / (rect.width / 2);
      let dy = (e.clientY - (rect.top + rect.height / 2)) / (rect.height / 2);
      const m = Math.hypot(dx, dy);
      if (m > 1) { dx /= m; dy /= m; }
      knob.style.transform = `translate(${dx * 32}px, ${dy * 32}px)`;
      this.stick = [-dy, -dx];
      this.goal = null;
      this.onTakeOver();
    };
    const release = () => {
      id = null;
      this.stick = [0, 0];
      knob.style.transform = '';
    };
    base.addEventListener('pointerdown', (e) => { id = e.pointerId; base.setPointerCapture(id); set(e); });
    base.addEventListener('pointermove', (e) => { if (e.pointerId === id) set(e); });
    base.addEventListener('pointerup', release);
    base.addEventListener('pointercancel', release);
    if (runButton) {
      runButton.addEventListener('pointerdown', () => { this.touchRun = true; });
      const stop = () => { this.touchRun = false; };
      runButton.addEventListener('pointerup', stop);
      runButton.addEventListener('pointercancel', stop);
      runButton.addEventListener('pointerleave', stop);
    }
  }

  // Write the wish into the simulation (world frame)
  apply(sim) {
    let fwd = this.stick[0], left = this.stick[1];
    for (const code of this.down) { fwd += KEYS[code][0]; left += KEYS[code][1]; }
    const m = Math.hypot(fwd, left);
    if (m > 1) { fwd /= m; left /= m; }
    const h = this.scene.viewHeading();
    sim.input.x = fwd * Math.cos(h) - left * Math.sin(h);
    sim.input.y = fwd * Math.sin(h) + left * Math.cos(h);
    sim.input.run = this.run || !!this.touchRun;
    if (m > 0) {
      sim.input.goal = null;
      this.goal = null;
    } else if (this.goal) {
      sim.input.goal = this.goal; // the simulation drops it on arrival
      this.goal = null;
    }
  }
}
