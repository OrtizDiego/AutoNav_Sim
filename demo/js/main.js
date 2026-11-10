// Glue: the simulation runs on the main thread at 60 Hz of simulation time,
// the robot's software in a worker. Sensor data goes over at 30 Hz, velocity
// commands and display state come back.

import { World } from './world.js';
import { Sim } from './sim.js';
import { Scene3D } from './scene.js';
import { Input } from './input.js';
import { UI } from './ui.js';
import { Tour, howItWorks } from './tour.js';

const $ = (id) => document.getElementById(id);
const SEND_EVERY = 2; // simulation steps per message to the robot (30 Hz)

async function main() {
  const config = await (await fetch('config.json')).json();
  const world = await World.load('assets/');
  const sim = new Sim(world);
  let scene;
  try {
    scene = new Scene3D($('view'), world);
  } catch (err) {
    $('fatal').hidden = false;
    $('fatal').textContent = 'This demo needs WebGL, which this browser or device does not provide.';
    throw err;
  }

  let paused = false;
  let viz = null;
  let brainReady = false;
  let pending = 0;      // messages the robot has not answered yet
  let brainMs = 0;
  let epoch = 0;        // bumped on reset: answers from before it are dropped

  const ui = new UI(scene, {
    togglePause: () => { paused = !paused; return paused; },
    reset: () => reset(),
    autopilot: () => { sim.autopilot = true; sim.input.goal = null; },
    toggleEstop: () => { sim.estop = !sim.estop; return sim.estop; },
  });
  const input = new Input($('view'), scene, () => {
    if (sim.autopilot) { sim.autopilot = false; sim.person.mode = 'WALK'; sim.person.stamina = 1; }
  });
  if (window.matchMedia('(pointer: coarse)').matches) {
    $('stick').hidden = false;
    $('controls').querySelector('.keys').textContent = 'Use the stick to walk, tap the floor to walk there, drag to orbit.';
    input.bindStick($('stick-base'), $('stick-knob'), $('stick-run'));
  }

  const tour = new Tour($('tour'), ui, scene);
  $('btn-tour').addEventListener('click', () => (tour.open ? tour.close() : tour.start()));
  const modal = $('modal');
  const closeModal = () => { modal.hidden = true; modal.innerHTML = ''; };
  $('btn-how').addEventListener('click', () => {
    modal.innerHTML = howItWorks();
    modal.hidden = false;
    modal.querySelector('[data-close]').focus();
  });
  modal.addEventListener('click', (e) => { if (e.target === modal || e.target.closest('[data-close]')) closeModal(); });
  window.addEventListener('keydown', (e) => {
    if (e.code !== 'Escape') return;
    if (!modal.hidden) closeModal(); else if (tour.open) tour.close();
  });
  window.addEventListener('resize', () => scene.resize());
  new ResizeObserver(() => scene.resize()).observe($('view'));

  // -- the robot's computer ----------------------------------------------------
  const worker = new Worker('js/brain-worker.js');
  const boot = () => worker.postMessage({
    type: 'boot',
    pyodideURL: new URL(config.pyodide, location.href).href,
    bundleURL: new URL('robot.zip', location.href).href,
    startTime: sim.t,
    epoch,
  });
  worker.onmessage = (event) => {
    const m = event.data;
    if (m.type === 'status') {
      ui.boot(m.text);
    } else if (m.type === 'ready') {
      if (m.epoch !== epoch) return;
      brainReady = true;
      pending = 0;
      scene.setZones(m.info);
      ui.setInfo(m.info);
      ui.boot('The project\'s ROS 2 nodes are running in your browser.', 'ready');
    } else if (m.type === 'out') {
      pending = Math.max(0, pending - 1);
      if (m.epoch !== epoch) return;
      brainMs += (m.ms - brainMs) * 0.05;
      const out = m.out;
      if (out.cmd) sim.setCommand(out.cmd[0], out.cmd[1]);
      sim.personCmd = out.person_cmd;
      if (sim.autopilot) sim.person.mode = out.person_mode;
      viz = out.viz;
      scene.applyViz(sim, viz);
      ui.applyViz(sim, viz);
    } else if (m.type === 'error') {
      console.error('robot software:', m.text);
      if (!brainReady) ui.boot(m.text.split('\n').slice(-2).join(' ').slice(0, 200), 'error');
    }
  };
  worker.onerror = (e) => ui.boot(String(e.message || 'the worker failed to load'), 'error');
  ui.boot('Loading Python');
  boot();

  function reset() {
    epoch++;
    brainReady = false;
    viz = null;
    sim.reset();
    scene.clearRun();
    ui.boot('Restarting the ROS 2 nodes');
    worker.postMessage({ type: 'restart', startTime: sim.t, epoch });
  }

  // -- main loop ------------------------------------------------------------------
  let last = performance.now();
  let acc = 0;
  let stepCount = 0;
  let simRate = 1;
  let rateT = last, rateSim = sim.t;

  function frame(now) {
    requestAnimationFrame(frame);
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    input.apply(sim);
    let newFrame = false;
    if (!paused) {
      acc += dt;
      while (acc >= sim.dt) {
        acc -= sim.dt;
        sim.step();
        stepCount++;
        if (sim.frameDue) { sim.frameDue = false; newFrame = true; }
        if (brainReady && stepCount % SEND_EVERY === 0) {
          const { msg, scan } = sim.takeMessage();
          if (pending < 4) { // a busy robot computer drops data, like a real one
            pending++;
            worker.postMessage({ type: 'sense', msg, scan, epoch }, scan ? [scan.buffer] : []);
          }
        }
      }
    }
    scene.update(sim);
    if (newFrame) ui.drawCamera(scene.renderRobotCamera(sim), sim, viz);
    scene.render();
    if (now - rateT > 1000) {
      simRate = (sim.t - rateSim) / ((now - rateT) / 1000);
      rateT = now; rateSim = sim.t;
    }
    ui.update(sim, brainReady
      ? `sim ×${simRate.toFixed(2)} · robot software ${brainMs.toFixed(1)} ms/step`
      : `sim ×${simRate.toFixed(2)}`);
  }
  requestAnimationFrame(frame);

  // For the browser tests
  window.__demo = { sim, scene, world, get viz() { return viz; }, get ready() { return brainReady; } };

  if (!sessionStorage.getItem('autonav-tour-seen')) {
    try { sessionStorage.setItem('autonav-tour-seen', '1'); } catch (e) { /* private mode */ }
    tour.start();
  }
}

main().catch((err) => {
  console.error(err);
  const el = document.getElementById('fatal');
  if (el.hidden) { el.hidden = false; el.textContent = `The demo could not start: ${err.message}`; }
});
