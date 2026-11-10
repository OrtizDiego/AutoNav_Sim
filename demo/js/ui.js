// The panels: what RViz, the terminal and `ros2 topic echo` show in the real
// project. Everything here is read from the robot's software, not computed
// by the page.

import { CAMERA } from './sim.js';

const $ = (id) => document.getElementById(id);

const STATES = {
  PatrolProtocol: { label: 'Patrol', colour: 'var(--blue)', note: 'scheduled zone waypoints' },
  IntruderProtocol: { label: 'Following intruder', colour: 'var(--red)', note: 'keeps 2.5 m away' },
  SearchProtocol: { label: 'Searching', colour: 'var(--amber)', note: 'where the track was heading' },
  SweepProtocol: { label: 'Sweeping rooms', colour: 'var(--orange)', note: 'viewpoints behind cover' },
  ReportProtocol: { label: 'Reporting', colour: 'var(--violet)', note: 'closes the incident' },
  EmergencyStop: { label: 'Emergency stop', colour: 'var(--red)', note: 'halts everything' },
};

const LAYERS = [
  ['scan', 'Lidar scan', '#5ee6ff'],
  ['fov', 'Camera field of view', '#ffffff'],
  ['track', 'Intruder track (EKF)', '#ff4d5e'],
  ['follow', 'Follow line', '#ff4d5e'],
  ['plan', 'Navigation plan', '#3ee08a'],
  ['search', 'Search goal + sweep viewpoints', '#ff7a2f'],
  ['zones', 'Patrol routes', '#4f9dff'],
  ['restricted', 'Restricted areas', '#ff4d5e'],
  ['sightings', 'Sightings', '#ff4d5e'],
];

const EVENTS = {
  intruder_detected: ['Intruder detected', 'var(--red)'],
  intruder_lost: ['Lost sight of the intruder', 'var(--amber)'],
  intruder_reacquired: ['Intruder found again', 'var(--red)'],
  restricted_area: ['Intruder in a restricted area', 'var(--violet)'],
  incident_closed: ['Incident closed', 'var(--green)'],
};

function clock(t) {
  const s = Math.max(0, Math.floor(t));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

export class UI {
  constructor(scene, handlers) {
    this.scene = scene;
    this.cam = $('robot-cam').getContext('2d');
    this._frame = null;
    this._rows = [];
    this._logLines = 0;

    // Layers menu
    const menu = $('layers');
    for (const [key, text, colour] of LAYERS) {
      const row = document.createElement('label');
      row.innerHTML = `<input type="checkbox" checked><i style="background:${colour}"></i>${text}`;
      const box = row.querySelector('input');
      box.dataset.key = key;
      box.addEventListener('change', (e) => scene.setLayer(key, e.target.checked));
      menu.appendChild(row);
    }
    $('btn-layers').addEventListener('click', (e) => {
      menu.hidden = !menu.hidden;
      e.currentTarget.setAttribute('aria-expanded', String(!menu.hidden));
    });
    document.addEventListener('pointerdown', (e) => {
      if (!menu.hidden && !e.target.closest('.menu')) menu.hidden = true;
    });

    for (const button of document.querySelectorAll('#views button')) {
      button.addEventListener('click', () => this.setView(button.dataset.view));
    }
    $('btn-pause').addEventListener('click', () => {
      const paused = handlers.togglePause();
      $('btn-pause').textContent = paused ? 'Resume' : 'Pause';
    });
    $('btn-reset').addEventListener('click', handlers.reset);
    $('btn-autopilot').addEventListener('click', handlers.autopilot);
    $('btn-estop').addEventListener('click', () => {
      const on = handlers.toggleEstop();
      $('btn-estop').classList.toggle('on', on);
      $('btn-estop').textContent = on ? 'Release emergency stop' : 'Emergency stop';
    });
  }

  setView(mode) {
    this.scene.setView(mode);
    for (const b of document.querySelectorAll('#views button')) b.classList.toggle('on', b.dataset.view === mode);
  }

  boot(text, state = 'loading') {
    const el = $('boot');
    el.classList.toggle('ready', state === 'ready');
    $('boot-title').textContent = state === 'ready' ? 'Robot computer running'
      : (state === 'error' ? 'Robot computer failed to start' : 'Robot computer starting');
    $('boot-text').textContent = text;
    if (state === 'ready') setTimeout(() => { el.hidden = true; }, 6000);
    else el.hidden = false;
  }

  fatal(text) {
    const el = $('fatal');
    el.hidden = false;
    el.textContent = text;
  }

  // The behaviour tree's shape, once the nodes are up
  setInfo(info) {
    const list = $('bt-tree');
    list.innerHTML = '';
    this._rows = info.tree.map((node) => {
      if (node.depth === 0) return null;
      const li = document.createElement('li');
      li.className = node.depth === 1 ? 'proto' : 'depth-2';
      const state = STATES[node.name];
      li.innerHTML = `<span class="dot"></span><span>${node.name}</span>` +
        (state ? `<small>${state.note}</small>` : '');
      li.dataset.name = node.name;
      list.appendChild(li);
      return li;
    });
    $('events').innerHTML = '<li class="empty">Nothing yet. Step into the robot\'s view.</li>';
    $('log').textContent = '';
    this._logLines = 0;
  }

  // The robot's camera image with the detection drawn on it, like
  // /sensor_fusion/image in the real project
  drawCamera(frame, sim, viz) {
    if (frame) this._frame = frame;
    if (!this._frame) return;
    const ctx = this.cam;
    ctx.putImageData(this._frame, 0, 0);
    const box = sim.box;
    const status = $('cam-status');
    if (!box) {
      status.textContent = 'no person in view';
      status.classList.remove('hit');
      return;
    }
    const [x, y, w, h] = box;
    ctx.strokeStyle = '#33e07f';
    ctx.lineWidth = 2;
    ctx.strokeRect(x, y, w, h);
    const target = viz && viz.target;
    let text = 'person';
    if (target && target[1] > 0) text += ` ${target[1].toFixed(2)} m`;
    ctx.font = '600 12px ui-monospace, monospace';
    const tw = ctx.measureText(text).width + 8;
    const ty = y > 18 ? y - 16 : y + 2;
    ctx.fillStyle = '#33e07f';
    ctx.fillRect(Math.min(x, CAMERA.width - tw), ty, tw, 15);
    ctx.fillStyle = '#04140b';
    ctx.fillText(text, Math.min(x, CAMERA.width - tw) + 4, ty + 11);
    if (target) {
      const deg = (target[0] * 180 / Math.PI).toFixed(1);
      status.textContent = `bearing ${deg > 0 ? '+' : ''}${deg}°` +
        (viz.source ? ` · range from ${viz.source}` : ' · no range');
      status.classList.add('hit');
    }
  }

  // Once per message from the robot's software
  applyViz(sim, viz) {
    const state = STATES[viz.state];
    const pill = $('state-pill');
    pill.textContent = state ? state.label : 'starting';
    pill.style.background = state ? state.colour : '';
    $('zone').textContent = viz.zone ? `zone: ${viz.zone.replace(/_/g, ' ')} · waypoint ${viz.waypoint + 1}` : '';

    viz.bt.forEach((status, i) => {
      const li = this._rows[i];
      if (!li) return;
      li.classList.toggle('running', status === 'RUNNING');
      li.classList.toggle('success', status === 'SUCCESS');
      if (li.classList.contains('proto')) li.classList.toggle('active', li.dataset.name === viz.state);
    });

    const trState = $('tr-state');
    trState.textContent = viz.intruder;
    trState.className = viz.intruder;
    const t = viz.track;
    if (t && viz.intruder !== 'none') {
      const r = sim.robot;
      const age = Math.min(1, Math.max(0, sim.t - t.t));
      const dx = t.x + t.vx * age - r.x, dy = t.y + t.vy * age - r.y;
      let bearing = Math.atan2(dy, dx) - r.yaw;
      bearing = Math.atan2(Math.sin(bearing), Math.cos(bearing));
      $('tr-range').textContent = `${Math.hypot(dx, dy).toFixed(2)} m`;
      $('tr-bearing').textContent = `${(bearing * 180 / Math.PI).toFixed(0)}°`;
      $('tr-speed').textContent = `${Math.hypot(t.vx, t.vy).toFixed(2)} m/s`;
      $('tr-sigma').textContent = `±${t.a.toFixed(2)} m`;
    } else {
      for (const id of ['tr-range', 'tr-bearing', 'tr-speed', 'tr-sigma']) $(id).textContent = '–';
    }
    $('tr-source').textContent = viz.source || '–';

    for (const e of viz.events) this._addEvent(e);
    for (const entry of viz.log) this._addLog(entry);
  }

  _addEvent(e) {
    const list = $('events');
    const empty = list.querySelector('.empty');
    if (empty) empty.remove();
    const [title, colour] = EVENTS[e.event] || [e.event, 'var(--muted)'];
    let detail = '';
    if (e.event === 'restricted_area') detail = `: ${String(e.area).replace(/_/g, ' ')}`;
    else if (e.event === 'incident_closed') detail = ` after ${Math.round(e.duration)} s, ${e.sightings} sighting${e.sightings === 1 ? '' : 's'}`;
    else if (e.x != null) detail = ` at (${e.x.toFixed(1)}, ${e.y.toFixed(1)})`;
    const li = document.createElement('li');
    li.innerHTML = `<time>${clock(e.t)}</time><span class="ev-dot" style="background:${colour}"></span>` +
      `<span>#${e.incident} ${title}${detail}</span>`;
    list.prepend(li);
    while (list.children.length > 40) list.lastChild.remove();
  }

  _addLog([t, level, node, text]) {
    if (text.startsWith('behaviour tree:') || text.startsWith('security event:')) return;
    const log = $('log');
    const line = document.createElement('div');
    if (level === 'warn' || level === 'error') line.className = 'w';
    line.innerHTML = `[${t.toFixed(1)}] <span class="n">${node}</span>: `;
    line.appendChild(document.createTextNode(text));
    log.appendChild(line);
    if (++this._logLines > 150) { log.firstChild.remove(); this._logLines--; }
    log.scrollTop = log.scrollHeight;
  }

  // Every frame
  update(sim, perf) {
    const p = sim.person;
    $('who').textContent = sim.autopilot ? 'Intruder: on autopilot' : 'You are the intruder';
    $('btn-autopilot').hidden = sim.autopilot;
    const bar = $('stamina');
    bar.style.width = `${Math.round((sim.autopilot ? 1 : p.stamina) * 100)}%`;
    bar.parentElement.classList.toggle('out', !sim.autopilot && p.mode === 'EXHAUSTED');
    $('perf').textContent = perf;
  }
}
