// The "Recorded runs" mode: a run picker, the report's verdict and numbers,
// and a timeline (play, pause, speed, seek, event ticks).

import { ReplayPlayer } from './replay.js';

const $ = (id) => document.getElementById(id);

const EVENT_COLOURS = {
  intruder_detected: '#ff4d5e', intruder_reacquired: '#ff4d5e', intruder_lost: '#ffb020',
  restricted_area: '#9d7bff', incident_closed: '#3ee08a',
};

const METRICS = [
  ['time_to_detect_s', 'detected after', ' s'],
  ['tracked_in_view_pct', 'tracked while in view', ' %'],
  ['track_breaks_per_min', 'track breaks', '/min'],
  ['false_alarms', 'false alarms', ''],
  ['track_switches', 'track switches', ''],
  ['real_time_factor', 'sim speed', '×'],
];

const clock = (t) => `${Math.floor(t / 60)}:${String(Math.floor(t % 60)).padStart(2, '0')}`;
const nice = (name) => name.replace(/_/g, ' ').replace(/^\w/, (c) => c.toUpperCase());

export class ReplayMode {
  constructor({ scene, ui, base, list }) {
    this.scene = scene;
    this.ui = ui;
    this.base = base;
    this.list = list;
    this.player = null;
    this.active = false;
    this._loading = null;
    this._lastImage = -1;
    this._scrubbing = false;

    const select = $('rp-select');
    for (const run of list) {
      const option = document.createElement('option');
      option.value = run.name;
      option.textContent = nice(run.name) + (run.result ? ` · ${run.result}` : '');
      select.appendChild(option);
    }
    select.addEventListener('change', () => this.load(select.value));
    $('rp-play').addEventListener('click', () => this.togglePlay());
    $('rp-speed').addEventListener('change', (e) => {
      if (this.player) this.player.speed = Number(e.target.value);
    });
    const range = $('rp-time');
    range.addEventListener('input', () => {
      this._scrubbing = true;
      this.seek(Number(range.value));
    });
    range.addEventListener('change', () => { this._scrubbing = false; });
    window.addEventListener('keydown', (e) => {
      if (!this.active || e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
      if (e.code === 'Space') { this.togglePlay(); e.preventDefault(); }
      if (e.code === 'ArrowRight') this.seek(this.player.t + 5);
      if (e.code === 'ArrowLeft') this.seek(this.player.t - 5);
    });
  }

  async enter() {
    this.active = true;
    document.body.classList.add('replaying');
    $('replay').hidden = false;
    if (!this.player) await this.load($('rp-select').value || this.list[0].name);
    else this._restart();
  }

  exit() {
    this.active = false;
    document.body.classList.remove('replaying');
    $('replay').hidden = true;
  }

  async load(name) {
    const run = this.list.find((r) => r.name === name) || this.list[0];
    $('rp-select').value = run.name;
    this._loading = run.name;
    this.ui.clearEvents('Loading the recording…');
    let player;
    try {
      player = await ReplayPlayer.load(`${this.base}${run.name}/`);
    } catch (err) {
      this.ui.clearEvents(`Could not load ${run.name}: ${err.message}`);
      return;
    }
    if (this._loading !== run.name) return; // another run was picked meanwhile
    player.speed = Number($('rp-speed').value);
    this.player = player;
    this._describe(player.data);
    this._ticks(player.data);
    this._restart();
  }

  // Start a little before the first security event: the runs open with a
  // minute or more of patrol, which the timeline still has
  _startTime() {
    const first = this.player.data.events[0];
    return first ? Math.max(0, first.t - 10) : 0;
  }

  _restart() {
    this.seek(this._startTime());
    this.player.playing = true;
    $('rp-play').textContent = 'Pause';
  }

  togglePlay() {
    const p = this.player;
    if (!p) return;
    if (p.finished) p.seek(0);
    p.playing = !p.playing;
    $('rp-play').textContent = p.playing ? 'Pause' : 'Play';
    $('rp-play').setAttribute('aria-label', p.playing ? 'Pause' : 'Play');
  }

  seek(t) {
    const p = this.player;
    if (!p) return;
    p.seek(t);
    this.scene.clearRun();
    this.ui.clearEvents('No security events yet in this run.');
    this.ui.clearLog();
    this._lastImage = -1;
    this._apply(); // replays every event up to t
  }

  _describe(data) {
    const badge = $('rp-result');
    badge.textContent = data.result || '';
    badge.className = 'badge' + (data.result === 'PASS' ? ' pass' : data.result ? ' fail' : '');
    badge.title = data.result === 'PASS' ? 'Met every target in config/scenarios + behavior_params'
      : (data.failures || []).join('\n');
    $('rp-desc').textContent = data.description || '';
    const box = $('rp-metrics');
    box.innerHTML = '';
    const missed = (data.failures || []).join(' ');
    for (const [key, text, unit] of METRICS) {
      const v = data.metrics && data.metrics[key];
      if (v == null) continue;
      const span = document.createElement('span');
      if (missed.includes(key)) span.className = 'miss';
      span.innerHTML = `${text} <b>${typeof v === 'number' ? +v.toFixed(2) : v}${unit}</b>`;
      box.appendChild(span);
    }
    const range = $('rp-time');
    range.max = String(data.duration);
  }

  _ticks(data) {
    const el = $('rp-ticks');
    el.innerHTML = '';
    for (const e of data.events) {
      const tick = document.createElement('i');
      tick.style.left = `${(100 * e.t) / Math.max(1, data.duration)}%`;
      tick.style.background = EVENT_COLOURS[e.event] || '#8d9ab0';
      tick.title = `${clock(e.t)} ${e.event.replace(/_/g, ' ')}`;
      el.appendChild(tick);
    }
  }

  _apply() {
    const p = this.player;
    const viz = p.update();
    this.scene.applyViz(p, viz);
    this.ui.applyViz(p, viz);
    const frame = p.image();
    if (frame && frame.index !== this._lastImage) {
      const img = p.imageElement(frame.index);
      const status = `recorded ${clock(frame.t)} · YOLOv8n + tracker · track ${viz.intruder}`;
      if (img.complete) {
        this._lastImage = frame.index;
        this.ui.drawImage(img, status);
      } else {
        img.onload = () => {
          if (this.player === p && p.image() && p.image().index === frame.index) {
            this._lastImage = frame.index;
            this.ui.drawImage(img, status);
          }
        };
      }
    } else if (!frame && this._lastImage !== -2 && !p.data.images.length) {
      this._lastImage = -2;
      this.ui.drawImage(null, 'no camera frames in this recording');
    }
    if (!this._scrubbing) $('rp-time').value = String(p.t);
    $('rp-clock').textContent = `${clock(p.t)} / ${clock(p.duration)}`;
    if (p.finished && $('rp-play').textContent !== 'Replay') $('rp-play').textContent = 'Replay';
  }

  // Every animation frame while active
  frame(dt) {
    const p = this.player;
    if (!p) return null;
    p.advance(dt);
    this._apply();
    return p;
  }
}
