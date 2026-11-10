// The guided tour: what each part of the system is, how it works and why it
// was built that way, with the difference between the real project and
// this page stated at every step where there is one.

const REPO = 'https://github.com/OrtizDiego/AutoNav_Sim';

const STEPS = [
  {
    title: 'You are the intruder',
    body: `
      <p>This is <b>AutoNav Sim</b>: a security-guard robot that patrols a museum, spots a person,
      follows them, and searches the rooms when they get away. Here you play the person.</p>
      <p>The robot's decisions are not scripted for this page. The project's actual Python ROS 2 nodes
      run in your browser, compiled to WebAssembly, fed by a small simulation of the robot's sensors.</p>
      <p class="try">Walk with <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> or click the floor.
      Hold <kbd>Shift</kbd> to sprint. Until you do, the intruder walks on autopilot.</p>`,
  },
  {
    title: 'The world is the robot\'s own map',
    view: 'top',
    body: `
      <p>The floor plan you see is the occupancy grid the robot built by driving around with
      <b>SLAM Toolbox</b> (lidar scans + odometry, 5 cm cells). Walls are extruded from it.</p>
      <div class="pair">
        <div class="real"><b>Real project</b>Gazebo simulates the museum, physics, the robot and a walking
        person, in Docker with ROS 2 Humble.</div>
        <div class="here"><b>This page</b>A light simulation on that saved map: differential-drive
        kinematics, ray-cast lidar, collisions.</div>
      </div>
      <p><b>Why a simulator:</b> every behaviour can be replayed, scored and tested in CI before it
      touches hardware.</p>`,
  },
  {
    title: 'Two sensors: lidar and camera',
    spot: 'card-camera',
    layers: ['scan', 'fov'],
    view: 'robot',
    body: `
      <p>The <span style="color:var(--cyan)">cyan dots</span> are the 2D lidar: 360 beams, 10 times a
      second, 12 cm above the floor, so it sees walls and legs. The panel on the right is the camera:
      320×240 pixels at 15 Hz, mounted 10 cm above the floor.</p>
      <p><b>Why both:</b> the camera can tell a person from a statue but not how far away they are.
      The lidar measures distance to a centimetre but cannot tell what it hit. The rest of the
      pipeline exists to combine them.</p>`,
  },
  {
    title: 'Detection: finding the person in the image',
    spot: 'card-camera',
    body: `
      <div class="pair">
        <div class="real"><b>Real project</b><strong>YOLOv8n</strong>
        (ONNX Runtime) finds the person; an OpenCV KCF tracker and a Kalman filter carry the box between
        detections. YOLO runs in a worker thread so a slow inference never delays a frame.</div>
        <div class="here"><b>This page</b>The green box is computed from where you stand and what the
        walls hide, with pixel noise, a few missed frames and 120 ms of delay.</div>
      </div>
      <p><b>Why the delay matters:</b> by the time a box arrives, the robot has already turned.
      Every box carries the timestamp of its image so later stages can correct for that.</p>`,
  },
  {
    title: 'Sensor fusion: where is that box in the room?',
    spot: 'card-camera',
    body: `
      <p><code>sensor_fusion.py</code> turns the box into a bearing and a range. The bearing comes from
      the box centre through the camera's pinhole model. The range is a low percentile of the lidar
      beams inside the box's angular span, which finds the near surface even when beams slip between
      the legs.</p>
      <p>It is cross-checked against a camera-only estimate from the person's apparent height. If the
      lidar disagrees (its beams hit the wall behind), the camera estimate wins. The label under the
      image says which one was used.</p>`,
  },
  {
    title: 'Tracking: an EKF that knows where you are heading',
    spot: 'card-track',
    layers: ['track', 'follow'],
    view: 'overview',
    body: `
      <p>The <span style="color:var(--red)">red marker</span> is not your position. It is the robot's
      estimate: a constant-velocity <b>extended Kalman filter</b> (<code>track_filter.py</code>).
      The ellipse is its 2σ uncertainty, the arrow its velocity estimate.</p>
      <p>Each measurement is placed using where the robot was when the image was taken (odometry
      history), which removes the detection delay. A track is confirmed after 3 hits, rejects
      outliers with a chi-square gate, and is kept alive by person-sized lidar clusters when the
      camera drops out.</p>
      <p><b>Why:</b> steering on raw, stale detections made the robot overshoot; a track with
      velocity lets it lead a moving target.</p>`,
  },
  {
    title: 'Deciding what to do: a behaviour tree',
    spot: 'card-guard',
    body: `
      <p>The guard is a <b>py_trees</b> behaviour tree, ticked 10 times a second. Branches are tried
      top to bottom, so priority is the order: emergency stop, follow an intruder, search, sweep,
      report, and only then patrol. The lit rows are the ones running right now.</p>
      <p><b>Why a tree and not a state machine:</b> a higher-priority branch pre-empts a lower one
      without a transition having to be written for every pair of states, and each leaf is small
      enough to unit-test alone.</p>
      <p class="try">Step into the robot's view and watch the tree switch to IntruderProtocol.</p>`,
  },
  {
    title: 'Getting there: navigation',
    layers: ['plan', 'zones'],
    body: `
      <p>The <span style="color:var(--green)">green line</span> is the plan to the next goal. The
      <span style="color:var(--blue)">blue routes</span> are the patrol zones, cycled on a schedule
      from <code>guard_zones.yaml</code>.</p>
      <div class="pair">
        <div class="real"><b>Real project</b><strong>Nav2</strong>:
        AMCL localises on the map, a global planner finds the route, the DWB controller follows it
        around obstacles in live costmaps.</div>
        <div class="here"><b>This page</b>A stand-in behind the same interface: A* on the saved map and a
        look-ahead path follower. Odometry is exact, so no localisation is needed.</div>
      </div>
      <p>While following you the robot does not use the planner at all: a stand-off controller
      holds 2.5 m, with a feedforward term from the track's velocity.</p>`,
  },
  {
    title: 'When you get away: search, then sweep',
    spot: 'card-events',
    layers: ['search', 'restricted', 'sightings'],
    body: `
      <p>Lose the robot and it drives to where your track was <em>heading</em>, not where it last saw
      you. If that fails it plans a sweep: viewpoints chosen on the map by a greedy set cover so that
      between them they see the floor around your last position
      (<span style="color:var(--orange)">orange arrows</span>).</p>
      <p>Every sighting, loss and entry into a
      <span style="color:var(--red)">restricted area</span> is published as an event (and, in the
      real project, written to an incident log with a camera snapshot).</p>
      <p class="try">Try it: sprint around a corner with <kbd>Shift</kbd> and stand still behind
      a wall.</p>`,
  },
  {
    title: 'How it is tested',
    body: `
      <p>The trick that runs these nodes in your browser comes from the test suite: a fake of the
      ROS client library lets every node run under plain pytest without ROS installed.</p>
      <p>On top of that, scripted scenarios (walk in, sprint, hide, two people crossing) run in
      headless Gazebo on GitHub Actions and are scored against ground truth: time to detect,
      share of time tracked, track breaks, false alarms.</p>
      <p>Open <b>How it works</b> for the full pipeline and a table of what is real on this page,
      or read the code on <a href="${REPO}" target="_blank" rel="noopener" style="color:var(--blue)">GitHub</a>.</p>
      <p class="try">Now go and be a nuisance to the robot.</p>`,
  },
];

export function howItWorks() {
  return `
  <div class="sheet" role="dialog" aria-label="How it works">
    <button class="close" data-close>Close</button>
    <h3>How it works</h3>
    <p>The robot's software is the project's own Python, unchanged, running in a Web Worker on
    Pyodide (CPython compiled to WebAssembly). The page replaces only what cannot run in a
    browser: the simulator, the neural network and Nav2.</p>
    <div class="legend">
      <span><i style="border-color:var(--green)"></i>project code, running here</span>
      <span><i style="border-color:var(--amber)"></i>stand-in for this page</span>
    </div>
    <div class="pipeline">
      <div class="node sub"><b>Simulation</b>robot, person, lidar rays, camera image</div>
      <span class="arrow">→</span>
      <div class="node sub"><b>Person detector</b>box in the image, with noise and delay</div>
      <span class="arrow">→</span>
      <div class="node real"><b>sensor_fusion</b>box + lidar → bearing and range</div>
      <span class="arrow">→</span>
      <div class="node real"><b>target_tracker</b>EKF track in the odom frame</div>
      <span class="arrow">→</span>
      <div class="node real"><b>security_guard_bt</b>behaviour tree: follow, search, sweep, patrol</div>
      <span class="arrow">→</span>
      <div class="node sub"><b>Navigation</b>A* + path follower</div>
    </div>

    <h4>What is real on this page</h4>
    <table>
      <tr><th>Part</th><th>Real project</th><th>This page</th></tr>
      <tr><td>Robot behaviour</td><td>py_trees behaviour tree, <code>security_guard_bt.py</code></td><td>The same file, unchanged</td></tr>
      <tr><td>Tracking</td><td>Constant-velocity EKF, lidar clusters, latency compensation</td><td>The same files, unchanged</td></tr>
      <tr><td>Sensor fusion</td><td>Camera box + lidar range, monocular cross-check</td><td>The same file, unchanged</td></tr>
      <tr><td>Room sweep, zones, incident log</td><td>Set cover on the map, YAML zones, JSON events</td><td>The same files, unchanged</td></tr>
      <tr><td>Intruder autopilot</td><td><code>person_controller.py</code>: walks, flees when detected, tires</td><td>The same file, unchanged (until you take over)</td></tr>
      <tr><td>Map</td><td>Built with SLAM Toolbox</td><td>The same saved map</td></tr>
      <tr><td>OpenCV</td><td>Reads the map image, computes distance to walls</td><td>A two-function numpy stand-in, tested to give the same result (the real library is 49 MB in a browser)</td></tr>
      <tr><td>ROS 2 middleware</td><td>rclpy, DDS topics, TF</td><td>A small in-process fake: topics, timers, sim clock</td></tr>
      <tr><td>Simulator</td><td>Gazebo (physics, sensors, animated actor)</td><td>Kinematics and ray casting in JavaScript</td></tr>
      <tr><td>Person detection</td><td>YOLOv8n + KCF tracker + Kalman filter</td><td>Box computed from geometry and line of sight</td></tr>
      <tr><td>Navigation</td><td>Nav2: AMCL, planner, DWB controller, costmaps</td><td>A* on the map + look-ahead follower</td></tr>
      <tr><td>3D view</td><td>Gazebo client + RViz</td><td>three.js</td></tr>
    </table>

    <h4>Technologies in the project</h4>
    <ul>
      <li><b>ROS 2 Humble</b> for the node graph, parameters and launch files; everything runs in Docker.</li>
      <li><b>Gazebo</b> with a custom C++ actor plugin for the pedestrian, driven by velocity commands.</li>
      <li><b>SLAM Toolbox</b> for mapping, <b>Nav2</b> for localisation, planning and control, plus a frontier explorer that maps a new world on its own.</li>
      <li><b>YOLOv8n</b> on ONNX Runtime with an OpenCV tracker for person detection.</li>
      <li><b>py_trees</b> for the behaviour tree; <b>numpy</b> for the EKF.</li>
      <li><b>pytest</b> without ROS, scripted scenarios scored against ground truth, and headless Gazebo runs on <b>GitHub Actions</b>.</li>
    </ul>
    <p><a href="${REPO}" target="_blank" rel="noopener">Source code on GitHub</a></p>
  </div>`;
}

export class Tour {
  constructor(el, ui, scene) {
    this.el = el;
    this.ui = ui;
    this.scene = scene;
    this.index = -1;
  }

  get open() { return this.index >= 0; }

  start() { this.show(0); }

  close() {
    this.index = -1;
    this.el.hidden = true;
    document.body.classList.remove('touring');
    this._spot(null);
  }

  show(i) {
    this.index = i;
    const step = STEPS[i];
    const dots = STEPS.map((_, k) => `<i class="${k === i ? 'on' : ''}"></i>`).join('');
    this.el.innerHTML = `
      <div class="step">Tour · ${i + 1} of ${STEPS.length}</div>
      <h3>${step.title}</h3>
      ${step.body}
      <footer>
        <div class="dots">${dots}</div>
        <div class="nav">
          <button data-act="close">${i === STEPS.length - 1 ? 'Finish' : 'Skip'}</button>
          ${i > 0 ? '<button data-act="back">Back</button>' : ''}
          ${i < STEPS.length - 1 ? '<button class="primary" data-act="next">Next</button>' : ''}
        </div>
      </footer>`;
    this.el.hidden = false;
    document.body.classList.add('touring');
    this.el.querySelector('[data-act="close"]').onclick = () => this.close();
    const back = this.el.querySelector('[data-act="back"]');
    if (back) back.onclick = () => this.show(i - 1);
    const next = this.el.querySelector('[data-act="next"]');
    if (next) { next.onclick = () => this.show(i + 1); next.focus({ preventScroll: true }); }
    this._spot(step.spot || null);
    if (step.view) this.ui.setView(step.view);
    for (const key of step.layers || []) {
      this.scene.setLayer(key, true);
    }
    for (const box of document.querySelectorAll('#layers input')) {
      const key = box.dataset.key;
      if (key && (step.layers || []).includes(key)) box.checked = true;
    }
  }

  _spot(id) {
    for (const card of document.querySelectorAll('.card.spot')) card.classList.remove('spot');
    if (!id) return;
    const card = document.getElementById(id);
    if (card) {
      card.classList.add('spot');
      card.scrollIntoView({ behavior: 'smooth', block: 'nearest', inline: 'nearest' });
    }
  }
}
