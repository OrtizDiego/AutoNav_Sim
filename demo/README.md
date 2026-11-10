# AutoNav Sim web demo

A page where a visitor plays the intruder and the security-guard robot
chases them, in 3D, with the panels RViz shows in the real project: the
robot's camera with the detection, the live behaviour tree, the EKF track,
the security events and the node log. A guided tour explains each part.

It is a static site. Nothing runs on a server.

## What is real

The robot's software is the package's own Python, unchanged, running in the
browser on [Pyodide](https://pyodide.org) (CPython compiled to WebAssembly):

| Runs unchanged | Replaced for the browser |
| --- | --- |
| `sensor_fusion.py` (camera box + lidar → bearing, range) | Gazebo → `js/sim.js`: kinematics, ray-cast lidar, camera box |
| `target_tracker.py` + `track_filter.py` (EKF track) | YOLOv8 + tracker → the person's box computed from geometry and line of sight, with noise and 120 ms delay |
| `security_guard_bt.py` (py_trees behaviour tree) | Nav2 → `py/demo_nav.py`: A* on the saved map + path follower behind `BasicNavigator`'s interface |
| `person_controller.py` (intruder autopilot) | rclpy and the message packages → `py/rosfake.py`: topics, timers, a simulation clock |
| `room_sweep.py`, `guard_zones.py`, `incident_log.py`, `clearance_map.py` | OpenCV → `py/shims/cv2.py`: the two calls the map code uses, in numpy |
| `config/behavior_params.yaml`, `config/guard_zones.yaml`, `maps/my_map` | Gazebo client + RViz → three.js |

`rosfake.py` is the idea behind `src/my_bot/test/conftest.py` (run the nodes
without ROS) turned into a runtime: one topic graph, one clock, timers fired
in time order. The page states these substitutions in its tour and in "How
it works".

## Run it

On the host (not in the Docker container); needs Node.js and Python 3:

```bash
make demo        # = cd demo && npm ci && python3 build.py --serve
```

then open http://localhost:8000. `build.py` writes `demo/_site/`:

- `index.html`, `css/`, `js/`: the page;
- `vendor/`: three.js from `node_modules`;
- `assets/`: the map image and its metadata;
- `robot.zip`: what the in-browser Python loads: the `my_bot` modules the
  four nodes need, the config and map, `py/`, and pure-Python `py_trees`
  (with `pydot` and `pyparsing`), downloaded once into `.wheels/`.

Pyodide itself (Python, numpy, PyYAML: about 20 MB, cached by the browser)
comes from the jsDelivr CDN at run time. `python3 build.py --pyodide <url>/`
points the page at another copy.

## Publish it

`.github/workflows/demo.yml` tests, builds and, on `main`, deploys the site
to GitHub Pages. Enable it once: repository **Settings → Pages → Source:
GitHub Actions**. The site then lives at
`https://<user>.github.io/AutoNav_Sim/`.

To show it inside a portfolio page:

```html
<iframe src="https://<user>.github.io/AutoNav_Sim/" title="AutoNav Sim live demo"
        style="width:100%;height:80vh;border:0;border-radius:12px" allow="fullscreen"></iframe>
```

Any static host works too: upload the contents of `demo/_site/`.

## Tests

```bash
pip install numpy opencv-python-headless pyyaml py_trees pytest
python3 -m pytest demo/test          # the robot software on rosfake, no browser
node demo/test/browser_smoke.js      # the built site in headless Chromium (needs Playwright)
```

`test_brain.py` drives the nodes with `minisim.py`, a Python copy of the
page's simulation: patrol, detect and follow at 2.5 m, lose → search → sweep
→ report → patrol, emergency stop, the autopilot intruder, and the OpenCV
stand-in against the real OpenCV. They run in their own pytest session
because the package's tests fake ROS in a different way.

## Layout

```
demo/
├── index.html, css/style.css
├── js/
│   ├── main.js          loop: simulation at 60 Hz, sensor data to the robot at 30 Hz
│   ├── world.js         the occupancy grid: collisions, ray casting, line of sight
│   ├── sim.js           robot, person, lidar, camera box (the Gazebo part)
│   ├── scene.js         three.js: museum from the map, overlays, the robot's camera
│   ├── brain-worker.js  Web Worker: Pyodide + the nodes
│   ├── ui.js            panels (camera, behaviour tree, track, events, log)
│   ├── tour.js          guided tour and "How it works"
│   └── input.js         keys, click-to-walk, touch stick
├── py/
│   ├── brain.py         builds the node graph, steps it, returns display state
│   ├── rosfake.py       the ROS 2 stand-in
│   ├── demo_nav.py      the Nav2 stand-in
│   └── shims/cv2.py     the OpenCV stand-in
├── test/                test_brain.py, minisim.py, browser_smoke.js
└── build.py             assembles _site/
```
