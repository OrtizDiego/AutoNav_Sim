"""The robot software as the web demo runs it: real nodes on rosfake.

Run from the repository root:  python3 -m pytest demo/test
(needs numpy, opencv-python-headless, pyyaml, py_trees; no ROS). The
package's own tests fake ROS differently (src/my_bot/test/conftest.py), so
these run in a separate pytest session.
"""

import ast
import importlib.util
import math
import os

import cv2
import numpy as np
import pytest

from minisim import MiniSim, PKG

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def states(sim):
    return [s for _, s in sim.states]


def events(sim):
    return [e['event'] for e in sim.events]


@pytest.fixture
def shim():
    spec = importlib.util.spec_from_file_location(
        'cv2_shim', os.path.join(DEMO, 'py', 'shims', 'cv2.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_cv2_shim_matches_opencv(shim):
    """The browser's numpy stand-in gives OpenCV's map and distances."""
    path = os.path.join(PKG, 'maps', 'my_map.pgm')
    real = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    assert np.array_equal(shim.imread(path), real)
    free = (real >= 250).astype(np.uint8)
    want = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    assert np.abs(shim.distanceTransform(free) - want).max() < 1e-3
    # a second map: the cache must not hand back the first one's result
    other = np.ones((40, 60), np.uint8)
    other[10, 20] = 0
    want = cv2.distanceTransform(other, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    assert np.abs(shim.distanceTransform(other) - want).max() < 1e-3


def test_bundle_has_every_module_the_nodes_import():
    """build.py's module list covers the my_bot imports of what it bundles."""
    spec = importlib.util.spec_from_file_location('demo_build', os.path.join(DEMO, 'build.py'))
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)
    for name in build.MODULES:
        with open(os.path.join(PKG, 'my_bot', name + '.py')) as f:
            tree = ast.parse(f.read())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith('my_bot.'):
                assert node.module.split('.')[1] in build.MODULES, (name, node.module)
    for name in build.CONFIG:
        assert os.path.exists(os.path.join(PKG, 'config', name))


def test_planner_routes_around_walls():
    sim = MiniSim()
    grid = sim.brain.nav._grid
    path = grid.plan((0.0, 0.0), (-7.5, 4.5))  # origin -> west wing
    assert path and path[0] == (0.0, 0.0)
    assert math.hypot(path[-1][0] + 7.5, path[-1][1] - 4.5) < 0.15
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
    assert length > math.hypot(7.5, 4.5)  # not a straight line through the walls
    for a, b in zip(path, path[1:]):
        assert sim.cmap.segment_clearance(a, b) >= 0.2
    assert grid.plan((0.0, 0.0), (50.0, 50.0)) is None  # off the map


def test_patrols_the_scheduled_zone():
    sim = MiniSim(person=(-7.0, -8.5, 0.0))  # nobody in sight
    sim.run(45)
    assert states(sim)[-1] == 'PatrolProtocol'
    assert sim.last['viz']['zone'] == 'north_hall'
    assert math.hypot(sim.robot[0], sim.robot[1]) > 3.0  # it left the spawn
    assert sim.brain.guard._metrics['intruder_detections'] == 0
    visited = [line for line in sim.log if 'heading to north_hall waypoint 1' in line[3]]
    assert visited, 'the first waypoint was not reached in 45 s'


def test_detects_follows_and_keeps_its_distance():
    sim = MiniSim(person=(5.0, 6.0, 0.0))
    assert sim.run(30, until=lambda out: out['viz']['state'] == 'IntruderProtocol')
    assert events(sim) == ['intruder_detected']
    sim.person_vel = (0.0, 0.8)  # walks away north
    sim.run(4)
    sim.person_vel = (0.0, 0.0)
    sim.run(8)
    viz = sim.last['viz']
    assert viz['state'] == 'IntruderProtocol' and viz['intruder'] == 'confirmed'
    gap = math.hypot(sim.person[0] - sim.robot[0], sim.person[1] - sim.robot[1])
    assert 2.0 < gap < 3.2, gap  # desired_distance 2.5 m
    track = viz['track']
    assert math.hypot(track['x'] - sim.person[0], track['y'] - sim.person[1]) < 0.4


def test_lost_intruder_search_sweep_report_then_patrol():
    sim = MiniSim(person=(5.0, 6.0, 0.0))
    assert sim.run(30, until=lambda out: out['viz']['state'] == 'IntruderProtocol')
    sim.person[0], sim.person[1] = -7.0, -8.5  # gone
    assert sim.run(120, until=lambda out: 'incident_closed' in events(sim))
    sim.run(2)
    assert states(sim)[-5:] == [
        'IntruderProtocol', 'SearchProtocol', 'SweepProtocol', 'ReportProtocol',
        'PatrolProtocol']
    assert events(sim) == ['intruder_detected', 'intruder_lost', 'incident_closed']


def test_emergency_stop_halts_the_robot():
    sim = MiniSim(person=(-7.0, -8.5, 0.0))
    sim.run(8)
    assert abs(sim.robot[3]) > 0.1  # driving
    sim.estop = True
    sim.run(3)
    assert states(sim)[-1] == 'EmergencyStop'
    assert sim.robot[3] == 0.0
    sim.estop = False
    sim.run(3)
    assert states(sim)[-1] == 'PatrolProtocol'


def test_autopilot_person_walks_and_stays_on_the_floor():
    sim = MiniSim(person=(4.6, 3.2, math.pi))
    sim.autopilot = True
    start = tuple(sim.person[:2])
    worst = math.inf
    for _ in range(40 * 30):
        sim.step()
        worst = min(worst, sim.cmap.clearance(sim.person[0], sim.person[1]))
    assert math.hypot(sim.person[0] - start[0], sim.person[1] - start[1]) > 1.0
    assert worst > 0.3  # person_controller's min_clearance is 0.5


def test_step_output_is_plain_json():
    import json
    sim = MiniSim(person=(5.0, 6.0, 0.0))
    for _ in range(90):
        out = sim.step()
        json.dumps(out, allow_nan=False)  # what the page parses


def test_synthetic_replay_has_what_the_player_reads(tmp_path):
    """fake_replay.py -> my_bot.replay_export -> the fields js/replay.js uses."""
    import json
    import fake_replay
    from my_bot import replay_export as rx
    out = tmp_path / 'synthetic'
    replay = rx.export(str(tmp_path), str(out), image_fps=2.0,
                       messages=fake_replay.messages(20.0))
    data = json.loads((out / 'replay.json').read_text())
    assert data == json.loads(json.dumps(replay))
    frames = data['frames']
    n = len(frames['t'])
    assert n > 150 and all(len(frames[k]) == n for k in
                           ('robot', 'person', 'person2', 'guard', 'intruder', 'track', 'mode'))
    assert data['scan']['samples'] == 360 and len(data['scans']) > 60
    assert {data['guard_states'][g] for g in frames['guard']} >= {'PatrolProtocol',
                                                                 'IntruderProtocol'}
    assert [e['event'] for e in data['events']][:1] == ['intruder_detected']
    assert data['plans'] and data['images']
    assert all((out / img['file']).exists() for img in data['images'])
