# Copyright 2026 AutoNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Phase 5: scripted scenarios, the scripted walker and their wiring."""

import json
import math
import os
import re

from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import Bool

from my_bot import person_controller as pc
from my_bot.clearance_map import ClearanceMap
from my_bot.scenario_script import Leg, Scenario, load_scenarios, parse_scenarios

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(os.path.dirname(PKG_DIR))
MAP_YAML = os.path.join(PKG_DIR, 'maps', 'my_map.yaml')
SCENARIOS_YAML = os.path.join(PKG_DIR, 'config', 'scenarios.yaml')
SPAWN = (0.5, 7.5)  # person.world's actor pose


@pytest.fixture(scope='module')
def museum():
    return ClearanceMap.from_yaml(MAP_YAML)


@pytest.fixture(scope='module')
def scenarios():
    return load_scenarios(SCENARIOS_YAML)


def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _run(walker, start=SPAWN, seconds=300.0, dt=0.05, robot=(0.5, 7.0),
         alarmed=lambda t: True, after=0.0):
    """Integrate the walker like the actor plugin; return a trace per step.

    Stops ``after`` seconds after the walker is done.
    """
    p = pc.PersonPose(start[0], start[1], 0.0, 0.0)
    trace = []
    t, end = 0.0, seconds
    while t < end:
        if walker.done:
            end = min(end, t + after)
        v, w = walker.step(dt, p, robot, alarmed(t))
        p.speed = v = max(0.0, v)
        p.yaw = pc.wrap_angle(p.yaw + w * dt)
        p.x += v * dt * math.cos(p.yaw)
        p.y += v * dt * math.sin(p.yaw)
        t += dt
        trace.append((t, p.x, p.y, v, walker.mode, walker.leg_index, walker.phase))
    return trace


# ---------------------------------------------------------------------------
# scenario_script
# ---------------------------------------------------------------------------

class TestScenarioFile:

    def test_defines_the_planned_scenarios(self, scenarios):
        assert {'walk_in', 'sprint', 'hide'} <= set(scenarios)
        for sc in scenarios.values():
            assert sc.description and sc.legs

    def test_legs(self, scenarios):
        sprint = scenarios['sprint']
        assert sprint.legs[0].wait_detected and sprint.legs[0].to is None
        assert sprint.legs[1] == Leg(to=(-4.2, 7.5), speed=2.5)
        assert scenarios['walk_in'].legs[0].wait_robot_within == 6.0
        assert scenarios['hide'].legs[5].pause == 40.0

    @pytest.mark.parametrize('bad, message', [
        ({'x': {'legs': [{'to': [1, 2], 'sped': 1.0}]}}, 'unknown keys'),
        ({'x': {'legs': [{'to': [1, 2, 3]}]}}, 'to must be'),
        ({'x': {'legs': [{'to': [1, 2], 'speed': 0}]}}, 'speed must be positive'),
        ({'x': {'legs': [{'pause': -1}]}}, '>= 0'),
        ({'x': {'legs': ['walk']}}, 'must be a mapping'),
        ({'x': {'legs': []}}, 'non-empty legs'),
        ({}, 'must map names'),
    ])
    def test_rejects_mistakes(self, bad, message):
        with pytest.raises(ValueError, match=message):
            parse_scenarios(bad)

    def test_waypoints(self, scenarios):
        assert scenarios['sprint'].waypoints()[0] == (-4.2, 7.5)
        assert None not in scenarios['hide'].waypoints()


# Every leg is a straight line the walker can follow: open floor at both
# ends and person_controller's min_clearance all along it.
@pytest.mark.parametrize('name', ['walk_in', 'sprint', 'hide'])
def test_routes_are_open_floor(museum, scenarios, name):
    min_clear = pc.PersonParams().min_clearance
    route = [SPAWN] + scenarios[name].waypoints()
    for a, b in zip(route, route[1:]):
        assert museum.clearance(*b) >= min_clear + 0.1, b
        assert museum.segment_clearance(a, b) >= min_clear, (a, b)


@pytest.mark.parametrize('name', ['walk_in', 'sprint', 'hide'])
def test_scenarios_run_to_the_end_without_touching_walls(museum, scenarios, name):
    walker = pc.ScriptedWalker(museum, scenarios[name])
    trace = _run(walker)
    assert walker.done, walker.status()
    min_clear = min(museum.clearance(x, y) for _, x, y, *_ in trace)
    assert min_clear >= pc.PersonParams().min_clearance - 0.05
    end = scenarios[name].waypoints()[-1]
    assert math.dist(trace[-1][1:3], end) < 0.4


def test_sprint_runs_then_walks(museum, scenarios):
    trace = _run(pc.ScriptedWalker(museum, scenarios['sprint']))
    sprinting = [v for *_, v, mode, leg, _ in trace if leg in (1, 2)]
    # Short legs: it turns around from the spawn yaw, then rounds a corner
    assert max(sprinting) > 2.0
    assert {mode for *_, mode, leg, _ in trace if leg in (1, 2)} == {pc.RUN}
    walk_start = next(t for t, *_, leg, _ in trace if leg == 3)
    assert max(v for t, *_, v, _, leg, _ in trace
               if leg >= 3 and t > walk_start + 1.0) <= 0.5 + 1e-9


# ---------------------------------------------------------------------------
# ScriptedWalker
# ---------------------------------------------------------------------------

def _open_floor():
    return ClearanceMap.from_bounds(10.0)


class TestScriptedWalker:

    def test_waits_until_detected(self):
        sc = Scenario('s', legs=[Leg(wait_detected=True), Leg(to=(3.0, 0.0))])
        walker = pc.ScriptedWalker(_open_floor(), sc)
        trace = _run(walker, start=(0.0, 0.0), seconds=10.0,
                     alarmed=lambda t: t > 2.0)
        assert all(v == 0.0 for t, _, _, v, *_ in trace if t <= 2.0)
        assert walker.done

    def test_waits_for_the_robot_to_come_close(self):
        sc = Scenario('s', legs=[Leg(wait_robot_within=5.0), Leg(to=(3.0, 0.0))])
        walker = pc.ScriptedWalker(_open_floor(), sc)
        p = pc.PersonPose()
        assert walker.step(0.05, p, (8.0, 0.0), False) == (0.0, 0.0)
        assert walker.phase == pc.WAITING
        walker.step(0.05, p, (4.0, 0.0), False)
        assert walker.phase == pc.MOVING

    def test_pauses_on_arrival(self):
        sc = Scenario('s', legs=[Leg(to=(2.0, 0.0), pause=3.0), Leg(to=(4.0, 0.0))])
        trace = _run(pc.ScriptedWalker(_open_floor(), sc), start=(0.0, 0.0))
        paused = [t for t, *_, phase in trace if phase == pc.PAUSING]
        assert max(paused) - min(paused) == pytest.approx(3.0, abs=0.1)

    def test_arrives_standing_where_it_stops(self):
        sc = Scenario('s', legs=[Leg(to=(5.0, 0.0), speed=2.5)])
        trace = _run(pc.ScriptedWalker(_open_floor(), sc), start=(0.0, 0.0),
                     after=1.0)
        arrived = next(i for i, step in enumerate(trace) if step[-1] == pc.DONE)
        assert trace[arrived][3] < 1.0
        _, x, _, v, *_ = trace[-1]
        assert v == 0.0
        assert x == pytest.approx(5.0, abs=0.3)

    def test_passes_through_intermediate_points(self):
        sc = Scenario('s', legs=[Leg(to=(3.0, 0.0)), Leg(to=(6.0, 0.0))])
        trace = _run(pc.ScriptedWalker(_open_floor(), sc), start=(0.0, 0.0))
        assert min(v for _, x, _, v, *_ in trace if 2.5 < x < 3.5) > 0.5

    def test_status_and_done(self):
        sc = Scenario('s', legs=[Leg(pause=0.1)])
        walker = pc.ScriptedWalker(_open_floor(), sc)
        assert walker.status() == {'scenario': 's', 'leg': 0, 'legs': 1,
                                   'phase': pc.WAITING}
        for _ in range(5):
            walker.step(0.05, pc.PersonPose(), (0.0, 0.0), False)
        assert walker.done and walker.status()['phase'] == pc.DONE
        assert walker.step(0.05, pc.PersonPose(speed=1.0), (0.0, 0.0), False)[0] < 1.0


class TestScriptedPersonNode:

    @pytest.fixture
    def node(self, ros_params):
        ros_params.update(map_yaml=MAP_YAML, scenario='sprint',
                          scenarios_file=SCENARIOS_YAML)
        return pc.PersonControllerNode()

    def _tick(self, node, n=1):
        for _ in range(n):
            node.clock.advance(node._dt)
            node.timers[0]()

    def test_runs_the_scenario(self, node):
        assert isinstance(node._brain, pc.ScriptedWalker)
        assert node._brain.scenario.name == 'sprint'
        assert '/scenario/status' in node.publishers
        assert any('scripted scenario sprint' in m for m in node.logger.messages('info'))

    def test_status_follows_the_script(self, node):
        status = node.publishers['/scenario/status']
        node.subscriptions['/person/odom'](_odom(*SPAWN))
        node.subscriptions['/odom'](_odom(0.5, 4.0))
        self._tick(node, 5)
        assert json.loads(status.last.data)['phase'] == pc.WAITING
        assert node.publishers['/person/cmd_vel'].last.linear.x == 0.0
        node.subscriptions['/person_detected'](Bool(data=True))
        self._tick(node)
        assert json.loads(status.last.data) == {
            'scenario': 'sprint', 'leg': 1, 'legs': 5, 'phase': pc.MOVING}
        assert node.publishers['/person_mode'].last.data == pc.RUN
        published = len(status.msgs)
        self._tick(node, 3)  # no change: nothing new until the 1 s timer
        assert len(status.msgs) == published
        node.timers[1]()
        assert len(status.msgs) == published + 1

    def test_unknown_scenario(self, ros_params):
        ros_params.update(map_yaml=MAP_YAML, scenario='moonwalk',
                          scenarios_file=SCENARIOS_YAML)
        with pytest.raises(ValueError, match='moonwalk'):
            pc.PersonControllerNode()


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------

def _read(*parts):
    with open(os.path.join(*parts)) as f:
        return f.read()


def test_scenario_launch_wiring():
    src = _read(PKG_DIR, 'launch', 'scenario.launch.py')
    assert "'person_sim.launch.py'" in src
    assert "launch_arguments={'scenario': scenario}" in src
    assert "executable='scenario_evaluator'" in src
    assert 'OnProcessExit' in src and 'Shutdown' in src
    for arg in ("'gui', default_value='false'", "'rviz', default_value='false'"):
        assert arg in src
    person = _read(PKG_DIR, 'launch', 'person_sim.launch.py')
    assert "ParameterValue(LaunchConfiguration('scenario'), value_type=str)" in person
    assert "'scenarios_file': os.path.join(pkg, 'config', 'scenarios.yaml')" in person
    sim = _read(PKG_DIR, 'launch', 'sim.launch.py')
    assert "'rviz', default_value='true'" in sim
    assert "condition=IfCondition(LaunchConfiguration('rviz'))" in sim


def test_make_scenario(scenarios):
    makefile = os.path.join(REPO_ROOT, 'Makefile')
    if not os.path.exists(makefile):
        pytest.skip('Makefile not available (only src/ is mounted)')
    src = _read(makefile)
    target = re.search(r'^scenario:\n\t(.*)$', src, re.M).group(1)
    assert target.startswith('$(EXEC) "$(STOP_LEFTOVERS) $(NEED_YOLO) $(SOURCE) && ')
    assert 'scenario.launch.py scenario:=$(NAME)' in target
    assert 'gui:=$(call ON_REQUEST,GUI) rviz:=$(call ON_REQUEST,RVIZ)' in target
    names = re.search(r'^SCENARIO_NAMES := (.*)$', src, re.M).group(1).split()
    assert sorted(names) == sorted(scenarios)
