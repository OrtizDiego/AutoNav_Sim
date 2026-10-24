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

"""Phase 6: frontier exploration, map saving and the explorer node."""

import json
import math

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import yaml

from my_bot import explorer as ex
from my_bot.clearance_map import ClearanceMap
from my_bot.frontier import (
    Frontier, Grid, choose_frontier, find_frontiers, frontier_mask, known_area,
    unknown_near)
from my_bot.map_io import FREE_PIXEL, OCCUPIED_PIXEL, UNKNOWN_PIXEL, grid_to_pgm, save_map

RES = 0.05
UNKNOWN, FREE, WALL = -1, 0, 100


def _room(known_x=4.0, size=6.0, door=None):
    """A walled size x size m room at the origin, known up to x = known_x.

    ``door``: (y0, y1) gap in the east wall. Returns a Grid.
    """
    n = int(round(size / RES))
    data = np.full((n, n), UNKNOWN, dtype=np.int8)
    k = int(round(known_x / RES))
    data[:, :k] = FREE
    data[0, :k] = data[-1, :k] = WALL       # south and north walls
    data[:, 0] = WALL                        # west wall
    if known_x >= size:
        data[:, -1] = WALL                   # east wall
        if door:
            data[int(door[0] / RES):int(door[1] / RES), -1] = FREE
    return Grid(data=data, resolution=RES, origin=(0.0, 0.0))


# ---------------------------------------------------------------------------
# frontier.py
# ---------------------------------------------------------------------------

class TestFrontiers:

    def test_the_unexplored_side_is_one_frontier(self):
        grid = _room(known_x=4.0)
        frontiers = find_frontiers(grid)
        assert len(frontiers) == 1
        f = frontiers[0]
        assert f.goal[0] == pytest.approx(4.0 - RES / 2, abs=RES)
        assert 0.3 < f.goal[1] < 5.7
        assert f.centroid[1] == pytest.approx(3.0, abs=0.1)
        assert f.size == pytest.approx(6.0 / RES - 2, abs=2)
        assert f.clearance >= 0.3

    def test_a_closed_room_has_none(self):
        grid = _room(known_x=6.0)
        assert not frontier_mask(grid.data).any()
        assert find_frontiers(grid) == []

    def test_short_frontiers_are_noise(self):
        grid = _room(known_x=6.0)
        grid.data[30:33, 58] = UNKNOWN  # a 3-cell unseen spot mid-room
        # its 8 free neighbours (3 + 3 beside it, 1 above, 1 below)
        assert find_frontiers(grid, min_size=9) == []
        assert find_frontiers(grid, min_size=8)[0].size == 8

    def test_a_gap_too_narrow_for_the_robot_is_dropped(self):
        n = 120  # 6 m
        data = np.full((n, n), WALL, dtype=np.int8)
        data[10:110, 10:60] = FREE
        data[58:62, 60:70] = FREE            # a 0.2 m corridor east
        data[50:70, 70:] = UNKNOWN
        grid = Grid(data=data, resolution=RES, origin=(0.0, 0.0))
        assert find_frontiers(grid, min_size=2, min_clearance=0.3) == []
        assert len(find_frontiers(grid, min_size=2, min_clearance=0.05)) == 1

    def test_choose_prefers_near_and_long(self):
        near = Frontier(goal=(1.0, 0.0), size=10, centroid=(1.0, 0.0), clearance=1.0)
        far_long = Frontier(goal=(3.0, 0.0), size=100, centroid=(3.0, 0.0), clearance=1.0)
        far_short = Frontier(goal=(3.0, 0.0), size=10, centroid=(3.0, 0.0), clearance=1.0)
        # 100 cells = 5 m of frontier, worth 2.5 m of travel at size_weight 0.5
        assert choose_frontier([near, far_long], (0.0, 0.0)) is far_long
        assert choose_frontier([near, far_short], (0.0, 0.0)) is near
        assert choose_frontier([near, far_long], (0.0, 0.0), size_weight=0.0) is near

    def test_choose_skips_the_blacklist(self):
        a = Frontier(goal=(1.0, 0.0), size=10, centroid=(1.0, 0.0), clearance=1.0)
        b = Frontier(goal=(5.0, 0.0), size=10, centroid=(5.0, 0.0), clearance=1.0)
        assert choose_frontier([a, b], (0.0, 0.0), blacklist=[(1.2, 0.1)]) is b
        assert choose_frontier([a], (0.0, 0.0), blacklist=[(1.0, 0.0)]) is None

    def test_unknown_near(self):
        grid = _room(known_x=4.0)
        assert unknown_near(grid, (3.8, 3.0), 0.5)
        assert not unknown_near(grid, (2.0, 3.0), 0.5)
        assert unknown_near(grid, (50.0, 50.0), 0.5)  # off the map

    def test_known_area(self):
        grid = _room(known_x=6.0)
        assert known_area(grid) == pytest.approx((6.0 - 2 * RES) ** 2, rel=0.05)

    def test_grid_cells_and_world(self):
        grid = Grid(data=np.zeros((10, 20), np.int8), resolution=0.5, origin=(-2.0, 1.0))
        assert grid.to_cell(-1.9, 1.1) == (0, 0)
        assert grid.to_cell(7.9, 5.9) == (9, 19)
        x, y = grid.to_world(9, 19)
        assert (float(x), float(y)) == (7.75, 5.75)
        assert grid.inside(9, 19) and not grid.inside(10, 0)


# ---------------------------------------------------------------------------
# map_io.py
# ---------------------------------------------------------------------------

def test_saved_map_loads_like_a_map_server_map(tmp_path):
    grid = _room(known_x=4.0)
    grid.origin = (-1.0, -2.0)
    yaml_path, pgm_path = save_map(str(tmp_path / 'maps' / 'w'), grid)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    assert meta == {'image': 'w.pgm', 'mode': 'trinary', 'resolution': RES,
                    'origin': [-1.0, -2.0, 0.0], 'negate': 0,
                    'occupied_thresh': 0.65, 'free_thresh': 0.25}
    with open(pgm_path, 'rb') as f:
        assert f.read(2) == b'P5'
    cmap = ClearanceMap.from_yaml(yaml_path)
    assert cmap.clearance(1.0, 1.0) > 0.5    # free floor, 1 m inside the room
    assert cmap.clearance(-1.0 + RES / 2, 1.0) == 0.0  # the west wall
    assert cmap.clearance(4.5, 1.0) == 0.0   # unknown is not free


def test_pgm_rows_run_top_down():
    data = np.full((4, 3), UNKNOWN, np.int8)
    data[0, :] = FREE        # bottom row (min y) in the grid
    data[3, 0] = WALL        # top-left
    image = grid_to_pgm(Grid(data=data, resolution=1.0, origin=(0.0, 0.0)))
    assert list(image[-1]) == [FREE_PIXEL] * 3
    assert image[0, 0] == OCCUPIED_PIXEL and image[1, 1] == UNKNOWN_PIXEL


# ---------------------------------------------------------------------------
# explorer node
# ---------------------------------------------------------------------------

def _msg(grid):
    msg = OccupancyGrid()
    msg.info.resolution = grid.resolution
    msg.info.height, msg.info.width = grid.data.shape
    msg.info.origin.position.x, msg.info.origin.position.y = grid.origin
    msg.data = grid.data.ravel().tolist()
    return msg


class TestExplorerNode:

    @pytest.fixture
    def nav(self):
        return BasicNavigator()

    @pytest.fixture
    def node(self, ros_params, nav, tmp_path):
        ros_params.update(start_delay=2.0, goal_timeout=30.0, done_checks=2,
                          map_save_path=str(tmp_path / 'warehouse'))
        node = ex.ExplorerNode(navigator=nav)
        node._tf_buffer.transforms[('map', 'base_link')] = (1.0, 3.0, 0.0)
        return node

    def _tick(self, node, secs=1.0):
        node.clock.advance(secs)
        node.timers[0]()

    def _status(self, node):
        return json.loads(node.publishers['/exploration/status'].last.data)

    def test_interface(self, node):
        assert set(node.subscriptions) == {'/map'}
        assert set(node.publishers) == {'/exploration/status'}
        assert node.timers[0].period == pytest.approx(1.0)

    def test_waits_for_nav2_with_slam(self, ros_params, monkeypatch):
        made = []
        monkeypatch.setattr(ex, 'BasicNavigator',
                            lambda: made.append(BasicNavigator()) or made[-1])
        ex.ExplorerNode()
        assert made[0].active
        assert made[0].wait_kwargs == {'localizer': 'controller_server'}

    def test_waits_for_the_map_and_the_start_delay(self, node, nav):
        self._tick(node)
        assert nav.goals == []                   # no map yet
        node.subscriptions['/map'](_msg(_room(known_x=4.0)))
        self._tick(node, 0.5)
        assert nav.goals == []                   # start_delay
        self._tick(node, 1.0)
        assert len(nav.goals) == 1

    def test_drives_to_the_frontier(self, node, nav):
        node.subscriptions['/map'](_msg(_room(known_x=4.0)))
        self._tick(node, 3.0)
        goal = nav.goals[-1]
        assert goal.header.frame_id == 'map'
        assert goal.pose.position.x == pytest.approx(4.0, abs=0.1)
        assert node.goal is not None and node.goals_sent == 1
        status = self._status(node)
        assert status['state'] == ex.EXPLORING and status['frontiers'] == 1
        self._tick(node)                          # still driving
        assert len(nav.goals) == 1

    def test_a_failed_goal_is_blacklisted(self, node, nav):
        node.subscriptions['/map'](_msg(_room(known_x=4.0)))
        self._tick(node, 3.0)
        nav.done, nav.result = True, TaskResult.FAILED
        self._tick(node)
        assert node.failures == 1 and len(node.blacklist) == 1
        assert len(nav.goals) == 1   # the only frontier is blacklisted
        assert any('skipped' in m for m in node.logger.messages('warn'))

    def test_a_goal_that_takes_too_long_is_cancelled(self, node, nav):
        node.subscriptions['/map'](_msg(_room(known_x=4.0)))
        self._tick(node, 3.0)
        self._tick(node, 31.0)
        assert nav.cancelled == 1 and node.failures == 1

    def test_a_goal_whose_frontier_was_seen_is_dropped(self, node, nav):
        node.subscriptions['/map'](_msg(_room(known_x=4.0)))
        self._tick(node, 3.0)
        node.subscriptions['/map'](_msg(_room(known_x=6.0, door=(2.5, 3.5))))
        self._tick(node)
        assert nav.cancelled == 1 and node.failures == 0
        assert not node.blacklist

    def test_saves_the_map_and_goes_home(self, node, nav, tmp_path):
        node.subscriptions['/map'](_msg(_room(known_x=6.0)))
        self._tick(node, 3.0)
        assert node.state == ex.EXPLORING        # one empty check of two
        self._tick(node)
        assert node.state == ex.RETURNING
        assert (tmp_path / 'warehouse.yaml').exists()
        assert (tmp_path / 'warehouse.pgm').exists()
        home = nav.goals[-1].pose.position
        assert (home.x, home.y) == (0.0, 0.0)
        assert self._status(node)['map'] == str(tmp_path / 'warehouse.yaml')
        nav.done = True
        self._tick(node)
        assert node.state == ex.DONE
        assert self._status(node)['state'] == ex.DONE
        goals = len(nav.goals)
        self._tick(node)
        assert len(nav.goals) == goals

    def test_without_a_save_path_nothing_is_written(self, ros_params, nav, tmp_path):
        ros_params.update(start_delay=0.0, done_checks=1, return_home=False)
        node = ex.ExplorerNode(navigator=nav)
        node._tf_buffer.transforms[('map', 'base_link')] = (1.0, 3.0, 0.0)
        node.subscriptions['/map'](_msg(_room(known_x=6.0)))
        self._tick(node)
        assert node.state == ex.DONE and node.saved == []
        assert list(tmp_path.iterdir()) == []

    def test_no_tf_no_goal(self, node, nav):
        node._tf_buffer.transforms.clear()
        node.subscriptions['/map'](_msg(_room(known_x=4.0)))
        self._tick(node, 3.0)
        assert nav.goals == []


def test_grid_from_msg_keeps_rows_and_origin():
    grid = _room(known_x=4.0)
    grid.origin = (-3.0, 2.0)
    back = ex.grid_from_msg(_msg(grid))
    assert back.origin == (-3.0, 2.0) and back.resolution == RES
    assert np.array_equal(back.data, grid.data)
    assert math.isclose(known_area(back), known_area(grid))
