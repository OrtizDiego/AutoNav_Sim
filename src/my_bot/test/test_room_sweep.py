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

"""Tests for the room-sweep viewpoint planner."""

import math
import os
import time

import numpy as np
import pytest

from my_bot.clearance_map import ClearanceMap
from my_bot.room_sweep import _visible, plan_sweep

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.path.join(PKG, 'maps', 'my_map.yaml')
RES = 0.05


def _two_rooms(door=True):
    """12 x 6 m: rooms x < 6 and x > 6 joined by a 1.5 m door at y 0.75-2.25."""
    free = np.ones((int(6 / RES), int(12 / RES)), dtype=bool)
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False
    wall = int(6 / RES)
    free[:, wall - 2:wall + 2] = False
    rows = free.shape[0]
    if door:
        free[rows - int(2.25 / RES):rows - int(0.75 / RES), wall - 2:wall + 2] = True
    return ClearanceMap(free, RES, (0.0, 0.0))


def test_clearance_many_matches_clearance():
    cmap = _two_rooms()
    xs = np.array([[0.5, 3.0, 6.0], [11.9, -1.0, 6.1]])
    ys = np.array([[0.5, 3.0, 4.0], [5.9, 3.0, 1.5]])
    many = cmap.clearance_many(xs, ys)
    assert many.shape == xs.shape
    for i in range(2):
        for j in range(3):
            assert many[i, j] == pytest.approx(cmap.clearance(xs[i, j], ys[i, j]))
    assert many[1, 1] == 0.0                      # off the map


def test_walls_block_the_view():
    cmap = _two_rooms()
    targets = np.array([[4.0, 3.0], [8.0, 4.5], [8.0, 1.5], [3.0, 3.0]])
    seen = _visible(cmap, (3.0, 1.5), targets, view_range=6.0)
    assert seen.tolist() == [True, False, True, True]  # 2nd is behind the wall
    assert not _visible(cmap, (3.0, 1.5), targets, view_range=0.5).any()


def test_sweep_covers_the_room_behind_the_door():
    cmap = _two_rooms()
    # Lost at the door, heading into the right room; the robot stands in
    # the left room and has only seen through the door.
    points = plan_sweep(cmap, center=(8.5, 3.0), start=(4.0, 1.5),
                        seen=[(4.0, 1.5)], radius=4.0, max_points=4)
    assert 1 <= len(points) <= 4
    for x, y, yaw in points:
        assert cmap.clearance(x, y) >= 0.4
        assert -math.pi <= yaw <= math.pi
    assert any(x > 6.0 for x, _, _ in points)     # it goes through the door
    # Between them the viewpoints see most of the right room
    room = np.array([[x, y] for x in np.arange(6.5, 11.6, 0.5)
                     for y in np.arange(0.5, 5.6, 0.5)])
    covered = np.zeros(len(room), dtype=bool)
    for x, y, _ in points:
        covered |= _visible(cmap, (x, y), room, 4.0)
    assert covered.mean() > 0.8


def test_unreachable_floor_gets_no_viewpoint():
    points = plan_sweep(_two_rooms(door=False), center=(9.0, 3.0), start=(3.0, 3.0),
                        seen=[(3.0, 3.0)], radius=3.0)
    assert points == []                           # the right room is sealed


def test_nothing_to_sweep_once_everything_was_seen():
    cmap = _two_rooms()
    assert plan_sweep(cmap, center=(3.0, 3.0), start=(3.0, 3.0),
                      seen=[(3.0, 3.0)], radius=2.0) == []
    off_map = plan_sweep(cmap, center=(50.0, 50.0), start=(3.0, 3.0), radius=2.0)
    assert off_map == []


def test_viewpoints_are_ordered_from_the_start():
    cmap = _two_rooms()
    points = plan_sweep(cmap, center=(6.0, 3.0), start=(1.0, 1.0),
                        radius=6.0, max_points=4, coverage=1.0, min_gain=1)
    assert len(points) >= 2
    dists = [math.hypot(x - 1.0, y - 1.0) for x, y, _ in points]
    assert dists[0] == min(dists)


def test_museum_sweep_is_quick_and_on_open_floor():
    cmap = ClearanceMap.from_yaml(MAP_YAML)
    start = time.perf_counter()
    points = plan_sweep(cmap, center=(-5.5, 2.0), start=(-3.0, 4.5),
                        seen=[(-3.0, 4.5)])
    assert time.perf_counter() - start < 1.0      # runs in the 10 Hz tick
    assert 1 <= len(points) <= 4
    for x, y, _ in points:
        assert cmap.clearance(x, y) >= 0.4
