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

"""Tests for patrol zones, the schedule, restricted areas and their file."""

import os

import pytest

from my_bot.clearance_map import ClearanceMap
from my_bot.guard_zones import GuardZones, point_in_polygon

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ZONES_YAML = os.path.join(PKG, 'config', 'guard_zones.yaml')
MAP_YAML = os.path.join(PKG, 'maps', 'my_map.yaml')

SQUARE = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]


def test_point_in_polygon():
    assert point_in_polygon(1.0, 1.0, SQUARE)
    assert point_in_polygon(1.0, 1.0, SQUARE[::-1])          # either winding
    assert not point_in_polygon(3.0, 1.0, SQUARE)
    assert not point_in_polygon(1.0, -0.1, SQUARE)
    l_shape = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
    assert point_in_polygon(0.5, 1.5, l_shape)
    assert not point_in_polygon(1.5, 1.5, l_shape)            # the notch


def test_schedule_cycles_on_the_clock():
    zones = GuardZones({'a': [(0, 0)], 'b': [(1, 1), (2, 2)]},
                       [('a', 10.0), ('b', 5.0)])
    assert [zones.zone_at(t) for t in (0.0, 9.9, 10.0, 14.9, 15.0, 31.0)] == [
        'a', 'a', 'b', 'b', 'a', 'a']
    assert zones.route_at(12.0) == ('b', [(1.0, 1.0), (2.0, 2.0)])


def test_without_a_schedule_the_first_zone_is_patrolled():
    assert GuardZones({'a': [(0, 0)], 'b': [(1, 1)]}).zone_at(1e6) == 'a'
    single = GuardZones.single([[1.0, 2.0]])
    assert single.route_at(0.0) == ('patrol', [(1.0, 2.0)])
    assert single.restricted == {} and single.areas_containing(1.0, 2.0) == []


@pytest.mark.parametrize('zones, schedule, restricted', [
    ({}, [], {}),                                      # no zone
    ({'a': []}, [], {}),                               # empty zone
    ({'a': [(0, 0)]}, [('b', 1.0)], {}),               # unknown zone
    ({'a': [(0, 0)]}, [('a', 0.0)], {}),               # empty slot
    ({'a': [(0, 0)]}, [], {'r': [(0, 0), (1, 1)]}),    # not a polygon
])
def test_invalid_zones_are_rejected(zones, schedule, restricted):
    with pytest.raises(ValueError):
        GuardZones(zones, schedule, restricted)


def test_restricted_areas():
    zones = GuardZones({'a': [(5, 5)]}, restricted={'vault': SQUARE})
    assert zones.areas_containing(1.0, 1.0) == ['vault']
    assert zones.areas_containing(5.0, 5.0) == []


def test_from_yaml(tmp_path):
    path = tmp_path / 'zones.yaml'
    path.write_text(
        'patrol_zones:\n  a: [[0, 0], [1, 0]]\n  b: [[3, 3]]\n'
        'patrol_schedule:\n  - {zone: b, secs: 2}\n  - {zone: a, secs: 3}\n'
        'restricted_areas:\n  r: [[0, 0], [1, 0], [1, 1]]\n')
    zones = GuardZones.from_yaml(str(path))
    assert zones.zone_at(0.0) == 'b' and zones.zone_at(2.5) == 'a'
    assert list(zones.restricted) == ['r']


# -- the shipped config/guard_zones.yaml on the museum map ------------------

@pytest.fixture(scope='module')
def museum():
    return GuardZones.from_yaml(ZONES_YAML), ClearanceMap.from_yaml(MAP_YAML)


def test_config_has_zones_schedule_and_areas(museum):
    zones, _ = museum
    assert len(zones.zones) >= 2
    assert {name for name, _ in zones.schedule} == set(zones.zones)
    assert zones.restricted


def test_config_waypoints_are_open_floor_outside_restricted_areas(museum):
    zones, cmap = museum
    for name, waypoints in zones.zones.items():
        for x, y in waypoints:
            assert cmap.clearance(x, y) >= 0.5, f'{name} ({x}, {y}) near a wall'
            assert zones.areas_containing(x, y) == [], f'{name} ({x}, {y}) restricted'


def test_config_restricted_areas_are_on_the_map(museum):
    zones, cmap = museum
    for name, poly in zones.restricted.items():
        cx = sum(x for x, _ in poly) / len(poly)
        cy = sum(y for _, y in poly) / len(poly)
        assert cmap.clearance(cx, cy) > 0.0 or any(
            cmap.clearance(x, y) > 0.0 for x, y in poly), f'{name} is off the floor'
