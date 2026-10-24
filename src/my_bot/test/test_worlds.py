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

"""Phase 6: the world registry, world choice at launch and its wiring."""

import os
import re

import pytest

from my_bot.worlds import (
    GAZEBO_DEFAULT_RESOURCE_PATH, World, gazebo_env, get_world, load_worlds,
    maps_source_dir, parse_worlds, without_shadows)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(os.path.dirname(PKG_DIR))
WORLDS_YAML = os.path.join(PKG_DIR, 'config', 'worlds.yaml')


@pytest.fixture(scope='module')
def worlds():
    return load_worlds(WORLDS_YAML)


def _read(*parts):
    with open(os.path.join(*parts)) as f:
        return f.read()


# ---------------------------------------------------------------------------
# config/worlds.yaml
# ---------------------------------------------------------------------------

class TestRegistry:

    def test_museum_is_this_packages_mapped_world(self, worlds):
        museum = worlds['museum']
        assert not museum.external
        assert museum.world_file(PKG_DIR) == os.path.join(PKG_DIR, 'worlds', 'room.world')
        assert os.path.isfile(museum.world_file(PKG_DIR))
        # The scenarios' tests and the guard's zones all use this map
        assert museum.map_yaml(os.path.join(PKG_DIR, 'maps')).endswith('maps/my_map.yaml')
        assert os.path.isfile(museum.map_yaml(os.path.join(PKG_DIR, 'maps')))
        assert museum.spawn == (0.0, 0.0, 0.0)  # maps/my_map's frame = world frame

    def test_warehouse_comes_from_the_dockerfile(self, worlds):
        warehouse = worlds['warehouse']
        assert warehouse.external
        assert warehouse.world_file(PKG_DIR) == (
            '/opt/worlds/aws-robomaker-small-warehouse-world/worlds/'
            'no_roof_small_warehouse/no_roof_small_warehouse.world')
        assert warehouse.model_paths() == [
            '/opt/worlds/aws-robomaker-small-warehouse-world/models']
        # Its meshes are file://models/...: the root must be a resource path
        assert warehouse.resource_paths() == ['/opt/worlds/aws-robomaker-small-warehouse-world']
        assert warehouse.map == 'warehouse'
        dockerfile = os.path.join(REPO_ROOT, 'Dockerfile')
        if not os.path.exists(dockerfile):
            pytest.skip('Dockerfile not available (only src/ is mounted)')
        src = _read(dockerfile)
        assert re.search(r'ARG WAREHOUSE_COMMIT=[0-9a-f]{40}\n', src)
        assert f'        {warehouse.root} \\' in src
        assert 'aws-robotics/aws-robomaker-small-warehouse-world.git' in src

    def test_every_world_has_a_description_and_a_distinct_map(self, worlds):
        assert all(w.description for w in worlds.values())
        maps = [w.map for w in worlds.values()]
        assert len(maps) == len(set(maps))

    def test_get_world(self, worlds):
        assert get_world(worlds, 'museum').name == 'museum'
        with pytest.raises(ValueError, match="unknown world 'moon'.*museum"):
            get_world(worlds, 'moon')

    @pytest.mark.parametrize('bad, message', [
        ({}, 'must map names'),
        ({'x': 'room.world'}, 'must be a mapping'),
        ({'x': {'world': 'a.world'}}, 'needs map'),
        ({'x': {'map': 'a'}}, 'needs world'),
        ({'x': {'world': 'a', 'map': 'a', 'size': 3}}, 'unknown keys'),
        ({'x': {'world': 'a', 'map': 'a', 'root': 'rel/dir'}}, 'root must be absolute'),
        ({'x': {'world': 'a', 'map': 'a', 'model_path': 'models'}}, 'need a root'),
        ({'x': {'world': 'a', 'map': 'a', 'spawn': [1.0, 2.0]}}, 'spawn must be'),
        ({'x': {'world': 'a', 'map': 'a', 'root': '/r', 'model_path': [1]}},
         'list of paths'),
    ])
    def test_rejects_mistakes(self, bad, message):
        with pytest.raises(ValueError, match=message):
            parse_worlds(bad)


# ---------------------------------------------------------------------------
# Gazebo environment, shadows, map folder
# ---------------------------------------------------------------------------

WAREHOUSE = World(name='w', world='w.world', map='w', root='/opt/w',
                  model_path=['models'], resource_path=['.'])


def test_gazebo_env_prepends_and_keeps_gazebos_media():
    env = gazebo_env(WAREHOUSE, {'GAZEBO_MODEL_PATH': '/share/my_bot/models'})
    assert env['GAZEBO_MODEL_PATH'] == os.pathsep.join(['/opt/w/models', '/share/my_bot/models'])
    # Unset, Gazebo would use its own media; setting it must not lose them
    assert env['GAZEBO_RESOURCE_PATH'] == os.pathsep.join(
        ['/opt/w', GAZEBO_DEFAULT_RESOURCE_PATH])
    env = gazebo_env(WAREHOUSE, {'GAZEBO_RESOURCE_PATH': '/custom'})
    assert env['GAZEBO_RESOURCE_PATH'] == os.pathsep.join(['/opt/w', '/custom'])
    assert env['GAZEBO_MODEL_PATH'] == '/opt/w/models'


def test_gazebo_env_of_an_internal_world_is_empty(worlds):
    assert gazebo_env(worlds['museum'], {}) == {}


@pytest.mark.parametrize('sdf, scene', [
    ('<sdf><world name="d">\n<model/></world></sdf>', '<scene><shadows>0</shadows></scene>'),
    ('<sdf><world name="d"><scene><shadows>1</shadows></scene></world></sdf>',
     '<scene><shadows>0</shadows></scene>'),
    ('<sdf><world name="d"><scene><ambient>1 1 1 1</ambient></scene></world></sdf>',
     '<scene><ambient>1 1 1 1</ambient><shadows>0</shadows></scene>'),
])
def test_without_shadows(sdf, scene):
    out = without_shadows(sdf)
    assert scene in out
    assert out.count('<scene>') == 1 and out.count('<shadows>') == 1


def test_without_shadows_needs_a_world():
    with pytest.raises(ValueError, match='no <world>'):
        without_shadows('<sdf/>')


def test_maps_source_dir_follows_the_symlink_install(tmp_path):
    src = tmp_path / 'src' / 'maps'
    src.mkdir(parents=True)
    (src / 'my_map.yaml').write_text('image: my_map.pgm\n')
    share = tmp_path / 'install' / 'share' / 'my_bot'
    (share / 'maps').mkdir(parents=True)
    os.symlink(src / 'my_map.yaml', share / 'maps' / 'my_map.yaml')
    assert maps_source_dir(str(share)) == str(src)


def test_maps_source_dir_without_symlinks(tmp_path):
    (tmp_path / 'maps').mkdir()
    (tmp_path / 'maps' / 'my_map.yaml').write_text('image: my_map.pgm\n')
    assert maps_source_dir(str(tmp_path)) == str(tmp_path / 'maps')


# ---------------------------------------------------------------------------
# Launch and make wiring
# ---------------------------------------------------------------------------

def test_sim_launch_takes_a_world_name():
    src = _read(PKG_DIR, 'launch', 'sim.launch.py')
    assert "'world_name', default_value=''" in src
    assert 'OpaqueFunction(function=_gazebo_and_robot)' in src
    assert "'-x', str(spawn[0]), '-y', str(spawn[1]), '-Y', str(spawn[2])" in src
    assert 'gazebo_env(world, context.environment)' in src
    assert 'without_shadows(' in src and 'make image' in src


def test_nav_sim_maps_a_world_without_a_map():
    src = _read(PKG_DIR, 'launch', 'nav_sim.launch.py')
    assert "'world_name', default_value='museum'" in src
    assert 'world.map_yaml(maps_source_dir(pkg))' in src
    assert 'if os.path.exists(map_yaml):' in src
    # With a map: Nav2 bringup (map_server + AMCL) on it
    assert "'navigation.launch.py'" in src and "'map': map_yaml" in src
    # Without: SLAM + Nav2 without map_server/AMCL + the explorer saving it
    assert "executable='async_slam_toolbox_node'" in src
    assert "'navigation_launch.py'" in src
    assert "executable='explorer'" in src
    assert "'map_save_path': os.path.splitext(map_yaml)[0]" in src


def test_make_world():
    makefile = os.path.join(REPO_ROOT, 'Makefile')
    if not os.path.exists(makefile):
        pytest.skip('Makefile not available (only src/ is mounted)')
    src = _read(makefile)
    assert re.search(r'^WORLD \?= museum$', src, re.M)
    for launch in ('sim', 'nav_sim'):
        assert f'{launch}.launch.py $(SCENARIO_ARGS) world_name:=$(WORLD)"' in src
