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

"""The environments the robot can run in (config/worlds.yaml).

`make sim WORLD=warehouse` / `make nav-sim WORLD=warehouse` pick one by
name; sim.launch.py and nav_sim.launch.py resolve it here. Keys:

world          the Gazebo world file; relative to ``root``, or to this
               package's worlds/ when there is no root
root           directory of an external world (the Dockerfile clones it)
model_path     directories (relative to root) added to GAZEBO_MODEL_PATH
resource_path  directories (relative to root) added to GAZEBO_RESOURCE_PATH,
               for meshes referenced as file://<relative path>
map            maps/<map>.yaml; nav-sim builds it when it is missing
spawn          [x, y, yaw] of the robot in the world frame

Pure Python + PyYAML, no ROS: the launch files and the tests share it.
"""

from dataclasses import dataclass, field
import os
import re
from typing import Dict, List, Mapping, Tuple

import yaml

WORLD_KEYS = {'description', 'world', 'root', 'model_path', 'resource_path',
              'map', 'spawn'}
# Gazebo's own media (materials, the actor skins): Gazebo uses it only while
# GAZEBO_RESOURCE_PATH is unset, so setting the variable must keep it.
GAZEBO_DEFAULT_RESOURCE_PATH = '/usr/share/gazebo-11'


@dataclass
class World:
    """One environment: where its files are and where the robot starts."""

    name: str
    world: str
    map: str
    description: str = ''
    root: str = ''
    model_path: List[str] = field(default_factory=list)
    resource_path: List[str] = field(default_factory=list)
    spawn: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def external(self) -> bool:
        """True for a world outside this package (fetched by the Dockerfile)."""
        return bool(self.root)

    def world_file(self, pkg_share: str) -> str:
        """Return the absolute path of the world file."""
        base = self.root or os.path.join(pkg_share, 'worlds')
        return os.path.join(base, self.world)

    def model_paths(self) -> List[str]:
        return [os.path.normpath(os.path.join(self.root, p)) for p in self.model_path]

    def resource_paths(self) -> List[str]:
        return [os.path.normpath(os.path.join(self.root, p)) for p in self.resource_path]

    def map_yaml(self, maps_dir: str) -> str:
        """Return the path of the world's map YAML (which may not exist yet)."""
        return os.path.join(maps_dir, self.map + '.yaml')


def _paths(value, where):
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, list) and all(isinstance(v, str) for v in value):
        return list(value)
    raise ValueError(f'{where}: expected a path or a list of paths')


def parse_worlds(data: dict) -> Dict[str, World]:
    """Turn the loaded YAML mapping into {name: World}."""
    if not isinstance(data, dict) or not data:
        raise ValueError('worlds file must map names to worlds')
    worlds = {}
    for name, body in data.items():
        if not isinstance(body, dict):
            raise ValueError(f'{name}: a world must be a mapping')
        unknown = set(body) - WORLD_KEYS
        if unknown:
            raise ValueError(f'{name}: unknown keys {sorted(unknown)}')
        for key in ('world', 'map'):
            if not body.get(key):
                raise ValueError(f'{name}: needs {key}')
        root = str(body.get('root', ''))
        if root and not os.path.isabs(root):
            raise ValueError(f'{name}: root must be absolute')
        extra = _paths(body.get('model_path'), f'{name} model_path') + _paths(
            body.get('resource_path'), f'{name} resource_path')
        if extra and not root:
            raise ValueError(f'{name}: model_path and resource_path need a root')
        spawn = body.get('spawn', [0.0, 0.0, 0.0])
        if not isinstance(spawn, list) or len(spawn) != 3:
            raise ValueError(f'{name}: spawn must be [x, y, yaw]')
        worlds[name] = World(
            name=name, world=str(body['world']), map=str(body['map']),
            description=str(body.get('description', '')).strip(), root=root,
            model_path=_paths(body.get('model_path'), name),
            resource_path=_paths(body.get('resource_path'), name),
            spawn=tuple(float(v) for v in spawn))
    return worlds


def load_worlds(path: str) -> Dict[str, World]:
    """Read every world from a worlds YAML file."""
    with open(path) as f:
        return parse_worlds(yaml.safe_load(f))


def get_world(worlds: Mapping[str, World], name: str) -> World:
    """Return world ``name``; ValueError lists the known ones."""
    if name not in worlds:
        raise ValueError(f'unknown world {name!r}: one of {sorted(worlds)}')
    return worlds[name]


def maps_source_dir(pkg_share: str) -> str:
    """Directory new maps are saved to and looked up in.

    With `colcon build --symlink-install` (make build) every installed map
    is a symlink into src/my_bot/maps, the folder mounted from the host, so
    a map saved there survives the container and is found without a
    rebuild. Without symlinks this is the installed share/my_bot/maps.
    """
    installed = os.path.join(pkg_share, 'maps')
    for entry in sorted(os.listdir(installed)) if os.path.isdir(installed) else []:
        path = os.path.join(installed, entry)
        if os.path.islink(path):
            return os.path.dirname(os.path.realpath(path))
    return installed


def gazebo_env(world: World, environ: Mapping[str, str]) -> Dict[str, str]:
    """Return the GAZEBO_* variables to set for ``world`` (prepended to ``environ``)."""
    env = {}
    for var, paths, default in (
            ('GAZEBO_MODEL_PATH', world.model_paths(), ''),
            ('GAZEBO_RESOURCE_PATH', world.resource_paths(), GAZEBO_DEFAULT_RESOURCE_PATH)):
        if paths:
            current = environ.get(var, '') or default
            env[var] = os.pathsep.join(paths + ([current] if current else []))
    return env


_SCENE = re.compile(r'<scene>(.*?)</scene>', re.S)
_SHADOWS = re.compile(r'<shadows>[^<]*</shadows>')


def without_shadows(sdf: str) -> str:
    """Return the world SDF with shadows off (SDF's default is on).

    Shadows cost the software renderer (CPU container) real-time factor; our
    own worlds set <scene><shadows>0</shadows></scene>, external ones may not.
    """
    match = _SCENE.search(sdf)
    if match:
        body = match.group(1)
        body = (_SHADOWS.sub('<shadows>0</shadows>', body) if _SHADOWS.search(body)
                else body + '<shadows>0</shadows>')
        return sdf[:match.start(1)] + body + sdf[match.end(1):]
    head = re.search(r'<world[^>]*>', sdf)
    if head is None:
        raise ValueError('no <world> element')
    return sdf[:head.end()] + '\n    <scene><shadows>0</shadows></scene>' + sdf[head.end():]
