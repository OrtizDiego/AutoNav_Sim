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

"""Save an occupancy grid as a map_server map (YAML + PGM).

The explorer writes the map it built with this instead of calling
map_saver_cli: same "trinary" format (occupied 0, free 254, unknown 205;
row 0 of the image = maximum y), so map_server, AMCL and ClearanceMap
load it like any map saved by `make save-map`.

Pure Python + numpy, no ROS: importable by unit tests.
"""

import os
from typing import Tuple

import numpy as np
import yaml

from my_bot.frontier import Grid, classify

OCCUPIED_PIXEL, FREE_PIXEL, UNKNOWN_PIXEL = 0, 254, 205


def grid_to_pgm(grid: Grid, free_below: int = 25, occupied_from: int = 65) -> np.ndarray:
    """Return the map image (uint8, row 0 = top = maximum y)."""
    free, _, occupied = classify(grid.data, free_below, occupied_from)
    image = np.full(grid.data.shape, UNKNOWN_PIXEL, dtype=np.uint8)
    image[free] = FREE_PIXEL
    image[occupied] = OCCUPIED_PIXEL
    return np.flipud(image)


def save_map(path: str, grid: Grid, yaw: float = 0.0) -> Tuple[str, str]:
    """Write ``path``.yaml and ``path``.pgm; return both paths.

    ``path`` is without extension, like map_saver_cli's -f.
    """
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    image = grid_to_pgm(grid)
    pgm_path, yaml_path = path + '.pgm', path + '.yaml'
    height, width = image.shape
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n# CREATOR: my_bot explorer {grid.resolution:.3f} m/pix\n'
                f'{width} {height}\n255\n'.encode())
        f.write(image.tobytes())
    meta = {'image': os.path.basename(pgm_path), 'mode': 'trinary',
            'resolution': float(grid.resolution),
            'origin': [float(grid.origin[0]), float(grid.origin[1]), float(yaw)],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    return yaml_path, pgm_path
