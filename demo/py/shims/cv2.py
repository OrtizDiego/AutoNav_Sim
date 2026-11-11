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

"""The two OpenCV calls the web demo needs, in numpy.

Pyodide's OpenCV wheel is ~49 MB; the nodes the demo runs only use OpenCV
to read the map image and to turn it into a distance-to-wall grid
(my_bot.clearance_map). This module is put on the path as ``cv2`` in the
browser only. test_brain.py checks it against the real OpenCV.
"""

import numpy as np

IMREAD_GRAYSCALE = 0
DIST_L2 = 2
DIST_MASK_PRECISE = 0

_cache = {}


def imread(path, flags=IMREAD_GRAYSCALE):
    """Read a binary PGM (P5, 8 bit), what map_saver writes; None if not one."""
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except OSError:
        return None
    if data[:2] != b'P5':
        return None
    fields, pos = [], 2
    while len(fields) < 3:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(int(data[pos:end]))
        pos = end
    width, height, maxval = fields
    if maxval > 255:
        return None
    pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos + 1)
    return pixels.reshape(height, width).copy()


def distanceTransform(src, distanceType=DIST_L2, maskSize=DIST_MASK_PRECISE):
    """Exact Euclidean distance (pixels) from each non-zero pixel to the nearest zero."""
    src = np.asarray(src)
    key = (src.shape, src.tobytes())
    if key in _cache:
        return _cache[key].copy()
    h, w = src.shape
    big = float(h + w)
    zero = src == 0
    # Distance to the nearest zero in the same column
    rows = np.arange(h, dtype=np.float32)[:, None]
    above = np.maximum.accumulate(np.where(zero, rows, -big), axis=0)
    below = np.minimum.accumulate(np.where(zero, rows, 2 * big)[::-1], axis=0)[::-1]
    g2 = np.minimum(rows - above, below - rows).astype(np.float32) ** 2
    # d2(i, j) = min over k of g2(i, j + k) + k^2; no k beyond sqrt(max d2) helps
    d2 = g2.copy()
    k = 1
    while k < w and k * k < float(d2.max()):
        np.minimum(d2[:, :-k], g2[:, k:] + k * k, out=d2[:, :-k])
        np.minimum(d2[:, k:], g2[:, :-k] + k * k, out=d2[:, k:])
        k += 1
    out = np.sqrt(d2).astype(np.float32)
    _cache[key] = out
    return out.copy()


def __getattr__(name):
    raise AttributeError(
        f'cv2.{name}: the web demo ships a two-function stand-in for OpenCV')
