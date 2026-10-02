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

"""Edge cases of the pure helpers that the scenario tests do not reach."""

import math
import random

import cv2
import numpy as np
import py_trees
import pytest

from my_bot import ball_controller as bc
from my_bot import follow_control as fc
from my_bot import security_guard_bt as sg
from my_bot import sensor_fusion as sf
from my_bot.clearance_map import ClearanceMap, safe_heading
from my_bot.object_detector import postprocess


# ---------------------------------------------------------------------------
# follow_control
# ---------------------------------------------------------------------------

class TestFrontClearance:

    N = 360
    INC = 2.0 * math.pi / N

    def _ranges(self, value=10.0):
        return [value] * self.N

    def _set(self, ranges, deg, value):
        ranges[int(round((math.radians(deg) + math.pi) / self.INC)) % self.N] = value

    def test_empty_scan_is_clear(self):
        assert fc.front_clearance([], -math.pi, self.INC) == math.inf

    def test_closest_return_ahead(self):
        ranges = self._ranges()
        self._set(ranges, 10, 1.5)
        self._set(ranges, -20, 0.8)
        assert fc.front_clearance(ranges, -math.pi, self.INC) == pytest.approx(0.8)

    def test_ignores_returns_outside_the_cone(self):
        ranges = self._ranges()
        self._set(ranges, 90, 0.3)
        self._set(ranges, 180, 0.3)
        assert fc.front_clearance(ranges, -math.pi, self.INC) == pytest.approx(10.0)

    def test_ignores_invalid_and_too_close_returns(self):
        ranges = self._ranges(float('inf'))
        self._set(ranges, 0, float('nan'))
        self._set(ranges, 5, 0.05)          # below the lidar's minimum range
        assert fc.front_clearance(ranges, -math.pi, self.INC) == math.inf

    def test_cone_wraps_for_a_zero_to_two_pi_scan(self):
        ranges = [10.0] * self.N
        ranges[self.N - 5] = 0.9            # -5 deg on a [0, 2pi) scan
        assert fc.front_clearance(ranges, 0.0, self.INC) == pytest.approx(0.9)


def test_search_turns_left_without_a_last_bearing():
    assert fc.search_command(0.0, 0.6) == pytest.approx(0.6)
    assert fc.search_command(-0.2, 0.6) == pytest.approx(-0.6)


# ---------------------------------------------------------------------------
# clearance_map
# ---------------------------------------------------------------------------

def _write_map(tmp_path, pixels, negate=0, image='room.pgm'):
    cv2.imwrite(str(tmp_path / 'room.pgm'), pixels)
    yaml_path = tmp_path / 'room.yaml'
    yaml_path.write_text(
        f'image: {image}\nresolution: 0.1\norigin: [-1.0, -1.0, 0.0]\n'
        f'negate: {negate}\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return str(yaml_path)


def _room():
    """20x20 cells (2 m x 2 m): free inside a 1-cell wall."""
    pixels = np.full((20, 20), 254, dtype=np.uint8)
    pixels[0, :] = pixels[-1, :] = pixels[:, 0] = pixels[:, -1] = 0
    return pixels


class TestClearanceMapFiles:

    def test_loads_a_relative_image(self, tmp_path):
        cmap = ClearanceMap.from_yaml(_write_map(tmp_path, _room()))
        assert cmap.clearance(0.0, 0.0) == pytest.approx(0.9, abs=0.1)

    def test_loads_an_absolute_image(self, tmp_path):
        cv2.imwrite(str(tmp_path / 'room.pgm'), _room())
        path = _write_map(tmp_path, _room(), image=str(tmp_path / 'room.pgm'))
        assert ClearanceMap.from_yaml(path).clearance(0.0, 0.0) > 0.5

    def test_negated_map(self, tmp_path):
        cmap = ClearanceMap.from_yaml(_write_map(tmp_path, 255 - _room(), negate=1))
        assert cmap.clearance(0.0, 0.0) == pytest.approx(0.9, abs=0.1)

    def test_missing_image_raises(self, tmp_path):
        path = _write_map(tmp_path, _room(), image='missing.pgm')
        with pytest.raises(IOError, match='missing.pgm'):
            ClearanceMap.from_yaml(path)


def test_random_free_point_raises_when_nothing_is_open_enough():
    cmap = ClearanceMap.from_bounds(1.0)
    with pytest.raises(ValueError, match='clearance'):
        cmap.random_free_point(50.0, random.Random(0))


def test_safe_heading_returns_the_most_open_direction_when_boxed_in():
    cmap = ClearanceMap.from_bounds(1.0)
    heading, free = safe_heading(cmap, 0.5, 0.0, 0.0, lookahead=5.0, min_clear=0.2)
    assert free < 5.0                             # nothing has 5 m free
    # Best is away from the near wall (+x), toward the far corners
    assert math.cos(heading) < -0.5
    fan = [cmap.free_distance(0.5, 0.0, math.radians(d), 5.0, 0.2)
           for d in range(-180, 181, 10)]
    assert free == pytest.approx(max(fan))


# ---------------------------------------------------------------------------
# object_detector
# ---------------------------------------------------------------------------

def test_score_exactly_at_threshold_is_dropped_by_nms():
    """The mask keeps score >= threshold, NMSBoxes only score > threshold."""
    out = np.zeros((1, 84, 8400), dtype=np.float32)
    out[0, :4, 0] = [320.0, 320.0, 50.0, 100.0]
    out[0, 4, 0] = 0.5
    assert postprocess(out, (640, 640), conf_threshold=0.5) == []


def test_postprocess_accepts_an_unbatched_output():
    out = np.zeros((84, 8400), dtype=np.float32)
    out[:4, 0] = [320.0, 320.0, 50.0, 100.0]
    out[4, 0] = 0.9
    assert len(postprocess(out, (640, 640))) == 1


# ---------------------------------------------------------------------------
# sensor_fusion
# ---------------------------------------------------------------------------

def test_scan_window_range_of_an_empty_scan():
    assert sf.scan_window_range([], -math.pi, 0.01, 0.1, -0.1, 0.3, 12.0) is None


def test_monocular_range_none_when_head_cut_and_feet_above_horizon():
    # Head cut off at the top, feet visible but not below the horizon (cy)
    assert sf.monocular_range((300.0, 0.0, 40.0, 240.0), 480, 500.0, 240.0,
                              1.72, 0.1) is None


# ---------------------------------------------------------------------------
# ball_controller
# ---------------------------------------------------------------------------

def test_robot_view_on_top_of_the_robot_falls_back_to_world_axes():
    assert bc.robot_view_to_world(0.3, -0.2, (1.0, 1.0), (1.0, 1.0)) == (0.3, -0.2)


# ---------------------------------------------------------------------------
# security_guard_bt
# ---------------------------------------------------------------------------

def test_wait_at_waypoint_keeps_running_during_the_dwell():
    py_trees.blackboard.Blackboard.clear()
    try:
        wait = sg.WaitAtWaypoint(dwell_secs=60.0)
        assert wait.update() == py_trees.common.Status.RUNNING   # starts dwell
        assert wait.update() == py_trees.common.Status.RUNNING   # still dwelling
        assert py_trees.blackboard.Blackboard.get(sg.BB_DWELL_START) is not None
    finally:
        py_trees.blackboard.Blackboard.clear()
