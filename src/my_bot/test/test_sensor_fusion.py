# Copyright 2026 root
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

"""Unit tests for sensor_fusion pure math functions.

These tests import module-level functions directly and do not require
a running ROS node or any ROS infrastructure; conftest.py stubs the ROS
packages so they run in a plain Python environment.
"""

import math

from my_bot.sensor_fusion import (
    compute_bearing,
    bearing_to_scan_index,
    validate_range,
)

# Camera parameters matching camera.xacro (FOV=1.089 rad, width=640)
CX = 320.0
FX = 320.0 / math.tan(1.089 / 2.0)   # ≈ 534.8

# LaserScan parameters matching lidar.xacro (360 samples, -π to +π)
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / 360.0
N_SAMPLES = 360


class TestComputeBearing:

    def test_center_pixel_gives_zero_bearing(self):
        """Pixel at image center should produce bearing of exactly 0."""
        bearing = compute_bearing(320.0, CX, FX)
        assert abs(bearing) < 1e-9

    def test_left_edge_gives_positive_bearing(self):
        """Pixel at left edge (0) is to the robot's left: positive (REP-103)."""
        bearing = compute_bearing(0.0, CX, FX)
        assert bearing > 0.0

    def test_right_edge_gives_negative_bearing(self):
        """Pixel at right edge (639) is to the robot's right: negative (REP-103)."""
        bearing = compute_bearing(639.0, CX, FX)
        assert bearing < 0.0

    def test_symmetric_about_center(self):
        """Pixels equidistant from center should produce equal-magnitude bearings."""
        b_left = compute_bearing(CX - 100, CX, FX)
        b_right = compute_bearing(CX + 100, CX, FX)
        assert abs(abs(b_left) - abs(b_right)) < 1e-9
        assert b_left > 0.0
        assert b_right < 0.0

    def test_full_fov_magnitude(self):
        """Bearing at far edge should be approximately half the camera FOV."""
        bearing = compute_bearing(639.0, CX, FX)
        expected = -math.atan2(319.0, FX)
        assert abs(bearing - expected) < 1e-6

    def test_left_pixel_maps_to_left_lidar_beam(self):
        """A target left of centre must look up a beam with angle > 0."""
        bearing = compute_bearing(100.0, CX, FX)
        idx = bearing_to_scan_index(bearing, ANGLE_MIN, ANGLE_INC, N_SAMPLES)
        assert ANGLE_MIN + idx * ANGLE_INC > 0.0


class TestBearingToScanIndex:

    def test_zero_bearing_maps_to_center(self):
        """Bearing 0 (straight ahead) → index 180 (middle of 0..359 scan)."""
        idx = bearing_to_scan_index(0.0, ANGLE_MIN, ANGLE_INC, N_SAMPLES)
        assert idx == 180

    def test_positive_bearing_above_center(self):
        """Positive bearing should map to index > 180."""
        idx = bearing_to_scan_index(0.5, ANGLE_MIN, ANGLE_INC, N_SAMPLES)
        assert idx > 180

    def test_negative_bearing_below_center(self):
        """Negative bearing should map to index < 180."""
        idx = bearing_to_scan_index(-0.5, ANGLE_MIN, ANGLE_INC, N_SAMPLES)
        assert idx < 180

    def test_clamps_to_zero_at_min(self):
        """Bearing below angle_min should clamp to index 0."""
        idx = bearing_to_scan_index(-100.0, ANGLE_MIN, ANGLE_INC, N_SAMPLES)
        assert idx == 0

    def test_clamps_to_max_at_max(self):
        """Bearing above angle_max should clamp to last valid index."""
        idx = bearing_to_scan_index(100.0, ANGLE_MIN, ANGLE_INC, N_SAMPLES)
        assert idx == N_SAMPLES - 1


class TestValidateRange:

    def test_valid_range_accepted(self):
        assert validate_range(3.0, 0.3, 12.0) is True

    def test_nan_rejected(self):
        assert validate_range(float('nan'), 0.3, 12.0) is False

    def test_inf_rejected(self):
        assert validate_range(float('inf'), 0.3, 12.0) is False

    def test_neg_inf_rejected(self):
        assert validate_range(float('-inf'), 0.3, 12.0) is False

    def test_below_min_rejected(self):
        assert validate_range(0.1, 0.3, 12.0) is False

    def test_above_max_rejected(self):
        assert validate_range(15.0, 0.3, 12.0) is False

    def test_exactly_at_min_accepted(self):
        assert validate_range(0.3, 0.3, 12.0) is True

    def test_exactly_at_max_accepted(self):
        assert validate_range(12.0, 0.3, 12.0) is True
