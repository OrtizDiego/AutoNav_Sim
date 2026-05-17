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

"""Tests for verifying that URDF/Xacro files are well-formed and contain
required sensor fidelity elements."""

import os
import subprocess
import xml.etree.ElementTree as ET


def _urdf_dir():
    urdf_dir = os.environ.get('URDF_DIR')
    if urdf_dir:
        return urdf_dir
    test_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(test_dir), 'urdf')


def test_xacro_parsing():
    """Verify the main robot xacro file can be processed without errors."""
    xacro_file = os.path.join(_urdf_dir(), 'robot.urdf.xacro')
    assert os.path.exists(xacro_file), f"Xacro file not found at {xacro_file}"

    try:
        result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True, check=True)
        assert result.returncode == 0
        assert 'robot' in result.stdout
    except subprocess.CalledProcessError as err:
        assert False, f"Xacro parsing failed: {err.stderr}"
    except FileNotFoundError:
        assert False, "xacro command not found"


def test_lidar_xacro_is_valid_xml():
    """lidar.xacro must be well-formed XML."""
    path = os.path.join(_urdf_dir(), 'lidar.xacro')
    assert os.path.exists(path), f"lidar.xacro not found at {path}"
    tree = ET.parse(path)
    assert tree is not None


def test_lidar_xacro_has_noise():
    """lidar.xacro must contain a Gaussian noise model."""
    path = os.path.join(_urdf_dir(), 'lidar.xacro')
    tree = ET.parse(path)
    root = tree.getroot()
    # Find any <noise type="gaussian"> element
    noise_elems = root.findall('.//{*}noise') + root.findall('.//noise')
    gaussian = [e for e in noise_elems if e.get('type') == 'gaussian']
    assert len(gaussian) > 0, "No Gaussian noise model found in lidar.xacro"


def test_camera_xacro_is_valid_xml():
    """camera.xacro must be well-formed XML."""
    path = os.path.join(_urdf_dir(), 'camera.xacro')
    assert os.path.exists(path), f"camera.xacro not found at {path}"
    tree = ET.parse(path)
    assert tree is not None


def test_camera_xacro_has_noise():
    """camera.xacro must contain a Gaussian noise model."""
    path = os.path.join(_urdf_dir(), 'camera.xacro')
    tree = ET.parse(path)
    root = tree.getroot()
    noise_elems = root.findall('.//{*}noise') + root.findall('.//noise')
    gaussian = [e for e in noise_elems if e.get('type') == 'gaussian']
    assert len(gaussian) > 0, "No Gaussian noise model found in camera.xacro"


def test_camera_update_rate_is_30():
    """camera.xacro must have update_rate of 30 Hz."""
    path = os.path.join(_urdf_dir(), 'camera.xacro')
    tree = ET.parse(path)
    root = tree.getroot()
    rate_elems = root.findall('.//{*}update_rate') + root.findall('.//update_rate')
    assert len(rate_elems) > 0, "No <update_rate> element in camera.xacro"
    assert rate_elems[0].text.strip() == '30', (
        f"Expected update_rate=30, got {rate_elems[0].text}")


def test_gazebo_control_xacro_has_wheel_friction():
    """gazebo_control.xacro must have friction (mu1/mu2) on drive wheels."""
    path = os.path.join(_urdf_dir(), 'gazebo_control.xacro')
    assert os.path.exists(path), f"gazebo_control.xacro not found at {path}"
    with open(path) as f:
        content = f.read()
    assert content.count('<mu1 value="1.0"/>') >= 2, (
        "Expected mu1=1.0 on at least 2 wheel gazebo elements")
