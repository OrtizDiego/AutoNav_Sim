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

"""Tests for verifying that URDF files can be parsed correctly using xacro."""

import os
import subprocess


def test_xacro_parsing():
    """Verify the main robot xacro file can be processed without errors."""
    # Get the path to the xacro file
    urdf_dir = os.environ.get('URDF_DIR')
    if urdf_dir:
        xacro_file = os.path.join(urdf_dir, 'robot.urdf.xacro')
    else:
        # Fallback logic
        test_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_path = os.path.dirname(test_dir)
        xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    assert os.path.exists(xacro_file), f"Xacro file not found at {xacro_file}"

    # Try to process the xacro file
    try:
        result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True, check=True)
        assert result.returncode == 0
        assert 'robot' in result.stdout
    except subprocess.CalledProcessError as err:
        assert False, f"Xacro parsing failed: {err.stderr}"
    except FileNotFoundError:
        # xacro command not found
        assert False, "xacro command not found"
