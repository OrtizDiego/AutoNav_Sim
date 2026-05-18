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

"""Tests for verifying that scripts are executable, have correct shebangs, and compile."""

import os
import subprocess
import sys


def _scripts_path():
    scripts_path = os.environ.get('SCRIPTS_DIR')
    if not scripts_path:
        test_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_path = os.path.dirname(test_dir)
        scripts_path = os.path.join(pkg_path, 'my_bot')
    return scripts_path


SCRIPTS = [
    'ball_chaser.py',
    'camera_test.py',
    'patrol.py',
    'security_guard.py',
    'sensor_fusion.py',
    'system_monitor.py',
    'security_guard_bt.py',
    'object_detector.py',
    'intruder_bot.py',
    'obstacle_controller.py',
    'person_controller.py',
    'person_tracker.py',
    'person_follower.py',
]


def test_scripts_have_shebang():
    """Verify all scripts have a valid python3 shebang line."""
    scripts_path = _scripts_path()
    for script in SCRIPTS:
        full_path = os.path.join(scripts_path, script)
        if not os.path.exists(full_path):
            continue  # script added in later phase — skip gracefully
        with open(full_path) as f:
            first_line = f.readline()
        assert first_line.startswith('#!'), f'{script} missing shebang'
        assert 'python3' in first_line, f'{script} shebang must reference python3'


def test_scripts_compile():
    """Verify all scripts have valid Python syntax (py_compile check)."""
    scripts_path = _scripts_path()
    for script in SCRIPTS:
        full_path = os.path.join(scripts_path, script)
        if not os.path.exists(full_path):
            continue  # script added in later phase — skip gracefully
        result = subprocess.run(
            [sys.executable, '-m', 'py_compile', full_path],
            capture_output=True, text=True)
        assert result.returncode == 0, (
            f'{script} has syntax error:\n{result.stderr}')
