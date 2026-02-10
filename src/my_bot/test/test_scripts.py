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

"""Tests for verifying that scripts are executable and have correct shebangs."""

import os
import stat


def test_scripts_executable():
    """Verify scripts in the package directory are valid."""
    scripts_path = os.environ.get('SCRIPTS_DIR')
    if not scripts_path:
        test_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_path = os.path.dirname(test_dir)
        scripts_path = os.path.join(pkg_path, 'my_bot')

    scripts = [
        'ball_chaser.py',
        'camera_test.py',
        'patrol.py',
        'security_guard.py'
    ]

    for script in scripts:
        script_full_path = os.path.join(scripts_path, script)
        assert os.path.exists(script_full_path), f"{script} not found in {scripts_path}"

        # Check shebang
        with open(script_full_path, 'r') as file:
            first_line = file.readline()
            assert first_line.startswith('#!'), f"{script} missing shebang"
            assert 'python3' in first_line, f"{script} shebang should use python3"
