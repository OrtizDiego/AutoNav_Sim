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
import re
import subprocess
import sys

import pytest


def _scripts_path():
    scripts_path = os.environ.get('SCRIPTS_DIR')
    if not scripts_path:
        test_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_path = os.path.dirname(test_dir)
        scripts_path = os.path.join(pkg_path, 'my_bot')
    return scripts_path


# Executable nodes (setup.py console_scripts)
SCRIPTS = [
    'ball_controller.py',
    'ball_teleop.py',
    'ball_chaser.py',
    'sensor_fusion.py',
    'person_controller.py',
    'person_tracker.py',
    'security_guard_bt.py',
    'system_monitor.py',
]

PKG_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(os.path.dirname(PKG_PATH))

# make target -> (launch file, world, rviz config)
SCENARIOS = {
    'sim': ('sim.launch.py', 'room.world', 'navigation.rviz'),
    'nav-sim': ('nav_sim.launch.py', 'room.world', 'navigation.rviz'),
    'ball-sim': ('ball_sim.launch.py', 'ball.world', 'perception.rviz'),
    'person-sim': ('person_sim.launch.py', 'person.world', 'person.rviz'),
    'yolo-sim': ('yolo_sim.launch.py', 'yolo.world', 'perception.rviz'),
}


def test_scripts_have_shebang():
    """Verify all scripts have a valid python3 shebang line."""
    scripts_path = _scripts_path()
    for script in SCRIPTS:
        full_path = os.path.join(scripts_path, script)
        with open(full_path) as f:
            first_line = f.readline()
        assert first_line.startswith('#!'), f'{script} missing shebang'
        assert 'python3' in first_line, f'{script} shebang must reference python3'


def test_scripts_compile():
    """Verify all scripts have valid Python syntax (py_compile check)."""
    scripts_path = _scripts_path()
    for script in SCRIPTS:
        full_path = os.path.join(scripts_path, script)
        result = subprocess.run(
            [sys.executable, '-m', 'py_compile', full_path],
            capture_output=True, text=True)
        assert result.returncode == 0, (
            f'{script} has syntax error:\n{result.stderr}')


def test_setup_entry_points_match_scripts():
    """Every console_script points at an existing module with a main()."""
    with open(os.path.join(PKG_PATH, 'setup.py')) as f:
        setup_src = f.read()
    entries = re.findall(r"'(\w+) = my_bot\.(\w+):main'", setup_src)
    assert sorted(f'{mod}.py' for _, mod in entries) == sorted(SCRIPTS)
    for _, mod in entries:
        with open(os.path.join(_scripts_path(), f'{mod}.py')) as f:
            assert 'def main(' in f.read(), mod


def _make_targets():
    makefile = os.path.join(REPO_ROOT, 'Makefile')
    if not os.path.exists(makefile):
        # Inside the Docker container only src/ is mounted.
        pytest.skip('Makefile not available (only src/ is mounted)')
    with open(makefile) as f:
        return dict(re.findall(r'^([a-z-]+):\n\t(.*)$', f.read(), re.M))


@pytest.mark.parametrize('target', sorted(SCENARIOS))
def test_scenario_wiring(target):
    """make <scenario> launches the right file, world and RViz layout."""
    launch, world, rviz = SCENARIOS[target]
    with open(os.path.join(PKG_PATH, 'launch', launch)) as f:
        src = f.read()
    if world != 'room.world':  # room.world is sim.launch.py's default
        assert f"'{world}'" in src
    assert f"'{rviz}'" in src
    assert os.path.exists(os.path.join(PKG_PATH, 'worlds', world))
    assert os.path.exists(os.path.join(PKG_PATH, 'config', rviz))
    assert f'ros2 launch $(PACKAGE_NAME) {launch}' in _make_targets()[target]


def test_gazebo_launch_arguments_do_not_leak():
    """Gazebo's params_file:='' must not reach Nav2 (make nav-sim crashed on it)."""
    with open(os.path.join(PKG_PATH, 'launch', 'sim.launch.py')) as f:
        assert 'GroupAction(scoped=True' in f.read()
    for launch in os.listdir(os.path.join(PKG_PATH, 'launch')):
        with open(os.path.join(PKG_PATH, 'launch', launch)) as f:
            src = f.read()
        if "'navigation.launch.py'" in src:
            assert "'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml')" in src, launch


def test_makefile_runs_only_existing_nodes():
    for target, cmd in _make_targets().items():
        for node in re.findall(r'ros2 run \$\(PACKAGE_NAME\) (\w+)', cmd):
            assert f'{node}.py' in SCRIPTS, f'make {target} runs unknown {node}'


@pytest.mark.parametrize('launch_file', sorted(
    f for f in os.listdir(os.path.join(PKG_PATH, 'launch')) if f.endswith('.launch.py')))
def test_launch_description_builds(launch_file):
    """Import and build each launch file (needs a ROS environment)."""
    # Skip only without a sourced ROS environment; in CI a broken import
    # must fail here, not be skipped.
    if not os.environ.get('ROS_DISTRO'):
        pytest.skip('ROS environment not sourced')
    path = os.path.join(PKG_PATH, 'launch', launch_file)
    # Run in a fresh interpreter: conftest.py replaces rclpy and the message
    # packages with stubs in sys.modules, which breaks importing launch_ros.
    # colcon test does not put my_bot itself on the ament index, so resolve
    # its share directory to the source package instead.
    script = (
        'import importlib.util, sys\n'
        'import ament_index_python.packages as pkgs\n'
        'pkgs.get_package_share_directory = lambda _pkg: sys.argv[1]\n'
        'spec = importlib.util.spec_from_file_location("launch_under_test", sys.argv[2])\n'
        'module = importlib.util.module_from_spec(spec)\n'
        'spec.loader.exec_module(module)\n'
        'assert module.generate_launch_description().entities\n'
    )
    result = subprocess.run(
        [sys.executable, '-c', script, PKG_PATH, path],
        capture_output=True, text=True)
    assert result.returncode == 0, (
        f'{launch_file} failed to build:\n{result.stderr}')
