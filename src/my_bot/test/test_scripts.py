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
    'perf_monitor.py',
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


def test_scenarios_can_run_without_the_gazebo_window():
    """make <scenario> GUI=false reaches gazebo.launch.py's gui argument."""
    targets = _make_targets()
    for target in SCENARIOS:
        assert targets[target].endswith('.launch.py $(SCENARIO_ARGS)"'), target
    with open(os.path.join(REPO_ROOT, 'Makefile')) as f:
        assert 'SCENARIO_ARGS := gui:=$(GUI)' in f.read()
    with open(os.path.join(PKG_PATH, 'launch', 'sim.launch.py')) as f:
        src = f.read()
    assert "'gui', default_value='true'" in src
    assert "'gui': LaunchConfiguration('gui')" in src


def test_gazebo_launch_arguments_do_not_leak():
    """Gazebo's params_file:='' must not reach Nav2 (make nav-sim crashed on it)."""
    with open(os.path.join(PKG_PATH, 'launch', 'sim.launch.py')) as f:
        assert 'GroupAction(scoped=True' in f.read()
    for launch in os.listdir(os.path.join(PKG_PATH, 'launch')):
        if not launch.endswith('.launch.py'):
            continue
        with open(os.path.join(PKG_PATH, 'launch', launch)) as f:
            src = f.read()
        if "'navigation.launch.py'" in src:
            assert "'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml')" in src, launch


def test_makefile_runs_only_existing_nodes():
    for target, cmd in _make_targets().items():
        for node in re.findall(r'ros2 run \$\(PACKAGE_NAME\) (\w+)', cmd):
            assert f'{node}.py' in SCRIPTS, f'make {target} runs unknown {node}'


def test_yolo_scenarios_need_the_model_and_make_image_provides_it():
    """No model: stop with the fix instead of a sim that tracks nothing.

    The fix (make image, named by person_tracker too) must recreate the
    container: docker compose build alone leaves it on the old image.
    """
    targets = _make_targets()
    for target in ('person-sim', 'yolo-sim'):
        assert '$(NEED_YOLO) $(SOURCE) && ros2 launch' in targets[target]
    with open(os.path.join(REPO_ROOT, 'Makefile')) as f:
        image = re.search(r'^image:\n((?:\t.*\n)+)', f.read(), re.M).group(1)
    assert 'docker compose build' in image
    assert 'docker compose up -d --force-recreate $(SERVICE)' in image


def test_scenarios_stop_leftovers_first():
    """A closed terminal leaves the old run going; it breaks the next one.

    Its gzserver keeps Gazebo's port (the old world stays on screen) and its
    nodes share names with the new ones (Nav2 bringup aborts: no map frame).
    """
    targets = _make_targets()
    for target in SCENARIOS:
        assert targets[target].startswith('$(EXEC) "$(STOP_LEFTOVERS) '), target
    assert targets['stop'] == '$(EXEC) "$(STOP_LEFTOVERS)"'
    script = os.path.join(REPO_ROOT, 'src', 'stop_sim.sh')
    with open(script) as f:
        src = f.read()
    for name in ('gzserver', 'component_container', 'ros2 launch'):
        assert name in src
    assert 'skip' in src  # spares the scenario's own shell, which matches


def test_compose_isolates_ros_and_gazebo():
    """network_mode: host shares ports with every other container on the host."""
    compose = os.path.join(REPO_ROOT, 'compose.yaml')
    if not os.path.exists(compose):
        pytest.skip('compose.yaml not available (only src/ is mounted)')
    with open(compose) as f:
        src = f.read()
    assert 'ROS_DOMAIN_ID=${AUTONAV_ROS_DOMAIN_ID:-' in src
    assert 'GAZEBO_MASTER_URI=http://localhost:${AUTONAV_GAZEBO_PORT:-' in src
    assert ':-11345}' not in src  # Gazebo's default: what the others use


def test_dockerfile_installs_pip_before_using_it():
    """osrf/ros and ubuntu ship without pip: a bare pip3 exits with 127."""
    dockerfile = os.path.join(REPO_ROOT, 'Dockerfile')
    if not os.path.exists(dockerfile):
        pytest.skip('Dockerfile not available (only src/ is mounted)')
    with open(dockerfile) as f:
        stages = re.split(r'^FROM ', f.read(), flags=re.M)[1:]
    assert any('pip3 install' in stage for stage in stages)
    for stage in stages:
        if 'pip3 install' in stage:
            before_pip = stage.split('pip3 install')[0]
            assert 'python3-pip' in before_pip, stage.splitlines()[0]


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
