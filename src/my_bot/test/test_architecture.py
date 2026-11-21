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

"""The package's structure, checked on the source (nothing is imported).

Three layers, each allowed to import only from the ones above it:

  LIBRARIES   pure Python (numpy, OpenCV, PyYAML): the algorithms. No ROS
              import at all, so they run and are tested anywhere.
  MESSAGES    build or consume ROS messages, but hold no node, clock or
              executor (no rclpy, tf2, Nav2).
  NODES       the console scripts, plus NODE_SUPPORT they share.

TOOLS are offline command-line programs (python3 -m my_bot.<tool>) that read
recorded ROS data; they may import what they need and nothing imports them.

A new module must be put in one of the lists below, which is where its
author decides what it may depend on. Functions stay under a branching
limit, so the next long callback is split when it is written, not later.
"""

import ast
import os
import re

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PKG_DIR, 'my_bot')

LIBRARIES = {
    'clearance_map', 'conversions', 'follow_control', 'frontier', 'guard_zones',
    'incident_log', 'map_io', 'object_detector', 'room_sweep', 'scenario_metrics',
    'scenario_script', 'scenario_summary', 'target_estimate', 'track_filter',
    'worlds',
}
MESSAGES = {'guard_markers', 'guard_tree', 'markers'}
NODE_SUPPORT = {'node_utils'}
TOOLS = {'replay_export'}

ROS_RUNTIME = {'rclpy', 'tf2_ros', 'nav2_simple_commander', 'cv_bridge',
               'launch', 'launch_ros', 'ament_index_python'}
# McCabe complexity (decision points + 1) no function may exceed
MAX_COMPLEXITY = 15


def _nodes():
    with open(os.path.join(PKG_DIR, 'setup.py')) as f:
        return set(re.findall(r"'\w+ = my_bot\.(\w+):main'", f.read()))


def _modules():
    return {name[:-3] for name in os.listdir(SRC)
            if name.endswith('.py') and name != '__init__.py'}


def _tree(module):
    with open(os.path.join(SRC, module + '.py')) as f:
        return ast.parse(f.read())


def _imports(module):
    """(top-level packages, my_bot modules) imported anywhere in ``module``."""
    packages, internal = set(), set()
    for node in ast.walk(_tree(module)):
        names = []
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            names = [node.module]
            if node.module == 'my_bot':
                names = [f'my_bot.{alias.name}' for alias in node.names]
        for name in names:
            parts = name.split('.')
            if parts[0] == 'my_bot':
                internal.add(parts[1])
            else:
                packages.add(parts[0])
    return packages, internal


def _is_message_package(package):
    return package.endswith(('_msgs', '_srvs')) or package == 'builtin_interfaces'


def test_every_module_has_a_layer():
    layers = [LIBRARIES, MESSAGES, NODE_SUPPORT, TOOLS, _nodes()]
    listed = set().union(*layers)
    assert _modules() == listed, (
        'put new modules in a layer of test_architecture.py; drop removed ones: '
        f'{sorted(_modules() ^ listed)}')
    assert sum(len(layer) for layer in layers) == len(listed), 'a module is in two layers'


@pytest.mark.parametrize('module', sorted(LIBRARIES))
def test_libraries_are_pure_python(module):
    packages, internal = _imports(module)
    ros = {p for p in packages if p in ROS_RUNTIME or _is_message_package(p)}
    assert not ros, f'{module} is a library but imports {sorted(ros)}'
    assert internal <= LIBRARIES, (
        f'{module} is a library but imports {sorted(internal - LIBRARIES)}')


@pytest.mark.parametrize('module', sorted(MESSAGES))
def test_message_layer_needs_no_running_ros(module):
    packages, internal = _imports(module)
    assert not packages & ROS_RUNTIME, (
        f'{module} imports {sorted(packages & ROS_RUNTIME)}')
    assert internal <= LIBRARIES | MESSAGES, (
        f'{module} imports {sorted(internal - LIBRARIES - MESSAGES)}')


def test_nothing_imports_a_node_or_a_tool():
    """They are entry points; code two of them need belongs in a lower layer."""
    entry_points = _nodes() | TOOLS
    for module in sorted(_modules()):
        _, internal = _imports(module)
        assert not internal & entry_points, (
            f'{module} imports the entry point(s) {sorted(internal & entry_points)}')


_BRANCHES = (ast.If, ast.For, ast.While, ast.ExceptHandler, ast.IfExp,
             ast.Assert, ast.comprehension)


def _complexity(func):
    """Decision points of ``func`` + 1, nested functions counted on their own."""
    score = 1
    stack = list(ast.iter_child_nodes(func))
    while stack:
        node = stack.pop()
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            continue
        if isinstance(node, _BRANCHES):
            score += 1 + len(getattr(node, 'ifs', ()))  # a comprehension's filters
        elif isinstance(node, ast.BoolOp):
            score += len(node.values) - 1
        stack.extend(ast.iter_child_nodes(node))
    return score


def test_complexity_counts_decision_points():
    func = ast.parse(
        'def f(a, b):\n'
        '    if a and b:\n'
        '        return [x for x in a if x]\n'
        '    for y in b:\n'
        '        pass\n'
        '    return 0\n').body[0]
    assert _complexity(func) == 6  # 1 + if + and + comprehension + its if + for


@pytest.mark.parametrize('module', sorted(_modules()))
def test_functions_stay_simple(module):
    too_complex = {
        node.name: _complexity(node) for node in ast.walk(_tree(module))
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and _complexity(node) > MAX_COMPLEXITY}
    assert not too_complex, (
        f'{module}: split these (complexity limit {MAX_COMPLEXITY}): {too_complex}')
