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

"""Stub the ROS Python packages so every node module imports without ROS.

The unit tests only exercise pure helpers and behaviour-tree leaves, so the
ROS client library and message packages are replaced by light fakes before
any test module is collected. numpy, OpenCV, PyYAML and py_trees are real.
Tests that need a real ROS environment (launch files) run in a subprocess.
"""

import os
import sys
import types


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Msg:
    """Accepts any keyword fields, like a generated ROS message."""

    def __init__(self, **fields):
        for k, v in fields.items():
            setattr(self, k, v)


class Twist(_Msg):
    def __init__(self, **fields):
        self.linear, self.angular = _Vec(), _Vec()
        super().__init__(**fields)


class Header(_Msg):
    def __init__(self, **fields):
        self.stamp, self.frame_id = None, ''
        super().__init__(**fields)


class PoseStamped(_Msg):
    def __init__(self, **fields):
        self.header = Header()
        self.pose = types.SimpleNamespace(
            position=_Vec(), orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        super().__init__(**fields)


class _Data(_Msg):
    def __init__(self, data=None, **fields):
        self.data = data
        super().__init__(**fields)


class Node:
    """Bare stand-in for rclpy.node.Node."""

    def __init__(self, *args, **kwargs):
        pass


def _module(name, **attrs):
    mod = sys.modules.get(name) or types.ModuleType(name)
    for key, value in attrs.items():
        if not hasattr(mod, key):
            setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


def _plain(*names):
    return {n: type(n, (_Msg,), {}) for n in names}


_module('rclpy', ok=lambda: True, init=lambda **kw: None,
        spin=lambda node: None, shutdown=lambda: None)
_module('rclpy.node', Node=Node)
_module('rclpy.qos', qos_profile_sensor_data=None, QoSProfile=_Msg,
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2))
_module('cv_bridge', CvBridge=_Msg)
_module('builtin_interfaces')
_module('builtin_interfaces.msg', Time=_Msg)
_module('geometry_msgs')
_module('geometry_msgs.msg', Twist=Twist, PoseStamped=PoseStamped,
        **_plain('TwistStamped', 'PointStamped', 'Pose'))
_module('sensor_msgs')
_module('sensor_msgs.msg', **_plain('Image', 'LaserScan'))
_module('std_msgs')
_module('std_msgs.msg', Header=Header, Bool=_Data, Float32=_Data, String=_Data,
        Float32MultiArray=_Data)
_module('std_srvs')
_module('std_srvs.srv', **_plain('Trigger'))
_module('nav_msgs')
_module('nav_msgs.msg', **_plain('Odometry'))
_module('diagnostic_msgs')
_module('diagnostic_msgs.msg', **_plain('DiagnosticArray', 'DiagnosticStatus', 'KeyValue'))
_module('visualization_msgs')
_module('visualization_msgs.msg', **_plain('Marker', 'MarkerArray'))
_module('nav2_simple_commander')
_module('nav2_simple_commander.robot_navigator', BasicNavigator=_Msg)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)
