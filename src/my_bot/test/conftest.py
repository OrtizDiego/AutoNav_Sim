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

The ROS client library and message packages are replaced by light fakes
before any test module is collected. numpy, OpenCV, PyYAML and py_trees are
real. The fake ``Node`` records its parameters, publishers, subscriptions,
services and timers and runs on a manual clock, so the node classes can be
driven directly: call a subscription callback, fire a timer, read what was
published. ``wire`` connects nodes like a ROS graph for integration tests.
Tests that need a real ROS environment (launch files) run in a subprocess.
"""

import os
import sys
import types

import pytest


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


def _quat():
    return types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


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


class _Stamped(_Msg):
    def __init__(self, **fields):
        self.header = Header()
        super().__init__(**fields)


class TwistStamped(_Stamped):
    def __init__(self, **fields):
        self.twist = Twist()
        super().__init__(**fields)


class PointStamped(_Stamped):
    def __init__(self, **fields):
        self.point = _Vec()
        super().__init__(**fields)


class PoseStamped(_Stamped):
    def __init__(self, **fields):
        self.pose = types.SimpleNamespace(position=_Vec(), orientation=_quat())
        super().__init__(**fields)


class Odometry(_Stamped):
    def __init__(self, **fields):
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=_Vec(), orientation=_quat()))
        super().__init__(**fields)


class Image(_Stamped):
    """Carries the numpy frame itself (see CvBridge below)."""

    def __init__(self, **fields):
        self.frame = None
        super().__init__(**fields)


class LaserScan(_Stamped):
    pass


class DiagnosticArray(_Stamped):
    def __init__(self, **fields):
        self.status = []
        super().__init__(**fields)


class DiagnosticStatus(_Msg):
    OK, WARN, ERROR, STALE = 0, 1, 2, 3

    def __init__(self, **fields):
        self.name = self.message = self.hardware_id = ''
        self.level = self.OK
        self.values = []
        super().__init__(**fields)


class Marker(_Stamped):
    SPHERE, TEXT_VIEW_FACING, ADD = 2, 9, 0

    def __init__(self, **fields):
        self.ns, self.id, self.type, self.action, self.text = '', 0, 0, 0, ''
        self.pose = types.SimpleNamespace(position=_Vec(), orientation=_quat())
        self.scale = _Vec()
        self.color = types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        super().__init__(**fields)


class MarkerArray(_Msg):
    def __init__(self, **fields):
        self.markers = []
        super().__init__(**fields)


class _Data(_Msg):
    def __init__(self, data=None, **fields):
        self.data = data
        super().__init__(**fields)


class Float32MultiArray(_Msg):
    def __init__(self, data=None, **fields):
        self.data = [] if data is None else data
        super().__init__(**fields)


class CvBridge:
    """Image messages carry the numpy frame, so conversion is a lookup."""

    def imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
        if getattr(msg, 'frame', None) is None:
            raise ValueError('image message has no frame')
        return msg.frame

    def cv2_to_imgmsg(self, frame, encoding='passthrough'):
        return Image(frame=frame, encoding=encoding)


# ---------------------------------------------------------------------------
# rclpy fakes
# ---------------------------------------------------------------------------

class Duration:
    def __init__(self, nanoseconds):
        self.nanoseconds = int(nanoseconds)


class Time:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = int(nanoseconds)

    def __sub__(self, other):
        return Duration(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return types.SimpleNamespace(sec=self.nanoseconds // 10**9,
                                     nanosec=self.nanoseconds % 10**9)


class Clock:
    """Manual clock; starts at t = 100 s so 'never' (-1e9) is far past."""

    def __init__(self, seconds=100.0):
        self.seconds = seconds

    def now(self):
        return Time(round(self.seconds * 1e9))

    def advance(self, seconds):
        self.seconds += seconds


class Logger:
    """Records (level, message) pairs."""

    def __init__(self):
        self.records = []

    def _log(self, level):
        return lambda msg, *a, **kw: self.records.append((level, msg))

    def __getattr__(self, level):
        if level.startswith('_'):
            raise AttributeError(level)
        return self._log(level)

    def messages(self, level=None):
        return [m for lv, m in self.records if level in (None, lv)]


class Publisher:
    """Keeps every message; forwards to wired subscribers."""

    def __init__(self, topic):
        self.topic = topic
        self.msgs = []
        self.callbacks = []
        self.subscribers = 0  # what get_subscription_count() reports

    def publish(self, msg):
        self.msgs.append(msg)
        for cb in list(self.callbacks):
            cb(msg)

    def get_subscription_count(self):
        return self.subscribers + len(self.callbacks)

    @property
    def last(self):
        return self.msgs[-1] if self.msgs else None


class Timer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback

    def __call__(self):
        self.callback()


class Node:
    """Stand-in for rclpy.node.Node that records what the node creates.

    ``Node.overrides`` plays the role of a --ros-args params file: values
    there replace the declared defaults (use the ``ros_params`` fixture).
    """

    overrides = {}

    def __init__(self, node_name='node', *args, **kwargs):
        self.node_name = node_name
        self.parameters = {}
        self.publishers = {}
        self.subscriptions = {}
        self.services = {}
        self.timers = []
        self.clock = Clock()
        self.logger = Logger()
        self.destroyed = False

    def declare_parameter(self, name, value=None, *args, **kwargs):
        self.parameters[name] = Node.overrides.get(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, msg_type, topic, qos):
        pub = Publisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return callback

    def create_service(self, srv_type, name, callback):
        self.services[name] = callback
        return callback

    def create_timer(self, period, callback):
        timer = Timer(period, callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        self.destroyed = True


def wire(*nodes):
    """Connect every publisher to same-topic subscriptions of the others."""
    for pub_node in nodes:
        for topic, pub in pub_node.publishers.items():
            for sub_node in nodes:
                cb = sub_node.subscriptions.get(topic)
                if sub_node is not pub_node and cb is not None:
                    pub.callbacks.append(cb)


class BasicNavigator:
    """Fake nav2_simple_commander navigator: goals finish when told to."""

    def __init__(self, *args, **kwargs):
        self.goals = []
        self.cancelled = 0
        self.done = True
        self.active = False

    def waitUntilNav2Active(self, *args, **kwargs):
        self.active = True

    def isTaskComplete(self):
        return self.done

    def cancelTask(self):
        self.cancelled += 1
        self.done = True

    def goToPose(self, goal):
        self.goals.append(goal)
        self.done = False


def _module(name, **attrs):
    """Install a fake module, replacing the real one if a plugin loaded it.

    With ROS sourced, pytest plugins (launch_testing_ros) may import the
    real rclpy before this file runs; the node tests need the fake anyway.
    """
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    sys.modules[name] = mod
    return mod


def _plain(*names):
    return {n: type(n, (_Msg,), {}) for n in names}


_module('rclpy', ok=lambda: True, init=lambda **kw: None,
        spin=lambda node: None, shutdown=lambda: None)
_module('rclpy.node', Node=Node)
_module('rclpy.qos', qos_profile_sensor_data=None, QoSProfile=_Msg,
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2))
_module('cv_bridge', CvBridge=CvBridge)
_module('builtin_interfaces')
_module('builtin_interfaces.msg', Time=_Msg)
_module('geometry_msgs')
_module('geometry_msgs.msg', Twist=Twist, PoseStamped=PoseStamped,
        TwistStamped=TwistStamped, PointStamped=PointStamped, **_plain('Pose'))
_module('sensor_msgs')
_module('sensor_msgs.msg', Image=Image, LaserScan=LaserScan)
_module('std_msgs')
_module('std_msgs.msg', Header=Header, Bool=_Data, Float32=_Data, String=_Data,
        Float32MultiArray=Float32MultiArray)
_module('std_srvs')
_module('std_srvs.srv', **_plain('Trigger'))
_module('nav_msgs')
_module('nav_msgs.msg', Odometry=Odometry)
_module('diagnostic_msgs')
_module('diagnostic_msgs.msg', DiagnosticArray=DiagnosticArray,
        DiagnosticStatus=DiagnosticStatus, **_plain('KeyValue'))
_module('visualization_msgs')
_module('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
_module('nav2_simple_commander')
_module('nav2_simple_commander.robot_navigator', BasicNavigator=BasicNavigator)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)


@pytest.fixture
def ros_params():
    """Parameter overrides for nodes created in the test (like a params file)."""
    Node.overrides = {}
    yield Node.overrides
    Node.overrides = {}
