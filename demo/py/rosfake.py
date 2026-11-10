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

"""A ROS 2 stand-in small enough for a browser (Pyodide) — no ROS install.

The web demo runs the package's real node classes (sensor_fusion,
target_tracker, security_guard_bt, person_controller). They only need the
slice of rclpy they use: parameters, publishers, subscriptions, timers, a
clock and a logger. ``install()`` puts fakes of rclpy and the message
packages into ``sys.modules`` (the same idea as test/conftest.py), with
three differences from the test fakes:

* one process-wide topic graph: a publish calls every subscription on that
  topic, in any node, synchronously;
* one shared simulation clock, advanced by ``spin_until(t)``, which also
  fires the timers that came due, in time order;
* parameters come from ``PARAMS[node_name]`` (behavior_params.yaml sections).

numpy, OpenCV, PyYAML and py_trees are the real packages.
"""

import math
import sys
import types

PARAMS = {}      # node name -> {parameter: value}
NODES = []       # every Node created, in order
LOG = []         # (sim time, level, node name, message), drained by the demo
_SUBS = {}       # topic -> [callback]
_TIMERS = []     # every Timer
_CLOCK = None


# ---------------------------------------------------------------------------
# Messages
# ---------------------------------------------------------------------------

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


class Stamp(_Msg):
    """builtin_interfaces/Time."""

    def __init__(self, sec=0, nanosec=0, **fields):
        self.sec, self.nanosec = sec, nanosec
        super().__init__(**fields)


def stamp(t: float) -> Stamp:
    ns = int(round(t * 1e9))
    return Stamp(sec=ns // 10**9, nanosec=ns % 10**9)


class Twist(_Msg):
    def __init__(self, **fields):
        self.linear, self.angular = _Vec(), _Vec()
        super().__init__(**fields)


class Header(_Msg):
    def __init__(self, **fields):
        self.stamp, self.frame_id = Stamp(), ''
        super().__init__(**fields)


class _Stamped(_Msg):
    def __init__(self, **fields):
        self.header = Header()
        super().__init__(**fields)


class PointStamped(_Stamped):
    def __init__(self, **fields):
        self.point = _Vec()
        super().__init__(**fields)


class Vector3Stamped(_Stamped):
    def __init__(self, **fields):
        self.vector = _Vec()
        super().__init__(**fields)


class PolygonStamped(_Stamped):
    def __init__(self, **fields):
        self.polygon = types.SimpleNamespace(points=[])
        super().__init__(**fields)


class PoseStamped(_Stamped):
    def __init__(self, **fields):
        self.pose = types.SimpleNamespace(position=_Vec(), orientation=_quat())
        super().__init__(**fields)


class Odometry(_Stamped):
    def __init__(self, **fields):
        self.child_frame_id = ''
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=_Vec(), orientation=_quat()),
            covariance=[0.0] * 36)
        self.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=_Vec(), angular=_Vec()),
            covariance=[0.0] * 36)
        super().__init__(**fields)


class Image(_Stamped):
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
    ARROW, SPHERE, CYLINDER, LINE_STRIP, TEXT_VIEW_FACING = 0, 2, 3, 4, 9
    ADD, DELETEALL = 0, 3

    def __init__(self, **fields):
        self.ns, self.id, self.type, self.action, self.text = '', 0, 0, 0, ''
        self.points = []
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


class CvBridge:
    """Image messages carry the numpy frame itself."""

    def imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
        if getattr(msg, 'frame', None) is None:
            raise ValueError('image message has no frame')
        return msg.frame

    def cv2_to_imgmsg(self, frame, encoding='passthrough'):
        return Image(frame=frame, encoding=encoding)


# ---------------------------------------------------------------------------
# rclpy
# ---------------------------------------------------------------------------

class Duration:
    def __init__(self, nanoseconds):
        self.nanoseconds = int(nanoseconds)


class Time:
    def __init__(self, nanoseconds=0, **kwargs):
        self.nanoseconds = int(nanoseconds)

    def __sub__(self, other):
        return Duration(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return Stamp(sec=self.nanoseconds // 10**9,
                     nanosec=self.nanoseconds % 10**9)


class Clock:
    """The simulation clock every node shares."""

    def __init__(self, seconds=0.0):
        self.seconds = seconds

    def now(self):
        return Time(round(self.seconds * 1e9))


class Logger:
    def __init__(self, name):
        self._name = name

    def __getattr__(self, level):
        if level.startswith('_'):
            raise AttributeError(level)

        def log(msg, *args, **kwargs):
            LOG.append((_CLOCK.seconds, level, self._name, str(msg)))
        return log


class Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        for callback in list(_SUBS.get(self.topic, ())):
            callback(msg)

    def get_subscription_count(self):
        return len(_SUBS.get(self.topic, ()))


class Timer:
    def __init__(self, period, callback):
        self.period = float(period)
        self.callback = callback
        self.next = _CLOCK.seconds + self.period

    def cancel(self):
        if self in _TIMERS:
            _TIMERS.remove(self)


class Node:
    """The part of rclpy.node.Node the package's nodes use."""

    def __init__(self, node_name='node', *args, **kwargs):
        self.node_name = node_name
        self.parameters = {}
        self._logger = Logger(node_name)
        NODES.append(self)

    def declare_parameter(self, name, value=None, *args, **kwargs):
        self.parameters[name] = PARAMS.get(self.node_name, {}).get(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, msg_type, topic, qos=None, **kwargs):
        return Publisher(topic)

    def create_subscription(self, msg_type, topic, callback, qos=None, **kwargs):
        _SUBS.setdefault(topic, []).append(callback)
        return callback

    def count_publishers(self, topic):
        return 1

    def create_service(self, srv_type, name, callback):
        return callback

    def create_timer(self, period, callback):
        timer = Timer(period, callback)
        _TIMERS.append(timer)
        return timer

    def get_clock(self):
        return _CLOCK

    def get_logger(self):
        return self._logger

    def get_name(self):
        return self.node_name

    def destroy_node(self):
        pass


def subscribe(topic, callback):
    """Listen to a topic from outside a node (the demo's own taps)."""
    _SUBS.setdefault(topic, []).append(callback)


def publish(topic, msg):
    Publisher(topic).publish(msg)


def now() -> float:
    return _CLOCK.seconds


def spin_until(t: float):
    """Fire every timer due up to ``t`` in time order, then set the clock."""
    while True:
        due = [timer for timer in _TIMERS if timer.next <= t]
        if not due:
            break
        timer = min(due, key=lambda tm: tm.next)
        _CLOCK.seconds = max(_CLOCK.seconds, timer.next)
        timer.next += timer.period
        if timer.next <= t - 1.0:  # a long pause: do not replay it tick by tick
            timer.next = t
        timer.callback()
    _CLOCK.seconds = max(_CLOCK.seconds, t)


# ---------------------------------------------------------------------------
# TF: the demo's odom frame is the map frame (ground-truth odometry)
# ---------------------------------------------------------------------------

class TransformException(Exception):
    pass


class TfBuffer:
    def __init__(self, *args, **kwargs):
        pass

    def lookup_transform(self, target, source, time, *args, **kwargs):
        return types.SimpleNamespace(transform=types.SimpleNamespace(
            translation=_Vec(), rotation=_quat()))


class TransformListener:
    def __init__(self, buffer, node, *args, **kwargs):
        self.buffer = buffer


class TaskResult:
    UNKNOWN, SUCCEEDED, CANCELED, FAILED = range(4)


# ---------------------------------------------------------------------------

def _module(name, **attrs):
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    sys.modules[name] = mod
    return mod


def _plain(*names):
    return {n: type(n, (_Msg,), {}) for n in names}


def install(navigator_class, start_time: float = 0.0):
    """Reset the graph and install the fake ROS modules.

    ``navigator_class`` stands in for nav2_simple_commander's BasicNavigator.
    """
    global _CLOCK
    _CLOCK = Clock(start_time)
    PARAMS.clear()
    del NODES[:]
    del LOG[:]
    _SUBS.clear()
    del _TIMERS[:]

    _module('rclpy', ok=lambda: True, init=lambda **kw: None,
            spin=lambda node: None, shutdown=lambda: None)
    _module('rclpy.node', Node=Node)
    _module('rclpy.time', Time=Time)
    _module('tf2_ros', Buffer=TfBuffer, TransformListener=TransformListener,
            TransformException=TransformException)
    _module('rclpy.qos', qos_profile_sensor_data=None, QoSProfile=_Msg,
            DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2),
            HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1, KEEP_ALL=2),
            ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2))
    _module('cv_bridge', CvBridge=CvBridge)
    _module('builtin_interfaces')
    _module('builtin_interfaces.msg', Time=Stamp)
    _module('geometry_msgs')
    _module('geometry_msgs.msg', Twist=Twist, PoseStamped=PoseStamped,
            PointStamped=PointStamped, Vector3Stamped=Vector3Stamped,
            PolygonStamped=PolygonStamped, Point=_Vec, Point32=_Vec,
            **_plain('Pose'))
    _module('sensor_msgs')
    _module('sensor_msgs.msg', Image=Image, LaserScan=LaserScan)
    _module('std_msgs')
    _module('std_msgs.msg', Header=Header, Bool=_Data, Float32=_Data, String=_Data)
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
    _module('nav2_simple_commander.robot_navigator',
            BasicNavigator=navigator_class, TaskResult=TaskResult)


def yaw_quaternion(yaw: float):
    q = _quat()
    q.z, q.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return q
