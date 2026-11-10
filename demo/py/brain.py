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

"""The robot's software for the web demo: the package's nodes on rosfake.

``Brain`` builds the person-sim graph without ROS:

    /person_bbox + /scan -> sensor_fusion -> /target
    /target + /scan + /odom -> target_tracker -> /intruder/track
    /intruder/track + /odom + /scan -> security_guard_bt -> /cmd_vel, Nav goals
    /person/odom + /odom + /person_detected -> person_controller -> /person/cmd_vel

Those four are the unmodified classes of my_bot, configured from
config/behavior_params.yaml and config/guard_zones.yaml like
person_sim.launch.py does. What the browser replaces: Gazebo (the page
simulates the robot, the person, the lidar and the camera box), YOLO (the
page sends the person's box as person_tracker would) and Nav2
(demo_nav.DemoNavigator).

``step()`` takes one batch of sensor data stamped with simulation time,
runs the timers that came due and returns the velocity commands plus
everything the page draws (the RViz part).
"""

import json
import math
import os

import numpy as np
import yaml

import demo_nav
import rosfake

SCAN_SAMPLES = 360
SCAN_ANGLE_MIN = -3.14
SCAN_ANGLE_INCREMENT = 6.28 / (SCAN_SAMPLES - 1)
SCAN_RANGE_MIN = 0.12
SCAN_RANGE_MAX = 12.0


class Brain:
    """The guard robot's nodes, stepped on simulation time."""

    def __init__(self, pkg_dir: str, start_time: float = 1.0):
        rosfake.install(demo_nav.DemoNavigator, start_time)
        map_yaml = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
        zones_file = os.path.join(pkg_dir, 'config', 'guard_zones.yaml')
        with open(os.path.join(pkg_dir, 'config', 'behavior_params.yaml')) as f:
            sections = yaml.safe_load(f)
        for name, section in sections.items():
            rosfake.PARAMS[name] = dict(section.get('ros__parameters', {}))
        # The per-scenario overrides of person_sim.launch.py
        rosfake.PARAMS['sensor_fusion']['mode'] = 'person'
        rosfake.PARAMS['person_controller']['map_yaml'] = map_yaml
        rosfake.PARAMS['security_guard_bt'].update(
            zones_file=zones_file, map_yaml=map_yaml, incident_dir='')

        # Imported after install(): they import rclpy and the message packages
        import py_trees
        from my_bot.clearance_map import ClearanceMap
        from my_bot.person_controller import PersonControllerNode
        from my_bot.security_guard_bt import SecurityGuardBTNode
        from my_bot.sensor_fusion import SensorFusionNode
        from my_bot.target_tracker import TargetTrackerNode

        self._py_trees = py_trees
        py_trees.blackboard.Blackboard.clear()
        demo_nav.DemoNavigator.cmap = ClearanceMap.from_yaml(map_yaml)

        self._cmd = None
        self._person_cmd = None
        self._person_mode = 'WALK'
        self._events = []
        self._target = None       # latest /target (bearing, range)
        self._fusion_source = ''
        self._guard_state = ''
        rosfake.subscribe('/cmd_vel', self._on_cmd)
        rosfake.subscribe('/person/cmd_vel', self._on_person_cmd)
        rosfake.subscribe('/person_mode', self._on_person_mode)
        rosfake.subscribe('/security_events', self._on_event)
        rosfake.subscribe('/target', self._on_target)
        rosfake.subscribe('/security_guard/state', self._on_guard_state)

        self.fusion = SensorFusionNode()
        self.tracker = TargetTrackerNode()
        self.person = PersonControllerNode()
        self.guard = SecurityGuardBTNode()
        self.nav = demo_nav.DemoNavigator.instance
        self._path_version = -1
        self._estop = False

    # -- taps ---------------------------------------------------------------

    def _on_cmd(self, msg):
        self._cmd = (float(msg.linear.x), float(msg.angular.z))

    def _on_person_cmd(self, msg):
        self._person_cmd = (float(msg.linear.x), float(msg.angular.z))

    def _on_person_mode(self, msg):
        self._person_mode = msg.data

    def _on_event(self, msg):
        self._events.append(json.loads(msg.data))

    def _on_target(self, msg):
        b, r = float(msg.vector.x), float(msg.vector.y)
        self._target = None if math.isnan(b) else (b, r)

    def _on_guard_state(self, msg):
        self._guard_state = msg.data

    # -- static description ---------------------------------------------------

    def static(self) -> dict:
        """What does not change during a run: zones, the tree's shape, limits."""
        zones = self.guard._zones
        nodes = []
        self._walk(self.guard._bt.root, 0, nodes)
        gp = rosfake.PARAMS['security_guard_bt']
        return {
            'zones': {k: [list(p) for p in v] for k, v in zones.zones.items()},
            'restricted': {k: [list(p) for p in v] for k, v in zones.restricted.items()},
            'tree': [{'name': n.name, 'depth': d,
                      'kind': type(n).__name__ if not n.children else
                      ('Selector' if isinstance(n, self._py_trees.composites.Selector)
                       else 'Sequence')} for n, d in nodes],
            'follow_distance': gp['desired_distance'],
            'scan': {'samples': SCAN_SAMPLES, 'angle_min': SCAN_ANGLE_MIN,
                     'angle_increment': SCAN_ANGLE_INCREMENT},
        }

    def _walk(self, node, depth, out):
        out.append((node, depth))
        for child in node.children:
            self._walk(child, depth + 1, out)

    # -- one step --------------------------------------------------------------

    def step(self, msg: dict, scan=None) -> dict:
        """Feed one batch of sensor data; return commands and display state.

        ``msg``: t (sim s), robot [x, y, yaw, v, w], person [x, y, yaw, v],
        optional bbox {t, box: [x, y, w, h] or None}, estop, scan_t.
        ``scan``: 360 ranges (m, inf = no return) when a scan is due.
        """
        t = float(msg['t'])
        rosfake.spin_until(t)

        estop = bool(msg.get('estop', False))
        if estop != self._estop:
            self._estop = estop
            rosfake.publish('/estop', rosfake._Data(data=estop))

        x, y, yaw, v, w = msg['robot']
        odom = rosfake.Odometry()
        odom.header.stamp = rosfake.stamp(t)
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = rosfake.yaw_quaternion(yaw)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        rosfake.publish('/odom', odom)

        px, py, pyaw, _pv = msg['person']
        podom = rosfake.Odometry()
        podom.header.stamp = rosfake.stamp(t)
        podom.pose.pose.position.x, podom.pose.pose.position.y = px, py
        podom.pose.pose.orientation = rosfake.yaw_quaternion(pyaw)
        rosfake.publish('/person/odom', podom)

        if scan is not None:
            ranges = scan if isinstance(scan, list) else list(scan)
            m = rosfake.LaserScan(
                ranges=ranges, angle_min=SCAN_ANGLE_MIN,
                angle_increment=SCAN_ANGLE_INCREMENT, range_min=SCAN_RANGE_MIN,
                range_max=SCAN_RANGE_MAX)
            m.header.stamp = rosfake.stamp(float(msg.get('scan_t', t)))
            m.header.frame_id = 'laser_frame'
            rosfake.publish('/scan', m)

        bbox = msg.get('bbox')
        if bbox is not None:
            self._publish_box(bbox)

        cmd, self._cmd = self._cmd, None
        return {
            't': t,
            'cmd': cmd,
            'person_cmd': self._person_cmd,
            'person_mode': self._person_mode,
            'viz': self._viz(),
        }

    def _publish_box(self, bbox):
        """What person_tracker publishes for one camera frame."""
        box = bbox.get('box')
        m = rosfake.PolygonStamped()
        m.header.stamp = rosfake.stamp(float(bbox['t']))
        m.header.frame_id = 'camera_link_optical'
        self._fusion_source = ''
        if box is not None:
            bx, by, bw, bh = box
            m.polygon.points = [rosfake._Vec(bx, by), rosfake._Vec(bx + bw, by + bh)]
            # For the camera panel's label only (the node does not publish it)
            _, self._fusion_source = self.fusion._range_for(
                (bx, by, bw, bh), float(bbox['t']))
        rosfake.publish('/person_bbox', m)
        rosfake.publish('/person_detected', rosfake._Data(data=box is not None))

    # -- display state -----------------------------------------------------------

    def _viz(self) -> dict:
        bb = self._py_trees.blackboard.Blackboard()

        def get(key, default=None):
            return bb.get(key) if bb.exists(key) else default

        nodes = []
        self._walk(self.guard._bt.root, 0, nodes)
        tracker = self.tracker._tracker
        track = None
        f = tracker.filter
        if f is not None:
            vals, vecs = np.linalg.eigh(f.P[:2, :2])
            px, py = f.position
            vx, vy = f.velocity
            track = {
                'x': float(px), 'y': float(py), 'vx': float(vx), 'vy': float(vy),
                't': float(f.t),
                # 2-sigma ellipse: semi-axes and the major axis' heading
                'a': 2.0 * math.sqrt(max(float(vals[1]), 1e-6)),
                'b': 2.0 * math.sqrt(max(float(vals[0]), 1e-6)),
                'yaw': math.atan2(float(vecs[1, 1]), float(vecs[0, 1])),
            }
        viz = {
            'state': self._guard_state,
            'bt': [n.status.value for n, _ in nodes],
            'zone': get('patrol_zone', ''),
            'waypoint': get('waypoint_index', 0),
            'intruder': tracker.state,
            'track': track,
            'target': self._target,
            'source': self._fusion_source,
            'follow': None,
            'search_goal': None,
            'sweep': [list(p) for p in (get('sweep_points') or [])],
            'nav_active': self.nav.active,
            'events': self._events,
            'log': [list(entry) for entry in rosfake.LOG],
            'metrics': dict(self.guard._metrics),
        }
        if self._guard_state == 'IntruderProtocol':
            viz['follow'] = [get('target_bearing', 0.0), get('target_range', -1.0),
                             get('target_yaw_rate', 0.0)]
        if self._guard_state == 'SearchProtocol':
            goal = get('search_goal')
            viz['search_goal'] = list(goal) if goal else None
        if self.nav.path_version != self._path_version:
            self._path_version = self.nav.path_version
            viz['path'] = [[round(px, 3), round(py, 3)] for px, py in self.nav.path]
        self._events = []
        del rosfake.LOG[:]
        return viz


# -- entry points for the page (Pyodide) ---------------------------------------

_brain = None


def start(pkg_dir: str, start_time: float = 1.0) -> str:
    global _brain
    _brain = Brain(pkg_dir, start_time)
    return json.dumps(_brain.static())


def step(msg_json: str, scan=None) -> str:
    if scan is not None and not isinstance(scan, list):
        scan = scan.to_py().tolist() if hasattr(scan, 'to_py') else list(scan)
    return json.dumps(_brain.step(json.loads(msg_json), scan))
