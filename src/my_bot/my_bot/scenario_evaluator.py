#!/usr/bin/env python3

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

"""Score a guard run against the intruder's ground truth, then exit.

`make scenario NAME=<name>` starts it next to person-sim with a scripted
person. It samples the robot, the person and the intruder track on the
node clock (sim time), and once person_controller reports the scenario done
it records ``linger_secs`` more (search, report) and finishes: writes
report.json, publishes it on /scenario/report, logs the summary with the
missed targets, and exits so the launch file shuts the run down.
``timeout_secs`` ends a run whose scenario never finishes (the patrol
never came close, the person was never seen). Ctrl+C writes what it has.

Metrics: scenario_metrics.py.

Subscribes
----------
/odom                    nav_msgs/Odometry  robot pose (odom = world at spawn)
/person/odom             nav_msgs/Odometry  person ground truth (actor plugin)
/intruder/track          nav_msgs/Odometry  target_tracker's confirmed track
/intruder/state          std_msgs/String    none | tentative | confirmed | lost
/security_guard/state    std_msgs/String    the guard's active protocol
/security_events         std_msgs/String    JSON alerts, counted by kind
/scenario/status         std_msgs/String    person_controller's progress

Publishes
---------
/scenario/report         std_msgs/String    the report as JSON (once, at the end)
"""

import json
import math
import os
import time
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from my_bot.clearance_map import ClearanceMap
from my_bot.scenario_metrics import (
    ScenarioMetrics, TARGETS, check_targets, format_report)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScenarioEvaluatorNode(Node):
    """Sample a run, then write and publish its metrics."""

    def __init__(self):
        super().__init__('scenario_evaluator')
        self.declare_parameter('scenario', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('report_path', '')
        self.declare_parameter('report_dir', '~/.ros/autonav_scenarios')
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('camera_hfov', 1.089)
        self.declare_parameter('view_range', 8.0)
        self.declare_parameter('match_radius', 1.5)
        self.declare_parameter('coverage_cell', 0.5)
        self.declare_parameter('track_timeout', 2.0)
        self.declare_parameter('linger_secs', 30.0)
        self.declare_parameter('timeout_secs', 600.0)
        # Targets (< 0 = not checked); the first three are the plan's.
        self.declare_parameter('max_time_to_detect_s', 5.0)
        self.declare_parameter('min_tracked_in_view_pct', 80.0)
        self.declare_parameter('max_track_breaks_per_min', 1.0)
        self.declare_parameter('max_false_alarms', 0.0)
        self.declare_parameter('min_patrol_coverage_pct', -1.0)

        def gp(name):
            return self.get_parameter(name).value

        self.scenario = str(gp('scenario'))
        map_yaml = str(gp('map_yaml'))
        if not map_yaml:
            raise ValueError('scenario_evaluator needs map_yaml (line of sight, coverage)')
        self.metrics = ScenarioMetrics(
            ClearanceMap.from_yaml(map_yaml), hfov=float(gp('camera_hfov')),
            view_range=float(gp('view_range')),
            match_radius=float(gp('match_radius')),
            coverage_cell=float(gp('coverage_cell')))
        self.limits = {name: float(gp(name)) for _, _, name in TARGETS}
        self._track_timeout = float(gp('track_timeout'))
        self._linger = float(gp('linger_secs'))
        self._timeout = float(gp('timeout_secs'))
        self.report_path = os.path.expanduser(str(gp('report_path'))) or os.path.join(
            os.path.expanduser(str(gp('report_dir'))),
            f"{self.scenario or 'run'}_{time.strftime('%Y%m%d-%H%M%S')}.json")

        self._robot = None
        self._person = None
        self._track = None  # (x, y, vx, vy, receipt time)
        self._track_state = 'none'
        self._guard_state = ''
        self._scenario_status = {}
        self._finish_at: Optional[float] = None
        self.finished = False
        self.report: Optional[dict] = None

        self._report_pub = self.create_publisher(String, '/scenario/report', 10)
        self.create_subscription(Odometry, '/odom', self._robot_cb, 10)
        self.create_subscription(Odometry, '/person/odom', self._person_cb, 10)
        self.create_subscription(Odometry, '/intruder/track', self._track_cb, 10)
        self.create_subscription(String, '/intruder/state', self._track_state_cb, 10)
        self.create_subscription(String, '/security_guard/state', self._guard_cb, 10)
        self.create_subscription(String, '/security_events', self._event_cb, 10)
        self.create_subscription(String, '/scenario/status', self._status_cb, 10)
        self.create_timer(1.0 / max(float(gp('sample_rate_hz')), 1.0), self._tick)
        self.get_logger().info(
            f"evaluating {self.scenario or 'an unscripted run'}; "
            f'report -> {self.report_path}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------

    def _robot_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._robot = (p.position.x, p.position.y, _yaw(p.orientation))

    def _person_cb(self, msg: Odometry) -> None:
        self._person = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _track_cb(self, msg: Odometry) -> None:
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self._track = (p.x, p.y, v.x, v.y, self._now())

    def _track_now(self, now: float):
        """Extrapolate the track to ``now``; None once it is too old.

        target_tracker publishes only on accepted measurements, so a
        confirmed track coasts silently between them.
        """
        if self._track is None:
            return None
        x, y, vx, vy, t = self._track
        age = now - t
        if age > self._track_timeout:
            return None
        return (x + vx * age, y + vy * age)

    def _track_state_cb(self, msg: String) -> None:
        self._track_state = msg.data

    def _guard_cb(self, msg: String) -> None:
        self._guard_state = msg.data

    def _event_cb(self, msg: String) -> None:
        try:
            kind = json.loads(msg.data).get('event', 'unknown')
        except (ValueError, AttributeError):
            kind = 'unparsed'
        self.metrics.event(kind)

    def _status_cb(self, msg: String) -> None:
        try:
            self._scenario_status = json.loads(msg.data)
        except ValueError:
            return
        if self._scenario_status.get('phase') == 'done' and self._finish_at is None:
            self._finish_at = self._now() + self._linger
            self.get_logger().info(
                f'scenario done; recording {self._linger:.0f} s more')

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self.finished or self._robot is None:
            return  # no odometry yet: the simulation is still starting
        now = self._now()
        self.metrics.add(now, self._robot, self._person, self._track_state,
                         self._track_now(now), self._guard_state)
        if self._finish_at is not None and now >= self._finish_at:
            self.finish('completed')
        elif now - self.metrics.start >= self._timeout:
            self.finish('timeout')

    def finish(self, outcome: str) -> dict:
        """Write, publish and log the report; the node is done after this."""
        report = self.metrics.report()
        failures = check_targets(report, self.limits)
        if outcome != 'completed':
            failures.append(f'run ended by {outcome} '
                            f'(scenario status {self._scenario_status or "none"})')
        report = {'scenario': self.scenario, 'outcome': outcome,
                  'passed': not failures, 'failures': failures,
                  'limits': self.limits, **report}
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.report_path, 'w') as f:
            json.dump(report, f, indent=2)
        try:
            self._report_pub.publish(String(data=json.dumps(report)))
        except Exception:  # noqa: BLE001 - Ctrl+C: the context may be gone
            pass
        self.get_logger().info(
            format_report(self.scenario or 'run', report, failures)
            + f'\nreport written to {self.report_path}')
        self.report = report
        self.finished = True
        return report


def main(args=None):
    """Spin until the run is scored; Ctrl+C scores what was recorded."""
    rclpy.init(args=args)
    node = ScenarioEvaluatorNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if not node.finished and node.metrics.start is not None:
            node.finish('interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
