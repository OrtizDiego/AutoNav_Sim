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

"""Measure where a running scenario spends its time (`make perf`).

Every ``report_period`` seconds this node logs, and publishes on
/perf_monitor (diagnostic_msgs/DiagnosticArray):

- real-time factor: simulation seconds per wall-clock second. Below ~0.8
  the simulator (rendering, physics) is the bottleneck.
- per topic: rate in simulation Hz, and for stamped topics the age of each
  message on arrival (now - header.stamp, simulation ms). Sensor rates
  below their configured value (camera 30 Hz, lidar 10 Hz) mean Gazebo
  cannot render them in time; a large /person_bbox or /target age means
  the detection pipeline is slow; /target age is the dead time the
  follow controller has to compensate.

Watched: /camera/image_raw (``watch_camera``; subscribing to raw images
costs transport itself), /scan, /odom, /person_bbox, /target, /cmd_vel.
Run with use_sim_time:=true so ages and rates are in simulation time.
"""

import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PolygonStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from my_bot.target_estimate import stamp_to_sec


class TopicStats:
    """Message count and arrival age over one report window."""

    def __init__(self):
        self.count = 0
        self.ages = []

    def add(self, now: float, stamp: Optional[float]) -> None:
        """Record a message received at ``now`` (sim s), stamped ``stamp``."""
        self.count += 1
        if stamp is not None:
            self.ages.append(now - stamp)

    def summary(self, sim_elapsed: float) -> dict:
        """Rate (sim Hz) and age (sim ms) since the last summary, then reset."""
        out = {'hz': self.count / sim_elapsed if sim_elapsed > 0.0 else 0.0}
        if self.ages:
            out['age_mean_ms'] = 1000.0 * sum(self.ages) / len(self.ages)
            out['age_max_ms'] = 1000.0 * max(self.ages)
        self.count, self.ages = 0, []
        return out


def real_time_factor(sim_elapsed: float, wall_elapsed: float) -> float:
    """Return simulation seconds per wall-clock second (0 if no wall time)."""
    return sim_elapsed / wall_elapsed if wall_elapsed > 0.0 else 0.0


def format_report(rtf: float, rows: dict) -> str:
    """Format the real-time factor, then one line per topic."""
    lines = [f'real-time factor {rtf:.2f}',
             f'  {"topic":<20} {"Hz (sim)":>9} {"age ms mean/max":>16}']
    for topic, s in rows.items():
        age = (f'{s["age_mean_ms"]:.0f}/{s["age_max_ms"]:.0f}'
               if 'age_mean_ms' in s else '-')
        lines.append(f'  {topic:<20} {s["hz"]:>9.1f} {age:>16}')
    return '\n'.join(lines)


class PerfMonitorNode(Node):
    """Reports real-time factor, topic rates and message ages."""

    def __init__(self):
        super().__init__('perf_monitor')
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('watch_camera', True)
        self._period = float(self.get_parameter('report_period').value)

        topics = [(LaserScan, '/scan', qos_profile_sensor_data),
                  (Odometry, '/odom', 10),
                  (PolygonStamped, '/person_bbox', 10),
                  (Vector3Stamped, '/target', 10),
                  (Twist, '/cmd_vel', 10)]
        if bool(self.get_parameter('watch_camera').value):
            topics.insert(0, (Image, '/camera/image_raw', qos_profile_sensor_data))
        self._stats = {}
        for msg_type, topic, qos in topics:
            self._stats[topic] = TopicStats()
            self.create_subscription(
                msg_type, topic, self._recorder(topic), qos)

        self._pub = self.create_publisher(DiagnosticArray, '/perf_monitor', 10)
        self._window = (self._sim_now(), time.monotonic())
        self.create_timer(self._period, self._report)

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _recorder(self, topic: str):
        stats = self._stats[topic]

        def record(msg):
            header = getattr(msg, 'header', None)
            stamp = stamp_to_sec(header.stamp) if header is not None else None
            stats.add(self._sim_now(), stamp)
        return record

    def _report(self) -> None:
        sim_now, wall_now = self._sim_now(), time.monotonic()
        sim_elapsed = sim_now - self._window[0]
        rtf = real_time_factor(sim_elapsed, wall_now - self._window[1])
        self._window = (sim_now, wall_now)
        rows = {t: s.summary(sim_elapsed) for t, s in self._stats.items()}
        self.get_logger().info(format_report(rtf, rows))

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        rtf_status = DiagnosticStatus(
            name='perf_monitor: real_time_factor', hardware_id='autonav_sim',
            message=f'{rtf:.2f}')
        rtf_status.level = DiagnosticStatus.OK if rtf >= 0.8 else DiagnosticStatus.WARN
        diag.status = [rtf_status]
        for topic, s in rows.items():
            status = DiagnosticStatus(
                name=f'perf_monitor: {topic}', hardware_id='autonav_sim',
                message=f'{s["hz"]:.1f} Hz')
            status.values = [KeyValue(key=k, value=f'{v:.1f}') for k, v in s.items()]
            diag.status.append(status)
        self._pub.publish(diag)


def main(args=None):
    """Initialize and spin the PerfMonitorNode."""
    rclpy.init(args=args)
    node = PerfMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
