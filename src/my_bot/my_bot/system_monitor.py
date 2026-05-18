#!/usr/bin/env python3

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

"""System watchdog — monitors sensor heartbeats and publishes health diagnostics."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_srvs.srv import Trigger


class SystemMonitorNode(Node):
    """Watchdog that monitors /scan and /camera/image_raw heartbeats.

    Publishes health diagnostics to /system_health every second.
    Provides a /trigger_estop service that deactivates the security_guard
    lifecycle node and zeros /cmd_vel.

    Parameters
    ----------
    scan_timeout      : float — seconds before /scan absence triggers warning
    camera_timeout    : float — seconds before camera absence triggers warning
    health_publish_rate: float — Hz at which /system_health is published
    """

    def __init__(self):
        """Initialize subscriptions, publishers, and watchdog timer."""
        super().__init__('system_monitor')

        self.declare_parameter('scan_timeout', 2.0)
        self.declare_parameter('camera_timeout', 2.0)
        self.declare_parameter('health_publish_rate', 1.0)

        self._scan_timeout = float(self.get_parameter('scan_timeout').value)
        self._cam_timeout = float(self.get_parameter('camera_timeout').value)
        rate_hz = float(self.get_parameter('health_publish_rate').value)

        now = self.get_clock().now()
        self._last_scan_time = now
        self._last_cam_time = now

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self._cam_sub = self.create_subscription(
            Image, '/camera/image_raw', self._cam_cb, qos_profile_sensor_data)

        self._health_pub = self.create_publisher(
            DiagnosticArray, '/system_health', 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Lifecycle change-state client for security_guard
        self._lc_client = self.create_client(
            ChangeState, '/security_guard/change_state')

        # E-stop service
        self._estop_srv = self.create_service(
            Trigger, '/trigger_estop', self._estop_cb)

        self._watchdog_timer = self.create_timer(
            1.0 / rate_hz, self._watchdog_cb)

    # ------------------------------------------------------------------

    def _scan_cb(self, _msg: LaserScan):
        self._last_scan_time = self.get_clock().now()

    def _cam_cb(self, _msg: Image):
        self._last_cam_time = self.get_clock().now()

    def _watchdog_cb(self):
        """Publish diagnostics; warn if sensors time out."""
        now = self.get_clock().now()
        scan_age = self._age_secs(now, self._last_scan_time)
        cam_age = self._age_secs(now, self._last_cam_time)
        scan_ok = scan_age < self._scan_timeout
        cam_ok = cam_age < self._cam_timeout

        if not scan_ok:
            self.get_logger().warn(
                f'LiDAR silent for {scan_age:.1f}s (timeout={self._scan_timeout}s)')
        if not cam_ok:
            self.get_logger().warn(
                f'Camera silent for {cam_age:.1f}s (timeout={self._cam_timeout}s)')

        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()
        status = DiagnosticStatus()
        status.name = 'system_monitor'
        status.hardware_id = 'autonav_sim'
        status.level = (DiagnosticStatus.OK
                        if (scan_ok and cam_ok)
                        else DiagnosticStatus.WARN)
        status.message = 'All sensors OK' if (scan_ok and cam_ok) else 'Sensor timeout'
        status.values = [
            KeyValue(key='scan_ok', value=str(scan_ok)),
            KeyValue(key='camera_ok', value=str(cam_ok)),
            KeyValue(key='scan_age_sec', value=f'{scan_age:.2f}'),
            KeyValue(key='camera_age_sec', value=f'{cam_age:.2f}'),
        ]
        diag.status = [status]
        self._health_pub.publish(diag)

    def _estop_cb(self, _request, response):
        """Zero velocity and request security_guard deactivation."""
        self.get_logger().warn('E-STOP triggered!')
        # Zero velocity immediately
        self._cmd_pub.publish(Twist())
        # Request lifecycle deactivation (best-effort — node may not exist)
        if self._lc_client.service_is_ready():
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            self._lc_client.call_async(req)
        response.success = True
        response.message = 'E-stop executed'
        return response

    @staticmethod
    def _age_secs(now, then) -> float:
        """Return elapsed time in seconds between two rclpy Time objects."""
        return (now - then).nanoseconds / 1e9


def main(args=None):
    """Initialize and spin the SystemMonitorNode."""
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
