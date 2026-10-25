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

"""Map a world autonomously: frontier exploration with Nav2, then save the map.

`make nav-sim WORLD=<name>` starts it (with SLAM Toolbox and Nav2 without a
map server or AMCL) when maps/<map>.yaml does not exist yet. Every
``update_period`` it looks at SLAM's /map, finds the frontiers (frontier.py),
and sends Nav2 to the best one. A goal ends when Nav2 reports it, when its
frontier has been seen (no unknown cell within ``seen_radius``), or after
``goal_timeout``; failed and timed-out goals are blacklisted. When no
frontier is left ``done_checks`` times in a row, the map is saved to
``map_save_path`` (.yaml + .pgm, map_io.py) and the robot drives back to
the start (``return_home``). The next nav-sim of that world uses the map.

Subscribes
----------
/map                    nav_msgs/OccupancyGrid  SLAM Toolbox's map (latched)
TF map -> base_link     the robot pose

Publishes
---------
/exploration/status     std_msgs/String (JSON)  state, frontiers, goals,
                                                failures, explored m², map
"""

import json
import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from my_bot.frontier import (
    Grid, choose_frontier, find_frontiers, known_area, unknown_near)
from my_bot.map_io import save_map

EXPLORING, RETURNING, DONE = 'exploring', 'returning', 'done'


def grid_from_msg(msg: OccupancyGrid) -> Grid:
    """Convert an OccupancyGrid message into a Grid (row 0 = minimum y)."""
    info = msg.info
    data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
    return Grid(data=data, resolution=float(info.resolution),
                origin=(float(info.origin.position.x), float(info.origin.position.y)))


class ExplorerNode(Node):
    """Drive Nav2 to frontiers until the map is complete, then save it."""

    def __init__(self, navigator: Optional[BasicNavigator] = None):
        super().__init__('explorer')
        self.declare_parameter('update_period', 1.0)
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('min_clearance', 0.3)
        self.declare_parameter('size_weight', 0.5)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('seen_radius', 0.6)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('done_checks', 3)
        self.declare_parameter('map_save_path', '')
        self.declare_parameter('return_home', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        def gp(name):
            return self.get_parameter(name).value

        self._min_size = int(gp('min_frontier_size'))
        self._min_clearance = float(gp('min_clearance'))
        self._size_weight = float(gp('size_weight'))
        self._blacklist_radius = float(gp('blacklist_radius'))
        self._seen_radius = float(gp('seen_radius'))
        self._goal_timeout = float(gp('goal_timeout'))
        self._done_checks = int(gp('done_checks'))
        self._start_delay = float(gp('start_delay'))
        self._save_path = str(gp('map_save_path'))
        self._return_home = bool(gp('return_home'))
        self._map_frame = str(gp('map_frame'))
        self._robot_frame = str(gp('robot_frame'))

        self.state = EXPLORING
        self.grid: Optional[Grid] = None
        self.goal: Optional[Tuple[float, float]] = None
        self._goal_since = 0.0
        self.blacklist: List[Tuple[float, float]] = []
        self.goals_sent = 0
        self.failures = 0
        self.frontiers = 0
        self._empty_checks = 0
        self.saved: List[str] = []

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched)
        self._status_pub = self.create_publisher(String, '/exploration/status', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._started = self._now()

        if navigator is None:
            navigator = BasicNavigator()
            self.get_logger().info('waiting for Nav2...')
            # SLAM Toolbox is not a lifecycle node and there is no AMCL to
            # wait for: wait on the controller server, which Nav2's
            # navigation launch activates together with the BT navigator.
            navigator.waitUntilNav2Active(localizer='controller_server')
        self._nav = navigator
        self.create_timer(float(gp('update_period')), self._tick)
        self.get_logger().info(
            f'exploring; the map goes to {self._save_path or "(not saved)"}')

    # ------------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.grid = grid_from_msg(msg)

    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._robot_frame, Time())
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _send(self, x: float, y: float, yaw: float = 0.0) -> None:
        goal = PoseStamped()
        goal.header.frame_id = self._map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self._nav.goToPose(goal)

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self.state == DONE:
            return
        if self.state == RETURNING:
            if self._nav.isTaskComplete():
                self.state = DONE
                self.get_logger().info('back at the start; exploration done')
                self._publish_status()
            return
        robot = self._robot_xy()
        if self.grid is None or robot is None or self._now() - self._started < self._start_delay:
            return  # SLAM or TF not up yet
        if self.goal is not None and not self._goal_finished():
            self._publish_status()
            return
        frontiers = find_frontiers(self.grid, self._min_size, self._min_clearance)
        self.frontiers = len(frontiers)
        target = choose_frontier(frontiers, robot, self.blacklist,
                                 self._blacklist_radius, self._size_weight,
                                 self.grid.resolution)
        if target is None:
            self._empty_checks += 1
            if self._empty_checks >= self._done_checks:
                self._finish()
            self._publish_status()
            return
        self._empty_checks = 0
        self.goal = target.goal
        self._goal_since = self._now()
        self.goals_sent += 1
        heading = math.atan2(target.goal[1] - robot[1], target.goal[0] - robot[0])
        self._send(*target.goal, yaw=heading)
        self.get_logger().info(
            f'frontier {self.goals_sent}: ({target.goal[0]:.1f}, {target.goal[1]:.1f}), '
            f'{target.size} cells, {len(frontiers)} left')
        self._publish_status()

    def _goal_finished(self) -> bool:
        """Tell whether the current goal is over; cancel or blacklist as needed."""
        if self._nav.isTaskComplete():
            if self._nav.getResult() != TaskResult.SUCCEEDED:
                self._fail('Nav2 could not reach it')
            self.goal = None
            return True
        if self._now() - self._goal_since > self._goal_timeout:
            self._nav.cancelTask()
            self._fail(f'not reached in {self._goal_timeout:.0f} s')
            self.goal = None
            return True
        if not unknown_near(self.grid, self.goal, self._seen_radius):
            self._nav.cancelTask()  # its frontier is mapped: move on
            self.goal = None
            return True
        return False

    def _fail(self, why: str) -> None:
        self.failures += 1
        self.blacklist.append(self.goal)
        self.get_logger().warn(
            f'frontier ({self.goal[0]:.1f}, {self.goal[1]:.1f}) skipped: {why}')

    def _finish(self) -> None:
        """Save the map, then drive home (or stop)."""
        area = known_area(self.grid)
        if self._save_path:
            self.saved = list(save_map(self._save_path, self.grid))
            self.get_logger().info(
                f'map complete ({area:.0f} m² free): saved {self.saved[0]}')
        else:
            self.get_logger().info(f'map complete ({area:.0f} m² free); not saved')
        if self._return_home:
            self.state = RETURNING
            self._send(0.0, 0.0)  # the spawn pose: SLAM's map origin
        else:
            self.state = DONE

    def _publish_status(self) -> None:
        status = {'state': self.state, 'frontiers': self.frontiers,
                  'goals': self.goals_sent, 'failures': self.failures,
                  'explored_m2': round(known_area(self.grid), 1) if self.grid else 0.0,
                  'map': self.saved[0] if self.saved else ''}
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    """Initialize and spin the ExplorerNode."""
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Ctrl+C: rclpy has already shut the context down
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
