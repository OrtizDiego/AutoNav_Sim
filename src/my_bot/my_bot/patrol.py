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

"""Simple two-waypoint patrol using Nav2 BasicNavigator."""

import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def main():
    """Navigate between two waypoints in a loop until shutdown."""
    rclpy.init()
    navigator = BasicNavigator()

    navigator.get_logger().info('Waiting for Nav2 to activate...')
    navigator.waitUntilNav2Active()

    goal_a = PoseStamped()
    goal_a.header.frame_id = 'map'
    goal_a.pose.position.x = 1.5
    goal_a.pose.position.y = 0.0
    goal_a.pose.orientation.w = 1.0

    goal_b = PoseStamped()
    goal_b.header.frame_id = 'map'
    goal_b.pose.position.x = 0.0
    goal_b.pose.position.y = 0.0
    goal_b.pose.orientation.w = 1.0

    while rclpy.ok():
        for label, goal in [('A', goal_a), ('B', goal_b)]:
            goal.header.stamp = navigator.get_clock().now().to_msg()
            navigator.get_logger().info(f'Going to Point {label}...')
            navigator.goToPose(goal)

            while not navigator.isTaskComplete():
                if not rclpy.ok():
                    break
                time.sleep(0.1)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info(
                    f'Reached Point {label}! Waiting 3 seconds...')
                time.sleep(3)
            else:
                navigator.get_logger().warn(f'Failed to reach Point {label}!')
                break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
