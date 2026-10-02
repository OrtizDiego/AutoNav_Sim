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

"""Keyboard control for the ball in ball-sim (``make teleop-ball``).

Unlike teleop_twist_keyboard there is no heading to steer: the ball moves
straight in the direction of the key, and only while the key is held
(it stops ~0.4 s after release). While this node runs the ball is all
yours; about a second after you quit, ball_controller hands it back to
the autopilot.

Two frames, toggled with ``m``:

  robot (default)  w = away from the robot, s = toward it,
                   a / d = the robot's left / right
                   -- matches what the robot's camera sees
  world            w / s = world +x / -x, a / d = world +y / -y

Publishes /ball/teleop (geometry_msgs/TwistStamped, frame_id robot|world).
"""

import math
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

HELP = """
Ball teleop  ({frame} frame, {speed:.2f} m/s)
---------------------------
   q   w   e        hold a key to move, release to stop
   a   s   d        x / space : stop now
   z       c        m : toggle robot/world frame
                    + / - : faster / slower
                    Ctrl-C : quit (autopilot resumes)
"""

# key -> (forward, left) unit direction
KEYS = {
    'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1),
    'q': (1, 1), 'e': (1, -1), 'z': (-1, 1), 'c': (-1, -1),
}

HOLD_SECS = 0.4  # covers the terminal's key-repeat delay


def direction_for(key: str):
    """Normalised (forward, left) for a key, or None."""
    d = KEYS.get(key.lower())
    if d is None:
        return None
    n = math.hypot(*d)
    return d[0] / n, d[1] / n


class BallTeleop(Node):
    """Reads the keyboard and publishes /ball/teleop at 20 Hz."""

    def __init__(self):
        super().__init__('ball_teleop')
        self._pub = self.create_publisher(TwistStamped, '/ball/teleop', 10)
        self.frame = 'robot'
        self.speed = 0.6
        self._dir = (0.0, 0.0)
        self._until = 0.0

    def press(self, key: str) -> bool:
        """Handle one key; return True if the help text should be reprinted."""
        d = direction_for(key)
        if d is not None:
            self._dir, self._until = d, time.monotonic() + HOLD_SECS
            return False
        if key in ('x', ' '):
            self._until = 0.0
        elif key == 'm':
            self.frame = 'world' if self.frame == 'robot' else 'robot'
            return True
        elif key in ('+', '='):
            self.speed = min(1.5, self.speed + 0.1)
            return True
        elif key in ('-', '_'):
            self.speed = max(0.1, self.speed - 0.1)
            return True
        return False

    def publish(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        if time.monotonic() < self._until:
            msg.twist.linear.x = self._dir[0] * self.speed
            msg.twist.linear.y = self._dir[1] * self.speed
        self._pub.publish(msg)


def _read_key(timeout: float) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main(args=None):
    """Run the keyboard loop until Ctrl-C."""
    rclpy.init(args=args)
    node = BallTeleop()
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(HELP.format(frame=node.frame, speed=node.speed))
        while rclpy.ok():
            key = _read_key(0.05)
            if key == '\x03':
                break
            if key and node.press(key):
                print(HELP.format(frame=node.frame, speed=node.speed))
            node.publish()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
