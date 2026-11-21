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

"""The security guard's behaviour tree: blackboard keys, leaves, factory.

Tree structure (highest priority first):

    Selector("SecurityGuard")
    ├── Sequence("EmergencyStop")
    │   ├── Condition("EStopActive")        /estop latched by system_monitor
    │   └── Action("HaltRobot")             cancel Nav2, publish zero cmd_vel
    ├── Sequence("IntruderProtocol")
    │   ├── Condition("IntruderVisible")    confirmed track seen < target_timeout ago
    │   ├── Action("CancelPatrol")          cancel the active Nav2 goal
    │   └── Action("FollowIntruder")        stand-off follow + turn-rate feedforward
    ├── Sequence("SearchProtocol")
    │   ├── Condition("IntruderRecentlyLost")  lost < search_secs ago
    │   └── Action("SearchPredictedPath")   Nav2 to where the track was heading,
    │                                       then turn toward the last-seen side
    ├── Sequence("SweepProtocol")
    │   ├── Condition("SweepPending")       viewpoints left, lost < search_secs
    │   │                                   + sweep_secs ago
    │   └── Action("SweepRooms")            Nav2 to each viewpoint, look, dwell
    ├── Sequence("ReportProtocol")
    │   ├── Condition("IncidentOpen")       an incident is still open
    │   └── Action("ReportIncident")        close it: incident_closed event
    └── Sequence("PatrolProtocol")
        ├── Action("NavigateToWaypoint")    Nav2 goToPose to the scheduled
        │                                   zone's waypoint, RUNNING until done
        ├── Action("WaitAtWaypoint")        timed dwell
        └── Action("IncrementWaypoint")     advance bb.waypoint_index

The leaves talk to the rest of the guard through three things only: the
py_trees blackboard (the ``BB_*`` keys below, written by the node from its
subscriptions), a navigator with BasicNavigator's ``goToPose`` /
``isTaskComplete`` / ``cancelTask``, and a ``publish(Twist)`` callable. So
the tree runs unchanged on Nav2, on the tests' fake navigator and on the
web demo's planner. security_guard_bt.py is the ROS node around it.
"""

from dataclasses import dataclass
import math
import time
from typing import Callable, Optional, Sequence, Tuple, Union

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
import py_trees

from my_bot.conversions import quaternion_zw
from my_bot.follow_control import compute_command, search_command


# ---------------------------------------------------------------------------
# Blackboard key constants
# ---------------------------------------------------------------------------
BB_ESTOP = 'estop'
BB_LAST_SEEN = 'target_last_seen'      # clock time (s) of the last sighting
BB_TARGET_RANGE = 'target_range'       # metres, -1.0 if unknown
BB_TARGET_BEARING = 'target_bearing'   # rad, positive left
BB_TARGET_YAW_RATE = 'target_yaw_rate'  # rad/s the intruder turns its bearing
BB_SEARCH_GOAL = 'search_goal'         # odom (x, y) to search first, or None
BB_SWEEP = 'sweep_points'              # map (x, y, yaw) viewpoints left
BB_FRONT_CLEAR = 'front_clear'         # closest lidar return ahead (m)
BB_WP_INDEX = 'waypoint_index'
BB_WP_VISITS = 'waypoints_visited'
BB_PATROL_ZONE = 'patrol_zone'
BB_NAV_GOAL_SENT = 'nav_goal_sent'
BB_DWELL_START = 'dwell_start_time'

Status = py_trees.common.Status
Publish = Callable[[Twist], None]
Route = Callable[[], Tuple[str, Sequence[Sequence[float]]]]  # (zone, waypoints)
Log = Optional[Callable[[str], None]]
Navigator = object  # BasicNavigator's goToPose / isTaskComplete / cancelTask


def bb_get(key, default):
    """Blackboard value of ``key``; ``default`` while nothing has set it."""
    bb = py_trees.blackboard.Blackboard()
    return bb.get(key) if bb.exists(key) else default


def bb_set(key, value) -> None:
    """Set blackboard ``key``."""
    py_trees.blackboard.Blackboard().set(key, value)


def _sighting_age(clock) -> float:
    """Seconds since the intruder was last seen (inf if never)."""
    return clock() - bb_get(BB_LAST_SEEN, -math.inf)


def _cancel_goal(navigator) -> None:
    """Cancel the Nav2 goal in progress, if any; the patrol sends a new one."""
    if not navigator.isTaskComplete():
        navigator.cancelTask()
    bb_set(BB_NAV_GOAL_SENT, False)


def _cancel_drive(navigator, driving: bool) -> None:
    """Cancel a leaf's own drive if it is still running."""
    if driving and not navigator.isTaskComplete():
        navigator.cancelTask()


def _no_log(_msg: str) -> None:
    pass


def _as_route(waypoints) -> Route:
    """Return a route callable for a fixed waypoint list (zone 'patrol') or a route."""
    if callable(waypoints):
        return waypoints
    return lambda: ('patrol', waypoints)


def _map_goal(x: float, y: float, yaw: Optional[float] = None,
              frame: str = 'map') -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = frame
    goal.header.stamp = Time()  # zero stamp: use the latest transform
    goal.pose.position.x = x
    goal.pose.position.y = y
    if yaw is None:
        goal.pose.orientation.w = 1.0
    else:
        goal.pose.orientation.z, goal.pose.orientation.w = quaternion_zw(yaw)
    return goal


def _twist(linear: float = 0.0, angular: float = 0.0) -> Twist:
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    return cmd


@dataclass
class FollowGains:
    """Stand-off follow parameters (see follow_control.compute_command)."""

    desired_distance: float = 2.5
    deadband: float = 0.15
    k_lin: float = 0.8
    k_yaw: float = 1.5
    max_linear_speed: float = 1.0
    max_back_speed: float = 0.2
    max_angular_speed: float = 1.5
    safety_distance: float = 0.6
    k_ff: float = 1.0  # bearing-rate feedforward (0 = P-loop only)


# ---------------------------------------------------------------------------
# Behavior Tree leaves
# ---------------------------------------------------------------------------

class EStopActive(py_trees.behaviour.Behaviour):
    """SUCCESS while the emergency stop is latched."""

    def __init__(self, name: str = 'EStopActive'):
        super().__init__(name)

    def update(self) -> Status:
        return Status.SUCCESS if bb_get(BB_ESTOP, False) else Status.FAILURE


class HaltRobot(py_trees.behaviour.Behaviour):
    """Cancel navigation and hold the robot still (RUNNING)."""

    def __init__(self, navigator: Navigator, publish: Publish,
                 name: str = 'HaltRobot'):
        super().__init__(name)
        self._nav = navigator
        self._publish = publish

    def update(self) -> Status:
        _cancel_goal(self._nav)
        self._publish(_twist())
        return Status.RUNNING


class IntruderVisible(py_trees.behaviour.Behaviour):
    """SUCCESS if the fused target was seen within ``timeout`` seconds."""

    def __init__(self, timeout: float = 0.5, clock=time.monotonic,
                 name: str = 'IntruderVisible'):
        super().__init__(name)
        self._timeout = timeout
        self._clock = clock

    def update(self) -> Status:
        age = _sighting_age(self._clock)
        return Status.SUCCESS if age <= self._timeout else Status.FAILURE


class IntruderRecentlyLost(py_trees.behaviour.Behaviour):
    """SUCCESS for ``search_secs`` after the target drops out of view."""

    def __init__(self, timeout: float = 0.5, search_secs: float = 6.0,
                 clock=time.monotonic, name: str = 'IntruderRecentlyLost'):
        super().__init__(name)
        self._timeout = timeout
        self._search_secs = search_secs
        self._clock = clock

    def update(self) -> Status:
        age = _sighting_age(self._clock)
        if self._timeout < age <= self._timeout + self._search_secs:
            return Status.SUCCESS
        return Status.FAILURE


class CancelPatrol(py_trees.behaviour.Behaviour):
    """Cancel any active Nav2 navigation goal."""

    def __init__(self, navigator: Navigator, name: str = 'CancelPatrol'):
        super().__init__(name)
        self._nav = navigator

    def update(self) -> Status:
        _cancel_goal(self._nav)
        return Status.SUCCESS


class FollowIntruder(py_trees.behaviour.Behaviour):
    """Keep ``desired_distance`` from the intruder (RUNNING)."""

    def __init__(self, publish: Publish, gains: FollowGains,
                 name: str = 'FollowIntruder'):
        super().__init__(name)
        self._publish = publish
        self._g = gains

    def update(self) -> Status:
        g = self._g
        rng = bb_get(BB_TARGET_RANGE, -1.0)
        v, w = compute_command(
            bb_get(BB_TARGET_BEARING, 0.0), rng if rng > 0 else None,
            g.desired_distance, g.k_lin, g.k_yaw, g.max_linear_speed,
            g.max_back_speed, g.max_angular_speed, g.deadband,
            bb_get(BB_FRONT_CLEAR, math.inf), g.safety_distance,
            g.k_ff * bb_get(BB_TARGET_YAW_RATE, 0.0))
        self._publish(_twist(v, w))
        return Status.RUNNING


class SearchPredictedPath(py_trees.behaviour.Behaviour):
    """Drive (Nav2) to where the intruder was heading, then turn (RUNNING).

    The goal is read from the blackboard when the search starts and sent in
    ``frame`` (odom: Nav2 transforms it to the map). Once Nav2 is done,
    whether it arrived or not, or at once without a goal, the robot turns
    toward the side the intruder was last seen on. The direction is latched
    when the turn starts: the blackboard bearing follows the robot's turn,
    and re-reading it would stop the spin once the robot faces the
    last-seen spot. Leaving the search cancels a drive still running.
    """

    def __init__(self, navigator: Navigator, publish: Publish,
                 speed: float = 0.6, frame: str = 'odom', log: Log = None,
                 name: str = 'SearchPredictedPath'):
        super().__init__(name)
        self._nav = navigator
        self._publish = publish
        self._speed = speed
        self._frame = frame
        self._log = log or _no_log
        self._goal = None
        self._driving = False
        self._turn = None

    def initialise(self) -> None:
        _cancel_goal(self._nav)  # the patrol goal
        self._goal = bb_get(BB_SEARCH_GOAL, None)
        self._driving = False
        self._turn = None
        if self._goal is not None:
            self._nav.goToPose(_map_goal(*self._goal, frame=self._frame))
            self._driving = True
            self._log('search: driving to ({:.2f}, {:.2f}) in {}'.format(
                *self._goal, self._frame))

    def update(self) -> Status:
        if self._driving:
            if not self._nav.isTaskComplete():
                return Status.RUNNING
            self._driving = False
        if self._turn is None:
            self._turn = search_command(bb_get(BB_TARGET_BEARING, 0.0), self._speed)
        self._publish(_twist(0.0, self._turn))
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        _cancel_drive(self._nav, self._driving)
        self._driving = False


class SweepPending(py_trees.behaviour.Behaviour):
    """SUCCESS while sweep viewpoints are left and the sweep window is open.

    The window follows the search: from ``timeout + search_secs`` to
    ``timeout + search_secs + sweep_secs`` after the last sighting.
    """

    def __init__(self, timeout: float = 0.5, search_secs: float = 6.0,
                 sweep_secs: float = 60.0, clock=time.monotonic,
                 name: str = 'SweepPending'):
        super().__init__(name)
        self._timeout = timeout
        self._search_secs = search_secs
        self._sweep_secs = sweep_secs
        self._clock = clock

    def update(self) -> Status:
        if not bb_get(BB_SWEEP, None):
            return Status.FAILURE
        age = _sighting_age(self._clock)
        if age <= self._timeout + self._search_secs + self._sweep_secs:
            return Status.SUCCESS
        return Status.FAILURE


class SweepRooms(py_trees.behaviour.Behaviour):
    """Nav2 to each sweep viewpoint (map frame), dwell, drop it (RUNNING).

    A viewpoint Nav2 cannot reach counts as visited: the next one may be.
    SUCCESS once none is left. Leaving the sweep cancels a drive still
    running; the viewpoint stays and is driven to again on return.
    """

    def __init__(self, navigator: Navigator, dwell_secs: float = 2.0,
                 clock=time.monotonic, log: Log = None, name: str = 'SweepRooms'):
        super().__init__(name)
        self._nav = navigator
        self._dwell = dwell_secs
        self._clock = clock
        self._log = log or _no_log
        self._driving = False
        self._arrived = None

    def initialise(self) -> None:
        _cancel_goal(self._nav)  # the patrol goal
        self._driving = False
        self._arrived = None

    def update(self) -> Status:
        points = list(bb_get(BB_SWEEP, None) or [])
        if not points:
            return Status.SUCCESS
        if not self._driving and self._arrived is None:
            x, y, yaw = points[0]
            self._nav.goToPose(_map_goal(x, y, yaw))
            self._driving = True
            self._log('sweep: viewpoint ({:.2f}, {:.2f}), {} left'.format(
                x, y, len(points)))
            return Status.RUNNING
        if self._driving:
            if not self._nav.isTaskComplete():
                return Status.RUNNING
            self._driving = False
            self._arrived = self._clock()
        if self._clock() - self._arrived < self._dwell:
            return Status.RUNNING
        self._arrived = None
        bb_set(BB_SWEEP, points[1:])
        return Status.RUNNING if points[1:] else Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        _cancel_drive(self._nav, self._driving)
        self._driving = False
        self._arrived = None


class IncidentOpen(py_trees.behaviour.Behaviour):
    """SUCCESS while an incident is open (``is_open()``)."""

    def __init__(self, is_open: Callable[[], bool], name: str = 'IncidentOpen'):
        super().__init__(name)
        self._is_open = is_open

    def update(self) -> Status:
        return Status.SUCCESS if self._is_open() else Status.FAILURE


class ReportIncident(py_trees.behaviour.Behaviour):
    """Close the open incident (``report()``); the patrol resumes next tick."""

    def __init__(self, report: Callable[[], None], name: str = 'ReportIncident'):
        super().__init__(name)
        self._report = report

    def update(self) -> Status:
        self._report()
        return Status.SUCCESS


class NavigateToWaypoint(py_trees.behaviour.Behaviour):
    """Send a Nav2 goal on first tick; poll completion on subsequent ticks.

    ``waypoints`` is a list, or a route callable returning (zone, waypoints)
    for the zone scheduled now. A new zone starts at its first waypoint.
    """

    def __init__(self, navigator: Navigator, waypoints,
                 log: Log = None, name: str = 'NavigateToWaypoint'):
        super().__init__(name)
        self._nav = navigator
        self._route = _as_route(waypoints)
        self._log = log or _no_log

    def update(self) -> Status:
        if not bb_get(BB_NAV_GOAL_SENT, False):
            zone, waypoints = self._route()
            previous = bb_get(BB_PATROL_ZONE, None)
            if zone != previous:
                bb_set(BB_PATROL_ZONE, zone)
                if previous is not None:  # a new zone starts at its first
                    bb_set(BB_WP_INDEX, 0)
                    self._log(f'patrol: zone {previous} -> {zone}')
            wp_idx = bb_get(BB_WP_INDEX, 0)
            wp = waypoints[wp_idx % len(waypoints)]
            self._nav.goToPose(_map_goal(wp[0], wp[1]))
            bb_set(BB_NAV_GOAL_SENT, True)
            self._log(f'patrol: heading to {zone} waypoint {wp_idx} {tuple(wp)}')

        if self._nav.isTaskComplete():
            bb_set(BB_NAV_GOAL_SENT, False)
            return Status.SUCCESS
        return Status.RUNNING


class WaitAtWaypoint(py_trees.behaviour.Behaviour):
    """Dwell at the current waypoint for a configured duration."""

    def __init__(self, dwell_secs: float = 2.0, clock=time.monotonic,
                 name: str = 'WaitAtWaypoint'):
        super().__init__(name)
        self._dwell = dwell_secs
        self._clock = clock

    def update(self) -> Status:
        start = bb_get(BB_DWELL_START, None)
        now = self._clock()
        if start is None:
            bb_set(BB_DWELL_START, now)
            return Status.RUNNING
        if (now - start) >= self._dwell:
            bb_set(BB_DWELL_START, None)
            return Status.SUCCESS
        return Status.RUNNING


class IncrementWaypoint(py_trees.behaviour.Behaviour):
    """Advance the waypoint index (wrapping) and count the visit."""

    def __init__(self, waypoints: Union[int, Route],
                 name: str = 'IncrementWaypoint'):
        super().__init__(name)
        self._count = (lambda: waypoints) if isinstance(waypoints, int) else (
            lambda: len(_as_route(waypoints)()[1]))

    def update(self) -> Status:
        bb_set(BB_WP_INDEX, (bb_get(BB_WP_INDEX, 0) + 1) % self._count())
        bb_set(BB_WP_VISITS, bb_get(BB_WP_VISITS, 0) + 1)
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Tree factory — exposed for unit tests
# ---------------------------------------------------------------------------

def build_security_guard_tree(
        navigator: Navigator,
        publish: Publish,
        waypoints,
        gains: FollowGains = FollowGains(),
        target_timeout: float = 0.5,
        search_secs: float = 6.0,
        search_speed: float = 0.6,
        dwell_secs: float = 2.0,
        sweep_secs: float = 60.0,
        sweep_dwell_secs: float = 2.0,
        log: Log = None,
        clock=time.monotonic,
        incident_open: Callable[[], bool] = lambda: False,
        report: Callable[[], None] = lambda: None,
) -> py_trees.trees.BehaviourTree:
    """Construct and return the security guard behaviour tree.

    ``waypoints`` is a fixed list or a route callable (zone, waypoints).
    ``clock`` returns seconds; the node passes its (sim-time) clock.
    ``incident_open`` and ``report`` connect ReportProtocol to the node's
    incident log.
    """
    estop = py_trees.composites.Sequence('EmergencyStop', memory=False)
    estop.add_children([EStopActive(), HaltRobot(navigator, publish)])

    intruder = py_trees.composites.Sequence('IntruderProtocol', memory=False)
    intruder.add_children([
        IntruderVisible(target_timeout, clock),
        CancelPatrol(navigator),
        FollowIntruder(publish, gains),
    ])

    search = py_trees.composites.Sequence('SearchProtocol', memory=False)
    search.add_children([
        IntruderRecentlyLost(target_timeout, search_secs, clock),
        SearchPredictedPath(navigator, publish, search_speed, log=log),
    ])

    sweep = py_trees.composites.Sequence('SweepProtocol', memory=False)
    sweep.add_children([
        SweepPending(target_timeout, search_secs, sweep_secs, clock),
        SweepRooms(navigator, sweep_dwell_secs, clock, log),
    ])

    report_seq = py_trees.composites.Sequence('ReportProtocol', memory=False)
    report_seq.add_children([IncidentOpen(incident_open), ReportIncident(report)])

    patrol = py_trees.composites.Sequence('PatrolProtocol', memory=True)
    patrol.add_children([
        NavigateToWaypoint(navigator, waypoints, log),
        WaitAtWaypoint(dwell_secs, clock),
        IncrementWaypoint(_as_route(waypoints)),
    ])

    root = py_trees.composites.Selector('SecurityGuard', memory=False)
    root.add_children([estop, intruder, search, sweep, report_seq, patrol])
    return py_trees.trees.BehaviourTree(root)


def active_protocol(tree: py_trees.trees.BehaviourTree) -> str:
    """Name of the root child that ran last tick, e.g. 'PatrolProtocol'."""
    for child in tree.root.children:
        if child.status in (Status.RUNNING, Status.SUCCESS):
            return child.name
    return 'Idle'
