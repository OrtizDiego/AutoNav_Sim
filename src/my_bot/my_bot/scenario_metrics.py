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

"""Score a guard run against the intruder's ground truth (Phase 5).

scenario_evaluator feeds samples of the robot pose, the person's true
position (/person/odom from the actor plugin) and target_tracker's track;
this module turns them into the numbers the plan asks for:

time to detect      first moment the person was in the camera's view to the
                    first confirmed track on the person
% time in view      after the first detection, share of time the person was
                    in the camera's view (did the guard keep them in sight?)
% tracked in view   share of in-view time with a confirmed track on the
                    person (tracking recall)
track breaks        confirmed -> not confirmed transitions, also per minute
                    after the first detection
false alarms        confirmed track episodes that never came within
                    ``match_radius`` of anyone; plus the share of
                    confirmed time spent on no one
track switches      (two-person scenarios) a confirmed track that jumps from
                    one person to the other without a break; plus the share
                    of confirmed time spent on the second person
patrol coverage     share of the floor the camera has seen (0.5 m cells,
                    within view range and in line of sight on the map)

"In view" is geometric ground truth: the person is within the camera's
horizontal field of view and ``view_range``, and the straight line from the
robot crosses no wall or exhibit of the saved map. The robot pose is
odometry, which equals the world frame at spawn and drifts a little;
``match_radius`` absorbs that.

The intruder is "the person"; other people (``others`` in add()) only
decide whether a track off the intruder is a false alarm or a switch.

Pure Python + numpy, no ROS: importable by unit tests.
"""

import json
import math
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from my_bot.clearance_map import ClearanceMap


Point = Tuple[float, float]
Pose = Tuple[float, float, float]  # x, y, yaw

CONFIRMED = 'confirmed'


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def line_of_sight(cmap: ClearanceMap, a: Point, b: Point) -> bool:
    """Tell whether the segment a -> b crosses no blocked cell of the map."""
    return cmap.segment_clearance(a, b) > 0.0


def in_camera_view(cmap: ClearanceMap, robot: Pose, target: Point,
                   hfov: float, view_range: float) -> bool:
    """Tell whether ``target`` is inside the camera's view cone and visible."""
    dx, dy = target[0] - robot[0], target[1] - robot[1]
    if math.hypot(dx, dy) > view_range:
        return False
    if abs(_wrap(math.atan2(dy, dx) - robot[2])) > hfov / 2.0:
        return False
    return line_of_sight(cmap, (robot[0], robot[1]), target)


class FloorCoverage:
    """Which floor cells the camera has seen so far."""

    def __init__(self, cmap: ClearanceMap, hfov: float, view_range: float,
                 cell: float = 0.5, min_clear: float = 0.3):
        """Cells are ``cell`` m squares whose centre has ``min_clear`` m."""
        self.cmap, self.hfov, self.view_range = cmap, hfov, view_range
        x0, y0 = cmap.origin
        xs = x0 + cell / 2.0 + cell * np.arange(int(cmap.width * cmap.resolution / cell))
        ys = y0 + cell / 2.0 + cell * np.arange(int(cmap.height * cmap.resolution / cell))
        gx, gy = np.meshgrid(xs, ys)
        gx, gy = gx.ravel(), gy.ravel()
        floor = cmap.clearance_many(gx, gy) >= min_clear
        self.xs, self.ys = gx[floor], gy[floor]
        self.seen = np.zeros(self.xs.shape, dtype=bool)

    @property
    def fraction(self) -> float:
        """Share of the floor cells seen, in [0, 1]."""
        return float(self.seen.mean()) if self.seen.size else 0.0

    def observe(self, robot: Pose) -> int:
        """Mark the cells visible from ``robot``; return how many were new."""
        dx, dy = self.xs - robot[0], self.ys - robot[1]
        dist = np.hypot(dx, dy)
        bearing = np.abs((np.arctan2(dy, dx) - robot[2] + np.pi) % (2 * np.pi) - np.pi)
        idx = np.nonzero(~self.seen & (dist <= self.view_range)
                         & (bearing <= self.hfov / 2.0))[0]
        if idx.size == 0:
            return 0
        # Sample every candidate's sight line at map resolution at once.
        steps = max(2, int(math.ceil(self.view_range / self.cmap.resolution)) + 1)
        t = np.linspace(0.0, 1.0, steps)[None, :]
        px = robot[0] + dx[idx, None] * t
        py = robot[1] + dy[idx, None] * t
        visible = (self.cmap.clearance_many(px, py) > 0.0).all(axis=1)
        self.seen[idx[visible]] = True
        return int(visible.sum())


class ScenarioMetrics:
    """Accumulate samples of one run; report() gives the metrics."""

    def __init__(self, cmap: ClearanceMap, hfov: float = 1.089,
                 view_range: float = 8.0, match_radius: float = 1.5,
                 coverage_cell: float = 0.5, coverage_period: float = 0.5):
        self.cmap, self.hfov, self.view_range = cmap, hfov, view_range
        self.match_radius = match_radius
        self.coverage = FloorCoverage(cmap, hfov, view_range, coverage_cell)
        self._coverage_period = coverage_period
        self._last_coverage = -math.inf
        self.start: Optional[float] = None
        self.last: Optional[float] = None
        self.first_in_view: Optional[float] = None
        self.first_detection: Optional[float] = None
        self.in_view_after_detection = 0.0
        self.after_detection = 0.0
        self.in_view_secs = 0.0
        self.tracked_in_view_secs = 0.0
        self.confirmed_secs = 0.0
        self.false_track_secs = 0.0
        self.other_track_secs = 0.0
        self.track_breaks = 0
        self.track_switches = 0
        self.saw_others = False
        self.episodes: List[bool] = []  # one per confirmed episode: on anyone?
        self._on: Optional[int] = None  # who the episode was last on: 0 = intruder
        self.follow_ranges: List[float] = []
        self.guard_secs: Dict[str, float] = {}
        self.events: Dict[str, int] = {}
        self._confirmed = False
        self._prev: Optional[dict] = None

    def event(self, kind: str) -> None:
        """Count a /security_events entry by its kind."""
        self.events[kind] = self.events.get(kind, 0) + 1

    def _on_whom(self, track_xy: Optional[Point],
                 people: Sequence[Optional[Point]]) -> Optional[int]:
        """Index of the person the track is on (nearest within match_radius)."""
        if track_xy is None:
            return None
        best, best_d = None, self.match_radius
        for i, xy in enumerate(people):
            if xy is not None and math.dist(track_xy, xy) <= best_d:
                best, best_d = i, math.dist(track_xy, xy)
        return best

    def add(self, t: float, robot: Pose, person: Optional[Point],
            track_state: str = 'none', track_xy: Optional[Point] = None,
            guard_state: str = '',
            others: Sequence[Optional[Point]] = ()) -> None:
        """Add the state at time ``t``; it holds until the next sample.

        ``person`` is the intruder; ``others`` are further people.
        """
        if self.start is None:
            self.start = t
        if t - self._last_coverage >= self._coverage_period:
            self.coverage.observe(robot)
            self._last_coverage = t
        confirmed = track_state == CONFIRMED
        in_view = person is not None and in_camera_view(
            self.cmap, robot, person, self.hfov, self.view_range)
        self.saw_others = self.saw_others or any(o is not None for o in others)
        on = self._on_whom(track_xy, [person, *others]) if confirmed else None
        matched = on == 0

        if in_view and self.first_in_view is None:
            self.first_in_view = t
        if matched and self.first_detection is None:
            self.first_detection = t
        if confirmed and not self._confirmed:
            self.episodes.append(False)
            self._on = None
        if self._confirmed and not confirmed:
            self.track_breaks += 1
        if on is not None:
            self.episodes[-1] = True
            if self._on is not None and on != self._on:
                self.track_switches += 1
            self._on = on
        if matched:
            self.follow_ranges.append(math.dist(robot[:2], person))
        self._confirmed = confirmed

        if self._prev is not None:
            self._accumulate(t - self.last, self._prev)
        # A confirmed track without a position (none received yet) is
        # neither on nor off the person.
        self._prev = {'in_view': in_view, 'matched': matched,
                      'on_other': on is not None and on > 0,
                      'confirmed': confirmed and track_xy is not None,
                      'guard': guard_state,
                      'detected': self.first_detection is not None}
        self.last = t

    def _accumulate(self, dt: float, s: dict) -> None:
        """Credit the interval since the previous sample to its state."""
        if s['detected']:
            self.after_detection += dt
            self.in_view_after_detection += dt if s['in_view'] else 0.0
        if s['in_view']:
            self.in_view_secs += dt
            self.tracked_in_view_secs += dt if s['matched'] else 0.0
        if s['confirmed']:
            self.confirmed_secs += dt
            if s['on_other']:
                self.other_track_secs += dt
            elif not s['matched']:
                self.false_track_secs += dt
        if s['guard']:
            self.guard_secs[s['guard']] = self.guard_secs.get(s['guard'], 0.0) + dt

    def report(self) -> dict:
        """Return the run's metrics; None where a metric has nothing to measure."""
        duration = (self.last - self.start) if self.start is not None else 0.0

        def pct(part, whole):
            return round(100.0 * part / whole, 1) if whole > 0.0 else None

        detect = None
        if self.first_detection is not None and self.first_in_view is not None:
            detect = round(max(0.0, self.first_detection - self.first_in_view), 2)
        minutes = self.after_detection / 60.0
        return {
            'duration_s': round(duration, 1),
            'first_in_view_s': (None if self.first_in_view is None
                                else round(self.first_in_view - self.start, 2)),
            'time_to_detect_s': detect,
            'in_view_pct': pct(self.in_view_after_detection, self.after_detection),
            'tracked_in_view_pct': pct(self.tracked_in_view_secs, self.in_view_secs),
            'track_breaks': self.track_breaks,
            'track_breaks_per_min': (round(self.track_breaks / minutes, 2)
                                     if minutes > 0.0 else None),
            'false_alarms': sum(1 for matched in self.episodes if not matched),
            'false_track_pct': pct(self.false_track_secs, self.confirmed_secs),
            'track_switches': self.track_switches,
            'second_person_track_pct': (pct(self.other_track_secs, self.confirmed_secs)
                                        if self.saw_others else None),
            'follow_range_mean_m': (round(float(np.mean(self.follow_ranges)), 2)
                                    if self.follow_ranges else None),
            'patrol_coverage_pct': round(100.0 * self.coverage.fraction, 1),
            'guard_state_pct': {k: pct(v, duration)
                                for k, v in sorted(self.guard_secs.items())},
            'events': dict(sorted(self.events.items())),
        }


# Report key, comparison, threshold parameter name.
TARGETS = (
    ('time_to_detect_s', 'max', 'max_time_to_detect_s'),
    ('tracked_in_view_pct', 'min', 'min_tracked_in_view_pct'),
    ('track_breaks_per_min', 'max', 'max_track_breaks_per_min'),
    ('false_alarms', 'max', 'max_false_alarms'),
    ('track_switches', 'max', 'max_track_switches'),
    ('patrol_coverage_pct', 'min', 'min_patrol_coverage_pct'),
)


def check_targets(report: dict, limits: Dict[str, float]) -> List[str]:
    """Return one line per missed target; a limit < 0 is not checked.

    A run that never detected the person fails time_to_detect: nothing else
    can be judged then.
    """
    failures = []
    for key, kind, limit_name in TARGETS:
        limit = limits.get(limit_name, -1.0)
        if limit < 0.0:
            continue
        value = report.get(key)
        if value is None:
            if key == 'time_to_detect_s':
                failures.append('the person was never detected')
            continue
        if (kind == 'max' and value > limit) or (kind == 'min' and value < limit):
            failures.append(f'{key} {value} ({kind} {limit})')
    return failures


def format_report(name: str, report: dict, failures: List[str]) -> str:
    """Human-readable summary for the log."""
    lines = [f'scenario {name}: ' + ('PASS' if not failures else 'FAIL')]
    for key, value in report.items():
        lines.append(f'  {key:<24} {value}')
    lines += [f'  missed: {f}' for f in failures]
    return '\n'.join(lines)


def smoke_failures(report: Optional[dict]) -> List[str]:
    """Return why a CI smoke run failed; empty = the whole stack ran.

    Looser than check_targets(): a CI runner renders on the CPU and runs
    well below real time, so only "every part came up and the loop closed"
    is checked: the scenario finished, the person was detected and the
    guard raised its alert.
    """
    if not report:
        return ['no report: the evaluator never finished']
    failures = []
    if report.get('outcome') != 'completed':
        failures.append(f"outcome {report.get('outcome')} (want completed)")
    if report.get('time_to_detect_s') is None:
        failures.append('the person was never detected')
    if not report.get('events', {}).get('intruder_detected'):
        failures.append('no intruder_detected event from the guard')
    if not report.get('guard_state_pct'):
        failures.append('no /security_guard/state received')
    return failures


def main(argv=None) -> int:
    """`python3 -m my_bot.scenario_metrics smoke <report.json>` (smoke_test.sh)."""
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2 or argv[0] != 'smoke':
        print('usage: python3 -m my_bot.scenario_metrics smoke <report.json>')
        return 2
    try:
        with open(argv[1]) as f:
            report = json.load(f)
    except (OSError, ValueError) as e:
        print(f'cannot read {argv[1]}: {e}')
        report = None
    failures = smoke_failures(report)
    print('smoke test: ' + ('PASS' if not failures else 'FAIL'))
    for failure in failures:
        print(f'  {failure}')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
