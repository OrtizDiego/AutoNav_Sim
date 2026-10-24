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

"""Scripted intruder scenarios (config/scenarios.yaml).

A scenario is a list of legs the pedestrian walks in order, so a run can be
repeated and compared: person_controller follows it instead of wandering
when its ``scenario`` parameter names one (`make scenario NAME=sprint`).

Each leg may wait for a trigger, then walk to a point, then pause::

    sprint:
      description: Waits to be seen, then sprints out of the hall.
      legs:
        - {wait_detected: true}
        - {to: [-4.2, 7.5], speed: 2.5}
        - {to: [-4.2, 4.0], speed: 2.5, pause: 5.0}

Leg keys (all optional):

to                 [x, y] in the map (= world) frame; no ``to`` = stand
speed              m/s while walking to ``to`` (default 0.7, a walk)
pause              seconds to stand after arriving
wait_detected      start only once the robot's tracker has the person
                   (person_controller's alarm: a lock within notice_radius)
wait_robot_within  start only once the robot is closer than this (m)

A scenario may also script a second person (``second_person: {legs: ...}``,
same leg keys). scenario.launch.py then runs it in two_person.world, whose
second actor starts at (5.0, 6.0); a second person_controller
(``scenario_person: second``) walks those legs. Its triggers react to the
same tracker lock as the intruder's: the tracker follows one person and
cannot tell which one it has.

Pure Python + PyYAML, no ROS: the launch file, the nodes and the tests all
read the same file.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import yaml


Point = Tuple[float, float]

LEG_KEYS = {'to', 'speed', 'pause', 'wait_detected', 'wait_robot_within'}
SCENARIO_KEYS = {'description', 'legs', 'second_person'}


@dataclass
class Leg:
    """One step of a scenario: optional trigger, walk, pause."""

    to: Optional[Point] = None
    speed: float = 0.7
    pause: float = 0.0
    wait_detected: bool = False
    wait_robot_within: float = 0.0

    @property
    def waits(self) -> bool:
        """True if the leg holds until a trigger fires."""
        return self.wait_detected or self.wait_robot_within > 0.0


@dataclass
class Scenario:
    """A named, ordered list of legs."""

    name: str
    description: str = ''
    legs: List[Leg] = field(default_factory=list)
    second_legs: List[Leg] = field(default_factory=list)

    def waypoints(self) -> List[Point]:
        """Return the points the legs walk to, in order."""
        return [leg.to for leg in self.legs if leg.to is not None]

    @property
    def has_second_person(self) -> bool:
        """True if the scenario scripts a second person too."""
        return bool(self.second_legs)

    def second_person(self) -> 'Scenario':
        """Return the second person's part as a scenario of its own."""
        if not self.second_legs:
            raise ValueError(f'{self.name}: no second_person')
        return Scenario(name=f'{self.name}/second_person',
                        description=self.description, legs=self.second_legs)


def parse_leg(raw: dict, where: str) -> Leg:
    """Build a Leg from its YAML mapping; ValueError names ``where``."""
    if not isinstance(raw, dict):
        raise ValueError(f'{where}: a leg must be a mapping, got {raw!r}')
    unknown = set(raw) - LEG_KEYS
    if unknown:
        raise ValueError(f'{where}: unknown keys {sorted(unknown)}')
    to = raw.get('to')
    if to is not None:
        if len(to) != 2:
            raise ValueError(f'{where}: to must be [x, y], got {to!r}')
        to = (float(to[0]), float(to[1]))
    leg = Leg(to=to,
              speed=float(raw.get('speed', Leg.speed)),
              pause=float(raw.get('pause', 0.0)),
              wait_detected=bool(raw.get('wait_detected', False)),
              wait_robot_within=float(raw.get('wait_robot_within', 0.0)))
    if leg.speed <= 0.0:
        raise ValueError(f'{where}: speed must be positive')
    if leg.pause < 0.0 or leg.wait_robot_within < 0.0:
        raise ValueError(f'{where}: pause and wait_robot_within must be >= 0')
    return leg


def parse_scenarios(data: dict) -> Dict[str, Scenario]:
    """Turn the loaded YAML mapping into {name: Scenario}."""
    if not isinstance(data, dict) or not data:
        raise ValueError('scenarios file must map names to scenarios')
    scenarios = {}
    for name, body in data.items():
        legs = body.get('legs') if isinstance(body, dict) else None
        if not legs:
            raise ValueError(f'{name}: needs a non-empty legs list')
        unknown = set(body) - SCENARIO_KEYS
        if unknown:
            raise ValueError(f'{name}: unknown keys {sorted(unknown)}')
        second = body.get('second_person')
        second_legs = []
        if second is not None:
            raw_legs = second.get('legs') if isinstance(second, dict) else None
            if not raw_legs:
                raise ValueError(f'{name}: second_person needs a non-empty legs list')
            second_legs = [parse_leg(raw, f'{name} second_person leg {i}')
                           for i, raw in enumerate(raw_legs)]
        scenarios[name] = Scenario(
            name=name, description=str(body.get('description', '')).strip(),
            legs=[parse_leg(raw, f'{name} leg {i}') for i, raw in enumerate(legs)],
            second_legs=second_legs)
    return scenarios


def load_scenarios(path: str) -> Dict[str, Scenario]:
    """Read every scenario from a scenarios YAML file."""
    with open(path) as f:
        return parse_scenarios(yaml.safe_load(f))
