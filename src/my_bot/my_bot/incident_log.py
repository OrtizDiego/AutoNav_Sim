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

"""Security incidents: alert events and an on-disk incident log.

An incident opens when the guard first follows an intruder, collects every
loss of sight and re-acquisition, and closes when the search gives up. Each
step is an event dict, published by security_guard_bt as JSON on
/security_events and appended to ``<directory>/incidents.jsonl``:

  intruder_detected   incident opened (position, snapshot)
  intruder_lost       out of sight, searching
  intruder_reacquired seen again before the search gave up (position)
  incident_closed     search gave up (duration, sightings, losses)

Positions carry the frame they are in (``map`` when the node could
transform them, else ``odom``). Times are simulation seconds (``t``) plus
the wall-clock time (``wall``) for people reading the log.

Pure Python (no ROS).
"""

import datetime
import json
import os
from typing import Optional, Tuple

DETECTED = 'intruder_detected'
LOST = 'intruder_lost'
REACQUIRED = 'intruder_reacquired'
CLOSED = 'incident_closed'

LOG_NAME = 'incidents.jsonl'


def _wall_now() -> str:
    return datetime.datetime.now().isoformat(timespec='seconds')


class IncidentLog:
    """Incident lifecycle; writes one JSON line per event if ``directory``."""

    def __init__(self, directory: str = '', wall_clock=_wall_now):
        self.directory = os.path.expanduser(directory) if directory else ''
        self._wall = wall_clock
        self.count = 0          # incidents opened this session
        self.current = None     # dict of the open incident, or None
        if self.directory:
            os.makedirs(self.directory, exist_ok=True)

    @property
    def is_open(self) -> bool:
        """True while an incident is open."""
        return self.current is not None

    def snapshot_path(self) -> Optional[str]:
        """Where the next event's camera snapshot goes (None: no directory)."""
        if not self.directory or self.current is None:
            return None
        n = self.current['sightings'] + self.current['losses']
        return os.path.join(
            self.directory, f'incident_{self.current["id"]:03d}_{n:02d}.jpg')

    # -- lifecycle ---------------------------------------------------------

    def sighted(self, t: float, where: Optional[Tuple[float, float]],
                frame: str, snapshot: bool = False) -> dict:
        """Intruder (re)acquired: opens an incident, or adds a sighting."""
        if self.current is None:
            self.count += 1
            self.current = {'id': self.count, 'start': t, 'sightings': 0,
                            'losses': 0}
            kind = DETECTED
        else:
            kind = REACQUIRED
        event = self._event(kind, t, where, frame)
        if snapshot:
            event['snapshot'] = self.snapshot_path()
        self.current['sightings'] += 1
        return self._write(event)

    def lost(self, t: float, where: Optional[Tuple[float, float]],
             frame: str) -> Optional[dict]:
        """Out of sight: the guard searches (None if no incident is open)."""
        if self.current is None:
            return None
        event = self._event(LOST, t, where, frame)
        self.current['losses'] += 1
        return self._write(event)

    def close(self, t: float) -> Optional[dict]:
        """Search gave up: report the incident; None if none is open."""
        if self.current is None:
            return None
        event = self._event(CLOSED, t, None, '')
        event.update(duration=round(t - self.current['start'], 2),
                     sightings=self.current['sightings'],
                     losses=self.current['losses'])
        self.current = None
        return self._write(event)

    # -- helpers -----------------------------------------------------------

    def _event(self, kind: str, t: float,
               where: Optional[Tuple[float, float]], frame: str) -> dict:
        event = {'event': kind, 'incident': self.current['id'],
                 't': round(t, 3), 'wall': self._wall()}
        if where is not None:
            event.update(frame=frame, x=round(where[0], 2), y=round(where[1], 2))
        return event

    def _write(self, event: dict) -> dict:
        if self.directory:
            with open(os.path.join(self.directory, LOG_NAME), 'a') as f:
                f.write(json.dumps(event) + '\n')
        return event
