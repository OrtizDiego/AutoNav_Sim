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

"""Tests for the incident lifecycle and its on-disk log."""

import json
import os

from my_bot.incident_log import (
    CLOSED, DETECTED, IncidentLog, LOG_NAME, LOST, REACQUIRED)


def _log(directory=''):
    return IncidentLog(directory, wall_clock=lambda: '2026-10-24T12:00:00')


def test_lifecycle_without_a_directory():
    log = _log()
    assert not log.is_open
    assert log.lost(1.0, (0.0, 0.0), 'odom') is None   # nothing open
    assert log.close(1.0) is None
    assert log.snapshot_path() is None

    first = log.sighted(10.0, (1.234, -2.0), 'map', snapshot=True)
    assert first == {'event': DETECTED, 'incident': 1, 't': 10.0,
                     'wall': '2026-10-24T12:00:00', 'frame': 'map',
                     'x': 1.23, 'y': -2.0, 'snapshot': None}
    assert log.is_open
    assert log.lost(12.0, (2.0, -2.0), 'map')['event'] == LOST
    again = log.sighted(14.0, None, '')
    assert again['event'] == REACQUIRED and 'x' not in again
    closed = log.close(30.5)
    assert closed['event'] == CLOSED
    assert (closed['duration'], closed['sightings'], closed['losses']) == (20.5, 2, 1)
    assert not log.is_open
    assert log.sighted(40.0, None, '')['incident'] == 2
    assert log.count == 2


def test_events_are_appended_to_the_log(tmp_path):
    directory = tmp_path / 'incidents'
    log = _log(str(directory))
    assert os.path.isdir(directory)                   # created
    event = log.sighted(5.0, (0.0, 1.0), 'odom', snapshot=True)
    assert event['snapshot'] == str(directory / 'incident_001_00.jpg')
    log.lost(6.0, (0.0, 1.0), 'odom')
    assert log.snapshot_path() == str(directory / 'incident_001_02.jpg')
    log.close(9.0)
    lines = (directory / LOG_NAME).read_text().splitlines()
    assert [json.loads(line)['event'] for line in lines] == [DETECTED, LOST, CLOSED]


def test_home_directory_is_expanded(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    log = IncidentLog('~/guard')
    assert log.directory == str(tmp_path / 'guard')
    assert log.sighted(0.0, None, '')['wall']         # real wall clock
