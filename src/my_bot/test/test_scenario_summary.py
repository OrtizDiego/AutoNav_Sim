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

"""Phase 7: the Markdown summary of several scenario reports."""

import json

from my_bot import scenario_summary as ss

PASSED = {'scenario': 'walk_in', 'outcome': 'completed', 'passed': True,
          'failures': [], 'real_time_factor': 0.8, 'time_to_detect_s': 1.2,
          'in_view_pct': 90.0, 'tracked_in_view_pct': 95.5,
          'track_breaks_per_min': 0.5, 'false_alarms': 0, 'track_switches': 0,
          'patrol_coverage_pct': 40.1}
FAILED = {**PASSED, 'scenario': 'sprint', 'passed': False,
          'failures': ['track_breaks_per_min 3.0 (max 1.0)'],
          'track_breaks_per_min': 3.0, 'time_to_detect_s': None}


def _write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(data if isinstance(data, str) else json.dumps(data))
    return str(path)


def test_result():
    assert ss.result(PASSED) == 'PASS'
    assert ss.result(FAILED) == 'FAIL'
    assert ss.result({**PASSED, 'outcome': 'timeout'}) == 'timeout'
    assert ss.result(None) == 'no report'


def test_load(tmp_path):
    assert ss.load(_write(tmp_path / 'a' / 'report.json', PASSED)) == ('walk_in', PASSED)
    # Unnamed or unreadable: named after the directory (the CI artifact)
    unnamed = {**PASSED, 'scenario': ''}
    assert ss.load(_write(tmp_path / 'hide' / 'report.json', unnamed))[0] == 'hide'
    assert ss.load(_write(tmp_path / 'broken' / 'report.json', '{')) == ('broken', None)
    assert ss.load(_write(tmp_path / 'list' / 'report.json', '[]')) == ('list', None)
    assert ss.load(str(tmp_path / 'crossing' / 'report.json')) == ('crossing', None)


def test_summary_table():
    lines = ss.summary_table([('walk_in', PASSED), ('sprint', FAILED),
                              ('crossing', None)]).splitlines()
    assert lines[0].startswith('| Scenario | Result | RTF | Detect |')
    assert lines[1] == '|' + '---|' * (2 + len(ss.COLUMNS))
    assert lines[2] == ('| walk_in | PASS | 0.8 | 1.2 s | 90.0 % | 95.5 % | 0.5/min'
                        ' | 0 | 0 | 40.1 % |')
    assert lines[3].startswith('| sprint | FAIL | 0.8 | – |')
    assert lines[4] == '| crossing | no report |' + ' – |' * len(ss.COLUMNS)
    assert lines[5:] == ['', 'Missed:', '- sprint: track_breaks_per_min 3.0 (max 1.0)']


def test_no_missed_section_when_all_pass():
    assert 'Missed' not in ss.summary_table([('walk_in', PASSED)])


def test_cli(tmp_path, capsys):
    path = _write(tmp_path / 'r' / 'report.json', PASSED)
    assert ss.main([path, str(tmp_path / 'hide' / 'report.json')]) == 0
    out = capsys.readouterr().out
    assert '| walk_in | PASS |' in out and '| hide | no report |' in out
    assert ss.main([]) == 2


def test_latest_reports(tmp_path):
    for run in ('walk_in_20261028-090000', 'walk_in_20261028-100000',
                'walk_in_extra_20261028-110000', 'sprint_20261027-080000', 'notes'):
        (tmp_path / run).mkdir()
    assert ss.latest_reports(str(tmp_path), ['walk_in', 'sprint', 'hide']) == [
        str(tmp_path / 'walk_in_20261028-100000' / 'report.json'),
        str(tmp_path / 'sprint_20261027-080000' / 'report.json'),
        str(tmp_path / 'hide' / 'report.json')]
    assert ss.latest_reports(str(tmp_path / 'missing'), ['hide']) == [
        str(tmp_path / 'missing' / 'hide' / 'report.json')]


def test_cli_latest(tmp_path, capsys):
    _write(tmp_path / 'walk_in_20261028-100000' / 'report.json', PASSED)
    assert ss.main(['--latest', str(tmp_path), 'walk_in', 'hide']) == 0
    out = capsys.readouterr().out
    assert '| walk_in | PASS |' in out and '| hide | no report |' in out
    assert ss.main(['--latest', str(tmp_path)]) == 2
