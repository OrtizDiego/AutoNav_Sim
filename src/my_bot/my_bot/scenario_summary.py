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

"""One Markdown table for several scenario reports (Phase 7).

    python3 -m my_bot.scenario_summary <report.json>...
    python3 -m my_bot.scenario_summary --latest <dir> <scenario>...

The CI scenario workflow appends it to the job summary; ``make scenarios``
prints it after the last run (``--latest``: the newest
``<dir>/<scenario>_<date-time>/report.json`` of each scenario, the layout
scenario.launch.py writes). A path that cannot be read becomes a
"no report" row named after its directory, so a run that crashed before the
evaluator wrote anything still shows up.

Only the standard library: the CI summary job runs it without ROS, numpy or
OpenCV.
"""

import json
import os
import re
import sys
from typing import List, Optional, Sequence, Tuple

# Header, report key, unit suffix.
COLUMNS = (
    ('RTF', 'real_time_factor', ''),
    ('Detect', 'time_to_detect_s', ' s'),
    ('In view', 'in_view_pct', ' %'),
    ('Tracked in view', 'tracked_in_view_pct', ' %'),
    ('Breaks', 'track_breaks_per_min', '/min'),
    ('False alarms', 'false_alarms', ''),
    ('Switches', 'track_switches', ''),
    ('Coverage', 'patrol_coverage_pct', ' %'),
)


def load(path: str) -> Tuple[str, Optional[dict]]:
    """Return (scenario name, report or None if it cannot be read)."""
    fallback = os.path.basename(os.path.dirname(os.path.abspath(path))) or path
    try:
        with open(path) as f:
            report = json.load(f)
    except (OSError, ValueError):
        return fallback, None
    if not isinstance(report, dict):
        return fallback, None
    return report.get('scenario') or fallback, report


def latest_reports(directory: str, names: Sequence[str]) -> List[str]:
    """Report path of each scenario's newest run in ``directory``.

    A scenario without a run gets ``<directory>/<name>/report.json``, which
    does not exist and so becomes its "no report" row.
    """
    try:
        entries = sorted(os.listdir(directory))
    except OSError:
        entries = []
    paths = []
    for name in names:
        runs = [e for e in entries if re.fullmatch(re.escape(name) + r'_\d{8}-\d{6}', e)]
        paths.append(os.path.join(directory, runs[-1] if runs else name, 'report.json'))
    return paths


def result(report: Optional[dict]) -> str:
    """PASS / FAIL against the run's targets, or why it was not scored."""
    if report is None:
        return 'no report'
    if report.get('outcome') != 'completed':
        return str(report.get('outcome'))
    return 'PASS' if report.get('passed') else 'FAIL'


def _cell(value, unit: str) -> str:
    return '–' if value is None else f'{value}{unit}'


def summary_table(runs: Sequence[Tuple[str, Optional[dict]]]) -> str:
    """Markdown table: one row per run, then the missed targets."""
    header = ['Scenario', 'Result'] + [c[0] for c in COLUMNS]
    lines = ['| ' + ' | '.join(header) + ' |',
             '|' + '---|' * len(header)]
    missed: List[str] = []
    for name, report in runs:
        cells = [name, result(report)]
        cells += [_cell((report or {}).get(key), unit) for _, key, unit in COLUMNS]
        lines.append('| ' + ' | '.join(cells) + ' |')
        missed += [f'- {name}: {f}' for f in (report or {}).get('failures', [])]
    if missed:
        lines += ['', 'Missed:'] + missed
    return '\n'.join(lines)


def main(argv=None) -> int:
    """Print the table for the reports named on the command line."""
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['--latest'] and len(argv) >= 3:
        argv = latest_reports(os.path.expanduser(argv[1]), argv[2:])
    elif not argv or argv[0] == '--latest':
        print('usage: python3 -m my_bot.scenario_summary <report.json>...\n'
              '       python3 -m my_bot.scenario_summary --latest <dir> <scenario>...')
        return 2
    print(summary_table([load(path) for path in argv]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
