#!/usr/bin/env python3
"""Assemble the web demo into a static site (default: demo/_site).

    npm ci                       # three.js, once
    python3 build.py             # -> _site/
    python3 build.py --serve     # build, then http://localhost:8000

The site is static: index.html, css/, js/, three.js (vendor/), the map
(assets/), the recorded runs (replays/, from demo/replays) and robot.zip, the robot's software for the in-browser Python:
my_bot's modules, its config and map, demo/py and pure-Python py_trees.
Nothing in src/ is copied by hand or changed; the demo always runs the
package as it is in the repository.
"""

import argparse
import functools
import http.server
import io
import json
import os
import shutil
import subprocess
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(os.path.dirname(HERE), 'src', 'my_bot')

PYODIDE = 'https://cdn.jsdelivr.net/pyodide/v0.26.4/full/'
# Pure-Python wheels unpacked into the bundle (py_trees needs pydot, which
# needs pyparsing). ROS 2 Humble ships py_trees 2.2; the tree API is the same.
WHEELS = ['py_trees==2.6.0', 'pydot==4.0.1', 'pyparsing==3.3.3']
# What the browser's nodes import (no YOLO, no OpenCV tracker, no launch)
MODULES = [
    '__init__', 'clearance_map', 'conversions', 'follow_control', 'guard_markers',
    'guard_tree', 'guard_zones', 'incident_log', 'markers', 'node_utils',
    'person_controller', 'room_sweep', 'scenario_script', 'security_guard_bt',
    'sensor_fusion', 'target_estimate', 'target_tracker', 'track_filter',
]
CONFIG = ['behavior_params.yaml', 'guard_zones.yaml']
MAP = 'my_map'


def read_map_meta():
    """image, resolution and origin of the map (flat `key: value` YAML)."""
    meta = {}
    with open(os.path.join(PKG, 'maps', MAP + '.yaml')) as f:
        for line in f:
            key, sep, value = line.partition(':')
            if sep and not key.strip().startswith('#'):
                meta[key.strip()] = value.strip()
    return {'image': meta['image'], 'resolution': float(meta['resolution']),
            'origin': json.loads(meta['origin'])[:2]}


def fetch_wheels(cache):
    os.makedirs(cache, exist_ok=True)
    paths = []
    for spec in WHEELS:
        name, version = spec.split('==')
        found = [f for f in os.listdir(cache)
                 if f.lower().startswith(f'{name.lower()}-{version}-') and f.endswith('.whl')]
        if not found:
            subprocess.check_call([
                sys.executable, '-m', 'pip', 'download', '--quiet', '--no-deps',
                '--only-binary', ':all:', '--dest', cache, spec])
            found = [f for f in os.listdir(cache)
                     if f.lower().startswith(f'{name.lower()}-{version}-') and f.endswith('.whl')]
        paths.append(os.path.join(cache, found[0]))
    return paths


def robot_bundle(wheels):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in MODULES:
            z.write(os.path.join(PKG, 'my_bot', name + '.py'), f'pkg/my_bot/{name}.py')
        for name in CONFIG:
            z.write(os.path.join(PKG, 'config', name), f'pkg/config/{name}')
        for ext in ('.yaml', '.pgm'):
            z.write(os.path.join(PKG, 'maps', MAP + ext), f'pkg/maps/{MAP}{ext}')
        for name in sorted(os.listdir(os.path.join(HERE, 'py'))):
            if name.endswith('.py'):
                z.write(os.path.join(HERE, 'py', name), f'py/{name}')
        for name in sorted(os.listdir(os.path.join(HERE, 'py', 'shims'))):
            if name.endswith('.py'):
                z.write(os.path.join(HERE, 'py', 'shims', name), f'shims/{name}')
        for wheel in wheels:
            with zipfile.ZipFile(wheel) as w:
                for info in w.infolist():
                    if '.dist-info/' in info.filename or info.is_dir():
                        continue
                    z.writestr('site/' + info.filename, w.read(info))
    return buf.getvalue()


def vendor_three(out):
    build = os.path.join(HERE, 'node_modules', 'three', 'build')
    controls = os.path.join(HERE, 'node_modules', 'three', 'examples', 'jsm',
                            'controls', 'OrbitControls.js')
    if not os.path.exists(os.path.join(build, 'three.module.min.js')):
        sys.exit('three.js is missing: run `npm ci` in demo/ first')
    os.makedirs(os.path.join(out, 'vendor'), exist_ok=True)
    shutil.copy(os.path.join(build, 'three.module.min.js'),
                os.path.join(out, 'vendor', 'three.module.min.js'))
    core = os.path.join(build, 'three.core.min.js')  # newer releases split the build
    if os.path.exists(core):
        shutil.copy(core, os.path.join(out, 'vendor', 'three.core.min.js'))
    shutil.copy(controls, os.path.join(out, 'vendor', 'OrbitControls.js'))
    with open(os.path.join(HERE, 'node_modules', 'three', 'LICENSE')) as f:
        with open(os.path.join(out, 'vendor', 'THREE-LICENSE.txt'), 'w') as g:
            g.write(f.read())


# Order of the recorded runs in the page's picker (others follow, by name)
REPLAY_ORDER = ['walk_in', 'sprint', 'hide', 'crossing']


def copy_replays(sources, out):
    """Copy every <source>/<name>/replay.json (+ images/); write replays/index.json."""
    runs = []
    found = {}
    for src in sources:
        for name in (os.listdir(src) if os.path.isdir(src) else []):
            found.setdefault(name, os.path.join(src, name))
    names = sorted(found, key=lambda n: (
        REPLAY_ORDER.index(n) if n in REPLAY_ORDER else len(REPLAY_ORDER), n))
    for name in names:
        replay = os.path.join(found[name], 'replay.json')
        if not os.path.isfile(replay):
            continue
        with open(replay) as f:
            data = json.load(f)
        shutil.copytree(found[name], os.path.join(out, 'replays', name))
        runs.append({'name': name, 'scenario': data.get('scenario', name),
                     'result': data.get('result', ''), 'duration': data.get('duration', 0),
                     'description': data.get('description', '')})
    if runs:
        with open(os.path.join(out, 'replays', 'index.json'), 'w') as f:
            json.dump({'replays': runs}, f)
    return runs


def build(out, pyodide_url, replays=None):
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(os.path.join(out, 'assets'))
    shutil.copy(os.path.join(HERE, 'index.html'), out)
    for folder in ('css', 'js'):
        shutil.copytree(os.path.join(HERE, folder), os.path.join(out, folder))
    vendor_three(out)

    meta = read_map_meta()
    shutil.copy(os.path.join(PKG, 'maps', meta['image']), os.path.join(out, 'assets'))
    with open(os.path.join(out, 'assets', 'world.json'), 'w') as f:
        json.dump(meta, f)
    bundle = robot_bundle(fetch_wheels(os.path.join(HERE, '.wheels')))
    with open(os.path.join(out, 'robot.zip'), 'wb') as f:
        f.write(bundle)
    with open(os.path.join(out, 'config.json'), 'w') as f:
        json.dump({'pyodide': pyodide_url}, f)
    runs = copy_replays(replays or [os.path.join(HERE, 'replays')], out)
    open(os.path.join(out, '.nojekyll'), 'w').close()
    print(f'built {out} (robot.zip {len(bundle) / 1024:.0f} kB, '
          f'{len(runs)} recorded run(s), Pyodide from {pyodide_url})')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--out', default=os.path.join(HERE, '_site'))
    parser.add_argument('--pyodide', default=PYODIDE,
                        help='URL of a Pyodide distribution, ending in /')
    parser.add_argument('--replays', action='append',
                        help='directory of recorded runs (<name>/replay.json, from make demo-bag); '
                             'repeatable; default demo/replays')
    parser.add_argument('--serve', action='store_true', help='serve the site after building')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    build(args.out, args.pyodide, args.replays)
    if args.serve:
        handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=args.out)
        print(f'http://localhost:{args.port}  (Ctrl+C to stop)')
        http.server.ThreadingHTTPServer(('', args.port), handler).serve_forever()


if __name__ == '__main__':
    main()
