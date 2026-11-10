// The robot's computer: a Web Worker running CPython (Pyodide, WebAssembly)
// with the project's ROS 2 nodes on a small fake of rclpy (py/rosfake.py).
// The page sends sensor data stamped with simulation time; the nodes answer
// with velocity commands and everything the page draws.

let pyodide = null;
let brain = null;

const post = (type, data = {}) => self.postMessage({ type, ...data });

async function boot({ pyodideURL, bundleURL, startTime, epoch }) {
  try {
    post('status', { stage: 'python', text: 'Starting Python (WebAssembly)' });
    importScripts(pyodideURL + 'pyodide.js');
    pyodide = await self.loadPyodide({ indexURL: pyodideURL });
    post('status', { stage: 'packages', text: 'Loading numpy and PyYAML' });
    await pyodide.loadPackage(['numpy', 'pyyaml']);
    post('status', { stage: 'code', text: 'Unpacking the robot software' });
    const bundle = await (await fetch(bundleURL)).arrayBuffer();
    pyodide.unpackArchive(bundle, 'zip', { extractDir: '/robot' });
    post('status', { stage: 'nodes', text: 'Starting the ROS 2 nodes' });
    pyodide.runPython(`
import sys
for path in ('/robot/py', '/robot/shims', '/robot/site', '/robot/pkg'):
    sys.path.insert(0, path)
`);
    brain = pyodide.pyimport('brain');
    const t0 = performance.now();
    const info = JSON.parse(brain.start('/robot/pkg', startTime));
    post('ready', { info, epoch, startMs: performance.now() - t0 });
  } catch (err) {
    post('error', { text: String(err && err.message ? err.message : err) });
  }
}

self.onmessage = (event) => {
  const m = event.data;
  if (m.type === 'boot') { boot(m); return; }
  if (m.type === 'restart' && brain) {
    try {
      post('ready', { info: JSON.parse(brain.start('/robot/pkg', m.startTime)), epoch: m.epoch });
    } catch (err) {
      post('error', { text: String(err && err.message ? err.message : err) });
    }
    return;
  }
  if (m.type === 'sense' && brain) {
    try {
      const t0 = performance.now();
      const out = brain.step(JSON.stringify(m.msg), m.scan || null);
      // Python's json writes NaN/Infinity bare; none is meant to reach here
      const data = JSON.parse(out.replace(/\b(NaN|-?Infinity)\b/g, 'null'));
      post('out', { out: data, epoch: m.epoch, ms: performance.now() - t0 });
    } catch (err) {
      post('error', { text: String(err && err.message ? err.message : err) });
    }
  }
};
