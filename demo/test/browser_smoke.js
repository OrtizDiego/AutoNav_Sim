// Headless-browser check of the built site: the page loads without errors,
// the robot's software boots under Pyodide, patrols, and detects the person.
//
//   python3 build.py && node test/browser_smoke.js [site directory]
//
// With recorded runs in the site it also plays one and goes back to live
// (CI adds a synthetic one: test/fake_replay.py).
//
// Needs Playwright with Chromium. Exits non-zero on failure.

const http = require('http');
const fs = require('fs');
const path = require('path');
const { chromium } = require('playwright');

const site = path.resolve(process.argv[2] || path.join(__dirname, '..', '_site'));
const TYPES = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.css': 'text/css',
  '.json': 'application/json', '.wasm': 'application/wasm', '.zip': 'application/zip',
};

function serve() {
  const server = http.createServer((req, res) => {
    let file = path.join(site, decodeURIComponent(req.url.split('?')[0]));
    if (!file.startsWith(site)) { res.writeHead(403); res.end(); return; }
    if (fs.existsSync(file) && fs.statSync(file).isDirectory()) file = path.join(file, 'index.html');
    fs.readFile(file, (err, data) => {
      if (err) { res.writeHead(404); res.end(); return; }
      res.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
      res.end(data);
    });
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

(async () => {
  const server = await serve();
  const url = `http://127.0.0.1:${server.address().port}/`;
  const browser = await chromium.launch({
    args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
  });
  const problems = [];
  let failed = null;
  try {
    const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
    page.on('pageerror', (e) => problems.push(`page error: ${e}`));
    page.on('console', (m) => { if (m.type() === 'error') problems.push(`console: ${m.text()}`); });
    page.on('requestfailed', (r) => problems.push(`request failed: ${r.url()}`));
    await page.goto(url);
    await page.waitForFunction(() => window.__demo && window.__demo.ready, null, { timeout: 120000 });
    console.log('robot software running');
    await page.waitForFunction(() => window.__demo.viz && window.__demo.viz.state === 'PatrolProtocol',
      null, { timeout: 20000 });
    console.log('patrolling');
    // Put the person 3 m in front of the robot, on open floor
    await page.evaluate(() => {
      const { sim } = window.__demo;
      sim.autopilot = false;
      sim.robot.x = 0; sim.robot.y = 0; sim.robot.yaw = 0;
      sim.person.x = 3; sim.person.y = 0;
    });
    await page.waitForFunction(() => window.__demo.viz.state === 'IntruderProtocol', null, { timeout: 20000 });
    console.log('intruder detected, following');
    const text = await page.evaluate(() => document.getElementById('events').innerText);
    if (!/Intruder detected/.test(text)) throw new Error(`no event in the panel: ${text}`);
    const lit = await page.evaluate(() => {
      const ctx = document.getElementById('robot-cam').getContext('2d');
      const d = ctx.getImageData(0, 0, 320, 240).data;
      let sum = 0;
      for (let i = 0; i < d.length; i += 4) sum += d[i] + d[i + 1] + d[i + 2];
      return sum / (d.length / 4);
    });
    if (lit < 30) throw new Error('the robot camera panel is black');
    console.log('camera panel drawn');

    // Recorded runs, when the site has any
    const hasReplays = await page.evaluate(() => !document.getElementById('modes').hidden);
    if (hasReplays) {
      await page.click('[data-mode=replay]');
      await page.waitForFunction(() => window.__demo.replay && window.__demo.replay.t > 1,
        null, { timeout: 30000 });
      const r = await page.evaluate(() => ({
        state: document.getElementById('state-pill').textContent,
        clock: document.getElementById('rp-clock').textContent,
        frames: window.__demo.replay.data.frames.t.length,
      }));
      if (!r.frames || r.state === 'starting') throw new Error(`replay not playing: ${JSON.stringify(r)}`);
      console.log(`recorded run playing (${r.clock}, ${r.state})`);
      await page.click('[data-mode=live]');
      await page.waitForFunction(() => window.__demo.ready && window.__demo.viz, null, { timeout: 60000 });
      console.log('back to live');
    } else {
      console.log('no recorded runs in this build');
    }
  } catch (err) {
    failed = err;
  }
  await browser.close();
  server.close();
  if (problems.length) console.error(problems.join('\n'));
  if (failed || problems.length) {
    console.error(`FAILED: ${failed ? failed.message : 'errors on the page'}`);
    process.exit(1);
  }
  console.log('OK');
})();
