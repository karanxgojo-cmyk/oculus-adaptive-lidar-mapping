const { spawn } = require('child_process');
const fs = require('fs');

async function run() {
  console.log('=== LAUNCHING HEADLESS CHROME FOR BROWSER CDP VERIFICATION ===');
  const chromePath = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';
  const chromeProc = spawn(chromePath, [
    '--headless=new',
    '--remote-debugging-port=9222',
    '--disable-gpu',
    '--no-sandbox',
    '--window-size=1600,900',
    'http://127.0.0.1:8050/'
  ]);

  await new Promise(r => setTimeout(r, 2500));

  const resp = await fetch('http://127.0.0.1:9222/json');
  const targets = await resp.json();
  const pageTarget = targets.find(t => t.type === 'page') || targets[0];
  console.log('Found page target:', pageTarget.url);

  const ws = new WebSocket(pageTarget.webSocketDebuggerUrl);
  let msgId = 1;
  const pending = new Map();
  const consoleLogs = [];

  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.id && pending.has(data.id)) {
      pending.get(data.id)(data);
      pending.delete(data.id);
    }
    if (data.method === 'Runtime.consoleAPICalled') {
      const text = data.params.args.map(a => a.value || a.description).join(' ');
      consoleLogs.push(`[${data.params.type}] ${text}`);
    }
    if (data.method === 'Runtime.exceptionThrown') {
      consoleLogs.push(`[EXCEPTION] ${JSON.stringify(data.params.exceptionDetails)}`);
    }
  };

  function send(method, params = {}) {
    return new Promise((resolve) => {
      const id = msgId++;
      pending.set(id, resolve);
      ws.send(JSON.stringify({ id, method, params }));
    });
  }

  await new Promise(r => { ws.onopen = r; });
  console.log('Connected to Chrome DevTools Protocol WebSocket.');

  await send('Runtime.enable');
  await send('Page.enable');

  console.log('Waiting 4.5 seconds for simulation render loop to pace and measure metrics...');
  await new Promise(r => setTimeout(r, 4500));

  const evalRes = await send('Runtime.evaluate', {
    expression: `(() => {
      const canvas = document.getElementById('lidar-canvas');
      const ctx = canvas ? canvas.getContext('2d') : null;
      let nonZeroPixels = 0;
      if (ctx) {
        const img = ctx.getImageData(0, 0, Math.min(300, canvas.width), Math.min(300, canvas.height));
        for (let i = 0; i < img.data.length; i += 4) {
          if (img.data[i] !== 3 || img.data[i+1] !== 5 || img.data[i+2] !== 10) {
            nonZeroPixels++;
          }
        }
      }

      const em = window.encounterManager;
      const ents = em ? em.getActiveEntities() : [];
      const vCount = ents.filter(e => e.type === 'vehicle').length;
      const pCount = ents.filter(e => e.type === 'pedestrian').length;
      const aCount = ents.filter(e => e.type === 'animal').length;
      const oCount = ents.filter(e => e.type === 'obstacle').length;

      const ego = window.osmRouter ? window.osmRouter.getEgoPose(0) : { x: 0, y: 0, yaw: 0 };
      const animalEnt = ents.find(e => e.type === 'animal');
      const animalInspector = animalEnt ? em.findEntityAt({ x: animalEnt.x, y: animalEnt.y }, ego) : null;

      return {
        measuredFps: document.getElementById('metric-fps')?.textContent,
        measuredLatency: document.getElementById('metric-latency')?.textContent,
        dynamicObjectsText: document.getElementById('metric-dynamic')?.textContent,
        totalEncounterEntities: ents.length,
        vehicles: vCount,
        pedestrians: pCount,
        animals: aCount,
        obstacles: oCount,
        nonZeroPixels,
        animalInspector,
        roadName: document.getElementById('route-road-name')?.textContent || document.getElementById('phase-title')?.textContent
      };
    })()`,
    returnByValue: true
  });

  if (evalRes.result?.exceptionDetails) {
    console.error('Runtime.evaluate exception:', JSON.stringify(evalRes.result.exceptionDetails));
  }
  const state = evalRes.result?.result?.value || evalRes.result?.value;
  if (!state) {
    console.error('evalRes dump:', JSON.stringify(evalRes));
  }
  console.log('\n=== REAL BROWSER MEASUREMENT RESULTS ===');
  console.log('Measured FPS:', state.measuredFps);
  console.log('Measured Latency:', state.measuredLatency);
  console.log('UI Dynamic Objects Count:', state.dynamicObjectsText);
  console.log('Active Encounter Entities:', state.totalEncounterEntities);
  console.log(`  - Vehicles: ${state.vehicles}`);
  console.log(`  - Pedestrians: ${state.pedestrians}`);
  console.log(`  - Animals: ${state.animals}`);
  console.log(`  - Static Obstacles: ${state.obstacles}`);
  console.log('Active Rendered Pixels:', state.nonZeroPixels);
  console.log('Current Roadway:', state.roadName);
  if (state.animalInspector) {
    console.log('Animal CAD Inspector Check:', {
      name: state.animalInspector.class_name,
      semantic_class: state.animalInspector.semantic_class,
      confidence: `${(state.animalInspector.confidence * 100).toFixed(0)}%`,
      base_elev: `${state.animalInspector.base_elev.toFixed(2)}m`,
      top_elev: `${state.animalInspector.top_elev.toFixed(2)}m`,
      height: `${state.animalInspector.height.toFixed(2)}m`,
      dimensions: state.animalInspector.dimensions
    });
  }

  const screenshotRes = await send('Page.captureScreenshot', { format: 'png' });
  const buf = Buffer.from(screenshotRes.result.data, 'base64');
  const screenshotPath = 'C:\\Users\\hp\\.gemini\\antigravity\\brain\\6420c690-a1d4-47e4-b98b-8dc72f0051c1\\local_verified_final.png';
  fs.writeFileSync(screenshotPath, buf);
  console.log(`Screenshot saved to: ${screenshotPath}`);

  const errors = consoleLogs.filter(l => l.toLowerCase().includes('error') || l.includes('EXCEPTION') || l.includes('TypeError'));
  console.log(`\nBrowser Console Errors: ${errors.length}`);
  if (errors.length > 0) {
    errors.forEach(e => console.error('  ', e));
  } else {
    console.log('  [PASS] 0 console errors detected in real browser session!');
  }

  ws.close();
  chromeProc.kill();
  process.exit(errors.length > 0 ? 1 : 0);
}

run().catch(err => {
  console.error('Test execution error:', err);
  process.exit(1);
});
