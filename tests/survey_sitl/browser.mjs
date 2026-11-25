// Capture actual rendered dashboard headings and screenshots through Chrome DevTools.
import { mkdir, writeFile, appendFile, access, mkdtemp, rm } from 'node:fs/promises';
import { setTimeout as sleep } from 'node:timers/promises';

import { execFileSync } from 'node:child_process';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

const [debugUrl, dashboard, output, option] = process.argv.slice(2);
const recordVideo = option === '--video';
if (recordVideo) execFileSync('ffmpeg', ['-version'], {stdio:'ignore'});
await mkdir(output, { recursive: true });
const page = await fetch(debugUrl + '/json/new?' + encodeURIComponent(dashboard), { method: 'PUT' })
  .then(reply => reply.json());
const socket = new WebSocket(page.webSocketDebuggerUrl);
await new Promise((resolve, reject) => {
  socket.addEventListener('open', resolve, { once: true });
  socket.addEventListener('error', reject, { once: true });
});
let nextId = 0;
const pending = new Map();
socket.addEventListener('message', ({ data }) => {
  const { id, result, error } = JSON.parse(data);
  if (!pending.has(id)) return;
  const { resolve, reject } = pending.get(id);
  pending.delete(id);
  if (error) reject(Error(error.message));
  else resolve(result);
});
function call(method, params = {}) {
  return new Promise((resolve, reject) => {
    const id = ++nextId;
    pending.set(id, { resolve, reject });
    socket.send(JSON.stringify({ id, method, params }));
  });
}

try {
  await call('Emulation.setDeviceMetricsOverride', {
    width: 1280, height: 1060, deviceScaleFactor: 1, mobile: false,
  });
  const videoDir = recordVideo ? await mkdtemp(join(tmpdir(), 'survey-dashboard-')) : null;
  const frames = [];
  let recording = false, finished = false;
  let previous;
  let count = 0;
  while (!(await access(output + '/stop').then(() => true, () => false))) {
    const evaluation = await call('Runtime.evaluate', {
      expression: `(() => {
        const heading = document.getElementById('replay-heading');
        const message = document.getElementById('replay-message');
        if (!heading?.checkVisibility()) return null;
        const survey = lastStatus?.survey;
        const image = document.getElementById('processed');
        const thumb = document.getElementById('sighting-photo');
        return {heading: heading.innerText, message: message.innerText,
          feed: document.getElementById('feed-label')?.textContent,
          feedVisible: Boolean(image?.naturalWidth && image.checkVisibility()),
          pinnedVisible: Boolean(thumb?.naturalWidth && thumb.checkVisibility()),
          hold_photo_ns: survey?.hold_photo_ns ?? 0, sighting_ns: survey?.sighting_ns, first_en: survey?.first_en, hold_en: survey?.hold_en,
          trail_points: survey?.trail.length,
          map_legend: document.querySelector('.map-key')?.innerText.trim().split(String.fromCharCode(10))};
      })()`,
      returnByValue: true,
    });
    const current = evaluation.result?.value;
    const visibleState = current && [current.heading,current.feed,current.feedVisible,current.pinnedVisible,Boolean(current.hold_en)].join('|');
    if (current && previous !== visibleState) {
      previous = visibleState;
      const screenshot = String(++count).padStart(2, '0') + '.png';
      const record = { observed_at: new Date().toISOString(), ...current, screenshot };
      const { data } = await call('Page.captureScreenshot', { format: 'png' });
      await writeFile(output + '/' + screenshot, Buffer.from(data, 'base64'));
      await appendFile(output + '/rendered.jsonl', JSON.stringify(record) + '\n');
    }
    if (recordVideo && !finished && current) {
      if (current.heading === 'Demo · Taking off') recording = true;
      const now = performance.now();
      if (recording && (!frames.length || now - frames.at(-1).time >= 450)) {
        const {data} = await call('Page.captureScreenshot', {format:'jpeg', quality:82});
        const file = 'frame-' + frames.length + '.jpg';
        await writeFile(join(videoDir, file), Buffer.from(data, 'base64'));
        frames.push({file, time:now});
        if (current.heading === 'Demo complete' || current.heading.startsWith('Return ')) finished = true;
      }
    }
    await sleep(100);
  }
  if (recordVideo && frames.length) {
    try {
      const timeline = frames.map(({file,time}, i) => {
        const duration = i + 1 < frames.length ? (frames[i+1].time - time)/1000 : 1;
        return "file '" + file + "'\nduration " + duration.toFixed(3);
      }).join('\n') + "\nfile '" + frames.at(-1).file + "'\n";
      await writeFile(join(videoDir, 'frames.txt'), timeline);
      execFileSync('ffmpeg', ['-nostdin','-loglevel','error','-f','concat','-safe','1','-i',
        join(videoDir,'frames.txt'),'-vf','fps=8','-c:v','libx264','-preset','veryfast',
        '-pix_fmt','yuv420p','-movflags','+faststart',output+'/replay.mp4']);
    } finally { await rm(videoDir, {recursive:true}); }
  }
} finally {
  socket.close();
  await fetch(debugUrl + '/json/close/' + page.id);
}
