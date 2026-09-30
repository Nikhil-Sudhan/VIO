#!/usr/bin/env python3
"""Local read-only camera/trajectory viewer for one actual recording session."""
import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
cv2.setNumThreads(1)

PAGE = '''<!doctype html><meta charset="utf-8"><title>Pi camera + IMU — actual rig</title>
<style>body{font:18px system-ui;background:#101820;color:#e8eef4;margin:20px}h1{font-size:25px}#status{padding:12px;background:#264658;border-radius:8px}.row{display:flex;gap:16px;margin-top:16px;flex-wrap:wrap}.panel{flex:1;min-width:400px}img,canvas{width:100%;background:#18232f;border-radius:8px}pre{white-space:pre-wrap;font:17px monospace}.note{color:#f4c575}</style>
<h1>Raspberry Pi camera + IMU</h1><div id="status">Connecting to actual recording…</div>
<div class="row"><div class="panel"><img id="camera"><p id="caminfo"></p></div><div class="panel"><canvas id="path" width="700" height="500"></canvas><pre id="numbers">Waiting for estimator output</pre></div></div>
<p id="detail" class="note"></p><p class="note">Development display. No reference dataset or simulated poses. A drawn trajectory does not establish accuracy. No flight-controller output.</p>
<script>
const canvas=document.getElementById('path'),ctx=canvas.getContext('2d');
function draw(rows){ctx.clearRect(0,0,700,500);ctx.fillStyle='#e8eef4';ctx.font='18px system-ui';ctx.fillText('Estimated local XY trajectory (metres)',20,28);if(!rows.length){ctx.fillText('No VIO pose available yet',20,90);return}let xs=rows.map(r=>r[0]),ys=rows.map(r=>r[1]),lo=[Math.min(...xs),Math.min(...ys)],hi=[Math.max(...xs),Math.max(...ys)],scale=Math.min(600/Math.max(hi[0]-lo[0],.5),380/Math.max(hi[1]-lo[1],.5));function xy(p){return[350+(p[0]-(lo[0]+hi[0])/2)*scale,245-(p[1]-(lo[1]+hi[1])/2)*scale]}ctx.strokeStyle='#65c8ff';ctx.lineWidth=3;ctx.beginPath();rows.forEach((p,i)=>{let q=xy(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.stroke();let q=xy(rows.at(-1));ctx.fillStyle='#8bebac';ctx.beginPath();ctx.arc(...q,6,0,7);ctx.fill();ctx.fillStyle='#e8eef4';ctx.fillText('Scale: '+(100/scale).toFixed(2)+' m per 100 px',20,477)}
async function update(){try{let s=await(await fetch('/state',{cache:'no-store'})).json();document.getElementById('status').textContent=s.status;document.getElementById('detail').textContent=s.message;document.getElementById('camera').src='/frame.jpg?v='+s.image_sequence;document.getElementById('caminfo').textContent='Frame '+s.image_sequence+' · FAST corners '+s.corners+' · '+s.capture_status;document.getElementById('numbers').textContent=s.pose?'Position m: '+s.pose.position.map(x=>x.toFixed(3)).join(', ')+'\\nVelocity m/s: '+s.pose.velocity.map(x=>x.toFixed(3)).join(', ')+'\\nSpeed: '+s.pose.speed.toFixed(3)+' m/s\\nSaved poses: '+s.pose_count:s.message;draw(s.trajectory)}catch(e){document.getElementById('status').textContent='Display disconnected — no fresh data'}setTimeout(update,500)}update();
</script>'''


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    p.add_argument('--open', action='store_true')
    a = p.parse_args()
    folder = a.session.resolve()
    state = dict(status='Waiting for camera', capture_status='starting', message='No estimator output yet',
                 image_sequence=-1, corners=0, pose=None, pose_count=0, trajectory=[])
    jpeg = b''
    lock = threading.Lock()

    def refresh():
        nonlocal jpeg, state
        last_image = None
        while True:
            try:
                s = state.copy()
                info = json.loads((folder / 'session.json').read_text())
                s['capture_status'] = info['status']
                files = sorted((folder / 'images').glob('*.pgm'))
                if files:
                    latest = files[-2] if len(files) > 1 else files[0]
                    if latest != last_image:
                        im = cv2.imread(str(latest), cv2.IMREAD_GRAYSCALE)
                        if im is not None:
                            s['corners'] = len(cv2.FastFeatureDetector_create(20).detect(im))
                            ok, data = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 75])
                            if ok:
                                with lock: jpeg = data.tobytes()
                            last_image = latest
                            s['image_sequence'] = int(latest.stem)
                    image_age = max(0, time.time() - latest.stat().st_mtime)
                else: image_age = float('inf')
                pose_path = folder / 'vio/poses.csv'
                s['status'] = 'LIVE CAMERA / RECORDING — VIO not connected'
                s['message'] = 'Capturing actual camera and IMU data; no position estimate available.'
                if info.get('estimator_config'):
                    s['status'] = 'LIVE CAMERA — waiting for OpenVINS initialization and visual update'
                    s['message'] = 'No VIO position yet. Hold still first; movement is required for visual-inertial updates.'
                if pose_path.exists():
                    text = pose_path.read_text()
                    complete = text[:text.rfind('\n') + 1]
                    rows = list(csv.DictReader(io.StringIO(complete)))
                    if rows:
                        last = rows[-1]
                        position = [float(last[k]) for k in ['px', 'py', 'pz']]
                        velocity = [float(last[k]) for k in ['vx', 'vy', 'vz']]
                        if all(math.isfinite(x) for x in position + velocity):
                            speed = math.sqrt(sum(x*x for x in velocity))
                            s['pose'] = dict(position=position, velocity=velocity, speed=speed)
                            s['pose_count'] = len(rows)
                            s['message'] = 'Experimental state; physical accuracy has not passed validation.'
                            if not any(int(r['msckf_features']) > 0 for r in rows):
                                s['message'] = 'No accepted MSCKF visual constraints in this run. Displayed motion must not be treated as valid VIO.'
                            s['trajectory'] = [[float(r['px']), float(r['py'])] for r in rows[::max(1, len(rows)//2000)]]
                            s['status'] = 'EXPERIMENTAL VIO OUTPUT — accuracy not validated'
                            if speed > 3 or math.sqrt(sum(x*x for x in position)) > 10:
                                s['status'] = 'UNRELIABLE for this short handheld demo — excessive estimated motion'
                if info['status'] != 'recording':
                    s['status'] = 'SAVED CAPTURE: ' + info['status'] + ' — display is not live'
                    if info.get('errors'): s['message'] = str(info['errors'][-1])[-800:]
                elif image_age > 2:
                    s['status'] = 'STALE CAMERA — no fresh image'
                elif s['pose'] and time.time() - pose_path.stat().st_mtime > 2:
                    s['status'] = 'STALE ESTIMATOR — camera may be live, but position is not fresh'
                with lock: state = s
            except Exception as e:
                with lock: state['message'] = 'Waiting for complete data: ' + str(e)
            time.sleep(.5)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = self.path.split('?')[0]
            with lock:
                if route == '/': body, kind = PAGE.encode(), 'text/html; charset=utf-8'
                elif route == '/state': body, kind = json.dumps(state, allow_nan=False).encode(), 'application/json'
                elif route == '/frame.jpg': body, kind = jpeg, 'image/jpeg'
                else: self.send_error(404); return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError): pass

        def log_message(self, *args): pass

    threading.Thread(target=refresh, daemon=True).start()
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    url = f'http://127.0.0.1:{server.server_port}/'
    (folder / 'viewer-server.json').write_text(json.dumps(dict(pid=os.getpid(), url=url, scope='read-only development display'))+'\n')
    print(url, flush=True)
    if a.open:
        env = os.environ.copy()
        env.update(WAYLAND_DISPLAY='wayland-0', XDG_RUNTIME_DIR='/run/user/1000')
        env.pop('LD_LIBRARY_PATH', None)
        subprocess.Popen(['/usr/bin/chromium', '--ozone-platform=wayland', '--new-window', url],
                         env=env, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    server.serve_forever()


if __name__ == '__main__': main()
