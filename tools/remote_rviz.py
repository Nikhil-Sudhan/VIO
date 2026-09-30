#!/usr/bin/env python3
"""Small LAN remote for the real RViz desktop and bounded native controller."""
import argparse
import io
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT/'data/native-control'
ENV = os.environ | {'WAYLAND_DISPLAY': 'wayland-0', 'XDG_RUNTIME_DIR': '/run/user/1000'}
PAGE = '''<!doctype html><html><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Live camera + IMU — RViz</title><style>
body{background:#141921;color:#eee;font:18px system-ui;margin:20px auto;max-width:1100px;padding:0 15px}
button,a{padding:12px;margin:6px 10px 6px 0;font-size:18px}button{cursor:pointer}a{color:#7cd9fa}
img{width:100%;background:#000}#status{padding:12px;background:#273342;white-space:pre-wrap}
</style><h2>Camera + IMU VIO — live RViz</h2>
<p>This is the Pi's actual desktop. Close the Pi Connect tab to reduce network load.</p>
<button onclick="action('start')">Start 60-second demo</button>
<button onclick="action('stop')">Stop and save</button>
<a href="/video" id="video">Download latest screen video</a>
<p>After starting: rest the rig for 5 seconds, then slide slowly sideways and back, keeping the grid visible.</p>
<div id="status">Connecting…</div><p id="screenstate"></p><img id="screen" alt="Actual Pi desktop">
<p>Manual prototype; calibration and distance accuracy remain unverified. The printed grid supplies visual features; position is estimated by OpenVINS using the camera and IMU.</p>
<script>
const img=document.getElementById('screen'),st=document.getElementById('status');
img.onload=()=>{document.getElementById('screenstate').textContent='Desktop received '+new Date().toLocaleTimeString();setTimeout(frame,400)};
img.onerror=()=>{document.getElementById('screenstate').textContent='Desktop delayed — retrying';setTimeout(frame,1000)};
function frame(){img.src='/screen.jpg?t='+Date.now()}frame();
async function action(action){try{let r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});if(!r.ok)throw Error(await r.text());st.textContent='Command sent: '+action}catch(e){st.textContent=e}}
async function state(){try{let x=await(await fetch('/state')).json();st.textContent=x.message;document.getElementById('video').style.display=x.video?'inline-block':'none'}catch(e){st.textContent='Connection interrupted — retrying'}setTimeout(state,1000)}state();
</script></html>'''
lock = threading.Lock()
cached = b''
captured = 0.

def status():
    videos = sorted((Path.home()/'Videos').glob('VIO-demo-*.mp4'))
    result = {'video': bool(videos), 'message': 'Ready. Click Start when the rig is resting.'}
    try:
        folder = Path((CONTROL/'current-session.txt').read_text().strip())
        session = json.loads((folder/'session.json').read_text())
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if session.get('boot_id') != boot:
            return result
        state = session['status']
        result['message'] = 'Capture: '+state
        if state in ('recording', 'starting'):
            snap = json.loads((folder/'vio/display.json').read_text())
            age = (time.clock_gettime_ns(time.CLOCK_BOOTTIME)-snap['emitted_boot_ns'])/1e9
            count = sum(len(snap.get(k, [])) for k in ('msckf','slam','aruco'))
            if age > 2:
                result['message'] += '\nEstimator delayed — do not move.'
            elif snap.get('stationary_ready') or snap.get('initialized'):
                result['message'] += f'\nInitialized — move slowly. Visual landmarks: {count}. Output age: {age:.2f}s.'
            else:
                result['message'] += '\nInitializing — keep the rig still.'
        elif state == 'failed':
            result['message'] += '\nRun stopped with an error; saved data and video are diagnostic only.'
        else:
            result['message'] += '\nStopped. The desktop shows the saved result; it is not live sensor data.'
    except (OSError,ValueError,KeyError):
        pass
    return result

class Handler(BaseHTTPRequestHandler):
    def send(self, body, kind):
        self.send_response(200)
        self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store')
        self.end_headers()
        try:self.wfile.write(body)
        except (BrokenPipeError,ConnectionResetError):pass

    def do_GET(self):
        global cached,captured
        route=urlparse(self.path).path
        if route=='/':return self.send(PAGE.encode(),'text/html; charset=utf-8')
        if route=='/state':return self.send(json.dumps(status()).encode(),'application/json')
        if route=='/screen.jpg':
            try:
                with lock:
                    if time.monotonic()-captured>.35:
                        pixels=subprocess.run(['grim','-s','0.8','-t','ppm','-'],env=ENV,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True,timeout=3).stdout
                        output=io.BytesIO()
                        Image.open(io.BytesIO(pixels)).save(output,format='JPEG',quality=65)
                        cached=output.getvalue()
                        captured=time.monotonic()
                    body=cached
                return self.send(body,'image/jpeg')
            except (OSError,subprocess.SubprocessError):return self.send_error(503,'Desktop unavailable')
        if route=='/video':
            videos=sorted((Path.home()/'Videos').glob('VIO-demo-*.mp4'))
            if not videos:return self.send_error(404,'No video saved')
            path=videos[-1]
            # A metadata sidecar is written only after the encoder closes.
            if not path.with_suffix('.json').exists():return self.send_error(409,'Video still recording; stop and save first')
            self.send_response(200);self.send_header('Content-Type','video/mp4')
            self.send_header('Content-Disposition','attachment; filename="'+path.name+'"')
            self.send_header('Content-Length',str(path.stat().st_size));self.end_headers()
            try:
                with path.open('rb') as f:
                    while data:=f.read(65536):self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
            return
        self.send_error(404)

    def do_POST(self):
        if self.path!='/control':return self.send_error(404)
        origin=self.headers.get('Origin')
        if origin and urlparse(origin).netloc!=self.headers.get('Host'):return self.send_error(403)
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<128:raise ValueError()
            command=json.loads(self.rfile.read(size))
            if command.get('action') not in ('start','stop'):raise ValueError()
            CONTROL.mkdir(exist_ok=True)
            with lock:
                temp=CONTROL/'request.tmp.json';temp.write_text(json.dumps(command));temp.replace(CONTROL/'request.json')
        except (ValueError,OSError):return self.send_error(400,'Invalid command')
        self.send(b'{"queued":true}','application/json')

    def log_message(self,*args):pass

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--port',type=int,default=8766)
    a=p.parse_args();print(f'http://{a.host}:{a.port}/',flush=True)
    ThreadingHTTPServer((a.host,a.port),Handler).serve_forever()
