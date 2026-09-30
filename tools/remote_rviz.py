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
<title>Live VIO</title><style>
html,body{width:100%;height:100%;margin:0;overflow:hidden;background:#000;color:white;font:18px system-ui}
#screen{display:block;width:100vw;height:100vh;object-fit:contain}
#controls{position:fixed;inset:0;display:flex;align-items:center;justify-content:center;flex-direction:column;background:#0009;gap:14px}
#controls.hidden{display:none}button,a{font:inherit;padding:14px 22px;border-radius:8px;border:0;cursor:pointer}a{color:white}#status{max-width:650px;white-space:pre-wrap;text-align:center}
</style><img id="screen" alt=""><div id="controls">
<button id="start" onclick="action('start')">Start 120-second demo + recording</button>
<button id="stop" onclick="action('stop')" hidden>Stop and save</button>
<a href="/video" id="video">Download latest screen video</a><div id="status">Ready</div></div>
<script>
const img=document.getElementById('screen'),st=document.getElementById('status'),controls=document.getElementById('controls');
let active=false,seenLive=false,requestedAt=0;
img.onload=()=>setTimeout(frame,400);
img.onerror=()=>{controls.classList.remove('hidden');st.textContent='Display disconnected — retrying';setTimeout(frame,1000)};
function frame(){img.src='/screen.jpg?t='+Date.now()}frame();
async function action(action){
 if(action==='start'){
  if(document.documentElement.requestFullscreen)document.documentElement.requestFullscreen().catch(()=>{});
  active=true;seenLive=false;requestedAt=Date.now();controls.classList.add('hidden');
 }
 try{let r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});if(!r.ok)throw Error(await r.text());if(action==='stop'){active=false;controls.classList.remove('hidden')}}
 catch(e){active=false;controls.classList.remove('hidden');st.textContent=e}
}
function reveal(){controls.classList.toggle('hidden')}
img.onclick=reveal;
document.addEventListener('keydown',e=>{if(e.key==='Escape')controls.classList.remove('hidden');if(e.key===' ')reveal()});
document.addEventListener('fullscreenchange',()=>{if(!document.fullscreenElement)controls.classList.remove('hidden')});
async function state(){try{
 let x=await(await fetch('/state')).json();st.textContent=x.message;
 document.getElementById('video').hidden=!x.video;
 if(x.running)seenLive=true;
 if(active&&!x.running&&(seenLive||Date.now()-requestedAt>20000)){active=false;controls.classList.remove('hidden')}
 document.getElementById('start').hidden=!!x.running||active;
 document.getElementById('stop').hidden=!x.running&&!active;
}catch(e){controls.classList.remove('hidden');st.textContent='Connection interrupted — retrying'}setTimeout(state,1000)}state();
</script></html>'''

lock = threading.Lock()
cached = b''
captured = 0.

def status():
    videos = sorted((Path.home()/'Videos').glob('VIO-demo-*.mp4'))
    result = {'video': bool(videos), 'running': False, 'message': 'Ready. Rest the rig for 5 seconds after starting. Click the picture or press Escape for controls.'}
    try:
        controller = json.loads((CONTROL/'controller-state.json').read_text())
        age = (time.clock_gettime_ns(time.CLOCK_BOOTTIME)-controller['emitted_boot_ns'])/1e9
        if controller['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip() and 0 <= age < 3:
            result.update({k: controller[k] for k in ('running', 'message', 'phase')})
            return result
    except (OSError, ValueError, KeyError):
        pass
    try:
        folder = Path((CONTROL/'current-session.txt').read_text().strip())
        session = json.loads((folder/'session.json').read_text())
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if session.get('boot_id') != boot:
            return result
        state = session['status']
        result['running'] = state in ('recording', 'starting')
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
