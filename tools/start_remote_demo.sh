#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/rviz_env.sh"
cd "$VIO_ROOT"
/usr/bin/python3 - <<'PY'
import fcntl,json,socket,subprocess,time,urllib.request
from pathlib import Path
p=Path('data/native-control');p.mkdir(exist_ok=True)
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.connect(('192.168.137.1',9));host=sock.getsockname()[0];sock.close()
lock=(p/'controller.lock').open('a')
try:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    fcntl.flock(lock,fcntl.LOCK_UN)
    with (p/'remote-controller.log').open('ab') as log:
        controller=subprocess.Popen(['bash','tools/start_native_demo.sh'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
except BlockingIOError:
    controller=None
url=f'http://{host}:8766/'
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
try:opener.open(url+'state',timeout=2).read()
except OSError:
    with (p/'remote-display.log').open('ab') as log:
        server=subprocess.Popen(['/usr/bin/python3','tools/remote_rviz.py','--host',host],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    time.sleep(2)
for route in ('state','screen.jpg'):
    with opener.open(url+route,timeout=5) as response:
        body=response.read();print(route,response.status,'bytes',len(body))
(p/'remote-display.json').write_text(json.dumps({'url':url,'controller_pid':controller.pid if controller else None,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}))
print(url)
PY
