#!/usr/bin/env python3
"""Manual localhost desktop control for bounded Pi VIO development captures."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from session_view import SessionView

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'data'
CAMERA = ROOT / 'config/camera-candidate.json'
ESTIMATOR = ROOT / 'calibration/rig-manual-demo-v2/estimator_config.yaml'
PYTHON = '/home/pluto/vio-env/bin/python'


def atomic_json(path, value):
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    tmp.replace(path)


def open_browser(url):
    env = os.environ.copy()
    env.update(WAYLAND_DISPLAY='wayland-0', XDG_RUNTIME_DIR='/run/user/1000')
    env.pop('LD_LIBRARY_PATH', None)
    subprocess.Popen(['/usr/bin/chromium', '--ozone-platform=wayland', '--new-window', url],
                     env=env, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL, start_new_session=True)


class Controller:
    def __init__(self):
        self.lock = threading.RLock()
        self.proc = None
        self.run_root = None
        self.selected = None
        self.reader = None
        self.state = dict(stage='idle', title='Ready for a daylight camera check',
                          instruction='Place the rig facing detailed objects, then click Check camera.',
                          detail='Physical VIO accuracy has not passed validation.', pose=None, trajectory=[])
        self.jpeg = b''
        self.started = None
        self.duration = None
        self.stop_requested = False
        self.finished = False
        self.log = None
        self.resource_log = None
        self.shutdown_event = threading.Event()
        self.last_resource = 0
        self.resources = {}
        self.notice = ''
        self.worker = threading.Thread(target=self.refresh, daemon=True)
        self.worker.start()

    def sessions(self):
        paths = list((DATA/'demo-sessions').glob('*/*/session.json'))
        paths += list((DATA/'live-demo').glob('*/session.json'))
        return {str(p.parent.relative_to(DATA)): p.parent for p in sorted(paths, reverse=True)[:50]}

    def active(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, mode):
        with self.lock:
            if self.active():
                raise ValueError('A capture is already running. Stop it before starting another.')
            if self.proc is not None and not self.finished:
                raise ValueError('Previous capture is saving its summary; wait a moment')
            if mode not in ('preview', 'vio'):
                raise ValueError('Unknown capture mode')
            if not Path(PYTHON).exists() or not CAMERA.exists():
                raise ValueError('Camera environment/configuration is missing; see setup instructions')
            if mode == 'vio':
                from calibration_guard import checked_bundle
                checked_bundle(ESTIMATOR, CAMERA, experimental=True)
                if not (ROOT/'build/adapter/vio_stream').is_file():
                    raise ValueError('Build vio_stream before starting VIO')
            self.duration = 30 if mode == 'preview' else 180
            if shutil.disk_usage(DATA).free < 3_000_000_000:
                raise ValueError('Less than 3 GB free; preserve and archive recordings before continuing')
            self.run_root = DATA/'demo-sessions'/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
            self.run_root.mkdir(parents=True)
            command = [PYTHON, '-u', str(ROOT/'tools/record.py'), '--seconds', str(self.duration),
                       '--finish-on-usr1', '--config', str(CAMERA), '--output', str(self.run_root)]
            if mode == 'vio':
                command += ['--estimator-config', str(ESTIMATOR), '--experimental-calibration']
            self.log = (self.run_root/'capture.log').open('xb')
            self.resource_log = (self.run_root/'resources.jsonl').open('x')
            atomic_json(self.run_root/'launch.json', dict(command=command, purpose=mode,
                        automatic_acceptance=False, created_unix_s=time.time()))
            self.proc = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                         stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            self.started = time.monotonic()
            self.stop_requested = self.finished = False
            self.selected = self.reader = None
            self.jpeg = b''
            self.state = dict(stage='starting', title='Starting camera and IMU',
                              instruction='Keep the whole assembly resting.', pose=None, trajectory=[])
            self.notice = ''

    def stop(self):
        with self.lock:
            if not self.active():
                return
            if not self.selected:
                raise ValueError('Recorder is still starting; wait for its session before stopping')
            info = json.loads((self.selected/'session.json').read_text())
            if info['status'] not in ('starting', 'recording'):
                return
            if info['status'] == 'starting':
                raise ValueError('Camera is still starting; wait a few seconds, then Stop')
            self.proc.send_signal(signal.SIGUSR1)
            self.stop_requested = True
            self.notice = 'Finishing capture and preserving the IMU tail…'

    def select(self, key):
        with self.lock:
            if self.active():
                raise ValueError('Stop the active capture before browsing saved runs')
            if self.proc is not None and not self.finished:
                raise ValueError('Previous capture is saving its summary; wait a moment')
            folder = self.sessions().get(key)
            if folder is None:
                raise ValueError('Saved run not found')
            self.selected = folder
            self.reader = SessionView(folder)
            self.jpeg = b''
            self.notice = ''

    def collect_resources(self):
        memory = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
        self.resources = dict(unix_s=time.time(), boottime_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME),
                              temperature_c=int(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000,
                              available_memory_kib=int(memory['MemAvailable'].split()[0]),
                              free_disk_gb=shutil.disk_usage(DATA).free/1e9,
                              cpu_counters=Path('/proc/stat').read_text().splitlines()[0],
                              loadavg=Path('/proc/loadavg').read_text().strip())
        if self.active():
            try:
                self.resources['recorder_proc_stat'] = Path(f'/proc/{self.proc.pid}/stat').read_text().strip()
                children = Path(f'/proc/{self.proc.pid}/task/{self.proc.pid}/children').read_text().split()
                self.resources['child_proc_stats'] = {pid: Path(f'/proc/{pid}/stat').read_text().strip() for pid in children}
            except FileNotFoundError:
                pass
            try:
                self.resources['throttled'] = subprocess.check_output(['vcgencmd', 'get_throttled'], text=True, timeout=2).strip()
            except (OSError, subprocess.SubprocessError) as e:
                self.resources['throttled_unavailable'] = str(e)
            self.resource_log.write(json.dumps(self.resources)+'\n')
            self.resource_log.flush()

    def finish_report(self):
        result = dict(scope='Capture summary; no automatic VIO accuracy acceptance',
                      exit_code=self.proc.returncode, accuracy_validated=False,
                      session=None if self.selected is None else str(self.selected),
                      state_at_end=self.state)
        if self.selected:
            result['session_status'] = json.loads((self.selected/'session.json').read_text())['status']
        atomic_json(self.run_root/'run-summary.json', result)
        self.log.close()
        self.resource_log.close()
        self.finished = True

    def refresh(self):
        while not self.shutdown_event.wait(.5):
            with self.lock:
                try:
                    if self.proc and not self.finished and self.selected is None:
                        files = sorted(self.run_root.glob('*/session.json'))
                        if files:
                            self.selected = files[0].parent
                            self.reader = SessionView(self.selected)
                    if self.reader:
                        self.state = self.reader.snapshot()
                        self.jpeg = self.reader.jpeg
                    if self.proc and not self.finished and self.proc.poll() is not None:
                        self.finish_report()
                        if self.proc.returncode:
                            self.notice = 'Capture ended with an error; logs are preserved.'
                            self.state.update(stage='failed', title='Capture failed — inspect saved details')
                            if not self.selected:
                                with (self.run_root/'capture.log').open('rb') as f:
                                    f.seek(0,2); n=f.tell();f.seek(max(0,n-2000))
                                    self.state['detail']=f.read().decode(errors='replace')
                    if self.active() and time.monotonic()-self.started > self.duration+45:
                        self.proc.terminate()
                        self.notice = 'Recorder exceeded its bounded duration; termination requested, data preserved.'
                    if self.active() and time.monotonic()-self.started > self.duration+55:
                        self.proc.kill()
                    if time.monotonic()-self.last_resource > 2:
                        self.collect_resources()
                        self.last_resource=time.monotonic()
                except Exception as e:
                    self.state.update(stage='error', title='Unable to read current capture', detail=str(e))

    def snapshot(self):
        with self.lock:
            s = dict(self.state)
            if not self.active() and s.get('capture_status') in ('starting', 'recording'):
                s.update(stage='stale', saved=True, title='Stopped or interrupted capture — not live',
                         instruction='The recorder is absent; preserved status may predate interruption.')
            s.update(active=self.active(), stop_requested=self.stop_requested, notice=self.notice,
                     resources=self.resources, sessions=list(self.sessions()),
                     remaining_s=max(0, self.duration-(time.monotonic()-self.started)) if self.active() else 0)
            return s, self.jpeg

    def close(self):
        self.shutdown_event.set()
        self.worker.join(timeout=5)
        if self.active():
            try:
                self.stop()
                self.proc.wait(timeout=10)
            except (ValueError, subprocess.TimeoutExpired):
                self.proc.terminate()
                try:self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait(timeout=5)
        if self.proc and not self.finished:
            try:
                if self.reader:self.state=self.reader.snapshot()
                self.finish_report()
            except Exception:
                if self.log:self.log.close()
                if self.resource_log:self.resource_log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--open', action='store_true')
    args = p.parse_args()
    DATA.mkdir(exist_ok=True)
    lock = (DATA/'demo-app.lock').open('a')
    registry = DATA/'demo-app.json'
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # A double-click can arrive while the first instance is publishing its URL.
        for _ in range(20):
            if registry.exists():break
            time.sleep(.1)
        info = json.loads(registry.read_text())
        if info['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            raise RuntimeError('App metadata belongs to an earlier boot')
        if args.open:open_browser(info['url'])
        print(info['url']);return
    app = Controller()
    token = secrets.token_urlsafe(32)
    page = (ROOT/'ui/demo.html').read_text().replace('__TOKEN__', token).encode()

    class Handler(BaseHTTPRequestHandler):
        def send(self, code, body, kind='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):pass

        def local_host(self):
            return self.headers.get('Host') == f'127.0.0.1:{self.server.server_port}'

        def do_GET(self):
            if not self.local_host():
                self.send(403, b'{"error":"Use the local Pi address"}');return
            route = self.path.split('?')[0]
            if route == '/':self.send(200, page, 'text/html; charset=utf-8')
            elif route == '/guide':self.send(200, (ROOT/'docs/MORNING.md').read_bytes(), 'text/plain; charset=utf-8')
            elif route == '/report.json':
                state, _ = app.snapshot()
                self.send(200, json.dumps(state, indent=2, allow_nan=False).encode())
            elif route == '/poses.csv':
                with app.lock:
                    path = None if app.selected is None else app.selected/'vio/poses.csv'
                    if path is None or not path.is_file():self.send(404, b'{"error":"No pose log for this capture"}')
                    else:self.send(200, path.read_bytes(), 'text/csv; charset=utf-8')
            elif route in ('/state', '/frame.jpg'):
                state, jpeg = app.snapshot()
                self.send(200, json.dumps(state, allow_nan=False).encode() if route == '/state' else jpeg,
                          'application/json' if route == '/state' else 'image/jpeg')
            else:self.send(404, b'{"error":"Not found"}')

        def do_POST(self):
            origin = self.headers.get('Origin')
            expected = f'http://127.0.0.1:{self.server.server_port}'
            if not self.local_host() or self.headers.get('X-Demo-Token') != token or origin not in (None, expected):
                self.send(403, b'{"error":"Local control token required"}');return
            try:
                n = int(self.headers.get('Content-Length', '0'))
                if not 0 < n <= 4096:raise ValueError('Invalid request length')
                data = json.loads(self.rfile.read(n))
                if self.path == '/start':app.start(data['mode'])
                elif self.path == '/stop':app.stop()
                elif self.path == '/select':app.select(data['session'])
                else:raise ValueError('Unknown action')
                self.send(200, b'{"ok":true}')
            except (ValueError, KeyError, OSError) as e:
                self.send(400, json.dumps(dict(error=str(e))).encode())

        def log_message(self, *args):pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.daemon_threads = True
    url = f'http://127.0.0.1:{server.server_port}/'
    atomic_json(registry, dict(pid=os.getpid(), url=url,
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                capture_starts_automatically=False))
    print(url, flush=True)
    if args.open:open_browser(url)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        server.server_close()
        app.close()
        registry.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
