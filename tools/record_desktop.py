#!/usr/bin/env python3
"""Save a bounded video of the actual Wayland desktop during a manual demo."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('output', type=Path)
p.add_argument('--seconds', type=int, default=120)
a = p.parse_args()
if not 1 <= a.seconds <= 300:
    p.error('seconds must be 1..300')
if a.output.exists():
    p.error('output already exists')
a.output.parent.mkdir(parents=True, exist_ok=True)
env = os.environ | {'XDG_RUNTIME_DIR': '/run/user/1000', 'WAYLAND_DISPLAY': 'wayland-0'}
fps = 5
started = time.monotonic()
times = []
stopping = False
def stop(*_):
    global stopping
    stopping = True
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
encoder = subprocess.Popen(['ffmpeg', '-nostdin', '-v', 'error', '-n', '-f', 'image2pipe',
    '-vcodec', 'ppm', '-framerate', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
    '-threads', '1', '-preset', 'ultrafast', '-crf', '23', '-pix_fmt', 'yuv420p',
    '-movflags', '+faststart', str(a.output)], stdin=subprocess.PIPE)
try:
    for i in range(a.seconds * fps):
        if stopping:
            break
        time.sleep(max(0, started + i/fps - time.monotonic()))
        frame = subprocess.run(['grim', '-s', '0.6666667', '-t', 'ppm', '-'],
                               env=env, stdout=subprocess.PIPE, check=True)
        times.append(time.monotonic() - started)
        encoder.stdin.write(frame.stdout)
finally:
    encoder.stdin.close()
    code = encoder.wait(timeout=30)
    a.output.with_suffix('.json').write_text(json.dumps({
        'source': 'Actual live desktop screen capture; no synthetic poses or playback',
        'frames': len(times), 'fps': fps, 'capture_times_s': times,
        'elapsed_s': time.monotonic()-started, 'encoder_exit_code': code}, indent=2)+'\n')
if code:
    raise SystemExit(code)
print(a.output, flush=True)
