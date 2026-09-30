#!/usr/bin/env python3
"""Stop the manually launched desktop app after checking process identity."""
import json
import os
from pathlib import Path
import signal

root = Path(__file__).resolve().parents[1]
info = json.loads((root/'data/demo-app.json').read_text())
if info['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
    raise SystemExit('App belongs to an earlier boot; no signal sent')
pid = int(info['pid'])
args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
if not any(arg.endswith(b'/demo_app.py') for arg in args):
    raise SystemExit('PID is not this app; no signal sent')
os.kill(pid, signal.SIGTERM)
print('Requested app shutdown. Any active capture is stopped with its data preserved.')
