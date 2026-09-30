#!/usr/bin/env python3
"""Gracefully finish a specific live guided capture, preserving the IMU tail."""
import argparse
import json
import os
import signal
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path);a=p.parse_args()
r=json.loads((a.session/'session.json').read_text())
if r.get('status')!='recording' or not r.get('finish_on_usr1'):
    raise RuntimeError('This is not an active guided recording')
if r['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
    raise RuntimeError('Recording belongs to an earlier boot; do not signal a reused PID')
pid=r['pid'];cmd=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
if not any(x.endswith(b'/record.py') for x in cmd) or b'--finish-on-usr1' not in cmd:
    raise RuntimeError('PID does not identify the guided recorder')
os.kill(pid,signal.SIGUSR1)
print(f'Requested normal finish of guided recording PID {pid}; inspect final session.json for completion.')
