#!/usr/bin/python3
"""Stop a specifically requested short desk-motion demo if its estimate diverges."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--scope', default='Automatic rejection for the requested approximately 10 cm movement only; not a general tracking-quality or safety guarantee.')
    args = parser.parse_args()
    folder = args.session.resolve()
    original = json.loads((folder/'session.json').read_text())
    pid = original['pid']
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if original['boot_id'] != boot or not original.get('finish_on_usr1'):
        raise ValueError('Recorder identity or graceful-stop capability mismatch')
    while True:
        state = json.loads((folder/'session.json').read_text())
        if state['status'] not in ('starting', 'recording'):
            return
        command = Path(f'/proc/{pid}/cmdline')
        if not command.exists():
            return
        raw = command.read_bytes()
        if b'tools/record.py' not in raw or str(folder.parent).encode() not in raw:
            raise ValueError('PID no longer belongs to this recorder')
        try:
            snap = json.loads((folder/'vio/display.json').read_text())
        except (OSError, ValueError):
            time.sleep(.5)
            continue
        fresh = 0 <= time.clock_gettime_ns(time.CLOCK_BOOTTIME)-snap['emitted_boot_ns'] < 2e9
        if fresh and snap.get('initialized'):
            position = math.sqrt(sum(x*x for x in snap['position']))
            speed = math.sqrt(sum(x*x for x in snap['velocity']))
            if not math.isfinite(position+speed) or position > 5 or speed > 3:
                reason = f'Estimate exceeded the short desk-test bounds: position {position:.2f} m, speed {speed:.2f} m/s.'
                result = {'usable_vio': False, 'reason': reason,
                          'scope': args.scope,
                          'position_norm_m': position, 'speed_m_s': speed,
                          'limits': {'position_norm_m': 5, 'speed_m_s': 3}}
                (folder/'vio/display-review.json').write_text(json.dumps(result, indent=2)+'\n')
                os.kill(pid, signal.SIGUSR1)
                print(reason, flush=True)
                return
        time.sleep(.5)


if __name__ == '__main__':
    main()
