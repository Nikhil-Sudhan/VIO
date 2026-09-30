#!/usr/bin/env python3
"""Bounded unattended IMU capture and provisional analysis; never auto-accepts noise."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, required=True)
    parser.add_argument('--warmup-s', type=int, default=300)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 15 <= args.seconds <= 7200 or not 0 <= args.warmup_s <= args.seconds - 10:
        parser.error('Need 15..7200 seconds and at least 10 seconds after warmup')
    folder = args.output.resolve()
    folder.mkdir()
    recordings = folder / 'recordings'
    recordings.mkdir()
    (folder / 'source').mkdir()
    state = dict(status='starting', accepted_for_estimator=False,
                 created_utc=datetime.now(timezone.utc).isoformat(),
                 boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                 pid=__import__('os').getpid(), seconds=args.seconds, warmup_s=args.warmup_s,
                 purpose='IMU-only noise characterization; no camera, estimator, flight controller or automatic calibration acceptance',
                 source_sha256={})
    for name in ['record.py', 'imu_fifo_probe.py', 'analyse_recording.py', 'clock_mapping.py',
                 'imu_noise.py', 'monitor.py', 'run_noise_job.py', 'env.sh']:
        source = ROOT / 'tools' / name
        shutil.copy2(source, folder / 'source' / name)
        state['source_sha256'][name] = hashlib.sha256(source.read_bytes()).hexdigest()
    child = None
    stopping = False
    monitor = None

    def save():
        state['updated_utc'] = datetime.now(timezone.utc).isoformat()
        temporary = folder / 'job.json.tmp'
        temporary.write_text(json.dumps(state, indent=2) + '\n')
        temporary.replace(folder / 'job.json')

    def stop(signum, frame):
        nonlocal stopping
        stopping = True
        if child is not None and child.poll() is None:
            child.terminate()  # Recorder restores registers on SIGTERM.

    def run(command, log_name, status):
        nonlocal child
        if stopping:
            raise RuntimeError('Job interrupted; no further processing started')
        state['status'] = status
        state.setdefault('commands', []).append(command)
        save()
        with (folder / log_name).open('x') as output:
            child = subprocess.Popen(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT)
            state['child_pid'] = child.pid
            save()
            code = child.wait()
        state.setdefault('exit_codes', {})[status] = code
        if code or stopping:
            raise RuntimeError(f'{status} exited {code}; interrupted={stopping}')
        child = None

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    save()
    print(folder, flush=True)
    try:
        with (folder / 'monitor.log').open('x') as monitor_log:
            monitor = subprocess.Popen(['/usr/bin/python3', str(ROOT / 'tools/monitor.py'),
                '--seconds', str(args.seconds + 60), '--output', str(folder / 'monitor.jsonl')],
                cwd=ROOT, stdout=monitor_log, stderr=subprocess.STDOUT)
            run(['/home/pluto/vio-env/bin/python', str(ROOT / 'tools/record.py'), '--imu-only',
                 '--temperature', '--seconds', str(args.seconds), '--output', str(recordings)],
                'capture.log', 'recording')
            monitor.terminate()
            monitor.wait()
        sessions = list(recordings.glob('*/session.json'))
        if len(sessions) != 1 or json.loads(sessions[0].read_text())['status'] != 'completed':
            raise RuntimeError('Expected exactly one completed recording')
        session = sessions[0].parent
        state['session'] = str(session)
        # Do not silently analyze with edited code while a long capture ran.
        for name in ['analyse_recording.py', 'clock_mapping.py', 'imu_noise.py', 'env.sh']:
            if hashlib.sha256((ROOT / 'tools' / name).read_bytes()).hexdigest() != state['source_sha256'][name]:
                raise RuntimeError(f'{name} changed during capture; raw data saved for explicit review')
        prefix = ['bash', '-c', 'source tools/env.sh; exec /usr/bin/python3 "$@"', 'vio-noise-analysis']
        run(prefix + ['tools/analyse_recording.py', str(session)], 'analysis.log', 'analysing_capture')
        run(prefix + ['tools/imu_noise.py', str(session), '--start-s', str(args.warmup_s),
                      '--output', str(folder / 'provisional-noise')], 'noise.log', 'analysing_noise')
        state['status'] = 'completed_requires_stationarity_and_noise_review'
    except Exception:
        state['status'] = 'interrupted' if stopping else 'failed'
        state['error'] = traceback.format_exc()
    finally:
        if monitor is not None and monitor.poll() is None:
            monitor.terminate()
            monitor.wait()
        state['child_pid'] = None
        save()
    return 0 if state['status'] == 'completed_requires_stationarity_and_noise_review' else 1


if __name__ == '__main__':
    sys.exit(main())
