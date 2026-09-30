#!/usr/bin/env python3
"""Export checked Pi recordings to the ROS-free adapter's input format.

This exports measurements only; it supplies no guessed estimator calibration.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--allow-estimator-failure', action='store_true',
                   help='Offline diagnostic export only: accept an estimator failure when every raw integrity check passed')
    a = p.parse_args()
    folder = a.session.resolve()
    session = json.loads((folder/'session.json').read_text())
    report_path = folder/'analysis/report.json'
    report = json.loads(report_path.read_text())
    estimator_failure_only = (
        a.allow_estimator_failure and session['status'] == 'failed'
        and report['faults'] == ['Session did not complete normally']
        and session.get('live_transport', {}).get('estimator_exit_code') not in (None, 0)
        and bool(session.get('errors'))
        and all('tools/live_bridge.py' in e or e.startswith('Estimator exit ') for e in session['errors']))
    if (session['status'] != 'completed' or not report['raw_capture_checks_passed']) and not estimator_failure_only:
        raise ValueError('Recording must pass strict acquisition checks')
    if session.get('capture_mode') == 'imu-only' or not (folder/'frames.csv').exists():
        raise ValueError('Camera measurements are required')
    for clock in ['clock_start','clock_end']:
        c = session[clock]
        if abs(c['boottime_ns']-c['monotonic_ns']) > 1_000_000:
            raise ValueError('BOOTTIME/MONOTONIC equivalence not established for this recording')
    with (folder/'frames.csv').open() as f: frames = list(csv.DictReader(f))
    imu_path = folder/'analysis/imu_derived.csv'
    with imu_path.open() as f: imu = list(csv.DictReader(f))
    for row in imu:
        values=[float(row['g'+x+'_rad_s']) for x in 'xyz']+[float(row['a'+x+'_m_s2']) for x in 'xyz']
        if not all(math.isfinite(v) for v in values) or any(abs(v)>500*math.pi/180 for v in values[:3]):
            raise ValueError('Nonfinite or outside-nominal-range IMU measurement')
    for rows,key in [(frames,'sensor_timestamp_ns'),(imu,'mapped_boot_ns')]:
        ticks = [int(row[key]) for row in rows]
        if len(ticks)<2 or any(b<=x for x,b in zip(ticks,ticks[1:])):
            raise ValueError('Insufficient/nonmonotonic input times')
    paths = [(folder/r['filename']).resolve() for r in frames]
    if any(not p.is_file() or any(c in str(p) for c in ',\n\r') for p in paths):
        raise ValueError('Missing image or unsupported CSV path')
    a.output.mkdir()
    with (a.output/'images.csv').open('x',newline='') as f:
        w = csv.writer(f,lineterminator='\n');w.writerow(['#t_ns','path0'])
        for r,path in zip(frames,paths): w.writerow([r['sensor_timestamp_ns'],path])
    with (a.output/'imu.csv').open('x',newline='') as f:
        w = csv.writer(f,lineterminator='\n');w.writerow(['#t_ns','wx','wy','wz','ax','ay','az'])
        for r in imu:
            w.writerow([r['mapped_boot_ns'],*[r['g'+x+'_rad_s'] for x in 'xyz'],*[r['a'+x+'_m_s2'] for x in 'xyz']])
    evidence = {'scope':'Pi measurement export; not accepted estimator calibration or VIO output',
        'session':str(folder),'camera_config_sha256':session['camera_config_sha256'],
        'counts':{'images':len(frames),'imu':len(imu)},
        'source_sha256':{str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [report_path,imu_path,folder/'frames.csv',folder/'session.json']},
        'images':'Absolute paths to preserved original 8-bit images; no image transformation',
        'estimator_failure_only_exception':estimator_failure_only,
        'original_session_status':session['status'],
        'original_raw_report_faults':report['faults'],
        'camera_timestamp':'Unmodified SensorTimestamp, driver frame-start clock; no midpoint correction',
        'imu_timestamp':'Derived hardware counter clock mapping; no invented replacement samples',
        'units_and_axes':'rad/s and m/s^2 including gravity; native IMU XYZ',
        'required_before_estimation':'Reviewed calibration for this exact camera geometry, IMU settings and rigid mounting, including time offset and noise'}
    (a.output/'source.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,indent=2))


if __name__ == '__main__': main()
