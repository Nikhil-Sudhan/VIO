#!/usr/bin/env python3
"""Strictly decode timestamped FIFO groups and report acquisition evidence."""
import argparse
import csv
import json
import math
import struct
from pathlib import Path
import numpy as np
from clock_mapping import fit_clock


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return None
    return {k: float(v) for k, v in zip(('min', 'median', 'p95', 'max'), np.percentile(a, [0, 50, 95, 100]))}


def decode(lines):
    groups, anchors, faults = [], [], []
    current, last_tick, wraps = {}, None, 0
    batches = []
    for record in lines:
        if record['kind'] == 'clock_anchor':
            anchors.append(record)
            continue
        batches.append(record)
        status = bytes.fromhex(record['status_hex'])
        if len(status) != 2 or record['level_words'] != status[0] | ((status[1] & 3) << 8):
            raise ValueError('FIFO status/level mismatch')
        if bool(status[1] & 0x68) != record['overflow_or_full']:
            raise ValueError('FIFO overflow flag inconsistent with raw status')
        if record['overflow_or_full']:
            faults.append('FIFO overflow/full flag')
        payload = bytes.fromhex(record['raw_hex'])
        if len(payload) != record['read_words'] * 7:
            raise ValueError('Raw FIFO payload length mismatch')
        for i in range(0, len(payload), 7):
            word = payload[i:i+7]
            tag, count = word[0] >> 3, (word[0] >> 1) & 3
            if word[0].bit_count() % 2:
                faults.append('FIFO tag parity error')
            if tag == 4:
                if current:
                    if set(current) >= {'ticks', 'gyro', 'accel', 'count', 'receipt_ns'}:
                        groups.append(current)
                    else:
                        faults.append('Incomplete interior FIFO time slot')
                tick = int.from_bytes(word[1:5], 'little')
                if last_tick is not None and tick < last_tick:
                    if last_tick - tick > 2**31:
                        wraps += 2**32
                    else:
                        faults.append('Sensor timestamp reversed')
                last_tick = tick
                if word[5:] != b'\x00\x55':
                    faults.append('Unexpected FIFO batching rates')
                current = {'ticks': tick + wraps, 'count': count, 'receipt_ns': record['host_after_boot_ns']}
            elif tag in (1, 2):
                name = 'gyro' if tag == 1 else 'accel'
                if not current or current['count'] != count or name in current:
                    faults.append('Unexpected FIFO group order, duplicate or mismatched counter')
                    continue
                current[name] = struct.unpack('<3h', word[1:])
                current['receipt_ns'] = record['host_after_boot_ns']
            else:
                faults.append(f'Unexpected FIFO tag {tag}')
    trailing = False
    if current:
        if 'gyro' in current and 'accel' in current:
            groups.append(current)
        else:
            trailing = True  # explicitly trimmed incomplete stop boundary; never filled
    for a, b in zip(groups, groups[1:]):
        if (b['count'] - a['count']) % 4 != 1:
            faults.append('FIFO slot counter discontinuity')
        # Observed and documented BDR slot ticks; require monotonicity and flag gaps.
        if b['ticks'] <= a['ticks']:
            faults.append('Nonincreasing hardware timestamps')
        elif b['ticks'] - a['ticks'] != 192:
            faults.append('Unexpected hardware slot interval (configured 208 Hz batching)')
    return groups, anchors, batches, faults, trailing


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder', type=Path)
    args = p.parse_args()
    folder = args.folder.resolve()
    with (folder / 'raw.jsonl').open() as f:
        groups, anchors, batches, faults, trailing = decode(json.loads(line) for line in f)
    if len(groups) < 2 or len(anchors) < 10:
        raise RuntimeError('Insufficient FIFO groups or clock anchors')
    map_time, clock_info, residuals, widths, selected = fit_clock(anchors)
    times = map_time([g['ticks'] for g in groups])
    deltas = np.diff(times)
    if np.any(deltas > np.median(deltas)*1.5):
        faults.append('Hardware timestamp gap exceeds 1.5 observed periods')
    if clock_info['withheld_residual_p95_us'] > 1000:
        faults.append('Withheld clock mapping residual p95 exceeds 1 ms')
    if np.any(deltas <= 0):
        faults.append('Mapped IMU time is nonincreasing')
    report = {'calibrated': False, 'synchronization_validated': False,
              'imu_groups': len(groups), 'imu_observed_hz': float(1e9/np.mean(deltas)),
              'hardware_tick_deltas': stats(np.diff([g['ticks'] for g in groups])),
              'imu_interval_ms': stats(deltas / 1e6), 'fifo_max_words': max(b['level_words'] for b in batches),
              'clock_fit': {**clock_info,
                            'all_anchor_residual_us': stats(abs(residuals)/1000),
                            'anchor_read_width_us': stats(widths/1000)},
              'imu_receipt_latency_ms': stats((np.array([g['receipt_ns'] for g in groups])-times)/1e6),
              'trailing_incomplete_slot_trimmed': trailing,
              'near_full_scale_groups': sum(any(abs(v)>=32760 for v in (*g['gyro'],*g['accel'])) for g in groups)}
    if report['near_full_scale_groups']:
        faults.append('IMU near full scale')
    derived = folder / 'analysis'
    derived.mkdir(exist_ok=True)
    with (derived / 'imu_derived.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['mapped_boot_ns','sensor_ticks','fifo_slot_counter','gx_rad_s','gy_rad_s','gz_rad_s',
                    'ax_m_s2','ay_m_s2','az_m_s2','host_received_boot_ns'])
        for g, timestamp in zip(groups, times):
            w.writerow([round(timestamp),g['ticks'],g['count'],
                        *(v*math.pi/180*0.0175 for v in g['gyro']),
                        *(v*9.80665*0.000122 for v in g['accel']),g['receipt_ns']])
    accel=np.array([g['accel'] for g in groups])*9.80665*0.000122
    gyro=np.array([g['gyro'] for g in groups])*math.pi/180*0.0175
    report['accel_norm_m_s2']=stats(np.linalg.norm(accel,axis=1))
    report['gyro_norm_rad_s']=stats(np.linalg.norm(gyro,axis=1))
    report['outside_nominal_gyro_range_groups']=int(np.sum(np.any(np.abs(gyro)>500*math.pi/180,axis=1)))
    report['measurement_range_checks_passed']=not (report['near_full_scale_groups'] or report['outside_nominal_gyro_range_groups'])
    report['measurement_range_note']='Raw continuity checks do not prove physical stationarity or in-range sensor response. The historical near-full-scale field checks proximity to the ADC limit; nominal gyro range is checked separately.'
    if (folder / 'frames.csv').exists():
        with (folder / 'frames.csv').open() as f:
            frames=list(csv.DictReader(f))
        stamps=np.array([int(r['sensor_timestamp_ns']) for r in frames],dtype=np.int64)
        if len(stamps)<2:
            faults.append('Insufficient camera images')
        else:
            d=np.diff(stamps)
            report['camera']={'count':len(frames),'hz':float(1e9/np.mean(d)), 'interval_ms':stats(d/1e6),
                'receipt_minus_sensor_ms':stats([(int(r['host_received_boot_ns'])-int(r['sensor_timestamp_ns']))/1e6 for r in frames]),
                'write_minus_receipt_ms':stats([(int(r['host_written_boot_ns'])-int(r['host_received_boot_ns']))/1e6 for r in frames]),
                'exposure_us':stats([int(r['exposure_us']) for r in frames]),
                'request_sequence_gaps':int(sum(np.diff([int(r['request_sequence']) for r in frames])!=1))}
            if np.any(d<=0) or np.any(d>np.median(d)*1.5):
                faults.append('Camera timestamps nonmonotonic or gap exceeds 1.5 periods')
            if report['camera']['request_sequence_gaps']:
                faults.append('Camera request sequence discontinuity')
            if stamps[0] < times[0] or stamps[-1] > times[-1]:
                faults.append('IMU does not bracket all camera times')
        report['missing_images']=sum(not (folder/r['filename']).is_file() for r in frames)
        if report['missing_images']:
            faults.append('Missing images')
    session=json.loads((folder/'session.json').read_text())
    clocks = [session.get(k, {}) for k in ('clock_start', 'clock_end')]
    if all('boottime_ns' in c and 'monotonic_ns' in c for c in clocks):
        clock_difference = [c['boottime_ns']-c['monotonic_ns'] for c in clocks]
        report['boottime_monotonic_difference_change_ns'] = clock_difference[1]-clock_difference[0]
        if abs(report['boottime_monotonic_difference_change_ns']) > 1_000_000:
            faults.append('Clock domain discontinuity / possible suspend')
    if (folder/'frames.csv').exists() and 'camera_requested' in session:
        w,h = session['camera_requested']['main']['size']
        header = f'P5\n{w} {h}\n255\n'.encode()
        invalid = []
        image_format = session.get('image_format', 'pgm')
        if image_format == 'png':
            import cv2
        for frame in frames:
            path = folder/frame['filename']
            if not path.is_file():
                continue
            if image_format == 'png':
                with path.open('rb') as f:
                    signature_ok = f.read(8) == b'\x89PNG\r\n\x1a\n'
                decoded = cv2.imread(str(path), cv2.IMREAD_UNCHANGED) if signature_ok else None
                valid = decoded is not None and decoded.dtype == np.uint8 and decoded.shape == (h,w)
            else:
                with path.open('rb') as f:
                    valid = f.read(len(header)) == header and path.stat().st_size == len(header)+w*h
            if not valid:
                invalid.append(frame['filename'])
        report['invalid_image_files'] = invalid
        if invalid:
            faults.append('Image header/size differs from configured geometry')
    if session['status'] != 'completed':
        faults.append('Session did not complete normally')
    report['faults']=faults
    report['raw_capture_checks_passed']=not faults
    (derived/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return int(bool(faults))


if __name__=='__main__':
    raise SystemExit(main())
