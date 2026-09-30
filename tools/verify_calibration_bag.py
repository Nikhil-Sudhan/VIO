#!/usr/bin/env python3
"""Read back every exported sample and compare it to the retained source."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import rosbag


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    provenance = json.loads(a.bag.with_suffix('.provenance.json').read_text())
    source = Path(provenance['recording'])
    report = source / 'analysis/report.json'
    if hashlib.sha256(report.read_bytes()).hexdigest() != provenance['raw_analysis_sha256']:
        raise ValueError('Analysis changed since bag export')
    with (source / 'frames.csv').open() as f:
        frames = list(csv.DictReader(f))
    with (source / 'analysis/imu_derived.csv').open() as f:
        imu = list(csv.DictReader(f))
    counts = {'camera': 0, 'imu': 0}
    last = -1
    with rosbag.Bag(str(a.bag), 'r') as bag:
        for topic, msg, stamp in bag.read_messages():
            ns = stamp.to_nsec()
            if ns < last or ns != msg.header.stamp.to_nsec():
                raise ValueError('Bag ordering or header timestamp mismatch')
            last = ns
            if topic == '/cam0/image_raw':
                row = frames[counts['camera']]
                pixels = cv2.imread(str(source / row['filename']), cv2.IMREAD_GRAYSCALE)
                expected = (int(row['sensor_timestamp_ns']), int(row['request_sequence']),
                            'cam0_optical', pixels.shape[0], pixels.shape[1],
                            'mono8', pixels.shape[1], 0, pixels.tobytes())
                actual = (ns, msg.header.seq, msg.header.frame_id, msg.height, msg.width,
                          msg.encoding, msg.step, msg.is_bigendian, bytes(msg.data))
                if actual != expected:
                    raise ValueError(f"Image mismatch at {counts['camera']}")
                counts['camera'] += 1
            elif topic == '/imu0':
                row = imu[counts['imu']]
                if (ns != int(row['mapped_boot_ns']) or msg.header.seq != counts['imu'] or
                        msg.header.frame_id != 'imu0_native' or msg.orientation_covariance[0] != -1):
                    raise ValueError('IMU metadata mismatch')
                for axis in 'xyz':
                    if (getattr(msg.angular_velocity, axis) != float(row[f'g{axis}_rad_s']) or
                            getattr(msg.linear_acceleration, axis) != float(row[f'a{axis}_m_s2'])):
                        raise ValueError(f"IMU data mismatch at {counts['imu']}")
                counts['imu'] += 1
            else:
                raise ValueError(f'Unexpected topic: {topic}')
    if counts != provenance['counts'] or counts != {'camera': len(frames), 'imu': len(imu)}:
        raise ValueError('Sample counts differ')
    result = {'bag': str(a.bag.resolve()), 'source': str(source), 'counts': counts,
              'all_sample_timestamps_and_payloads_match': True,
              'scope': 'Export integrity only; camera/IMU synchronization and calibration remain unverified'}
    with a.output.open('x') as f:
        json.dump(result, f, indent=2)
        f.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
