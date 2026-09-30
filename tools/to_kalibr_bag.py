#!/usr/bin/env python3
"""Export a strictly checked raw recording for calibration, without guessed transforms."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import genpy
import rosbag
from sensor_msgs.msg import Image, Imu


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path);p.add_argument('output_bag',type=Path)
    p.add_argument('--compression',choices=['none','bz2','lz4'],default='none',help='Lossless bag compression; sensor payloads and timestamps are unchanged')
    p.add_argument('--start',type=float,default=0,help='First camera time, seconds after the first recorded image')
    p.add_argument('--end',type=float,help='Last camera time, seconds after the first recorded image')
    p.add_argument('--image-stride',type=int,default=1,help='Export every Nth selected original image; keep every IMU sample with one-second margins')
    a=p.parse_args();folder=a.recording.resolve()
    if a.start < 0 or a.image_stride < 1 or (a.end is not None and a.end <= a.start):
        p.error('Require start >= 0, end > start and image-stride >= 1')
    report=json.loads((folder/'analysis/report.json').read_text())
    if not report['raw_capture_checks_passed']:
        raise ValueError('Raw capture checks failed: do not calibrate from a known corrupt recording')
    if a.output_bag.exists():raise FileExistsError(a.output_bag)
    with (folder/'frames.csv').open() as f:frames=list(csv.DictReader(f))
    with (folder/'analysis/imu_derived.csv').open() as f:imu=list(csv.DictReader(f))
    camera_origin=int(frames[0]['sensor_timestamp_ns'])
    selection_requested=a.start != 0 or a.end is not None or a.image_stride != 1
    if selection_requested:
        frames=[r for r in frames if (int(r['sensor_timestamp_ns'])-camera_origin)*1e-9 >= a.start
                and (a.end is None or (int(r['sensor_timestamp_ns'])-camera_origin)*1e-9 <= a.end)][::a.image_stride]
        if len(frames)<2:raise ValueError('Selected interval needs at least two images')
        first=int(frames[0]['sensor_timestamp_ns'])-1_000_000_000
        last=int(frames[-1]['sensor_timestamp_ns'])+1_000_000_000
        imu=[r for r in imu if first <= int(r['mapped_boot_ns']) <= last]
        if len(imu)<2:raise ValueError('Selected interval needs IMU measurements')
    events=sorted([(int(r['sensor_timestamp_ns']),'camera',r) for r in frames]+
                  [(int(r['mapped_boot_ns']),'imu',r) for r in imu],key=lambda e:e[0])
    counts={'camera':0,'imu':0}
    with rosbag.Bag(str(a.output_bag),'w',compression=a.compression) as bag:
        for ns,kind,r in events:
            stamp=genpy.Time(ns//1_000_000_000,ns%1_000_000_000)
            if kind=='camera':
                pixels=cv2.imread(str(folder/r['filename']),cv2.IMREAD_GRAYSCALE)
                if pixels is None:raise ValueError('Unreadable camera image')
                m=Image();m.header.seq=int(r['request_sequence']);m.header.frame_id='cam0_optical'
                m.height,m.width=pixels.shape;m.encoding='mono8';m.is_bigendian=0;m.step=m.width;m.data=pixels.tobytes()
                topic='/cam0/image_raw'
            else:
                m=Imu();m.header.seq=counts[kind];m.header.frame_id='imu0_native'
                m.orientation_covariance[0]=-1 # orientation not measured by this IMU stream
                for axis in 'xyz':
                    setattr(m.angular_velocity,axis,float(r[f'g{axis}_rad_s']))
                    setattr(m.linear_acceleration,axis,float(r[f'a{axis}_m_s2']))
                topic='/imu0'
            m.header.stamp=stamp;bag.write(topic,m,stamp);counts[kind]+=1
    manifest={'recording':str(folder),'counts':counts,'compression':a.compression,
              'selection':{'requested':selection_requested,'camera_origin_ns':camera_origin,
                           'start_s':a.start,'end_s':a.end,'image_stride':a.image_stride,
                           'imu_margin_s':1 if selection_requested else None,
                           'first_exported_ns':events[0][0],'last_exported_ns':events[-1][0]},
              'purpose':'calibration input, not calibrated VIO',
              'camera_time':'original SensorTimestamp (driver frame start, no midpoint correction)',
              'imu_time':'hardware timestamp mapped by saved clock model; offset still uncalibrated',
              'gravity_retained':True,'axes':'native IMU XYZ and camera optical image convention',
              'raw_analysis_sha256':hashlib.sha256((folder/'analysis/report.json').read_bytes()).hexdigest(),
              'clock_fit':report['clock_fit']}
    a.output_bag.with_suffix('.provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
