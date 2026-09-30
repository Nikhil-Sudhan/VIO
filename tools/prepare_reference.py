#!/usr/bin/env python3
"""Verify and prepare the official TUM VI room4 EuRoC-format archive."""
import argparse
import csv
import hashlib
import json
import tarfile
from pathlib import Path


def sha(path, algorithm):
    h=hashlib.new(algorithm)
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def rows(path):
    with path.open() as f:
        result=list(csv.reader(line for line in f if line.strip() and not line.startswith('#')))
    ticks=[int(r[0]) for r in result]
    if len(ticks)<2 or any(b<=a for a,b in zip(ticks,ticks[1:])):
        raise ValueError(f'Insufficient/nonmonotonic records: {path}')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('archive',type=Path);p.add_argument('md5_file',type=Path)
    p.add_argument('output',type=Path)
    a=p.parse_args();expected=a.md5_file.read_text().split()[0]
    if len(expected)!=32 or sha(a.archive,'md5')!=expected:
        raise ValueError('Published archive checksum mismatch')
    a.output.mkdir()  # never overwrite an existing extraction/result
    with tarfile.open(a.archive) as t:t.extractall(a.output,filter='data')
    candidates=list(a.output.glob('**/mav0'))
    if len(candidates)!=1:raise ValueError('Expected exactly one mav0 directory')
    mav=candidates[0].resolve()
    camera=[rows(mav/f'cam{i}/data.csv') for i in range(2)]
    imu=rows(mav/'imu0/data.csv')
    if [r[0] for r in camera[0]] != [r[0] for r in camera[1]]:
        raise ValueError('Stereo timestamps do not match exactly')
    with (a.output/'images.csv').open('w',newline='') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['#t_ns','path0','path1'])
        for r0,r1 in zip(*camera):
            paths=[mav/f'cam{i}/data'/r[1] for i,r in enumerate((r0,r1))]
            if any(not x.is_file() for x in paths):raise ValueError('Missing reference image')
            if any(',' in str(x) or '\n' in str(x) for x in paths):raise ValueError('Unsupported filename')
            w.writerow([r0[0],*map(str,paths)])
    with (a.output/'images-mono.csv').open('w',newline='') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['#t_ns','path0'])
        for r in camera[0]:w.writerow([r[0],str(mav/'cam0/data'/r[1])])
    with (a.output/'imu.csv').open('w',newline='') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['#t_ns','wx','wy','wz','ax','ay','az']);w.writerows(imu)
    report={'scope':'reference dataset only; not the Pi rig',
            'dataset':'TUM VI room4, 512x512 16-bit EuRoC export',
            'url':'https://cdn2.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room4_512_16.tar',
            'published_md5':expected,'sha256':sha(a.archive,'sha256'),
            'images_per_camera':len(camera[0]),'imu_samples':len(imu),
            'mav0_directory':str(mav),'image_conversion':'uint16 -> uint8 with fixed scale 1/256',
            'calibration':'OpenVINS pinned upstream config/tum_vi; exclusively for this dataset'}
    (a.output/'source.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
