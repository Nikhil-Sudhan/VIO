#!/usr/bin/env python3
"""Export only the already-selected still images for independent camera fitting."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import genpy
import rosbag
from sensor_msgs.msg import Image


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('observations',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    observations=json.loads(a.observations.read_text());tables={};events=[]
    for o in observations:
        image=Path(o['image'])
        if hashlib.sha256(image.read_bytes()).hexdigest()!=o['image_sha256']:
            raise ValueError('Source image changed')
        session=image.parent.parent
        if session not in tables:
            info=json.loads((session/'session.json').read_text())
            if info['status']!='completed':raise ValueError('Incomplete session')
            with (session/'frames.csv').open() as f:tables[session]={r['filename']:r for r in csv.DictReader(f)}
        row=tables[session][str(image.relative_to(session))]
        events.append((int(row['sensor_timestamp_ns']),image,row,o['split']))
    events.sort(key=lambda x:x[0])
    if any(b[0]<=a[0] for a,b in zip(events,events[1:])):raise ValueError('Nonmonotonic source timestamps')
    records=[]
    with rosbag.Bag(str(a.output),'w') as bag:
        for ns,path,row,split in events:
            im=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
            msg=Image();msg.header.stamp=genpy.Time(ns//10**9,ns%10**9)
            msg.header.seq=int(row['request_sequence']);msg.header.frame_id='cam0_optical'
            msg.height,msg.width=im.shape;msg.encoding='mono8';msg.step=msg.width;msg.data=im.tobytes()
            bag.write('/cam0/image_raw',msg,msg.header.stamp)
            records.append({'source_image':str(path),'timestamp_ns':ns,'original_split':split})
    r={'purpose':'Camera-only cross-check using independently selected still views; no IMU samples',
       'selection_source':str(a.observations),'images':records,
       'note':'Kalibr will fit this selected set; it is not the original OpenCV held-out evaluation.'}
    a.output.with_suffix('.selection.json').write_text(json.dumps(r,indent=2)+'\n')
    print(f'Wrote {len(records)} selected images to {a.output}')


if __name__=='__main__':main()
