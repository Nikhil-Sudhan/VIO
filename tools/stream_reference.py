#!/usr/bin/env python3
"""Send recorded reference data through the transport, not through physical sensors."""
import argparse
import csv
import sys
import time
from pathlib import Path
import cv2
from stream_protocol import MAGIC, imu_packet, camera_packet, end_packet


def rows(path):
    with path.open() as f: r=list(csv.reader(line for line in f if line.strip() and not line.startswith('#')))
    if len(r)<2 or any(int(b[0])<=int(a[0]) for a,b in zip(r,r[1:])):
        raise ValueError('Insufficient/nonmonotonic input')
    return r


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('images',type=Path);p.add_argument('imu',type=Path)
    p.add_argument('--max-frames',type=int,default=0)
    p.add_argument('--pace',action='store_true',help='Deliver according to recorded times plus a simulated 30 ms camera receipt delay')
    a=p.parse_args()
    images=rows(a.images);imu=rows(a.imu)
    if any(len(r)!=2 for r in images) or any(len(r)!=7 for r in imu):
        raise ValueError('Monocular image CSV and six-axis IMU required')
    first,last=int(imu[0][0]),int(imu[-1][0])
    selected=[(i,r) for i,r in enumerate(images) if first<=int(r[0]) and int(r[0])+100_000_000<last]
    excluded=len(images)-len(selected)
    if a.max_frames: selected=selected[:a.max_frames]
    if not selected: raise ValueError('No bracketed camera frames')
    tail=int(selected[-1][1][0])+100_000_000
    events=[(int(r[0]),1,i,r) for i,r in enumerate(imu) if int(r[0])<=tail]
    events.extend((int(r[0])+30_000_000,2,i,r) for i,r in selected)
    events.sort(key=lambda e:(e[0],e[1]))
    out=sys.stdout.buffer;out.write(MAGIC);start=time.monotonic_ns();origin=events[0][0]
    for arrival,kind,index,r in events:
        if a.pace:
            delay=(start+arrival-origin-time.monotonic_ns())*1e-9
            if delay>0: time.sleep(delay)
        ns=int(r[0])
        if kind==1: packet=imu_packet(ns,0,index,list(map(float,r[1:])))
        else:
            pixels=cv2.imread(r[1],cv2.IMREAD_UNCHANGED)
            if pixels is None: raise ValueError(f'Cannot read {r[1]}')
            if pixels.dtype.name=='uint16': pixels=cv2.convertScaleAbs(pixels,alpha=1/256)
            packet=camera_packet(ns,0,index,pixels)
        out.write(packet)
        if a.pace:out.flush()
    out.write(end_packet());out.flush()
    print(f'Reference transport: {len(selected)} images; {excluded} unbracketed boundary images excluded explicitly; pace={a.pace}',file=sys.stderr)


if __name__=='__main__':main()
