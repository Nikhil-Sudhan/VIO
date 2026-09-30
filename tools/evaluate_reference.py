#!/usr/bin/env python3
"""Evaluate reference replay against TUM VI IMU-frame ground truth, without scale fitting."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def rigid_align(source,target):
    a,b=source.mean(axis=0),target.mean(axis=0)
    u,_,vt=np.linalg.svd((source-a).T@(target-b))
    fix=np.eye(3);fix[2,2]=np.linalg.det(vt.T@u.T)
    rotation=vt.T@fix@u.T
    return rotation,b-rotation@a


def summary(x):
    return {'rms':float(np.sqrt(np.mean(x*x))),'median':float(np.median(x)),
            'p95':float(np.percentile(x,95)),'max':float(np.max(x))}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('groundtruth',type=Path)
    a=p.parse_args()
    manifest=dict(line.split('=',1) for line in (a.run/'run.txt').read_text().splitlines() if '=' in line)
    if 'wall_seconds' not in manifest:raise ValueError('Replay has not completed')
    origin=int(manifest['origin_ns'])
    poses=np.genfromtxt(a.run/'poses.csv',delimiter=',',names=True)
    if poses.ndim!=1 or len(poses)<100:raise ValueError('Insufficient initialized states')
    with a.groundtruth.open() as f:gt=list(csv.reader(l for l in f if not l.startswith('#')))
    times=np.array([(int(r[0])-origin)*1e-9 for r in gt])
    raw=np.array([[float(v) for v in r[1:]] for r in gt])
    if np.any(np.diff(times)<=0):raise ValueError('Nonmonotonic ground truth')
    t=poses['t_rel_s'];within=(t>=times[0])&(t<=times[-1])
    bracket=np.clip(np.searchsorted(times,t),1,len(times)-1)
    within &= times[bracket]-times[bracket-1]<0.05
    pose_t=t[within]
    truth=np.column_stack([np.interp(pose_t,times,raw[:,j]) for j in range(3)])
    estimated=np.column_stack([poses[n][within] for n in ('px','py','pz')])
    rotation,translation=rigid_align(estimated,truth)
    aligned=estimated@rotation.T+translation
    error=np.linalg.norm(aligned-truth,axis=1)
    q_est=np.column_stack([poses[n][within] for n in ('qx','qy','qz','qw')])
    q_true=raw[:,[4,5,6,3]] # published qw,qx,qy,qz -> scipy xyzw
    true_rot=Slerp(times,Rotation.from_quat(q_true))(pose_t)
    aligned_rot=Rotation.from_matrix(rotation)*Rotation.from_quat(q_est)
    angular=(true_rot.inv()*aligned_rot).magnitude()*180/np.pi
    timing=np.genfromtxt(a.run/'timing.csv',delimiter=',',names=True)
    report={'scope':'TUM VI room4 reference, NOT the Raspberry Pi assembly',
            'alignment':'single SE(3) rotation/translation fit over matched positions; fixed metric scale=1',
            'groundtruth':'Published calibrated IMU-frame poses; no additional raw-data offset applied',
            'position_error_m':summary(error),'orientation_error_deg':summary(angular),
            'matched_states':len(pose_t),'initialized_states':len(poses),
            'first_state_s':float(t[0]),'last_state_s':float(t[-1]),
            'processing_ms':summary(timing['processing_ms']),
            'wall_seconds':float(manifest['wall_seconds']),
            'notes':'Offline replay ran alongside other project jobs; these processing times do not establish live latency.',
            'alignment_rotation':rotation.tolist(),'alignment_translation':translation.tolist()}
    np.savetxt(a.run/'aligned_reference.csv',np.column_stack([pose_t,aligned,truth,error,angular]),
               delimiter=',',header='t_rel_s,px,py,pz,gt_x,gt_y,gt_z,error_m,orientation_error_deg',comments='')
    (a.run/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
