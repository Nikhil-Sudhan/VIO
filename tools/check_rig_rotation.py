#!/usr/bin/python3
"""Compare short, slow rigid-rig turns to image bearing rotation; not calibration.

The image fit assumes rotation-dominant motion. Translation, rolling shutter and
camera calibration errors can bias it. This tool never writes estimator settings.
"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from calibration_guard import read_yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--camera', type=Path, default=Path('calibration/camera-820x616-v1/camera.yaml'))
    args = parser.parse_args()
    report = json.loads((args.session/'analysis/report.json').read_text())
    if not report['raw_capture_checks_passed']:
        raise ValueError('Recording must pass raw acquisition checks first')
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    data = np.genfromtxt(args.session/'analysis/imu_derived.csv', delimiter=',', names=True)
    origin = data['mapped_boot_ns'][0]
    t = (data['mapped_boot_ns']-origin)*1e-9
    w = np.column_stack([data['g'+a+'_rad_s'] for a in 'xyz'])
    bias = np.median(w[(t>2)&(t<5)], axis=0)
    w = w-bias
    q = [Rotation.identity()]
    for dw in (w[:-1]+w[1:])*.5*np.diff(t)[:,None]:
        q.append(q[-1]*Rotation.from_rotvec(dw))
    interp = Slerp(t, Rotation.concatenate(q))
    cam = read_yaml(args.camera)['cam0']
    fx,fy,cx,cy = cam['intrinsics']
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    D = np.array(cam['distortion_coeffs'])
    Rc = Rotation.from_matrix([[0,-1,0],[0,0,-1],[1,0,0]])
    rows = list(csv.DictReader((args.session/'frames.csv').open()))
    ts = np.array([(int(r['sensor_timestamp_ns'])-origin)*1e-9 for r in rows])
    pairs = []
    for i in range(0,len(rows)-6,4):
        j=i+6
        if ts[i]<t[0] or ts[j]>t[-1]:
            continue
        ims=[cv2.imread(str(args.session/rows[k]['filename']),0) for k in [i,j]]
        points=cv2.goodFeaturesToTrack(ims[0],300,.015,12)
        if points is None or len(points)<30:
            continue
        end,st,_=cv2.calcOpticalFlowPyrLK(ims[0],ims[1],points,None)
        back,sb,_=cv2.calcOpticalFlowPyrLK(ims[1],ims[0],end,None)
        keep=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back-points,axis=2)[:,0]<1)
        if keep.sum()<30:
            continue
        x=cv2.undistortPoints(points[keep],K,D).reshape(-1,2)
        y=cv2.undistortPoints(end[keep],K,D).reshape(-1,2)
        H,mask=cv2.findHomography(x,y,cv2.RANSAC,2/fx)
        if H is None or mask.sum()<25:
            continue
        mask=mask[:,0]>0
        x=np.c_[x[mask],np.ones(mask.sum())]
        y=np.c_[y[mask],np.ones(mask.sum())]
        x/=np.linalg.norm(x,axis=1)[:,None]
        y/=np.linalg.norm(y,axis=1)[:,None]
        u,_,vt=np.linalg.svd(y.T@x)
        visual=Rotation.from_matrix(u@np.diag([1,1,np.linalg.det(u@vt)])@vt)
        predicted=Rc*interp(ts[j]).inv()*interp(ts[i])*Rc.inv()
        dt=ts[j]-ts[i]
        residual=np.degrees(np.arccos(np.clip(np.sum(visual.apply(x)*y,axis=1),-1,1)))
        pairs.append({'t0':float(ts[i]),'t1':float(ts[j]),'tracks':int(mask.sum()),
            'visual_rotvec':visual.as_rotvec().tolist(),'gyro_camera_rotvec':predicted.as_rotvec().tolist(),
            'image_fit_residual_deg_median':float(np.median(residual)),
            'rotation_difference_deg':float(np.degrees((visual.inv()*predicted).magnitude())),
            'visual_rate_deg_s':(np.degrees(visual.as_rotvec())/dt).tolist(),
            'gyro_camera_rate_deg_s':(np.degrees(predicted.as_rotvec())/dt).tolist()})
    moving=[r for r in pairs if np.linalg.norm(r['gyro_camera_rate_deg_s'])>1.0]
    result={'scope':'Axis/motion consistency diagnostic only; pure-rotation image approximation, no calibration acceptance.',
        'session':str(args.session.resolve()),'gyro_bias_rad_s':bias.tolist(),
        'imu_axis_rotation_activity_deg':np.degrees(np.trapezoid(abs(w),t,axis=0)).tolist(),
        'pair_count':len(pairs),'moving_pair_count':len(moving),'pairs':pairs}
    if len(moving)>=8:
        v=np.array([r['visual_rate_deg_s'] for r in moving])
        g=np.array([r['gyro_camera_rate_deg_s'] for r in moving])
        result['moving_summary']={
            'rotation_difference_deg_p50_p90':np.percentile([r['rotation_difference_deg'] for r in moving],[50,90]).tolist(),
            'yaw_rate_correlation':float(np.corrcoef(v[:,1],g[:,1])[0,1]) if np.std(v[:,1])>1e-6 and np.std(g[:,1])>1e-6 else None,
            'yaw_visual_to_gyro_slope':float(np.dot(v[:,1],g[:,1])/np.dot(g[:,1],g[:,1])) if np.dot(g[:,1],g[:,1])>1e-9 else None}
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='pairs'},indent=2))


if __name__=='__main__':
    main()
