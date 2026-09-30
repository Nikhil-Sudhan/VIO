#!/usr/bin/python3
"""Read-only camera/gyro rotation check; essential rotations are not ground truth."""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from calibration_guard import read_yaml

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('session', type=Path)
p.add_argument('output', type=Path)
p.add_argument('--calibration', type=Path, help='Explicit candidate imucam.yaml for raw-only recordings; diagnostic use only')
a = p.parse_args()
cv2.setNumThreads(1)
cv2.setRNGSeed(0)
imu = np.genfromtxt(a.session/'analysis/imu_derived.csv', delimiter=',', names=True)
origin = imu['mapped_boot_ns'][0]
t = (imu['mapped_boot_ns']-origin)*1e-9
w = np.column_stack([imu['g'+v+'_rad_s'] for v in 'xyz'])
bias = np.median(w[(t>3)&(t<20)], axis=0)
w -= bias
rotations = [Rotation.identity()]
for dw in (w[:-1]+w[1:])*.5*np.diff(t)[:,None]:
    rotations.append(rotations[-1]*Rotation.from_rotvec(dw))
slerp = Slerp(t, Rotation.concatenate(rotations))
c = read_yaml(a.calibration or a.session/'calibration/imucam.yaml')['cam0']
fx,fy,cx,cy = c['intrinsics']
K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
D = np.array(c['distortion_coeffs'])
Rc = Rotation.from_matrix(np.array(c['T_cam_imu'])[:3,:3])
dt = c['timeshift_cam_imu']
rows = list(csv.DictReader((a.session/'frames.csv').open()))
ts = np.array([(int(r['sensor_timestamp_ns'])-origin)*1e-9 for r in rows])
pairs = []
for i in range(0, len(rows)-4, 6):
    j = i+3
    if ts[i]+dt<t[0] or ts[j]+dt>t[-1]:
        continue
    imu_relative = slerp(ts[j]+dt).inv()*slerp(ts[i]+dt)
    angle = np.degrees(imu_relative.magnitude())
    if not 1.5<angle<20:
        continue
    im0,im1 = [cv2.imread(str(a.session/rows[k]['filename']),0) for k in [i,j]]
    if im0 is None or im1 is None:
        continue
    points = cv2.goodFeaturesToTrack(im0,350,.01,10)
    if points is None or len(points)<35:
        continue
    v,st,_ = cv2.calcOpticalFlowPyrLK(im0,im1,points,None)
    back,sb,_ = cv2.calcOpticalFlowPyrLK(im1,im0,v,None)
    ok = (st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back-points,axis=2)[:,0]<1)
    if ok.sum()<30:
        continue
    u = cv2.undistortPoints(points[ok],K,D).reshape(-1,2)
    v = cv2.undistortPoints(v[ok],K,D).reshape(-1,2)
    E,mask = cv2.findEssentialMat(u,v,np.eye(3),method=cv2.RANSAC,prob=.999,threshold=1/fx)
    if E is None or E.shape!=(3,3):
        continue
    n,R,tr,cm = cv2.recoverPose(E,u,v,np.eye(3),mask=mask.copy())
    visual = Rotation.from_matrix(R)
    predicted = Rc*imu_relative*Rc.inv()
    pairs.append(dict(t=float(ts[i]),tracks=int(ok.sum()),inliers=int(mask.sum()),positive_depth=int(n),
                      gyro_angle_deg=float(angle),visual_angle_deg=float(np.degrees(visual.magnitude())),
                      difference_deg=float(np.degrees((visual.inv()*predicted).magnitude())),
                      imu_rotvec=imu_relative.as_rotvec().tolist(),visual_rotvec=visual.as_rotvec().tolist()))
report = {'scope':'Diagnostic only. Essential rotation can be degenerate; no calibration promotion.',
          'gyro_bias_rad_s':bias.tolist(),'pairs':pairs}
good = [r for r in pairs if r['positive_depth']>30 and .6<r['visual_angle_deg']/r['gyro_angle_deg']<1.4]
if len(good)>=8:
    u = np.array([r['imu_rotvec'] for r in good])
    v = np.array([r['visual_rotvec'] for r in good])
    fit,_ = Rotation.align_vectors(v,u)
    error = np.linalg.norm(fit.apply(u)-v,axis=1)
    keep = error<np.quantile(error,.75)
    fit,_ = Rotation.align_vectors(v[keep],u[keep])
    report['diagnostic_alignment'] = {'pairs':len(good),'R_cam_imu':fit.as_matrix().tolist(),
       'difference_from_user_axes_deg':float(np.degrees((fit*Rc.inv()).magnitude())),
       'rotation_vector_error_deg_p50_p90':np.degrees(np.percentile(np.linalg.norm(fit.apply(u)-v,axis=1),[50,90])).tolist(),
       'excitation_singular_values':np.linalg.svd(u,compute_uv=False).tolist()}
a.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'pairs':len(pairs),'median_difference_deg':float(np.median([r['difference_deg'] for r in pairs])) if pairs else None,
                  'diagnostic_alignment':report.get('diagnostic_alignment')},indent=2))
