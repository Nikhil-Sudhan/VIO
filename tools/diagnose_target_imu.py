#!/usr/bin/env python3
"""Independent target/gyro consistency diagnostic; never exports VIO poses."""
import argparse, csv, json, sys
from pathlib import Path
import cv2, numpy as np, yaml
from scipy.spatial.transform import Rotation, Slerp
from scipy.optimize import least_squares
from calibrate_intrinsics import object_corners

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('session',type=Path);p.add_argument('calibration',type=Path)
p.add_argument('output',type=Path);p.add_argument('--start',type=float,required=True)
p.add_argument('--end',type=float,required=True);p.add_argument('--stride',type=int,default=2)
p.add_argument('--bias-start',type=float);p.add_argument('--bias-end',type=float)
a=p.parse_args();a.output.mkdir();cv2.setNumThreads(1)
c=yaml.safe_load('\n'.join(l for l in a.calibration.read_text().splitlines() if not l.startswith('%')))['cam0']
fx,fy,cx,cy=c['intrinsics'];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
D=np.array(c['distortion_coeffs']);Rci=np.array(c['T_cam_imu'])[:3,:3];dt=c['timeshift_cam_imu']
imu=np.genfromtxt(a.session/'analysis/imu_derived.csv',delimiter=',',names=True)
origin=imu['mapped_boot_ns'][0];it=(imu['mapped_boot_ns']-origin)*1e-9
gyro=np.column_stack([imu['g'+x+'_rad_s'] for x in 'xyz'])
bias_start=a.start-10 if a.bias_start is None else a.bias_start
bias_end=a.start-2 if a.bias_end is None else a.bias_end
bias_samples=gyro[(it>bias_start)&(it<bias_end)]
if len(bias_samples)<100:raise ValueError('Insufficient samples in selected resting bias interval')
bias=bias_samples.mean(axis=0)
rots=[np.eye(3)]
for k in range(1,len(it)):
    rots.append(rots[-1]@Rotation.from_rotvec(((gyro[k]+gyro[k-1])/2-bias)*(it[k]-it[k-1])).as_matrix())
slerp=Slerp(it,Rotation.from_matrix(np.array(rots)))
params=cv2.aruco.DetectorParameters();params.markerBorderBits=2
params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_APRILTAG
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),params)
rows=list(csv.DictReader((a.session/'frames.csv').open()));obs=[];attempts=0
for row in rows[::a.stride]:
    t=(int(row['sensor_timestamp_ns'])-origin)*1e-9
    if not a.start<=t<=a.end:continue
    attempts+=1;im=cv2.imread(str(a.session/row['filename']),0);corners,ids,_=det.detectMarkers(im)
    if ids is None:continue
    keep=[(int(i),v.reshape(4,2)) for i,v in zip(ids.ravel(),corners) if 0<=i<36]
    if len(keep)<4 or len(set(i for i,v in keep))!=len(keep):continue
    obj=np.concatenate([object_corners(i)*.020 for i,v in keep]);pts=np.concatenate([v for i,v in keep])
    h,mask=cv2.findHomography(obj[:,:2],pts,cv2.RANSAC,2)
    if h is None or mask.mean()<.95:continue
    ok,r,tvec=cv2.solvePnP(obj,pts,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or tvec[2,0]<=0:continue
    rms=float(np.sqrt(np.mean(np.sum((cv2.projectPoints(obj,r,tvec,K,D)[0].reshape(-1,2)-pts)**2,axis=1))))
    if rms>.75:continue
    obs.append({'t':t,'frame':int(row['sequence']),'tags':len(keep),'rms':rms,'rvec':r.ravel().tolist(),'tvec':tvec.ravel().tolist()})
(a.output/'observations.json').write_text(json.dumps(obs,indent=2))
pairs=[]
for i,x in enumerate(obs):
    later=[y for y in obs[i+1:] if .18<=y['t']-x['t']<=.4]
    if not later:continue
    y=min(later,key=lambda y:abs(y['t']-x['t']-.3))
    dr=Rotation.from_rotvec(y['rvec']).as_matrix()@Rotation.from_rotvec(x['rvec']).as_matrix().T
    pairs.append((x['t'],y['t'],dr))
def residual(v,subset):
    R=Rotation.from_rotvec(v[:3]).as_matrix()@Rci
    ts=np.array([[x,y] for x,y,r in subset])+v[3]
    b=slerp(ts.ravel()).as_matrix().reshape(-1,2,3,3)
    return np.array([Rotation.from_matrix(dr@(R@bb[1].T@bb[0]@R.T).T).as_rotvec() for (x,y,dr),bb in zip(subset,b)]).ravel()
moving=[v for v in pairs if np.linalg.norm(Rotation.from_matrix(v[2]).as_rotvec())>np.deg2rad(.5)]
report={'scope':'Diagnostic only; nominal 20 mm target; no accepted calibration; target PnP is not ground truth',
        'attempts':attempts,'observations':len(obs),'moving_pairs':len(moving),'bias':bias.tolist(),
        'bias_interval_s':[bias_start,bias_end],'initial_dt':dt}
if len(moving)>=8:
    train=moving[::2];test=moving[1::2]
    result=least_squares(lambda v:residual(v,train),[0,0,0,dt],bounds=([-1,-1,-1,-.15],[1,1,1,.15]),loss='soft_l1',f_scale=.005,max_nfev=100)
    for name,v in [('fixed',np.array([0,0,0,dt])),('fitted',result.x)]:
        report[name]={'delta_rotation_deg':np.rad2deg(np.linalg.norm(v[:3])),'dt_s':v[3],
                     'R_cam_imu':(Rotation.from_rotvec(v[:3]).as_matrix()@Rci).tolist(),
                     'train_error_p50_p95_deg':np.percentile(np.rad2deg(np.linalg.norm(residual(v,train).reshape(-1,3),axis=1)),[50,95]).tolist(),
                     'held_error_p50_p95_deg':np.percentile(np.rad2deg(np.linalg.norm(residual(v,test).reshape(-1,3),axis=1)),[50,95]).tolist()}
    report['pair_errors']=[{'t0':x,'t1':y,'visual_angle_deg':np.rad2deg(Rotation.from_matrix(dr).magnitude()),'error_deg':e} for (x,y,dr),e in zip(moving,np.rad2deg(np.linalg.norm(residual(np.array([0,0,0,dt]),moving).reshape(-1,3),axis=1)))]
(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='pair_errors'},indent=2))
