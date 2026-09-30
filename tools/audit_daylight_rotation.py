#!/usr/bin/env python3
"""Read-only comparison of gyro rotation and image correspondences on daylight motion.
Camera essential-matrix rotations are diagnostics, not ground truth.
"""
import csv,json
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
root=Path(__file__).resolve().parents[1]
cv2.setNumThreads(1);cv2.setRNGSeed(0)
imu=np.loadtxt(root/'data/daylight-replay-input/imu.csv',delimiter=',',comments='#');origin=imu[0,0];t=(imu[:,0]-origin)*1e-9
bias=imu[(t>120)&(t<130),1:4].mean(axis=0)
keep=(t>130)&(t<145);t=t[keep];w=imu[keep,1:4]-bias
q=[Rotation.identity()]
for dw in (w[1:]+w[:-1])*.5*np.diff(t)[:,None]:q.append(q[-1]*Rotation.from_rotvec(dw))
s=Slerp(t,Rotation.concatenate(q))
c=yaml.safe_load('\n'.join(l for l in (root/'calibration/rig-manual-demo-v2/imucam.yaml').read_text().splitlines() if not l.startswith('%')))['cam0']
fx,fy,cx,cy=c['intrinsics'];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
D=np.array(c['distortion_coeffs']);Rc=Rotation.from_matrix(np.array(c['T_cam_imu'])[:3,:3]);dt=c['timeshift_cam_imu']
rows=list(csv.reader((root/'data/daylight-replay-input/images.csv').read_text().splitlines()[1:]));ts=np.array([(int(r[0])-origin)*1e-9 for r in rows]);out=[]
for start in np.arange(137.4,138.5,.1):
 i=np.searchsorted(ts,start);j=i+3;a=cv2.imread(rows[i][1],0);b=cv2.imread(rows[j][1],0)
 p=cv2.goodFeaturesToTrack(a,400,.01,10);v,st,e=cv2.calcOpticalFlowPyrLK(a,b,p,None);back,sb,eb=cv2.calcOpticalFlowPyrLK(b,a,v,None)
 ok=(st[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back-p,axis=2)[:,0]<1)
 u=cv2.undistortPoints(p[ok],K,D).reshape(-1,2);v=cv2.undistortPoints(v[ok],K,D).reshape(-1,2)
 E,mask=cv2.findEssentialMat(u,v,np.eye(3),method=cv2.RANSAC,prob=.999,threshold=1/fx)
 if E is None or E.shape!=(3,3):continue
 n,R,tr,cm=cv2.recoverPose(E,u,v,np.eye(3),mask=mask.copy())
 predicted=Rc*s(ts[j]+dt).inv()*s(ts[i]+dt)*Rc.inv()
 # Fit only translation direction with the gyro rotation, then report epipolar error.
 x=np.c_[u,np.ones(len(u))];y=np.c_[v,np.ones(len(v))];rx=predicted.apply(x)
 good=mask[:,0]>0;_,sing,V=np.linalg.svd(np.cross(rx[good],y[good]),full_matrices=False);trans=V[-1]
 line=np.cross(np.broadcast_to(trans,rx.shape),rx);err=np.abs(np.sum(y*line,axis=1))/np.linalg.norm(line[:,:2],axis=1)*fx
 out.append(dict(t1=float(ts[i]),t2=float(ts[j]),tracks=len(u),essential_inliers=int(good.sum()),positive_depth_inliers=int(n),gyro_rotation_deg=float(predicted.magnitude()*180/np.pi),essential_rotation_deg=float(Rotation.from_matrix(R).magnitude()*180/np.pi),rotation_difference_deg=float((Rotation.from_matrix(R).inv()*predicted).magnitude()*180/np.pi),fixed_gyro_epipolar_error_px_median_p95=np.percentile(err[good],[50,95]).tolist()))
r=dict(scope='Diagnostic image/gyro consistency; essential rotation is not independent ground truth and can be degenerate',bias_rad_s=bias.tolist(),pairs=out)
(root/'evidence/daylight-rotation-consistency.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
