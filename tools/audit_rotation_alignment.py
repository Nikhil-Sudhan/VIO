#!/usr/bin/env python3
"""Fit diagnostic rotation/time alignment to target pose differences, not VIO.

This uses measured gyro bias from a resting prefix and separate fitting/review
intervals. It does not replace joint calibration or establish independent truth.
"""
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

root = Path(__file__).resolve().parents[1]
obs = np.genfromtxt(root/'calibration/motion-preflight-poses4-run3/target-observations.csv', delimiter=',', names=True)
imu = np.loadtxt(root/'data/hardware-replay-poses4/imu.csv', delimiter=',', comments='#')
origin = imu[0, 0]
it = (imu[:, 0]-origin)*1e-9
ot = (obs['sensor_timestamp_ns']-origin)*1e-9
bias = imu[(it > 100) & (it < 110), 1:4].mean(axis=0)
selected = (it > 95) & (it < 255)
it = it[selected]; w = imu[selected, 1:4]-bias
increments = Rotation.from_rotvec((w[1:]+w[:-1])*.5*np.diff(it)[:, None])
q = np.empty((len(it), 4)); R = Rotation.identity(); q[0] = R.as_quat()
for i in range(len(increments)):
    R = R*increments[i]; q[i+1] = R.as_quat()
orientation = Slerp(it, Rotation.from_quat(q))
cam = Rotation.from_rotvec(np.column_stack([obs[k] for k in ('rvec_x', 'rvec_y', 'rvec_z')]))
cal_path = root/'calibration/rig-manual-demo-v1/imucam.yaml'
cal = yaml.safe_load('\n'.join(x for x in cal_path.read_text().splitlines() if not x.startswith('%')))['cam0']
Rci = Rotation.from_matrix(np.array(cal['T_cam_imu'])[:3, :3])

def pairs(start, end):
    rows = []
    for i in range(len(ot)):
        if not start <= ot[i] <= end-.5:continue
        j = np.searchsorted(ot, ot[i]+.45)
        if j >= len(ot) or not .4 <= ot[j]-ot[i] <= .7:continue
        if max(obs['rms_px'][i], obs['rms_px'][j]) > .8 or min(obs['tags'][i], obs['tags'][j]) < 12:continue
        rows.append((i,j))
    a,b = np.array(rows).T
    return a,b,cam[b]*cam[a].inv()

train = pairs(182.06, 242.05)
review = pairs(112.06, 172.05)

def residual(x, pair):
    a,b,observed = pair
    Rc = Rotation.from_rotvec(x[:3])*Rci
    inertial_relative = orientation(ot[b]+x[3]).inv()*orientation(ot[a]+x[3])
    if len(x) > 4:
        inertial_relative = Rotation.from_rotvec(inertial_relative.as_rotvec()*x[4])
    predicted = Rc*inertial_relative*Rc.inv()
    return (observed.inv()*predicted).as_rotvec().ravel()

initial = np.r_[np.zeros(3), cal['timeshift_cam_imu']]
fit = least_squares(residual, initial, args=(train,), bounds=([-0.15]*3+[-.05],[.15]*3+[.05]),
                    loss='soft_l1', f_scale=np.deg2rad(.2), max_nfev=60)
def stats(x, pair):
    error = np.linalg.norm(residual(x,pair).reshape(-1,3),axis=1)*180/np.pi
    return dict(pairs=len(error), rms_deg=float(np.sqrt(np.mean(error**2))),
                p50_p95_max_deg=np.percentile(error,[50,95,100]).tolist())
result = dict(scope='Diagnostic rotation/time fit only; not accepted calibration or independent ground truth',
              training_interval_s=[182.06,242.05], review_interval_s=[112.06,172.05],
              measured_static_gyro_bias_rad_s=bias.tolist(), original_offset_s=float(initial[3]),
              fit_converged=bool(fit.success), fitted_offset_s=float(fit.x[3]),
              rotation_change_deg=float(np.linalg.norm(fit.x[:3])*180/np.pi),
              fitted_R_cam_imu=(Rotation.from_rotvec(fit.x[:3])*Rci).as_matrix().tolist(),
              train_before=stats(initial,train), train_after=stats(fit.x,train),
              review_before=stats(initial,review), review_after=stats(fit.x,review))
(root/'evidence/rotation-alignment-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))

# A scale diagnostic on short relative rotations; not an intrinsic correction.
# Scaling the integrated rotation is an approximation during changing-axis motion.
scaled_initial = np.r_[initial,1.0]
scale_fit = least_squares(residual,scaled_initial,args=(train,),
    bounds=([-0.15]*3+[-.05,.8],[.15]*3+[.05,1.2]),loss='soft_l1',f_scale=np.deg2rad(.2),max_nfev=60)
scale_result = dict(scope='Approximate gyro scale diagnostic; must not be applied as measured calibration',
    scale_factor=float(scale_fit.x[4]), offset_s=float(scale_fit.x[3]),
    rotation_change_deg=float(np.linalg.norm(scale_fit.x[:3])*180/np.pi),
    train_before=stats(scaled_initial,train),train_after=stats(scale_fit.x,train),
    review_before=stats(scaled_initial,review),review_after=stats(scale_fit.x,review))
(root/'evidence/gyro-scale-audit.json').write_text(json.dumps(scale_result,indent=2)+'\n')
print(json.dumps(scale_result,indent=2))
