#!/usr/bin/env python3
"""Compare a replay to target-derived observations, never independent ground truth."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from calibration_guard import read_yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('observations',type=Path)
    p.add_argument('camera',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    poses=np.genfromtxt(a.run/'poses.csv',delimiter=',',names=True)
    obs=np.genfromtxt(a.observations,delimiter=',',names=True)
    manifest=dict(l.split('=',1) for l in (a.run/'run.txt').read_text().splitlines() if '=' in l)
    origin=int(manifest['origin_ns'])
    t=(obs['sensor_timestamp_ns']-origin)*1e-9
    cam_origin=t[0]
    keep=(t>=poses['t_rel_s'][0])&(t<=poses['t_rel_s'][-1])&(obs['rms_px']<.8)
    obs=obs[keep];t=t[keep]
    if len(t)<20:raise ValueError('Insufficient matching observations')
    pt=poses['t_rel_s']
    xyz=np.column_stack([np.interp(t,pt,poses[k]) for k in ('px','py','pz')])
    vio_rot=Slerp(pt,Rotation.from_quat(np.column_stack([poses[k] for k in ('qx','qy','qz','qw')])))(t)
    ci=np.asarray(read_yaml(a.camera)['cam0']['T_cam_imu'],float)
    ct=Rotation.from_rotvec(np.column_stack([obs['rvec_'+k] for k in 'xyz']))
    target_xyz=ct.inv().apply(ci[:3,3]-np.column_stack([obs['t'+k+'_m'] for k in 'xyz']))
    target_rot=ct.inv()*Rotation.from_matrix(ci[:3,:3])
    resting=(t-cam_origin>=20)&(t-cam_origin<40)
    if resting.sum()<10:raise ValueError('Need initial resting matches to anchor comparison')
    alignment=(vio_rot[resting]*target_rot[resting].inv()).mean()
    translation=np.median(xyz[resting]-alignment.apply(target_xyz[resting]),axis=0)
    observed_local=alignment.apply(target_xyz)+translation
    error=np.linalg.norm(xyz-observed_local,axis=1)
    angle=np.rad2deg((vio_rot.inv()*alignment*target_rot).magnitude())
    def summary(mask):
        if not mask.any():return {'samples':0}
        return {'samples':int(mask.sum()),'position_error_m_p50_p95_max':np.percentile(error[mask],[50,95,100]).tolist(),
                'position_rms_m':float(np.sqrt(np.mean(error[mask]**2))),
                'orientation_difference_deg_p50_p95_max':np.percentile(angle[mask],[50,95,100]).tolist()}
    report={'scope':'Consistency against target PnP using shared intrinsics, nominal target scale and fitted extrinsics; NOT independent accuracy or grid-free mapping validation',
            'alignment':'Initial resting interval 20–40 camera-relative seconds, rotation and translation only; no scale fit',
            'run':str(a.run.resolve()),'observations':str(a.observations.resolve()),
            'all':summary(np.ones(len(t),bool)),
            'moving':summary((t-cam_origin>=60)&(t-cam_origin<270)),
            'outside_both_joint_fit_intervals':summary(((t-cam_origin>=120)&(t-cam_origin<170))|((t-cam_origin>=230)&(t-cam_origin<260))),
            'windows':[{'start_camera_relative_s':start,**summary((t-cam_origin>=start)&(t-cam_origin<start+20))} for start in range(0,280,20)]}
    a.output.mkdir()
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savetxt(a.output/'comparison.csv',np.column_stack([t,xyz,observed_local,error,angle]),delimiter=',',
               header='t_rel_s,vio_x,vio_y,vio_z,target_x,target_y,target_z,position_difference_m,orientation_difference_deg',comments='')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,1,figsize=(10,7),layout='constrained')
    axs[0].plot(t-cam_origin,error);axs[0].set(ylabel='Position difference (m)')
    axs[1].plot(t-cam_origin,angle);axs[1].set(xlabel='Time from first image (s)',ylabel='Orientation difference (deg)')
    fig.suptitle('Natural-feature replay vs target-derived observations\nShared calibration; not independent ground truth')
    fig.savefig(a.output/'consistency.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k!='windows'},indent=2))


if __name__=='__main__':main()
