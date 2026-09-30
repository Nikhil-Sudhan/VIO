#!/usr/bin/env python3
"""Camera-only calibration from varied AprilGrid views; grid units are NOT metres.

The displayed tag size is deliberately one abstract unit. Intrinsics are scale
invariant. This does not measure camera/IMU translation, timing, or metric poses.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def object_corners(tag):
    x,y=(tag%6)*1.3,(tag//6)*1.3
    # Verified against digital generator: OpenCV's canonical corner 0 is BR
    # for these rendered codes; then BL, TL, TR. Board +X right, +Y up.
    return np.array([[x+1,y,0],[x,y,0],[x,y+1,0],[x+1,y+1,0]],np.float32)


def detect(path, detector):
    im=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
    if im is None:raise ValueError(f'Cannot decode {path}')
    corners,ids,_=detector.detectMarkers(im)
    if ids is None:return None
    keep=[(int(i),c.reshape(4,2)) for i,c in zip(ids.ravel(),corners) if 0<=i<36]
    if len(keep)<12 or len({i for i,_ in keep})!=len(keep):return None
    obj=np.concatenate([object_corners(i) for i,_ in keep])
    pts=np.concatenate([c for _,c in keep]).astype(np.float32)
    h,inliers=cv2.findHomography(obj[:,:2],pts,cv2.RANSAC,3)
    if h is None or inliers.mean()<.85:return None
    signature=cv2.perspectiveTransform(np.array([[[0,0],[7.5,0],[7.5,7.5],[0,7.5]]],np.float32),h)[0]
    return {'object':obj,'image':pts,'signature':signature,'path':str(path),'tags':len(keep),
            'ids':[i for i,_ in keep],'size':(im.shape[1],im.shape[0])}


def motion_score(path, neighbor_paths, points):
    image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
    values=[]
    for neighbor in neighbor_paths:
        other=cv2.imread(str(neighbor),cv2.IMREAD_GRAYSCALE)
        if other is None:return float('inf')
        dest,status,_=cv2.calcOpticalFlowPyrLK(image,other,points.reshape(-1,1,2),None,
                                             winSize=(15,15),maxLevel=3)
        if dest is None or status.mean()<.9:return float('inf')
        valid=status.ravel().astype(bool)
        values.append(float(np.median(np.linalg.norm(dest.reshape(-1,2)[valid]-points[valid],axis=1))))
    return max(values)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('sessions',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--max-motion-px',type=float,default=2.5,
                   help='Maximum median target motion to +/-2 neighboring frames (~100 ms each side)')
    p.add_argument('--corner-method',choices=['apriltag','subpix'],default='apriltag',
                   help='AprilTag edge fitting avoids unstable tiny-window refinement on small displayed tags')
    a=p.parse_args();a.output.mkdir()
    params=cv2.aruco.DetectorParameters();params.markerBorderBits=2
    params.cornerRefinementMethod=(cv2.aruco.CORNER_REFINE_APRILTAG if a.corner_method=='apriltag'
                                   else cv2.aruco.CORNER_REFINE_SUBPIX)
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),params)
    views=[];attempts=0;cfg_hash=None;size=None;motion_rejected=0
    for session in a.sessions:
        info=json.loads((session/'session.json').read_text())
        if info['status']!='completed':raise ValueError('Incomplete acquisition session')
        if cfg_hash is None:cfg_hash=info['camera_config_sha256']
        if cfg_hash!=info['camera_config_sha256']:raise ValueError('Mixed camera configurations')
        with (session/'frames.csv').open() as f:frames=list(csv.DictReader(f))
        last=-10**30
        for index,row in enumerate(frames):
            ns=int(row['sensor_timestamp_ns'])
            if ns-last<350_000_000:continue
            last=ns;attempts+=1
            v=detect(session/row['filename'],detector)
            if v is None:continue
            if index<2 or index+2>=len(frames):continue
            neighbors=[session/frames[k]['filename'] for k in (index-2,index+2)]
            v['motion_px']=motion_score(session/row['filename'],neighbors,v['image'])
            if v['motion_px']>a.max_motion_px:
                motion_rejected+=1;continue
            if size is None:size=v['size']
            if size!=v['size']:raise ValueError('Mixed image sizes')
            # Reject near-duplicate board projections before splitting held-out views.
            close=[i for i,old in enumerate(views) if np.sqrt(np.mean((v['signature']-old['signature'])**2))<22]
            if close:
                # Prefer the stiller frame of essentially the same viewing geometry.
                old=close[0]
                if v['motion_px']<views[old]['motion_px'] and v['tags']>=views[old]['tags']-2:views[old]=v
                continue
            views.append(v)
    evidence={'purpose':'camera-only intrinsics candidate, NOT metric camera/IMU calibration',
              'physical_target_size_measured':False,'board_unit':'one displayed tag width',
              'assumptions':['flat display','aspect ratio preserved','6x6 generated April36h11 grid, gap/tag=0.3'],
              'camera_config_sha256':cfg_hash,'attempted_frames':attempts,'distinct_views':len(views),
              'detector':{'library':'OpenCV','version':cv2.__version__,'dictionary':'AprilTag36h11',
                          'border_bits':2,'corner_method':a.corner_method},
              'motion_filter':{'max_median_motion_px':a.max_motion_px,'neighbor_frame_distance':2,'rejected_frames':motion_rejected},
              'views':[{'path':v['path'],'tags':v['tags'],'ids':v['ids'],'motion_px':v['motion_px'],'projected_grid_corners':v['signature'].tolist()} for v in views]}
    def save(): (a.output/'report.json').write_text(json.dumps(evidence,indent=2)+'\n')
    if len(views)<16:
        evidence['status']='insufficient varied views';save();raise ValueError('Need >=16 distinct target views')
    evidence.update(status='fitting candidate',accepted=False);save()
    print(f'Fitting {len(views)} distinct still views; every fourth view held out',flush=True)
    test=[v for i,v in enumerate(views) if i%4==0];train=[v for i,v in enumerate(views) if i%4!=0]
    rms,K,D,rvecs,tvecs,stdK,stdExt,perView=cv2.calibrateCameraExtended(
        [v['object'] for v in train],[v['image'] for v in train],size,None,None,
        flags=cv2.CALIB_FIX_K3,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,100,1e-10))
    held=[];test_poses=[]
    for v in test:
        ok,r,t=cv2.solvePnP(v['object'],v['image'],K,D)
        if not ok:raise ValueError('Held-out target pose solve failed')
        projected=cv2.projectPoints(v['object'],r,t,K,D)[0].reshape(-1,2)
        held.append(float(np.sqrt(np.mean(np.sum((projected-v['image'])**2,axis=1)))))
        test_poses.append((r,t))
    normals=np.array([cv2.Rodrigues(r)[0][:,2] for r in rvecs])
    angles=np.rad2deg(np.arccos(np.clip(normals@normals.T,-1,1)))
    coverage=np.zeros((5,5),bool)
    for v in train:
        bins=np.minimum(np.maximum((v['image']/np.array(size)*5).astype(int),0),4)
        coverage[bins[:,1],bins[:,0]]=True
    checks={'held_out_rms_under_1px':max(held)<1,
            'training_rms_under_1px':rms<1,
            'normal_span_over_20deg':float(angles.max())>20,
            'coverage_at_least_18_of_25_cells':int(coverage.sum())>=18,
            'focal_relative_std_under_5percent':bool(stdK[0,0]/K[0,0]<.05 and stdK[1,0]/K[1,1]<.05),
            'finite_plausible_focal':bool(np.isfinite(K).all() and 100<K[0,0]<3000 and 100<K[1,1]<3000)}
    evidence.update(status='candidate passes numerical gates; review report' if all(checks.values()) else 'candidate fails acceptance gates',
        image_size=list(size),model='pinhole-radtan (k1,k2,p1,p2), k3 fixed to zero',
        training_rms_px=float(rms),held_out_view_rms_px=held,
        training_view_rms_px=perView.ravel().tolist(),intrinsics_matrix=K.tolist(),
        distortion=D.ravel().tolist(),intrinsic_standard_deviations=stdK.ravel().tolist(),
        maximum_board_normal_separation_deg=float(angles.max()),coverage_grid=coverage.astype(int).tolist(),
        checks=checks,accepted=False)
    save()
    observations=[]
    for split,group,poses in [('train',train,zip(rvecs,tvecs)),('held_out',test,test_poses)]:
        for v,(r,t) in zip(group,poses):
            predicted=cv2.projectPoints(v['object'],r,t,K,D)[0].reshape(-1,2)
            observations.append({'split':split,'image':v['path'],'tag_ids':v['ids'],
                'image_sha256':hashlib.sha256(Path(v['path']).read_bytes()).hexdigest(),
                'object_points_tag_units':v['object'].tolist(),'detected_pixels':v['image'].tolist(),
                'projected_pixels':predicted.tolist(),'rvec':r.ravel().tolist(),
                'tvec_tag_units':t.ravel().tolist()})
    (a.output/'observations.json').write_text(json.dumps(observations,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,2,figsize=(12,9))
    for split,color in [('train','tab:blue'),('held_out','tab:orange')]:
        subset=[o for o in observations if o['split']==split]
        pixels=np.concatenate([o['detected_pixels'] for o in subset])
        predicted=np.concatenate([o['projected_pixels'] for o in subset])
        residual=pixels-predicted
        axs[0,0].scatter(pixels[:,0],pixels[:,1],s=3,alpha=.2,c=color,label=split)
        axs[0,1].scatter(residual[:,0],residual[:,1],s=3,alpha=.2,c=color,label=split)
        axs[1,0].hist(np.linalg.norm(residual,axis=1),bins=40,histtype='step',density=True,color=color,label=split)
    axs[0,0].set(xlim=(0,size[0]),ylim=(size[1],0),xlabel='Image x (px)',ylabel='Image y (px)',title='Corner coverage')
    axs[0,1].set(xlabel='Residual x (px)',ylabel='Residual y (px)',title='Detected minus projected corners')
    axs[1,0].set(xlabel='Residual magnitude (px)',ylabel='Density',title='Corner residual distributions')
    axs[1,0].legend()
    axs[1,1].plot(held,'o-',label='Held-out view RMS')
    axs[1,1].axhline(1,color='red',linestyle='--',label='1 px gate')
    axs[1,1].set(xlabel='Held-out viewpoint',ylabel='RMS (px)',title='Independent viewpoints; pose fitted, intrinsics fixed')
    axs[1,1].legend();fig.suptitle('Camera-only candidate — metric target scale unmeasured')
    fig.tight_layout();fig.savefig(a.output/'residuals.png',dpi=150);fig.savefig(a.output/'residuals.pdf');plt.close(fig)
    # Camera-only file: no invented IMU transform, noise or time offset.
    text='%YAML:1.0\n# CAMERA-ONLY CANDIDATE; review report.json before acceptance.\n# Board dimensions are unmeasured; target-frame translations are not metres.\ncam0:\n  camera_model: pinhole\n  distortion_model: radtan\n'
    text+=f'  intrinsics: {json.dumps([float(K[0,0]),float(K[1,1]),float(K[0,2]),float(K[1,2])])}\n'
    text+=f'  distortion_coeffs: {json.dumps(D.ravel()[:4].tolist())}\n  resolution: {json.dumps(list(size))}\n  rostopic: /cam0/image_raw\n'
    (a.output/'camera-only-CANDIDATE.yaml').write_text(text)
    print(json.dumps({k:v for k,v in evidence.items() if k!='views'},indent=2))


if __name__=='__main__':main()
