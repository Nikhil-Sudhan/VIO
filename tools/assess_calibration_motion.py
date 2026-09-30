#!/usr/bin/env python3
"""Inspect AprilGrid visibility and measured motion before joint calibration.

PnP target poses are observations, not VIO estimates or independent ground truth.
No camera/IMU transform, time offset or noise defaults are generated here.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from calibrate_intrinsics import detect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    p.add_argument('--camera', type=Path, default=Path('calibration/camera-820x616-v1/camera.yaml'))
    p.add_argument('--target', type=Path, default=Path('calibration/target/target-screen-MEASURED.yaml'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stride', type=int, default=2)
    p.add_argument('--allow-exposure-change', action='store_true',
                   help='For this diagnostic only, allow exposure/gain changes after checking all remaining camera settings')
    a = p.parse_args()
    if a.stride < 1: p.error('stride must be positive')
    source = json.loads((a.session/'analysis/report.json').read_text())
    session = json.loads((a.session/'session.json').read_text())
    if not source['raw_capture_checks_passed'] or session['status'] != 'completed':
        raise ValueError('Input must pass acquisition checks')
    camera_review = json.loads((a.camera.parent/'review.json').read_text())
    exposure_change = None
    if session['camera_config_sha256'] != camera_review['configuration_sha256']:
        if not a.allow_exposure_change:
            raise ValueError('Camera configuration differs from the reviewed calibration')
        reviewed_cfg = json.loads((a.camera.parent/'camera-config.json').read_text())
        capture_cfg = json.loads((a.session/'camera-config.json').read_text())
        def nonphotometric(cfg):
            result = {k:v for k,v in cfg.items() if k not in ('notes','status','controls')}
            result['controls'] = {k:v for k,v in cfg['controls'].items()
                                  if k not in ('ExposureTime','AnalogueGain','AeEnable')}
            return result
        if nonphotometric(reviewed_cfg) != nonphotometric(capture_cfg):
            raise ValueError('Camera settings other than exposure/gain differ from the reviewed calibration')
        exposure_change = {'reviewed_controls': reviewed_cfg['controls'],
                           'capture_controls': capture_cfg['controls'],
                           'scope': 'PnP visibility/motion diagnostic only; timing and moving-image quality remain unvalidated'}
    cam = yaml.safe_load('\n'.join(s for s in a.camera.read_text().splitlines() if not s.startswith('%')))['cam0']
    target = yaml.safe_load(a.target.read_text())
    if (target['tagRows'],target['tagCols'],target['tagSpacing']) != (6,6,.3):
        raise ValueError('Detector expects the prepared six-by-six target')
    fx,fy,cx,cy = cam['intrinsics']
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float)
    D = np.array(cam['distortion_coeffs'])
    params = cv2.aruco.DetectorParameters()
    params.markerBorderBits = 2
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),params)
    cv2.setNumThreads(1)
    with (a.session/'frames.csv').open() as f: frames = list(csv.DictReader(f))
    origin = int(frames[0]['sensor_timestamp_ns'])
    a.output.mkdir()
    observed = []
    with (a.output/'target-observations.csv').open('x') as f:
        writer = csv.writer(f)
        writer.writerow(['sensor_timestamp_ns','t_rel_s','tags','rms_px','rvec_x','rvec_y','rvec_z','tx_m','ty_m','tz_m'])
        for index,row in enumerate(frames[::a.stride]):
            view = detect(a.session/row['filename'],detector)
            if view is None: continue
            obj = view['object'] * target['tagSize']
            ok,rvec,tvec = cv2.solvePnP(obj,view['image'],K,D,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or tvec[2,0] <= 0: continue
            projected,_ = cv2.projectPoints(obj,rvec,tvec,K,D)
            rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-view['image'])**2,axis=1))))
            ns = int(row['sensor_timestamp_ns']);t = (ns-origin)*1e-9
            writer.writerow([ns,t,view['tags'],rms,*rvec.ravel(),*tvec.ravel()])
            observed.append((t,view['tags'],rms))
            if index % 250 == 0: print(f'checked={index+1}, target poses={len(observed)}',flush=True)
    imu = np.genfromtxt(a.session/'analysis/imu_derived.csv',delimiter=',',names=True)
    t = (imu['mapped_boot_ns']-origin)*1e-9
    w = np.column_stack([imu[f'g{x}_rad_s'] for x in 'xyz'])
    accel = np.column_stack([imu[f'a{x}_m_s2'] for x in 'xyz'])
    observations = np.array(observed).reshape(-1,3)
    bins = []
    duration = (int(frames[-1]['sensor_timestamp_ns'])-origin)*1e-9
    for start in np.arange(0,duration,20):
        end = min(start+20,duration)
        v = observations[(observations[:,0]>=start)&(observations[:,0]<end)]
        select = (t>=start)&(t<end)
        gyro = w[select];acc = accel[select]
        eig = np.linalg.eigvalsh(np.cov(gyro.T))
        count = int(sum(start <= (int(r['sensor_timestamp_ns'])-origin)*1e-9 < end for r in frames[::a.stride]))
        bins.append({'start_s':float(start),'end_s':float(end),'frames_checked':count,
            'target_poses':len(v),'target_pose_fraction':len(v)/max(1,count),
            'reprojection_rms_p50_p95_px':np.percentile(v[:,2],[50,95]).tolist() if len(v) else None,
            'gyro_covariance_eigenvalues_rad2_s2':eig.tolist(),
            'gyro_axis_std_rad_s':np.std(gyro,axis=0).tolist(),
            'accel_axis_std_m_s2':np.std(acc,axis=0).tolist()})
    result = {'scope':'Motion/target visibility preflight only; not an accepted joint calibration',
        'session':str(a.session.resolve()),'camera_sha256':hashlib.sha256(a.camera.read_bytes()).hexdigest(),
        'target_sha256':hashlib.sha256(a.target.read_bytes()).hexdigest(),
        'source_analysis_sha256':hashlib.sha256((a.session/'analysis/report.json').read_bytes()).hexdigest(),
        'opencv_version':cv2.__version__,'frame_stride':a.stride,'origin_camera_ns':origin,
        'exposure_change_review':exposure_change,
        'pose_convention':'R and t transform fixed target coordinates to camera optical coordinates',
        'caveats':['PnP moving poses inherit rolling-shutter distortion and corner noise.',
            'Gyro covariance only screens for excitation; full calibration observability remains to be evaluated.',
            'Timestamps remain separate clock-mapped observations; no synchronization is asserted.'],
        'windows':bins}
    (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__': main()
