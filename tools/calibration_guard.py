"""Reject missing, changed or reference-only calibration before physical VIO."""
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml


def read_yaml(path):
    return yaml.safe_load('\n'.join(line for line in path.read_text().splitlines() if not line.startswith('%')))


def checked_bundle(config_path,camera_config,experimental=False):
    config_path=Path(config_path).resolve();folder=config_path.parent
    review=json.loads((folder/'review.json').read_text())
    if review.get('invalidated') is True:
        raise ValueError('Calibration invalidated: '+review.get('invalidation_reason','assembly changed; recalibration required'))
    approved = review.get('accepted_for_hardware_estimation') is True
    if experimental:
        approved = approved or (review.get('approved_for_manual_development_demo') is True
                                and review.get('scope') == 'manual development demo; NOT validated navigation')
    if not approved:
        raise ValueError('Joint calibration has not been accepted for hardware estimation')
    if review.get('camera_config_sha256') != hashlib.sha256(Path(camera_config).read_bytes()).hexdigest():
        raise ValueError('Camera configuration changed since joint calibration')
    if not review.get('mounting_id') or not review.get('supporting_reports'):
        raise ValueError('Calibration needs assembly identity and supporting reports')
    files=review['files_sha256']
    config=read_yaml(config_path)
    paths=[config_path,folder/config['relative_config_imu'],folder/config['relative_config_imucam']]
    for path in paths:
        path=path.resolve()
        if path.parent != folder or files.get(path.name) != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError('Calibration file differs from its reviewed hash')
    for name,expected in review['supporting_reports'].items():
        path=(folder/name).resolve()
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError('Supporting report missing or changed')
    if config['max_cameras']!=1 or config.get('use_mask',False):
        raise ValueError('Physical adapter currently requires one camera without a dataset mask')
    camera=read_yaml(paths[2])['cam0']
    imu=read_yaml(paths[1])['imu0']
    intrinsics=np.asarray(camera['intrinsics'],float)
    distortion=np.asarray(camera['distortion_coeffs'],float)
    if camera['camera_model']!='pinhole' or camera['distortion_model'] not in ('radtan','equidistant'):
        raise ValueError('Unsupported camera model')
    if intrinsics.shape!=(4,) or distortion.shape!=(4,) or not np.isfinite(intrinsics).all() or not np.isfinite(distortion).all() or np.any(intrinsics[:2]<=0):
        raise ValueError('Invalid camera intrinsics/distortion')
    matrix=np.asarray(camera['T_cam_imu'],float)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3],[0,0,0,1],atol=1e-8):
        raise ValueError('Invalid IMU-to-camera homogeneous transform')
    R=matrix[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(R),1,atol=1e-5):
        raise ValueError('Extrinsic rotation is not a proper rotation')
    if not np.isfinite(camera['timeshift_cam_imu']) or abs(camera['timeshift_cam_imu'])>.5:
        raise ValueError('Camera-to-IMU time offset is missing or outside buffer range')
    for key in ['gyroscope_noise_density','gyroscope_random_walk','accelerometer_noise_density','accelerometer_random_walk','update_rate']:
        if not np.isfinite(imu[key]) or imu[key]<=0: raise ValueError(f'Missing/invalid measured noise model field {key}')
    if camera['resolution']!=json.loads(Path(camera_config).read_text())['main']['size']:
        raise ValueError('Calibrated image resolution mismatch')
    if not review.get('imu_configured_registers'):
        raise ValueError('Reviewed IMU register configuration missing')
    return review
