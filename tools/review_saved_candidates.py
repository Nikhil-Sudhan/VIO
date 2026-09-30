#!/usr/bin/env python3
"""Compare saved calibration candidates on identical held-out raw inputs.

Creates new offline-only bundles and preserves every replay and failure.
The target comparison uses the same images and nominal print scale: it is
a consistency diagnostic, never independent ground truth or acceptance.
"""
import argparse
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

from compare_joint_calibrations import read_candidate

ROOT = Path(__file__).resolve().parents[1]


def read_yaml(path):
    return yaml.safe_load('\n'.join(x for x in path.read_text().splitlines()
                                    if not x.startswith('%')))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(folder, code, target, source, camera, target_window=None):
    result = {'exit_code': code, 'completed': code == 0,
              'accepted_for_hardware_estimation': False}
    poses = np.atleast_1d(np.genfromtxt(folder/'poses.csv', delimiter=',', names=True))
    if not len(poses):
        return result | {'poses': 0}
    xyz = np.column_stack([poses['p'+x] for x in 'xyz'])
    speed = np.linalg.norm(np.column_stack([poses['v'+x] for x in 'xyz']), axis=1)
    if not np.isfinite(xyz).all() or not np.isfinite(speed).all():
        raise ValueError('Nonfinite replay output')
    result.update(poses=len(poses),
                  max_position_norm_m=float(np.linalg.norm(xyz, axis=1).max()),
                  max_excursion_from_first_pose_m=float(np.linalg.norm(xyz-xyz[0], axis=1).max()),
                  endpoint_from_first_pose_m=float(np.linalg.norm(xyz[-1]-xyz[0])),
                  max_speed_m_s=float(speed.max()),
                  pose_frames_with_msckf=int(np.count_nonzero(poses['msckf_features'])),
                  final_time_offset_s=float(poses['dt_cam_to_imu_s'][-1]))
    audit = (np.atleast_1d(np.genfromtxt(folder/'track-audit.csv', delimiter=',', names=True))
             if (folder/'track-audit.csv').exists() else [])
    if len(audit):
        result['max_slam_landmarks'] = int(audit['slam_landmarks'].max())
        result['sampled_zupt_fraction'] = float(audit['did_zupt'].mean())
    timing = np.atleast_1d(np.genfromtxt(folder/'timing.csv', delimiter=',', names=True))
    result['processed_frames'] = len(timing)
    result['offline_processing_p50_p95_max_ms'] = np.percentile(timing['processing_ms'], [50, 95, 100]).tolist()
    if target is None:
        return result
    manifest = dict(x.split('=', 1) for x in (folder/'run.txt').read_text().splitlines() if '=' in x)
    with (source/'analysis/imu_derived.csv').open() as f:
        import csv
        source_origin = int(next(csv.DictReader(f))['mapped_boot_ns'])
    offset = (source_origin-int(manifest['origin_ns']))*1e-9
    obs = json.loads(target.read_text())
    obs = [x for x in obs if poses['t_rel_s'][0] <= x['t']+offset <= poses['t_rel_s'][-1]]
    if target_window is not None:
        obs = [x for x in obs if target_window[0] <= x['t']+offset <= target_window[1]]
    if len(obs) < 2:
        result['target_comparison'] = {'paired_samples': len(obs), 'available': False}
        return result
    times = np.array([x['t']+offset for x in obs])
    # Do not interpolate through estimator outages.
    idx = np.searchsorted(poses['t_rel_s'], times).clip(1, len(poses)-1)
    keep = poses['t_rel_s'][idx]-poses['t_rel_s'][idx-1] < .101
    times = times[keep]
    obs = [x for x, k in zip(obs, keep) if k]
    if len(obs) < 2:
        result['target_comparison'] = {'paired_samples': len(obs), 'available': False}
        return result
    Tci = np.array(camera['T_cam_imu'])
    target_poses = []
    for x in obs:
        Tcb = np.eye(4)
        Tcb[:3, :3] = Rotation.from_rotvec(x['rvec']).as_matrix()
        Tcb[:3, 3] = x['tvec']
        target_poses.append(np.linalg.inv(Tcb) @ Tci)
    target_poses = np.array(target_poses)
    vio_xyz = np.column_stack([np.interp(times, poses['t_rel_s'], xyz[:, i]) for i in range(3)])
    vio_rot = Slerp(poses['t_rel_s'], Rotation.from_quat(np.column_stack(
        [poses['q'+x] for x in 'xyzw'])))(times).as_matrix()
    alignment = vio_rot[0] @ target_poses[0, :3, :3].T
    aligned = (alignment @ (target_poses[:, :3, 3]-target_poses[0, :3, 3]).T).T+vio_xyz[0]
    errors = np.linalg.norm(vio_xyz-aligned, axis=1)
    angles = Rotation.from_matrix(vio_rot @ np.transpose(alignment @ target_poses[:, :3, :3], (0, 2, 1))).magnitude()*180/np.pi
    result['target_comparison'] = {
        'scope': 'Same-image PnP, nominal 20 mm tags; fixed initial extrinsic for lever arm although filter refines extrinsic online. One initial rigid pose alignment, no fitted scale; NOT independent ground truth.',
        'paired_samples': len(obs), 'available': True,
        'position_disagreement_p50_p95_max_m': np.percentile(errors, [50, 95, 100]).tolist(),
        'orientation_disagreement_p50_p95_max_deg': np.percentile(angles, [50, 95, 100]).tolist(),
        'target_implied_max_excursion_m': float(np.linalg.norm(aligned-aligned[0], axis=1).max()),
        'estimated_paired_max_excursion_m': float(np.linalg.norm(vio_xyz-vio_xyz[0], axis=1).max()),
        'target_implied_endpoint_m': float(np.linalg.norm(aligned[-1]-aligned[0])),
        'estimated_paired_endpoint_m': float(np.linalg.norm(vio_xyz[-1]-vio_xyz[0])),
        'last_disagreement_m': float(errors[-1])}
    np.savetxt(folder/'target-consistency.csv', np.column_stack([times, vio_xyz, aligned, errors, angles]),
               delimiter=',', header='t_rel_s,vio_x,vio_y,vio_z,target_x,target_y,target_z,disagreement_m,disagreement_deg', comments='')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    output = a.output.resolve()
    output.mkdir()
    base = ROOT/'calibration/lights-return-20260929'
    default = base/'manual-grid-vio-fast'
    fits = {name: read_candidate(base/name) for name in ['all-axis-2340-fit', 'tilt-roll-fit']}
    comparison = []
    for (na, ca), (nb, cb) in itertools.combinations(fits.items(), 2):
        ta, tb = np.array(ca['T_cam_imu']), np.array(cb['T_cam_imu'])
        comparison.append({'a': na, 'b': nb,
            'rotation_difference_deg': float(Rotation.from_matrix(ta[:3, :3] @ tb[:3, :3].T).magnitude()*180/np.pi),
            'translation_difference_mm': float(np.linalg.norm(ta[:3, 3]-tb[:3, 3])*1000),
            'time_offset_difference_ms': abs(ca['camera_to_imu_shift_s']-cb['camera_to_imu_shift_s'])*1000})
    report = {'scope': 'Offline calibration review on two separate earlier recordings; no automatic promotion.',
              'accepted_for_hardware_estimation': False, 'fits': fits, 'pairwise_comparisons': comparison,
              'binary_sha256': sha(ROOT/'build/adapter/vio_replay'), 'runs': {},
              'limitations': ['Nominal unmeasured printed target scale.', 'Provisional 10x noise weights.',
                              'No independent trajectory ground truth.', 'No analytic calibration covariance.',
                              'Rolling shutter is unmodeled.']}
    cases = {
        'straight-slide': (ROOT/'data/straight-slide-motion-input-20260929',
                           ROOT/'data/native-live/20260929_220949_1xjelsnb', base/'straight-slide-target-gyro/observations.json'),
        'later-movement': (ROOT/'data/user-motion-selected-input-20260929',
                           ROOT/'data/native-live/20260929_232136_0zyrndg4', base/'user-motion-target-diagnostic/observations.json')}
    configs = {}
    for name in ['baseline', *fits]:
        cfg = output/name
        cfg.mkdir()
        for filename in ['estimator_config.yaml', 'imu.yaml', 'imucam.yaml']:
            shutil.copy2(default/filename, cfg/filename)
        if name != 'baseline':
            doc = read_yaml(base/name/'input-camchain-imucam.yaml')
            doc['cam0']['cam_overlaps'] = []
            # OpenCV requires indented sequence entries; preserve numeric bytes.
            class Dumper(yaml.SafeDumper):
                def increase_indent(self, flow=False, indentless=False):
                    return super().increase_indent(flow, False)
            (cfg/'imucam.yaml').write_text('%YAML:1.0\n---\n'+yaml.dump(doc, Dumper=Dumper, default_flow_style=None, sort_keys=False))
            assert read_yaml(cfg/'imucam.yaml') == doc
        (cfg/'review.json').write_text(json.dumps({'accepted_for_hardware_estimation': False,
            'scope': 'Offline candidate only; not authorized by runtime calibration guard.'}, indent=2)+'\n')
        configs[name] = cfg
    for case, (inputs, source, target) in cases.items():
        for name, cfg in configs.items():
            key = case+'--'+name
            folder = output/key
            command = [str(ROOT/'build/adapter/vio_replay'), str(cfg/'estimator_config.yaml'),
                       str(inputs/'images.csv'), str(inputs/'imu.csv'), str(folder), '0']
            print('START '+key, flush=True)
            started = time.monotonic()
            with (output/(key+'.log')).open('x') as log:
                code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                    env=os.environ | {'VIO_DIAGNOSTICS': '1', 'VIO_DIAGNOSTICS_EVERY': '20'}, timeout=600).returncode
            result = summarize(folder, code, target, source, read_yaml(cfg/'imucam.yaml')['cam0'])
            result['wall_seconds'] = time.monotonic()-started
            result['command'] = command
            result['input_sha256'] = {x: sha(inputs/x) for x in ['images.csv', 'imu.csv']}
            result['configuration_sha256'] = {x: sha(cfg/x) for x in ['estimator_config.yaml', 'imu.yaml', 'imucam.yaml']}
            (folder/'review.json').write_text(json.dumps(result, indent=2)+'\n')
            report['runs'][key] = result
            (output/'review.json').write_text(json.dumps(report, indent=2)+'\n')
            print('DONE '+key+' '+json.dumps({k: v for k, v in result.items() if k not in ['command', 'input_sha256', 'configuration_sha256']}), flush=True)
    print('REVIEW COMPLETE '+str(output/'review.json'), flush=True)


if __name__ == '__main__':
    main()
