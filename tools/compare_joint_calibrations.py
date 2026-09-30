#!/usr/bin/env python3
"""Archive and compare fitted Kalibr candidates without accepting a calibration."""
import argparse
import hashlib
import itertools
import json
import re
import shutil
from pathlib import Path

import numpy as np
import yaml


def read_candidate(folder):
    camera = yaml.safe_load((folder / 'input-camchain-imucam.yaml').read_text())['cam0']
    transform = np.asarray(camera['T_cam_imu'], dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f'Invalid transform: {folder}')
    rotation = transform[:3, :3]
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8) or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6):
        raise ValueError(f'Improper rigid transform: {folder}')
    dt = float(camera['timeshift_cam_imu'])
    if not np.isfinite(dt):
        raise ValueError(f'Nonfinite offset: {folder}')
    text = (folder / 'input-results-imucam.txt').read_text()
    residuals = {}
    for label, unit in [('Reprojection', 'px'), ('Gyroscope', 'rad/s'), ('Accelerometer', 'm/s^2')]:
        pattern = rf'{label} error .*?\[{re.escape(unit)}\]:\s*mean ([\d.eE+-]+), median ([\d.eE+-]+), std: ([\d.eE+-]+)'
        match = re.search(pattern, text)
        if not match:
            raise ValueError(f'Missing {label} residual summary: {folder}')
        mean, median, std = map(float, match.groups())
        if not np.isfinite([mean, median, std]).all():
            raise ValueError(f'Nonfinite residual summary: {folder}')
        residuals[label.lower()] = dict(unit=unit, mean_norm=mean, median_norm=median, std_norm=std, rms_norm=float(np.hypot(mean, std)))
    return dict(source=str(folder.resolve()), T_cam_imu=transform.tolist(),
                camera_to_imu_shift_s=dt, separation_m=float(np.linalg.norm(transform[:3, 3])),
                residuals=residuals, run_arguments=(folder / 'run-arguments.txt').read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('attempts', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    # Validate every input before creating a new archive. Never overwrite results.
    candidates = {p.name: read_candidate(p) for p in args.attempts}
    if len(candidates) != len(args.attempts):
        raise ValueError('Attempt directory names must be unique')
    args.output.mkdir()
    archive_names = ['input-camchain-imucam.yaml', 'input-imu.yaml', 'input-results-imucam.txt',
                     'input-report-imucam.pdf', 'noise-input.yaml', 'camera-input.yaml',
                     'target-input.yaml', 'run-arguments.txt', 'UNCERTAINTY_LIMITATION.txt']
    hashes = {}
    for folder in args.attempts:
        target = args.output / folder.name
        target.mkdir()
        for name in archive_names:
            shutil.copy2(folder / name, target / name)
            hashes[str((target / name).relative_to(args.output))] = hashlib.sha256((target / name).read_bytes()).hexdigest()
    comparisons = []
    for (name_a, a), (name_b, b) in itertools.combinations(candidates.items(), 2):
        ta, tb = np.array(a['T_cam_imu']), np.array(b['T_cam_imu'])
        cosine = np.clip((np.trace(ta[:3, :3] @ tb[:3, :3].T) - 1) / 2, -1, 1)
        comparisons.append(dict(a=name_a, b=name_b,
            rotation_difference_deg=float(np.degrees(np.arccos(cosine))),
            translation_difference_mm=float(1000 * np.linalg.norm(ta[:3, 3] - tb[:3, 3])),
            time_shift_difference_ms=1000 * abs(a['camera_to_imu_shift_s'] - b['camera_to_imu_shift_s'])))
    report = dict(accepted_for_hardware_estimation=False,
        scope='Candidate sensitivity/repeatability comparison; no automatic acceptance or statistical covariance',
        time_convention='t_imu = t_camera + timeshift_cam_imu; effective fitted offset, not established exposure midpoint',
        candidates=candidates, pairwise_comparisons=comparisons, files_sha256=hashes,
        limitations=['Shared source recording, camera intrinsics and measured screen scale create shared systematic errors.',
                     'Weight variants fit identical measurements and are not independent repeated calibrations.',
                     'Nonoverlapping intervals check motion dependence but are not a new acquisition or remounting test.',
                     'Factory IMU scale and axis alignment retained; bias splines estimated. Long-term sensor noise identification is incomplete.',
                     'Analytic covariance recovery is unavailable in the pinned native Kalibr path.',
                     'Approximately 12 ms rolling shutter is not modeled by OpenVINS or these joint fits.',
                     'Physical transform plausibility and independent replay/live validation remain required.'])
    (args.output / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(comparisons, indent=2))


if __name__ == '__main__':
    main()
