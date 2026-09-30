#!/usr/bin/env python3
"""Controlled candidate ablations, each preserving all unrelated settings."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
from review_saved_candidates import ROOT, read_yaml, sha, summarize

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('review', type=Path)
p.add_argument('--variant', choices=['fixed', 'one-corner', 'tag-landmarks'], default='fixed')
a = p.parse_args()
output = a.review.resolve()
source_config = output/'all-axis-2340-fit'
variant_name = 'all-axis-'+a.variant
cfg = output/variant_name
cfg.mkdir()
for name in ['estimator_config.yaml', 'imucam.yaml', 'imu.yaml', 'review.json']:
    shutil.copy2(source_config/name, cfg/name)
text = (cfg/'estimator_config.yaml').read_text()
if a.variant == 'fixed':
    for key in ['calib_cam_extrinsics', 'calib_cam_timeoffset']:
        assert text.count(key+': true') == 1
        text = text.replace(key+': true', key+': false')
elif a.variant == 'one-corner':
    assert 'aprilgrid_corners_per_tag:' not in text
    text += '\naprilgrid_corners_per_tag: 1\n'
else:
    assert text.count('max_slam: 50') == 1
    # The upstream cap applies only to ordinary persistent landmarks. Keep all
    # four corners of every tag, and keep ordinary KLT/MSCKF updates enabled.
    text = text.replace('max_slam: 50', 'max_slam: 0')
(cfg/'estimator_config.yaml').write_text(text)
base = ROOT/'calibration/lights-return-20260929'
cases = {
    'straight-slide': (ROOT/'data/straight-slide-motion-input-20260929',
                       ROOT/'data/native-live/20260929_220949_1xjelsnb', base/'straight-slide-target-gyro/observations.json'),
    'later-movement': (ROOT/'data/user-motion-selected-input-20260929',
                       ROOT/'data/native-live/20260929_232136_0zyrndg4', base/'user-motion-target-diagnostic/observations.json')}
for case, (inputs, source, target) in cases.items():
    key = case+'--'+variant_name
    folder = output/key
    command = [str(ROOT/'build/adapter/vio_replay'), str(cfg/'estimator_config.yaml'),
               str(inputs/'images.csv'), str(inputs/'imu.csv'), str(folder), '0']
    print('START '+key, flush=True)
    with (output/(key+'.log')).open('x') as log:
        code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
            env=os.environ | {'VIO_DIAGNOSTICS': '1', 'VIO_DIAGNOSTICS_EVERY': '20'}, timeout=600).returncode
    result = summarize(folder, code, target, source, read_yaml(cfg/'imucam.yaml')['cam0'])
    result['command'] = command
    result['input_sha256'] = {x: sha(inputs/x) for x in ['images.csv', 'imu.csv']}
    result['configuration_sha256'] = {x: sha(cfg/x) for x in ['estimator_config.yaml', 'imu.yaml', 'imucam.yaml']}
    (folder/'review.json').write_text(json.dumps(result, indent=2)+'\n')
    report = json.loads((output/'review.json').read_text())
    report['runs'][key] = result
    (output/'review.json').write_text(json.dumps(report, indent=2)+'\n')
    print('DONE '+key+' '+json.dumps(result), flush=True)
