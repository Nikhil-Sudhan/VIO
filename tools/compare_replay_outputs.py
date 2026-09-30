#!/usr/bin/env python3
"""Strict numerical regression for an implementation-only estimator change."""
import argparse
import json
from pathlib import Path
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('before', type=Path)
p.add_argument('after', type=Path)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
old = np.genfromtxt(a.before/'poses.csv', delimiter=',', names=True)
new = np.genfromtxt(a.after/'poses.csv', delimiter=',', names=True)
assert old.shape == new.shape and old.size > 0, 'Different pose counts'
assert old.dtype.names == new.dtype.names, 'Different pose fields'
np.testing.assert_array_equal(old['t_rel_s'], new['t_rel_s'])
np.testing.assert_array_equal(old['msckf_features'], new['msckf_features'])
errors = {}
for field in old.dtype.names:
    if not np.isfinite(old[field]).all() or not np.isfinite(new[field]).all():
        raise ValueError('Nonfinite pose field '+field)
    errors[field] = float(np.max(np.abs(old[field]-new[field])))
    if field not in ('t_rel_s', 'msckf_features'):
        np.testing.assert_allclose(old[field], new[field], rtol=1e-8, atol=1e-6)
cold = np.loadtxt(a.before/'covariance.csv', delimiter=',', comments='#')
cnew = np.loadtxt(a.after/'covariance.csv', delimiter=',', comments='#')
np.testing.assert_allclose(cold, cnew, rtol=1e-7, atol=1e-8)
report = {'passed': True, 'poses': int(old.size), 'identical_state_times_and_feature_counts': True,
          'max_absolute_difference_by_pose_field': errors,
          'max_absolute_covariance_difference': float(np.max(np.abs(cold-cnew))),
          'limits': {'pose_rtol': 1e-8, 'pose_atol': 1e-6, 'covariance_rtol': 1e-7, 'covariance_atol': 1e-8},
          'scope': 'Numerical regression only, not a physical accuracy measurement.'}
a.output.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
