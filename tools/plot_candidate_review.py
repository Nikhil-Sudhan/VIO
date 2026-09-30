#!/usr/bin/env python3
"""Compare all saved candidates over identical target-observation timestamps."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from review_saved_candidates import ROOT, read_yaml, summarize

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('review', type=Path)
a = p.parse_args()
output = a.review.resolve()
report = json.loads((output/'review.json').read_text())
base = ROOT/'calibration/lights-return-20260929'
cases = {
    'straight-slide': (ROOT/'data/native-live/20260929_220949_1xjelsnb', base/'straight-slide-target-gyro/observations.json'),
    'later-movement': (ROOT/'data/native-live/20260929_232136_0zyrndg4', base/'user-motion-target-diagnostic/observations.json')}
labels = {'baseline': 'Yesterday baseline', 'all-axis-2340-fit': 'New all-axis fit',
          'tilt-roll-fit': 'New tilt/roll fit', 'all-axis-fixed': 'All-axis, fixed calibration',
          'all-axis-one-corner': 'All-axis, one corner per tag',
          'all-axis-tag-landmarks': 'All-axis, tag landmarks + MSCKF'}
fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
table = []
for row, (case, (source, target)) in enumerate(cases.items()):
    runs = {key: value for key, value in report['runs'].items() if key.startswith(case+'--')}
    poses = {key: np.genfromtxt(output/key/'poses.csv', delimiter=',', names=True) for key in runs}
    window = [max(x['t_rel_s'][0] for x in poses.values()), min(x['t_rel_s'][-1] for x in poses.values())]
    comparison_times = None
    for key, result in runs.items():
        name = key.split('--', 1)[1]
        csv_path = output/key/'target-consistency.csv'
        saved = csv_path.with_name('per-run-target-consistency.csv')
        if not saved.exists():
            csv_path.rename(saved)
        summary = summarize(output/key, result['exit_code'], target, source,
                            read_yaml(output/name/'imucam.yaml')['cam0'], target_window=window)
        common = summary['target_comparison']
        common['identical_output_time_window_s'] = window
        if name == 'all-axis-fixed':
            common['scope'] = common['scope'].replace('although filter refines extrinsic online', 'with extrinsic held fixed in this ablation')
        result['common_window_target_comparison'] = common
        data = np.genfromtxt(csv_path, delimiter=',', names=True)
        if comparison_times is None:
            comparison_times = data['t_rel_s']
        else:
            np.testing.assert_array_equal(comparison_times, data['t_rel_s'])
        t = data['t_rel_s']-comparison_times[0]
        axes[row, 0].plot(t, data['disagreement_m']*100, label=labels[name])
        if name == 'all-axis-2340-fit':
            xyz = np.column_stack([data['vio_'+x] for x in 'xyz'])
            target_xyz = np.column_stack([data['target_'+x] for x in 'xyz'])
            axes[row, 1].plot(t, np.linalg.norm(xyz-xyz[0], axis=1)*100, label='All-axis VIO')
            axes[row, 1].plot(t, np.linalg.norm(target_xyz-target_xyz[0], axis=1)*100, '--', label='Same-image target diagnostic')
        table.append({'case': case, 'configuration': labels[name],
                      'paired_samples': common['paired_samples'],
                      'max_estimated_position_norm_m': result['max_position_norm_m'],
                      'target_disagreement_p95_cm': common['position_disagreement_p50_p95_max_m'][1]*100,
                      'target_disagreement_max_cm': common['position_disagreement_p50_p95_max_m'][2]*100,
                      'accepted_for_hardware_estimation': False})
    axes[row, 0].set(title=case+': same-time comparison', ylabel='Position disagreement (cm)')
    axes[row, 0].set_yscale('symlog', linthresh=1)
    axes[row, 1].set(title=case+': all-axis fit', ylabel='Excursion from first paired pose (cm)')
    for ax in axes[row]:
        ax.set_xlabel('Seconds from common comparison start')
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
fig.suptitle('VIO calibration review — 30 September 2026\nAccuracy NOT validated: same-image target diagnostic; nominal print scale; no independent ground truth', fontsize=12)
fig.savefig(output/'candidate-comparison.png', dpi=150)
fig.savefig(output/'candidate-comparison.pdf')
(output/'review.json').write_text(json.dumps(report, indent=2)+'\n')
(output/'comparison-table.json').write_text(json.dumps(table, indent=2)+'\n')
print(json.dumps(table, indent=2))
