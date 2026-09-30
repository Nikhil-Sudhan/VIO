#!/usr/bin/env python3
"""Plot the reviewed physical run, retaining failures and diagnostic limits."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('review', type=Path)
a = p.parse_args()
report = json.loads((a.review/'live-review.json').read_text())
session = Path(report['session'])
data = np.genfromtxt(session/'vio/target-consistency.csv', delimiter=',', names=True)
timing = np.genfromtxt(session/'vio/timing.csv', delimiter=',', names=True)
xyz = np.column_stack([data['vio_'+n] for n in 'xyz'])
target = np.column_stack([data['target_'+n] for n in 'xyz'])
xyz -= xyz[0]
target -= target[0]
fig, ax = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
ax[0, 0].plot(xyz[:, 0]*100, xyz[:, 1]*100, label='Actual OpenVINS output')
ax[0, 0].plot(target[:, 0]*100, target[:, 1]*100, '--', label='Same-image target diagnostic')
ax[0, 0].scatter(xyz[0, 0], xyz[0, 1], color='green', label='Comparison start')
ax[0, 0].set(title='Recorded path before the stop', xlabel='Local X (cm)', ylabel='Local Y (cm)', aspect='equal')
ax[0, 0].legend(fontsize=8)
ax[0, 1].plot(data['t_rel_s'], data['disagreement_m']*100)
ax[0, 1].set(title='Position disagreement, not ground-truth error', ylabel='Disagreement (cm)', xlabel='Estimator time (s)')
ax[1, 0].plot(timing['t_rel_s'], timing['sensor_to_output_ms'], label='Sensor timestamp to output')
ax[1, 0].axhline(150, color='orange', linestyle='--', label='150 ms target')
ax[1, 0].axhline(1000, color='red', linestyle='--', label='1 second freshness limit')
ax[1, 0].set(title='Latency grew until the stream stopped', ylabel='Latency (ms)', xlabel='Estimator time (s)')
ax[1, 0].legend(fontsize=8)
ax[1, 1].plot(timing['t_rel_s'], timing['processing_ms'], label='Filter processing per frame')
ax[1, 1].axhline(50, color='red', linestyle='--', label='20 Hz frame interval')
ax[1, 1].set(title='Processing time', ylabel='Time (ms)', xlabel='Estimator time (s)')
ax[1, 1].legend(fontsize=8)
for axis in ax.flat:
    axis.grid(alpha=.25)
fig.suptitle('Actual live VIO — 30 September 2026 — FAILED: input backlog\nNominal print scale; same-image target diagnostic; full out-and-back not confirmed in capture', fontsize=12)
fig.savefig(a.review/'live-result.png', dpi=150)
fig.savefig(a.review/'live-result.pdf')
print(a.review/'live-result.png')
