#!/usr/bin/env python3
"""Plot actual exported local landmarks and valid poses from ONE reset epoch."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--epoch', type=int, default=0)
    args = parser.parse_args()
    def read(name):
        with (args.run/name).open() as f:
            return [r for r in csv.DictReader(f) if int(r['reset_epoch']) == args.epoch]
    poses = read('valid-poses.csv')
    landmarks = {r['feature_id']: [float(r[k]) for k in ('x', 'y', 'z')] for r in read('landmarks.csv')}
    if not poses or not landmarks:
        raise ValueError('Selected epoch has no valid poses or mapped landmarks')
    xyz = np.array([[float(r[k]) for k in ('px', 'py', 'pz')] for r in poses])
    points = np.array(list(landmarks.values()))
    fig = plt.figure(figsize=(12, 5), constrained_layout=True)
    ax = fig.add_subplot(121, projection='3d')
    ax.scatter(*points.T, s=12, c='#287db3', label=f'{len(points)} real landmarks')
    ax.plot(*xyz.T, color='#da6b16', label='Valid camera/IMU trajectory')
    ax.set(xlabel='Local X (m)', ylabel='Local Y (m)', zlabel='Local Z (m)', title=f'Sparse local map · epoch {args.epoch}')
    ax.legend(loc='best', fontsize=8)
    bx = fig.add_subplot(122)
    t = np.array([float(r['t_rel_s']) for r in poses])
    for i, label in enumerate('XYZ'):
        bx.plot(t, xyz[:, i], label=label)
    bx.set(xlabel='Recorded time (s)', ylabel='Position (m)', title='Published positions in this local frame')
    bx.grid(alpha=.25); bx.legend()
    fig.suptitle('Recorded natural-feature VIO · no loop closure or independently measured accuracy', fontsize=11)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    main()
