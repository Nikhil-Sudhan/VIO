#!/usr/bin/env python3
"""Summarize supervised VIO evidence, keeping reset coordinate frames separate."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def review(folder):
    tracking = rows(folder/'tracking.csv')
    poses = rows(folder/'valid-poses.csv')
    events = rows(folder/'epochs.csv')
    landmarks = rows(folder/'landmarks.csv')
    segments = []
    for epoch in sorted({int(r['reset_epoch']) for r in tracking}):
        health = [r for r in tracking if int(r['reset_epoch']) == epoch]
        valid = [r for r in poses if int(r['reset_epoch']) == epoch]
        points = {r['feature_id']: [float(r[k]) for k in ('x', 'y', 'z')]
                  for r in landmarks if int(r['reset_epoch']) == epoch}
        item = dict(epoch=epoch, camera_frames=len(health), valid_pose_count=len(valid),
                    states=dict(Counter(r['state'] for r in health)),
                    lost_events=[r for r in events if int(r['reset_epoch']) == epoch and r['event'] == 'lost'],
                    persistent_landmarks=len(points))
        if valid:
            xyz = np.array([[float(r[k]) for k in ('px', 'py', 'pz')] for r in valid])
            velocity = np.array([[float(r[k]) for k in ('vx', 'vy', 'vz')] for r in valid])
            item.update(first_valid_time_s=float(valid[0]['t_rel_s']), last_valid_time_s=float(valid[-1]['t_rel_s']),
                        max_position_norm_m=float(np.linalg.norm(xyz, axis=1).max()),
                        final_position_norm_m=float(np.linalg.norm(xyz[-1])),
                        max_speed_m_s=float(np.linalg.norm(velocity, axis=1).max()),
                        final_speed_m_s=float(np.linalg.norm(velocity[-1])))
        segments.append(item)
    scope = 'Recorded-data software evaluation. Health gates are not ground truth or accuracy validation.'
    manifest = (folder/'run.txt').read_text().splitlines()
    if 'mode=live' in manifest:
        scope = 'Physical live-stream software evidence. Health gates alone do not validate moving accuracy or mapping.'
    for line in manifest:
        if line.startswith('images='):
            source = Path(line.split('=', 1)[1])
            if not source.is_absolute():
                source = Path(__file__).resolve().parents[1]/source
            fixture = source.parent/'fixture.json'
            if fixture.exists():
                scope = json.loads(fixture.read_text())['scope']
    return dict(scope=scope,
                folder=str(folder.resolve()), processed_frames=len(tracking),
                valid_pose_count=len(poses), segments=segments,
                lost_events=[r for r in events if r['event'] == 'lost'],
                map_scope='Sparse local landmarks only. Independent reset frames are not registered; no loop closure.',
                navigation_validated=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = review(args.folder)
    text = json.dumps(report, indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
