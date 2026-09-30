"""Read saved/live measurements for the desktop app; never opens sensors."""
import csv
import io
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

cv2.setNumThreads(1)


class SessionView:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.image_path = None
        self.jpeg = b''
        self.image_info = {}
        self.rows = []
        self.pose_offset = 0
        self.pose_header = None
        self.pose_pending = ''
        self.log_offset = 0
        self.log_pending = ''
        self.static_initialized = False
        self.last_init_message = ''
        self.last_error = ''

    def read_poses(self):
        path = self.folder / 'vio/poses.csv'
        if not path.exists():
            return
        with path.open() as f:
            f.seek(self.pose_offset)
            text = self.pose_pending + f.read()
            self.pose_offset = f.tell()
        lines = text.split('\n')
        self.pose_pending = lines.pop()
        for line in lines:
            if not line:
                continue
            fields = next(csv.reader([line]))
            if self.pose_header is None:
                self.pose_header = fields
                continue
            if len(fields) != len(self.pose_header):
                raise ValueError('Malformed pose row; refusing a misleading display')
            row = dict(zip(self.pose_header, map(float, fields)))
            if not all(math.isfinite(x) for x in row.values()):
                raise ValueError('Nonfinite estimator output')
            self.rows.append(row)

    def read_log(self):
        path = self.folder / 'estimator.log'
        if not path.exists():
            return
        with path.open(errors='replace') as f:
            f.seek(self.log_offset)
            text = self.log_pending + f.read()
            self.log_offset = f.tell()
        lines = text.split('\n')
        self.log_pending = lines.pop()
        for line in lines:
            if '[init]: successful initialization' in line:
                self.static_initialized = True
            if '[init]:' in line or '[init-s]:' in line:
                self.last_init_message = line[-300:]
            if 'ERROR:' in line or 'StateHelper::EKFUpdate() - diagonal' in line:
                self.last_error = line[-500:]

    def snapshot(self):
        info = json.loads((self.folder / 'session.json').read_text())
        self.read_log()
        self.read_poses()
        files = sorted((self.folder / 'images').glob('*.pgm'))
        image_age = None
        if files:
            # The newest file may still be being written. After shutdown it is safe.
            live = info['status'] in ('starting', 'recording')
            latest = files[-2] if live and len(files) > 1 else files[-1]
            image_age = max(0, time.time() - latest.stat().st_mtime)
            if latest != self.image_path:
                im = cv2.imread(str(latest), cv2.IMREAD_GRAYSCALE)
                if im is not None:
                    points = cv2.FastFeatureDetector_create(20).detect(im)
                    h, w = im.shape
                    cells = {(min(4, int(p.pt[0]*5/w)), min(4, int(p.pt[1]*5/h))) for p in points}
                    self.image_info = dict(sequence=int(latest.stem), corners=len(points),
                                           occupied_cells=len(cells), median=float(np.median(im)))
                    ok, encoded = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok:
                        self.jpeg = encoded.tobytes()
                        self.image_path = latest
        pose = None
        trajectory = []
        recent_visual = False
        if self.rows:
            r = self.rows[-1]
            p = [r[k] for k in ('px', 'py', 'pz')]
            v = [r[k] for k in ('vx', 'vy', 'vz')]
            pose = dict(t=r['t_rel_s'], position=p, velocity=v,
                        quaternion_xyzw=[r[k] for k in ('qx', 'qy', 'qz', 'qw')],
                        speed=math.sqrt(sum(x*x for x in v)), msckf_features=int(r['msckf_features']))
            recent_visual = any(x['msckf_features'] > 0 for x in self.rows[-40:])
            stride = max(1, math.ceil(len(self.rows)/2000))
            trajectory = [[x[k] for k in ('px', 'py', 'pz')] for x in self.rows[::stride]]
            if trajectory[-1] != p:
                trajectory.append(p)
        s = dict(session=str(self.folder), capture_status=info['status'], image=self.image_info,
                 image_age_s=image_age, pose=pose, pose_count=len(self.rows), trajectory=trajectory,
                 static_initialized=self.static_initialized, stage='waiting',
                 title='Waiting for camera', instruction='Leave the rig resting.',
                 detail='', accuracy_validated=False, saved=False)
        if not info.get('estimator_config'):
            s.update(stage='preview', title='Camera check — no position estimate',
                     instruction='Check that distinct objects are clear across the image.')
        elif not self.static_initialized:
            s.update(title='Hold still — waiting for stationary initialization',
                     instruction='Rest the whole rig on a stable surface. Do not pick it up yet.')
        elif pose is None:
            if self.image_info.get('corners', 0) >= 100 and self.image_info.get('occupied_cells', 0) >= 12:
                s.update(stage='ready', title='Stationary initialization complete',
                         instruction='Move the whole base sideways about 20 cm over 3 seconds, keeping the camera pointed the same way. Then set it down.')
            else:
                s.update(stage='weak', title='Initialized, but the image has too little distinct detail',
                         instruction='Stop this run before changing the scene. Use several detailed objects across the view.')
        else:
            s.update(stage='estimate', title='Experimental OpenVINS estimates — accuracy unvalidated',
                     instruction='Set the rig down and observe whether estimated motion settles.')
            if not recent_visual:
                s.update(stage='weak', title='Visual support not established — do not trust this path',
                         detail='No recent MSCKF feature constraints. This alone does not measure all SLAM updates.')
            if pose['speed'] > 3 or sum(x*x for x in pose['position']) > 100:
                s.update(stage='failed', title='Unreliable motion estimate for this short demo',
                         instruction='Stop this run. The saved logs will retain the failure.')
        if self.last_error:
            s.update(stage='failed', title='Estimator error', detail=self.last_error,
                     instruction='Stop and review the saved error. No automatic restart is performed.')
        if info['status'] not in ('recording', 'starting'):
            s.update(saved=True, title='Saved capture: '+info['status']+' — not live',
                     instruction='Start a new run to capture fresh data.')
            if info.get('errors'):
                s.update(stage='failed', detail=info['errors'][-1][-1600:])
        elif image_age is not None and image_age > 2:
            s.update(stage='stale', title='Camera stale — no fresh image', instruction='Stop and inspect the saved capture error.')
        elif pose and time.time() - (self.folder/'vio/poses.csv').stat().st_mtime > 2:
            s.update(stage='stale', title='Position stale — camera may still be updating', instruction='Do not treat this position as live.')
        timing = self.folder / 'vio/timing.csv'
        if timing.exists():
            with timing.open('rb') as f:
                f.seek(0, 2); end = f.tell(); f.seek(max(0, end-2048))
                lines = f.read().decode(errors='replace').split('\n')
            if len(lines) >= 3:
                try:
                    fields = lines[-2].split(',')
                    s['timing'] = dict(processing_ms=float(fields[2]), sensor_to_output_ms=float(fields[5]),
                                       pending_frames=int(fields[7]))
                except (ValueError, IndexError):
                    pass
        return s
