#!/usr/bin/env python3
"""Back up a bounded RAM capture while recording, then preserve its final files."""
import argparse
import json
import os
from pathlib import Path
import shutil
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('destination',type=Path)
a=p.parse_args()
if not str(a.source.resolve()).startswith('/dev/shm/'):
    raise ValueError('Expected a temporary RAM capture source')
a.destination.mkdir()
marker=a.destination/'RAM_BACKUP_STATUS.json'
marker.write_text(json.dumps({'complete':False,'source':str(a.source)})+'\n')
copied={}
deadline=time.monotonic()+600
while time.monotonic()<deadline:
    state=json.loads((a.source/'session.json').read_text())
    final='clock_end' in state and state['status'] in ('completed','failed','interrupted')
    for directory,_,files in os.walk(a.source,followlinks=True):
        for name in files:
            src=Path(directory)/name
            if '.tmp' in name or name.endswith('.saving'):continue
            relative=src.relative_to(a.source)
            try:
                stat=src.stat();identity=(stat.st_size,stat.st_mtime_ns)
                if copied.get(str(relative))==identity:continue
                dst=a.destination/relative;dst.parent.mkdir(parents=True,exist_ok=True)
                temporary=dst.with_name(dst.name+'.copying')
                shutil.copy2(src,temporary);temporary.replace(dst)
                copied[str(relative)]=identity
            except FileNotFoundError:
                if final:raise
    if final:
        marker.write_text(json.dumps({'complete':True,'source':str(a.source),
            'original_session_status':state['status'],'files':len(copied)},indent=2)+'\n')
        pointer=Path(__file__).resolve().parents[1]/'data/native-control/current-session.txt'
        if pointer.read_text().strip()==str(a.source):pointer.write_text(str(a.destination)+'\n')
        print('Final capture preserved:',a.destination,flush=True)
        break
    time.sleep(3)
else:
    raise RuntimeError('Capture did not finish within backup window; partial disk backup retained')
