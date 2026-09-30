#!/usr/bin/env python3
"""Fetch the pinned bench wheels; no controller or camera access."""
import hashlib
import json
from pathlib import Path
import urllib.request

root = Path(__file__).resolve().parents[1]
folder = root/'data/bench-wheels'
folder.mkdir(parents=True, exist_ok=True)
for package in json.loads((root/'config/bench-deps.lock.json').read_text())['packages']:
    path = folder/package['filename']
    if path.exists():
        if hashlib.sha256(path.read_bytes()).hexdigest() != package['sha256']:
            raise RuntimeError(f'Existing wheel checksum differs: {path}; preserved for inspection')
        continue
    data = urllib.request.urlopen(package['url'], timeout=120).read()
    if hashlib.sha256(data).hexdigest() != package['sha256']:
        raise RuntimeError(f'Download checksum differs: {path}')
    with path.open('xb') as f:f.write(data)
    print(path)
