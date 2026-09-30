#!/usr/bin/python3
"""Verify pinned Kalibr archive and apply recorded Debian compatibility patches."""
import hashlib
import json
import subprocess
import tarfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
lock = json.loads((root / 'config/sources.lock.json').read_text())['kalibr']
source = root / lock['source_directory']
archive = root / f"data/kalibr-{lock['commit'][:8]}.tar.gz"
if not archive.exists():
    archive.parent.mkdir(exist_ok=True)
    partial = archive.with_suffix('.partial')
    subprocess.run(['curl', '-4', '-fL', '--retry', '2', '--connect-timeout', '20',
                    f"https://codeload.github.com/ethz-asl/kalibr/tar.gz/{lock['commit']}",
                    '-o', str(partial)], check=True)
    if hashlib.sha256(partial.read_bytes()).hexdigest() != lock['archive_sha256']:
        raise ValueError('Kalibr archive checksum mismatch')
    partial.replace(archive)
if hashlib.sha256(archive.read_bytes()).hexdigest() != lock['archive_sha256']:
    raise ValueError('Kalibr archive checksum mismatch')
if not source.exists():
    with tarfile.open(archive) as t:
        t.extractall(root / 'vendor', filter='data')
for patch in sorted((root / 'patches').glob('kalibr-*.patch')):
    command = ['patch', '--batch', '-p1', '-i', str(patch)]
    forward = subprocess.run(command + ['--dry-run', '--forward'], cwd=source, capture_output=True)
    if forward.returncode == 0:
        subprocess.run(command + ['--forward'], cwd=source, check=True)
    else:
        reverse = subprocess.run(command + ['--dry-run', '--reverse'], cwd=source, capture_output=True)
        if reverse.returncode:
            raise RuntimeError(f'Patch does not match source: {patch.name}; review local changes')
        print(f'Already applied: {patch.name}')
ws = root / 'build/kalibr-ws/src'
ws.mkdir(parents=True, exist_ok=True)
for dest, target in [(ws / 'kalibr', source),
                     (ws / 'CMakeLists.txt', root / 'deps/usr/share/catkin/cmake/toplevel.cmake')]:
    if not dest.is_symlink() and not dest.exists():
        dest.symlink_to(target)
    if dest.resolve() != target.resolve():
        raise ValueError(f'Unexpected existing workspace entry: {dest}')
