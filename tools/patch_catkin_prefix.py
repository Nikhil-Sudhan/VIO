#!/usr/bin/python3
"""Avoid Debian catkin's /usr-path rewrite corrupting a relocated local prefix."""
from pathlib import Path
import difflib

root=Path(__file__).resolve().parents[1]
p=root/'deps/usr/share/catkin/cmake/catkin_package.cmake'
a=p.read_text()
lines=a.splitlines(True)
matches=[i for i,l in enumerate(lines) if 'string(REGEX REPLACE' in l and 'library_replace ${library}' in l]
if matches:
    if len(matches)!=1:raise RuntimeError('Unexpected catkin version: review before patching')
    backup=root/'evidence/catkin_package.debian-original.cmake'
    if not backup.exists():backup.write_text(a)
    lines[matches[0]]='    set(library_replace "${library}") # local-prefix patch: preserve absolute imported library paths\n'
    b=''.join(lines)
    patch=''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='a/usr/share/catkin/cmake/catkin_package.cmake',tofile='b/usr/share/catkin/cmake/catkin_package.cmake'))
    (root/'patches/catkin-local-prefix.patch').write_text(patch)
    p.write_text(b)
elif 'local-prefix patch:' not in a:
    raise RuntimeError('Unrecognized catkin source')
