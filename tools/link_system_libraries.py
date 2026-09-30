#!/usr/bin/python3
"""Complete extracted development-package symlinks using installed native runtimes."""
from pathlib import Path
import json

root = Path(__file__).resolve().parents[1]
records = []
for directory in ('usr/lib', 'usr/lib/aarch64-linux-gnu', 'usr/lib/aarch64-linux-gnu/blas', 'usr/lib/aarch64-linux-gnu/lapack'):
    dest = root / 'deps' / directory
    dest.mkdir(parents=True, exist_ok=True)
    for source in (Path('/') / directory).glob('*.so*'):
        target = dest / source.name
        if source.is_file() and not target.exists() and not target.is_symlink():
            target.symlink_to(source)
            records.append({'link': str(target.relative_to(root)), 'system_library': str(source)})
all_links = [{'link': str(p.relative_to(root)), 'system_library': str(p.readlink())}
             for p in (root / 'deps/usr/lib').rglob('*.so*')
             if p.is_symlink() and str(p.readlink()).startswith('/usr/')]
(root / 'evidence/system-library-links.json').write_text(json.dumps(all_links, indent=2) + '\n')
print(f'Linked {len(records)} installed native runtime libraries into local development prefix')
mpl = root / 'deps/usr/lib/python3/dist-packages/matplotlib/mpl-data'
if not mpl.exists():
    mpl.symlink_to(root / 'deps/usr/share/matplotlib/mpl-data')
