#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/env.sh"
cd "$VIO_ROOT"
python3 tools/fetch_debian_deps.py --profile rviz
touch deps/usr/.catkin
python3 - <<'PY'
from pathlib import Path
p=Path('deps/usr/lib')
for lib in (p/'aarch64-linux-gnu/ros').glob('*.so'):
    target=p/lib.name
    if not target.exists():
        target.symlink_to(lib.resolve())
p=Path('deps/usr/share/fonts/truetype/liberation')
p.mkdir(parents=True,exist_ok=True)
for source in [Path('/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'),
               Path('/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf')]:
    target=p/source.name
    if source.exists() and not target.exists():
        target.symlink_to(source)
PY
g++ -shared -fPIC -O2 adapter/rviz_local_ogre.cpp -o build/rviz_local_ogre.so
cmake --build build/adapter -j1
