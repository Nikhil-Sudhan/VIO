#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/env.sh"
cd "$VIO_ROOT"
/usr/bin/python3 tools/prepare_kalibr.py
/usr/bin/python3 tools/patch_catkin_prefix.py
cmake -S build/kalibr-ws/src -B build/kalibr-ws/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_DEVEL_PREFIX="$VIO_ROOT/build/kalibr-ws/devel" \
  -DCMAKE_INSTALL_PREFIX="$VIO_ROOT/build/kalibr-ws/install"
cmake --build build/kalibr-ws/build -j"${VIO_BUILD_JOBS:-1}"
