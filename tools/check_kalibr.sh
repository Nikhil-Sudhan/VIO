#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/kalibr_env.sh"
cd "$VIO_ROOT"
/usr/bin/python3 tools/check_kalibr.py
kalibr_source=$(/usr/bin/python3 -c 'import json; print(json.load(open("config/sources.lock.json"))["kalibr"]["source_directory"])')
/usr/bin/g++ -std=c++14 -O2 tests/kalibr_linalg_smoke.cpp \
  -I "$kalibr_source/aslam_incremental_calibration/incremental_calibration/include" \
  -I deps/usr/include/eigen3 -I deps/usr/include/suitesparse \
  -L build/kalibr-ws/devel/lib -L deps/usr/lib/aarch64-linux-gnu \
  -lincremental_calibration -lcholmod -o build/kalibr_linalg_smoke
build/kalibr_linalg_smoke
