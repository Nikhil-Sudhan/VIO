#!/bin/bash
# Rebuild the source-backed demo; does not restore recordings or recalibrate sensors.
set -euo pipefail
cd "$(dirname -- "$0")/.."
if [[ "$PWD" != /home/pluto/vio-project ]]; then
  echo 'This checkpoint expects /home/pluto/vio-project and /home/pluto/vio-env.' >&2
  exit 1
fi
case "${1:---check}" in
  --build)
    /usr/bin/python3 -c 'import apt, picamera2, libcamera, PyQt5, PIL'
    mkdir -p evidence data/native-control build vendor
    if [[ ! -x /home/pluto/vio-env/bin/python ]]; then
      /usr/bin/python3 -m venv --system-site-packages /home/pluto/vio-env
    fi
    /home/pluto/vio-env/bin/python -m pip install -r recovery/requirements-demo.txt
    if [[ ! -d vendor/open_vins/.git ]]; then
      git clone https://github.com/rpng/open_vins.git vendor/open_vins
      pinned=$(/usr/bin/python3 -c 'import json; print(json.load(open("config/sources.lock.json"))["openvins"]["commit"])')
      git -C vendor/open_vins checkout --detach "$pinned"
    fi
    /usr/bin/python3 tools/fetch_debian_deps.py --profile openvins
    /usr/bin/python3 tools/fetch_debian_deps.py --profile rviz
    /usr/bin/python3 tools/link_system_libraries.py
    VIO_BUILD_JOBS=1 bash tools/build_openvins.sh
    bash tools/setup_rviz.sh
    ;;
  --check) ;;
  *) echo 'Usage: bash tools/restore_demo.sh [--check|--build]' >&2; exit 2 ;;
esac
source tools/rviz_env.sh
test -x build/adapter/vio_stream
test -f build/rviz_local_ogre.so
test -f install/lib/libov_msckf_lib.so
command -v grim >/dev/null
command -v ffmpeg >/dev/null
command -v rviz >/dev/null
/home/pluto/vio-env/bin/python - <<'PY'
import sys
sys.path.insert(0, 'tools')
import picamera2, libcamera, cv2
from Adafruit_PureIO.smbus import SMBus
from calibration_guard import checked_bundle
checked_bundle('calibration/demo-recovery-20260930/manual-quiet-veto/estimator_config.yaml',
               'config/camera-room-demo.json', experimental=True)
print('Runtime imports, executable files and original calibration bundle check passed.')
print('No sensors opened. Launch: bash tools/start_remote_demo.sh')
PY
