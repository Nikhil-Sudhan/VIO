#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/rviz_env.sh"
cd "$VIO_ROOT"
exec env LD_PRELOAD="$VIO_ROOT/build/rviz_local_ogre.so" rviz -d "$VIO_ROOT/config/pi-live.rviz" "$@"
