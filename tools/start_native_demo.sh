#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/rviz_env.sh"
cd "$VIO_ROOT"
exec /usr/bin/python3 tools/native_demo.py "$@"
