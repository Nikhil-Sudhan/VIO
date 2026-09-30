#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/env.sh"
cd "$VIO_ROOT"
mkdir -p data
exec >>data/demo-app-console.log 2>&1
exec /usr/bin/python3 tools/demo_app.py --open
