#!/bin/bash
set -euo pipefail
cd "$(dirname -- "$0")/.."
unset PYTHONPATH LD_LIBRARY_PATH
/usr/bin/python3 tools/fetch_bench_deps.py
/usr/bin/python3 -m venv bench-env
bench-env/bin/python -m pip install --no-index --find-links=data/bench-wheels \
  --require-hashes -r config/bench-requirements.txt
