#!/bin/bash
# Source this file from Bash for project-local native Debian dependencies.
VIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export VIO_ROOT
export PATH="$VIO_ROOT/deps/usr/bin:$PATH"
export LD_LIBRARY_PATH="$VIO_ROOT/install/lib:$VIO_ROOT/deps/usr/lib:$VIO_ROOT/deps/usr/lib/aarch64-linux-gnu:$VIO_ROOT/deps/usr/lib/aarch64-linux-gnu/blas:$VIO_ROOT/deps/usr/lib/aarch64-linux-gnu/lapack${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$VIO_ROOT/deps/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export CMAKE_PREFIX_PATH="$VIO_ROOT/deps/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export CPLUS_INCLUDE_PATH="$VIO_ROOT/deps/usr/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
export C_INCLUDE_PATH="$VIO_ROOT/deps/usr/include${C_INCLUDE_PATH:+:$C_INCLUDE_PATH}"
export LIBRARY_PATH="$VIO_ROOT/deps/usr/lib/aarch64-linux-gnu:$VIO_ROOT/deps/usr/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export MATPLOTLIBRC="$VIO_ROOT/deps/etc/matplotlibrc"
export MPLCONFIGDIR="$VIO_ROOT/data/matplotlib"
