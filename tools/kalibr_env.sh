#!/bin/bash
# Source after the native Kalibr workspace has been built.
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
source "$VIO_ROOT/build/kalibr-ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$VIO_ROOT/build/kalibr-ws/src:$VIO_ROOT/deps/usr/share${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export MPLBACKEND=Agg
