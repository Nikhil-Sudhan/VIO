#!/bin/bash
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
export ROS_PACKAGE_PATH="$VIO_ROOT/deps/usr/share"
export ROS_MASTER_URI=http://127.0.0.1:11319
export ROS_IP=127.0.0.1
export ROS_HOME="$VIO_ROOT/data/ros-home"
export ROS_LOG_DIR="$VIO_ROOT/data/ros-logs"
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/run/user/1000
export QT_QPA_PLATFORM=xcb
export QT_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins
export VIO_NATIVE_DISPLAY=1
