#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
BUILD_ROS_NOETIC=${BUILD_ROS_NOETIC:-true}
BUILD_ROS_HUMBLE=${BUILD_ROS_HUMBLE:-true}
BUILD_ROS1_BRIDGE=${BUILD_ROS1_BRIDGE:-true}

if [ "${BUILD_ROS_NOETIC}" = "true" ] || [ "${BUILD_ROS1_BRIDGE}" = "true" ]; then
  echo "Building ROS image..."
  cd $SCRIPT_DIR/ros-noetic
  ./build.bash $@
  if [ "${BUILD_ROS1_BRIDGE}" = "true" ]; then
    echo "Building ROS1 bridge image..."
    cd $SCRIPT_DIR/ros1-bridge
    ./build.bash $@
  fi
fi

if [ "${BUILD_ROS_HUMBLE}" = "true" ]; then
  echo "Building ROS Humble image..."
  cd $SCRIPT_DIR/ros-humble
  ./build.bash $@
fi
