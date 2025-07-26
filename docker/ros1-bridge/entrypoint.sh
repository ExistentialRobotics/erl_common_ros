#!/usr/bin/sh

. /opt/ros/noetic/setup.sh && \
. /opt/ros/humble/setup.sh && \
. /opt/ros1_bridge/setup.sh && \
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
