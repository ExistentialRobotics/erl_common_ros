#! /usr/bin/bash

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
docker build --rm -t erl/ros-jazzy:cpu-common . \
    --build-arg BASE_IMAGE=erl/common:24.04 \
    --build-arg ROS_APT_SOURCE_VERSION=$ROS_APT_SOURCE_VERSION $@
